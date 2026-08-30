# Add lifespan support for startup/shutdown with strong typing
from contextlib import asynccontextmanager, suppress
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from mcp.server.fastmcp import Context, FastMCP
from typing import Any, Tuple

try:
    # Returning an image means returning this: the SDK base64-encodes it into an
    # ImageContent block. Guarded because a stubbed or older SDK may not have it,
    # and the map tools degrade to returning a file path rather than failing.
    from mcp.server.fastmcp import Image as MCPImage
except ImportError:  # pragma: no cover - depends on the installed SDK
    MCPImage = None
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import OffboardError, PositionNedYaw
from .rc_status_helpers import normalize_rc_status, rc_status_err
from .altitude_helpers import altitude_status_err, normalize_altitude
from .landed_state_helpers import (
    landed_state_status_err,
    normalize_landed_state,
)
from .distance_sensor_helpers import (
    distance_sensor_status_err,
    normalize_distance_sensor,
)
import argparse
import asyncio
import json
import os
import logging
from .endpoint import build_system_address
from .tool_dicts import format_gps_info, format_velocity_ned

from .armed_air_helpers import normalize_in_air, normalize_is_armed, status_err as aa_status_err

from .home_position_helpers import normalize_home_position, status_err as home_status_err
from .attitude_helpers import normalize_attitude_euler, status_err as attitude_status_err
from .health_helpers import normalize_health_flags, status_err as health_status_err
from .wind_helpers import normalize_wind, wind_status_err
from .odometry_helpers import normalize_odometry, odometry_status_err
from .unix_epoch_time_helpers import (
    normalize_unix_epoch_time,
    unix_epoch_time_status_err,
)
from .vtol_state_helpers import (
    normalize_vtol_state,
    vtol_state_status_err,
)

from . import (
    coverage_helpers,
    geo_helpers,
    map_source,
    map_view,
    plan_check_helpers,
    plan_store,
    tile_helpers,
)
from .geo_helpers import validate_lat_lon
from .map_transform import validate_pixels
from .plan_helpers import (
    STATUS_UPLOADED,
    STATUS_VALIDATED,
    build_plan,
    can_upload,
    plan_to_geojson,
    slugify_plan_id,
    utc_now_iso,
    validate_mission_points,
    waypoints_equal,
)

# Configure logger. StreamHandler defaults to stderr, which is what a stdio MCP
# server needs -- stdout carries the protocol. The guard matters because the
# module can be imported more than once in a process (tests reload it), and each
# import would otherwise stack another handler and duplicate every line.
logger = logging.getLogger("MAVLinkMCP")
logger.setLevel(logging.INFO)
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


def clamp_takeoff_altitude(takeoff_altitude: float, min_m: float = 0.5, max_m: float = 120.0) -> float:
    """Clamp takeoff altitude to a safe, finite range for agent-driven takeoff."""
    try:
        alt = float(takeoff_altitude)
    except (TypeError, ValueError) as e:
        raise ValueError(f"takeoff_altitude must be a number: {e}") from e
    if alt != alt:  # NaN
        raise ValueError("takeoff_altitude must be finite (got NaN)")
    if alt == float("inf") or alt == float("-inf"):
        raise ValueError("takeoff_altitude must be finite")
    if alt < min_m:
        return min_m
    if alt > max_m:
        return max_m
    return alt


def validate_relative_move(lr: float, fb: float, altitude: float, yaw: float, max_abs_m: float = 500.0) -> None:
    """Reject non-finite or absurdly large relative move requests (fail-closed)."""
    for name, val in (("lr", lr), ("fb", fb), ("altitude", altitude), ("yaw", yaw)):
        try:
            v = float(val)
        except (TypeError, ValueError) as e:
            raise ValueError(f"{name} must be a number: {e}") from e
        if v != v or v in (float("inf"), float("-inf")):
            raise ValueError(f"{name} must be finite")
        if name != "yaw" and abs(v) > max_abs_m:
            raise ValueError(f"{name} magnitude {abs(v)} exceeds max_abs_m={max_abs_m}")


def tool_ok(payload=None, **extra):
    """Structured success payload for MCP tool clients.

    Accepts either a positional dict (``tool_ok({"disarmed": True})``) or
    keyword fields (``tool_ok(armed=True)``); both merge into the result.
    """
    out = {"status": "success"}
    if isinstance(payload, dict):
        out.update(payload)
    elif payload is not None:
        out["result"] = payload
    out.update(extra)
    return out


def tool_err(message, **payload):
    """Structured failure payload for MCP tool clients (fail-closed)."""
    return {"status": "failed", "error": str(message), **payload}


def _image_result(path, payload):
    """Return ``[Image, dict]``: the picture, then the numbers describing it.

    The base64 encoding happens inside the SDK's Image type, which emits an
    ImageContent block. Putting base64 in the dict instead would produce a text
    block that most clients will not render and that costs roughly fifty times
    more tokens than the same pixels as an image.

    ``MAVLINKMCP_MAP_IMAGE_MODE=path`` returns just the dict with a local file
    path, for clients that cannot display images at all.
    """
    mode = os.environ.get("MAVLINKMCP_MAP_IMAGE_MODE", "image").strip().lower()
    if MCPImage is None or mode == "path":
        return {**payload, "image_mode": "path"}
    return [MCPImage(path=path), payload]


def clamp_imu_count(n, min_n: int = 1, max_n: int = 100) -> int:
    """Clamp requested IMU sample count to a safe inclusive range."""
    try:
        # bool is int subclass; reject explicitly for agent clarity
        if isinstance(n, bool):
            raise ValueError("n must be an integer count, not bool")
        v = int(n)
    except (TypeError, ValueError) as e:
        raise ValueError(f"imu count must be an integer: {e}") from e
    if v < min_n:
        return min_n
    if v > max_n:
        return max_n
    return v


def connect_timeout_s(value: str | float | None = None) -> float:
    """Seconds to keep trying the MAVLink link before declaring it unreachable."""
    if value is None:
        value = os.environ.get("MAVLINK_CONNECT_TIMEOUT", "60")
    try:
        t = float(str(value).strip())
    except (TypeError, ValueError) as e:
        raise ValueError(f"MAVLINK_CONNECT_TIMEOUT must be a number: {value!r}") from e
    if t != t:  # NaN
        raise ValueError("MAVLINK_CONNECT_TIMEOUT must be finite (got NaN)")
    if t in (float("inf"), float("-inf")):
        raise ValueError("MAVLINK_CONNECT_TIMEOUT must be finite")
    if t <= 0:
        raise ValueError(f"MAVLINK_CONNECT_TIMEOUT must be positive: {t}")
    return t


# Link states for MAVLinkConnector.link_state
LINK_CONNECTING = "connecting"
LINK_READY = "ready"
LINK_FAILED = "failed"


@dataclass
class MAVLinkConnector:
    drone: System
    last_offboard_position: PositionNedYaw = field(default_factory=lambda: PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    link_state: str = LINK_CONNECTING
    link_error: str | None = None

    @property
    def is_linked(self) -> bool:
        """True once the vehicle answered and tools may talk to it."""
        return self.link_state == LINK_READY

    def link_failure(self) -> str:
        """Why tools cannot run right now (empty string when they can)."""
        if self.link_state == LINK_READY:
            return ""
        if self.link_state == LINK_CONNECTING:
            return "MAVLink link is still coming up; retry shortly"
        return self.link_error or "MAVLink link unavailable"


def resolve_connector(ctx: Context) -> Tuple[MAVLinkConnector | None, dict | None]:
    """Return ``(connector, None)`` when the link is up, else ``(None, error)``.

    Tools go through this instead of reaching into the lifespan context, so an
    absent vehicle yields a structured failure rather than a MAVSDK call that
    blocks forever.
    """
    connector = ctx.request_context.lifespan_context
    if not connector.is_linked:
        return None, tool_err(
            connector.link_failure(), connected=False, link_state=connector.link_state
        )
    return connector, None


def resolve_drone(ctx: Context) -> Tuple[System | None, dict | None]:
    """Return ``(drone, None)`` when the link is up, else ``(None, error)``."""
    connector, link_err = resolve_connector(ctx)
    if link_err is not None:
        return None, link_err
    return connector.drone, None


async def wait_for_connection(drone: System) -> None:
    """Block until the autopilot reports a live connection."""
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def wait_for_position_estimate(drone: System) -> None:
    """Block until the vehicle has a global or home position estimate."""
    async for health in drone.telemetry.health():
        if health.is_global_position_ok or health.is_home_position_ok:
            logger.info(
                "Global position %s, home position %s",
                health.is_global_position_ok,
                health.is_home_position_ok,
            )
            return


async def connect_vehicle(drone: System, system_address: str) -> None:
    """Open the MAVLink link and wait for the autopilot to answer."""
    logger.info("Connecting to drone at %s", system_address)
    await drone.connect(system_address=system_address)

    logger.info("Waiting for drone to connect at %s", system_address)
    await wait_for_connection(drone)


async def bring_up_link(connector: MAVLinkConnector, system_address: str, timeout_s: float) -> None:
    """Establish the link in the background and record the outcome.

    Runs outside the handshake path, so a slow or absent vehicle delays tools
    rather than the MCP session itself.
    """
    try:
        await asyncio.wait_for(connect_vehicle(connector.drone, system_address), timeout_s)
    except asyncio.TimeoutError:
        connector.link_state = LINK_FAILED
        connector.link_error = (
            f"no MAVLink vehicle reachable on {system_address} after {timeout_s:g}s"
        )
        logger.error("%s; tools will fail closed", connector.link_error)
        return
    except Exception as e:
        connector.link_state = LINK_FAILED
        connector.link_error = f"MAVLink link to {system_address} failed: {e}"
        logger.error("%s; tools will fail closed", connector.link_error)
        return

    connector.link_state = LINK_READY
    logger.info("Connected to drone at %s!", system_address)

    # A position estimate is worth waiting for but must not gate tools: GPS can
    # keep converging long after the vehicle is reachable and commandable.
    logger.info("Waiting for drone to have a global position estimate...")
    try:
        await asyncio.wait_for(wait_for_position_estimate(connector.drone), timeout_s)
    except asyncio.TimeoutError:
        logger.warning(
            "No global/home position estimate after %gs; position tools may fail", timeout_s
        )
    except Exception as e:
        logger.warning("Could not read position health: %s", e)


@asynccontextmanager
async def app_lifespan(server: FastMCP) -> AsyncIterator[MAVLinkConnector]:
    """Manage application lifecycle with type-safe context.

    The link is brought up in the background on purpose. An MCP client blocks on
    the ``initialize`` handshake until this context manager yields, and on the
    HTTP transports the SDK enters the lifespan before anything drains the
    session's read stream -- so connecting here would wedge the handshake until
    the client gave up and tore the session down. Instead we yield at once and
    let tools fail closed until the vehicle answers.
    """
    # Initialize on startup
    system_address = build_system_address()
    timeout_s = connect_timeout_s()
    connector = MAVLinkConnector(drone=System())

    link_task = asyncio.create_task(bring_up_link(connector, system_address, timeout_s))
    try:
        yield connector
    finally:
        # Cleanup on shutdown
        link_task.cancel()
        with suppress(asyncio.CancelledError):
            await link_task
        logger.info("Disconnecting drone")
        try:
            await connector.drone.close()
        except Exception as e:
            logger.warning("Error closing drone connection: %s", e)


# Pass lifespan to server

def normalize_heading_deg(heading_deg: float) -> float:
    """Normalize heading in degrees to the half-open interval [0, 360)."""
    if heading_deg != heading_deg:  # NaN
        raise ValueError("heading_deg must be finite")
    if heading_deg in (float("inf"), float("-inf")):
        raise ValueError("heading_deg must be finite")
    h = float(heading_deg) % 360.0
    # Python % can yield -0.0; force [0, 360)
    if h < 0:
        h += 360.0
    return h if h < 360.0 else 0.0


def format_battery(remaining_percent: float | None = None, remaining_fraction: float | None = None, voltage_v: float | None = None) -> dict:
    """Build a battery dict from MAVSDK-like fields; reject nonsense remaining values when provided."""
    out = {}
    if remaining_fraction is not None:
        rf = float(remaining_fraction)
        if rf != rf or rf in (float("inf"), float("-inf")):
            raise ValueError("remaining_fraction must be finite")
        if not (0.0 <= rf <= 1.0):
            raise ValueError("remaining_fraction must be between 0 and 1")
        out["remaining_fraction"] = rf
    if remaining_percent is not None:
        rp = float(remaining_percent)
        if rp != rp or rp in (float("inf"), float("-inf")):
            raise ValueError("remaining_percent must be finite")
        if not (0.0 <= rp <= 100.0):
            raise ValueError("remaining_percent must be between 0 and 100")
        out["remaining_percent"] = rp
    if voltage_v is not None:
        v = float(voltage_v)
        if v != v or v in (float("inf"), float("-inf")):
            raise ValueError("voltage_v must be finite")
        if v < 0:
            raise ValueError("voltage_v must be non-negative")
        out["voltage_v"] = v
    if not out:
        raise ValueError("battery payload empty")
    return out


mcp = FastMCP("MAVLink MCP", lifespan=app_lifespan)


# ARM
@mcp.tool()
async def arm_drone(ctx: Context) -> dict:
    """Arm the drone. Returns a structured status dict (fail-closed on errors)."""
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Arming")
    try:
        await drone.action.arm()
        return tool_ok(armed=True)
    except Exception as e:
        logger.error("Arm failed: %s", e)
        return tool_err(e, armed=False)


# Get Position
@mcp.tool()
async def get_position(ctx: Context) -> dict:
    """
    Get the position of the drone in latitude/longitude degrees and altitude in meters.
    The drone must be connected and have a global position estimate.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: A dict with the position.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching drone position")

    try:
        async for position in drone.telemetry.position():
            return {"status": "success", "position": {
                "latitude_deg": position.latitude_deg,
                "longitude_deg": position.longitude_deg,
                "absolute_altitude_m": position.absolute_altitude_m,
                "relative_altitude_m": position.relative_altitude_m
            }}
    except Exception as e:
        logger.error(f"Failed to retrieve position: {e}")
        return tool_err(e)

async def start_offboard_mode(connector: MAVLinkConnector) -> bool:
    """
    Start the offboard mode for the drone and set the initial NED position.

    Args:
        connector (MAVLinkConnector): The MAVLinkConnector instance.

    Returns:
        bool: True if offboard mode was started successfully, False otherwise.
    """
    drone = connector.drone
    logger.info("Setting initial setpoint for offboard mode")
    await drone.offboard.set_position_ned(connector.last_offboard_position)

    logger.info("Starting offboard mode")
    try:
        await drone.offboard.start()
        logger.info("Offboard mode started successfully")
        return True
    except OffboardError as error:
        logger.error(f"Starting offboard mode failed with error code: {error._result.result}")
        return False

async def stop_offboard_mode(connector: MAVLinkConnector) -> bool:
    """
    Stop the offboard mode for the drone.

    Args:
        connector (MAVLinkConnector): The MAVLinkConnector instance.

    Returns:
        bool: True if offboard mode was stopped successfully, False otherwise.
    """
    drone = connector.drone
    logger.info("Stopping offboard mode")

    try:
        await drone.offboard.stop()
        logger.info("Offboard mode stopped successfully")
        return True
    except OffboardError as error:
        logger.error(f"Stopping offboard mode failed with error code: {error._result.result}")
        return False

@mcp.tool()
async def move_to_relative(ctx: Context, lr: float, fb: float, altitude: float, yaw: float) -> dict:
    """
    Move the drone relative to the current position. The drone must be armed and offboard mode must be active.

    Validates finite deltas and rejects absolute components over 500 m (fail-closed).

    Args:
        ctx (Context): the context.
        lr (float): distance in left/right axis. right is the positive.
        fb (float): distance along front/back axis. front is positive.
        altitude (float): the altitude relative to the current point.
        yaw (float): yaw change.

    Returns:
        dict: structured status (success or error).
    """
    connector, link_err = resolve_connector(ctx)
    if link_err is not None:
        return link_err
    drone = connector.drone

    try:
        validate_relative_move(lr, fb, altitude, yaw)
    except ValueError as e:
        logger.error("Invalid relative move: %s", e)
        return tool_err(e)

    # Activate offboard mode
    if not await start_offboard_mode(connector):
        return tool_err("failed to start offboard mode")

    try:
        # Update the last offboard position
        connector.last_offboard_position.north_m += fb
        connector.last_offboard_position.east_m += lr
        connector.last_offboard_position.down_m += -altitude
        connector.last_offboard_position.yaw_deg += yaw

        # Send the updated position
        logger.info(f"Sending updated offboard position: {connector.last_offboard_position}")
        await drone.offboard.set_position_ned(connector.last_offboard_position)
        return tool_ok(
            north_m=connector.last_offboard_position.north_m,
            east_m=connector.last_offboard_position.east_m,
            down_m=connector.last_offboard_position.down_m,
            yaw_deg=connector.last_offboard_position.yaw_deg,
        )
    except Exception as e:
        logger.error("Relative move failed: %s", e)
        return tool_err(e)

@mcp.tool()
async def takeoff(ctx: Context, takeoff_altitude: float = 3.0) -> dict:
    """Command the drone to initiate takeoff and ascend to a specified altitude. The drone must be armed.

    Args:
        ctx (Context): The context of the request.
        takeoff_altitude (float): Altitude in meters after takeoff. Default is 3.0 m.
            Values are clamped to [0.5, 120.0] for fail-closed agent use.

    Returns:
        dict: structured status including the altitude actually commanded.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        altitude = clamp_takeoff_altitude(takeoff_altitude)
    except ValueError as e:
        logger.error("Invalid takeoff altitude: %s", e)
        return tool_err(e)

    logger.info("Initiating takeoff to %s m", altitude)
    try:
        await drone.action.set_takeoff_altitude(altitude)
        await drone.action.takeoff()
        return tool_ok(takeoff_altitude_m=altitude)
    except Exception as e:
        logger.error("Takeoff failed: %s", e)
        return tool_err(e)

@mcp.tool()
async def land(ctx: Context) -> dict:
    """Command the drone to initiate landing at its current location.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: structured status (success or error).
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Initiating landing")
    try:
        await drone.action.land()
        return tool_ok(landing=True)
    except Exception as e:
        logger.error("Land failed: %s", e)
        return tool_err(e)

@mcp.tool()
async def print_status_text(ctx: Context) -> dict:
    """Print and return status text from the drone (structured status on failure)."""
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for status_text in drone.telemetry.status_text():
            logger.info(f"Status: {status_text.type}: {status_text.text}")
            return tool_ok(type=str(status_text.type), text=status_text.text)
        return tool_err("no status text available")
    except asyncio.CancelledError:
        return tool_err("status text stream cancelled")
    except Exception as e:
        logger.error("status text failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def get_imu(ctx: Context, n: int = 1) -> dict:
    """Fetch the first n IMU data points from the drone.

    Args:
        ctx (Context): The context of the request.
        n (int): Number of IMU samples (clamped to [1, 100] for fail-closed agent use).

    Returns:
        dict: structured status with imu list and count, or error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    telemetry = drone.telemetry

    try:
        count_target = clamp_imu_count(n)
    except ValueError as e:
        logger.error("Invalid IMU count: %s", e)
        return tool_err(e)

    try:
        # Set the rate at which IMU data is updated (in Hz)
        await telemetry.set_rate_imu(200.0)

        imu_data = []
        count = 0

        async for imu in telemetry.imu():
            imu_data.append({
                "timestamp_us": imu.timestamp_us,
                "acceleration": {
                    "x": imu.acceleration_frd.forward_m_s2,
                    "y": imu.acceleration_frd.right_m_s2,
                    "z": imu.acceleration_frd.down_m_s2
                },
                "angular_velocity": {
                    "x": imu.angular_velocity_frd.forward_rad_s,
                    "y": imu.angular_velocity_frd.right_rad_s,
                    "z": imu.angular_velocity_frd.down_rad_s
                },
                "magnetic_field": {
                    "x": imu.magnetic_field_frd.forward_gauss,
                    "y": imu.magnetic_field_frd.right_gauss,
                    "z": imu.magnetic_field_frd.down_gauss
                },
                "temperature_degc": imu.temperature_degc
            })
            count += 1
            if count >= count_target:
                break

        return tool_ok(imu=imu_data, count=len(imu_data))
    except Exception as e:
        logger.error("IMU fetch failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def print_mission_progress(ctx: Context) -> dict:
    """
    Print and return the current mission progress of the drone.

    Returns:
        dict: structured status with current/total, or error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for mission_progress in drone.mission.mission_progress():
            logger.info(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
            return tool_ok(current=mission_progress.current, total=mission_progress.total)
        return tool_err("no mission progress available")
    except Exception as e:
        logger.error("mission progress failed: %s", e)
        return tool_err(e)




@mcp.tool()
async def initiate_mission(ctx: Context, mission_points: list, return_to_launch: bool = True) -> dict:
    """
    Initiate a mission with a list of mission points. The drone must be armed.

    Validates waypoints (non-empty, lat/lon bounds, finite altitude/speed caps) and
    returns a structured status dict (fail-closed for agent use).

    Args:
        ctx (Context): The context of the request.
        mission_points (list): Waypoint dicts with latitude_deg, longitude_deg,
            relative_altitude_m, speed_m_s, is_fly_through (+ optional MissionItem fields).
        return_to_launch (bool): Whether to return to launch after completing the mission.

    Returns:
        dict: structured status (success with waypoint_count, or error).
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err

    try:
        points = validate_mission_points(mission_points)
    except ValueError as e:
        logger.error("Mission validation failed: %s", e)
        return tool_err(e)

    mission_items = []
    for point in points:
        mission_items.append(MissionItem(
            latitude_deg=point["latitude_deg"],
            longitude_deg=point["longitude_deg"],
            relative_altitude_m=point["relative_altitude_m"],
            speed_m_s=point["speed_m_s"],
            is_fly_through=point["is_fly_through"],
            gimbal_pitch_deg=point.get("gimbal_pitch_deg", float('nan')),
            gimbal_yaw_deg=point.get("gimbal_yaw_deg", float('nan')),
            camera_action=point.get("camera_action", MissionItem.CameraAction.NONE),
            loiter_time_s=point.get("loiter_time_s", float('nan')),
            camera_photo_interval_s=point.get("camera_photo_interval_s", float('nan')),
            acceptance_radius_m=point.get("acceptance_radius_m", float('nan')),
            yaw_deg=point.get("yaw_deg", float('nan')),
            camera_photo_distance_m=point.get("camera_photo_distance_m", float('nan')),
            vehicle_action=point.get("vehicle_action", MissionItem.VehicleAction.NONE)
        ))

    mission_plan = MissionPlan(mission_items)

    try:
        await drone.mission.set_return_to_launch_after_mission(return_to_launch)
        logger.info("Uploading mission (%s waypoints)", len(mission_items))
        await drone.mission.upload_mission(mission_plan)
        logger.info("Starting mission")
        await drone.mission.start_mission()
        return tool_ok(waypoint_count=len(mission_items), return_to_launch=bool(return_to_launch))
    except Exception as e:
        logger.error("Mission upload/start failed: %s", e)
        return tool_err(e)









@mcp.tool()
async def get_home_position(ctx: Context) -> dict:
    """
    Return the vehicle home position as a fail-closed dict.

    Uses mavsdk telemetry.home() when available.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for home in drone.telemetry.home():
            return normalize_home_position(
                home.latitude_deg,
                home.longitude_deg,
                home.absolute_altitude_m,
            )
        return home_status_err("no_home_telemetry")
    except Exception as e:
        logger.error("Failed to retrieve home position: %s", e)
        return home_status_err(str(e))


@mcp.tool()
async def get_is_armed(ctx: Context) -> dict:
    """Return whether the vehicle reports armed=True (fail-closed dict)."""
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for armed in drone.telemetry.armed():
            return normalize_is_armed(bool(armed))
        return aa_status_err("no_armed_telemetry")
    except Exception as e:
        logger.error("Failed to retrieve armed state: %s", e)
        return aa_status_err(str(e))


@mcp.tool()
async def get_in_air(ctx: Context) -> dict:
    """Return whether the vehicle reports in_air=True (fail-closed dict)."""
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for in_air in drone.telemetry.in_air():
            return normalize_in_air(bool(in_air))
        return aa_status_err("no_in_air_telemetry")
    except Exception as e:
        logger.error("Failed to retrieve in_air state: %s", e)
        return aa_status_err(str(e))


@mcp.tool()
async def get_velocity_ned(ctx: Context) -> dict:
    """
    Get NED velocity of the drone (north/east/down m/s). Fail-closed structured dict.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: success payload with velocity_ned or failed error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching velocity NED")
    try:
        async for velocity in drone.telemetry.velocity_ned():
            return tool_ok({"velocity_ned": format_velocity_ned(velocity)})
    except Exception as e:
        logger.error(f"Failed to retrieve velocity NED: {e}")
        return tool_err(e)


@mcp.tool()
async def get_gps_info(ctx: Context) -> dict:
    """
    Get GPS satellite / fix info. Fail-closed structured dict.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: success payload with gps_info or failed error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching GPS info")
    try:
        async for info in drone.telemetry.gps_info():
            return tool_ok({"gps_info": format_gps_info(info)})
    except Exception as e:
        logger.error(f"Failed to retrieve GPS info: {e}")
        return tool_err(e)


@mcp.tool()
async def get_battery(ctx: Context) -> dict:
    """Get a single battery telemetry sample (remaining fraction / voltage when available).

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: status + battery fields, or failed with error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching battery")
    try:
        async for bat in drone.telemetry.battery():
            remaining = getattr(bat, "remaining_percent", None)
            voltage = getattr(bat, "voltage_v", None)
            # MAVSDK uses 0-1 fractional remaining_percent historically on some versions
            payload = {}
            if remaining is not None:
                r = float(remaining)
                if 0.0 <= r <= 1.0:
                    payload = format_battery(remaining_fraction=r, voltage_v=voltage)
                else:
                    payload = format_battery(remaining_percent=r, voltage_v=voltage)
            elif voltage is not None:
                payload = format_battery(voltage_v=voltage)
            else:
                return tool_err("battery sample empty")
            return tool_ok({"battery": payload})
    except Exception as e:
        logger.error(f"Failed to retrieve battery: {e}")
        return tool_err(e)


@mcp.tool()
async def get_heading(ctx: Context) -> dict:
    """Get a single heading (degrees) sample from drone telemetry.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: status + heading_deg normalized to [0, 360), or failed with error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching heading")
    try:
        # Prefer dedicated heading stream; fall back to attitude Euler yaw_deg
        tele = drone.telemetry
        if hasattr(tele, "heading"):
            async for h in tele.heading():
                deg = getattr(h, "heading_deg", None)
                if deg is None:
                    deg = getattr(h, "deg", None)
                if deg is None:
                    return tool_err("heading sample missing degrees")
                return tool_ok({"heading_deg": normalize_heading_deg(float(deg))})
        async for att in tele.attitude_euler():
            yaw = getattr(att, "yaw_deg", None)
            if yaw is None:
                return tool_err("attitude sample missing yaw_deg")
            return tool_ok({"heading_deg": normalize_heading_deg(float(yaw)), "source": "attitude_euler.yaw_deg"})
    except Exception as e:
        logger.error(f"Failed to retrieve heading: {e}")
        return tool_err(e)


@mcp.tool()
async def disarm_drone(ctx: Context) -> dict:
    """Disarm the drone when it is safe to do so (typically on ground).

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: status success or failed with error message.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Disarming")
    try:
        await drone.action.disarm()
        return tool_ok({"disarmed": True})
    except Exception as e:
        logger.error(f"Disarm failed: {e}")
        return tool_err(e)


@mcp.tool()
async def return_to_launch(ctx: Context) -> dict:
    """Command the drone to return to launch (RTL).

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: status success or failed with error message.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Return to launch")
    try:
        await drone.action.return_to_launch()
        return tool_ok({"rtl": True})
    except Exception as e:
        logger.error(f"RTL failed: {e}")
        return tool_err(e)


@mcp.tool()
async def get_flight_mode(ctx: Context) -> dict:
    """
    Get the current flight mode of the drone.

    Returns:
        dict: structured status with mode string, or error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        flight_mode = await drone.telemetry.flight_mode().__anext__()
        logger.info(f"FlightMode: {flight_mode}")
        return tool_ok(mode=str(flight_mode))
    except StopAsyncIteration:
        logger.error("Failed to retrieve flight mode")
        return tool_err("no flight mode available", mode="Unknown")
    except Exception as e:
        logger.error("flight mode failed: %s", e)
        return tool_err(e)




@mcp.tool()
async def get_attitude_euler(ctx: Context) -> dict:
    """
    Get the drone attitude as Euler angles (roll/pitch/yaw degrees).

    Returns a structured dict for LLM/MCP clients. Non-finite samples fail closed.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching attitude_euler")
    try:
        async for att in drone.telemetry.attitude_euler():
            return normalize_attitude_euler(
                att.roll_deg,
                att.pitch_deg,
                att.yaw_deg,
            )
        return attitude_status_err("no attitude_euler samples")
    except Exception as e:
        logger.error("Failed to retrieve attitude_euler: %s", e)
        return attitude_status_err(str(e))



@mcp.tool()
async def get_health(ctx: Context) -> dict:
    """
    Get vehicle health / readiness flags (calibration, position, armable).

    Structured dict for MCP agents. Missing stream fails closed.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching health telemetry")
    try:
        async for h in drone.telemetry.health():
            flags = {
                "is_gyrometer_calibration_ok": h.is_gyrometer_calibration_ok,
                "is_accelerometer_calibration_ok": h.is_accelerometer_calibration_ok,
                "is_magnetometer_calibration_ok": h.is_magnetometer_calibration_ok,
                "is_local_position_ok": h.is_local_position_ok,
                "is_global_position_ok": h.is_global_position_ok,
                "is_home_position_ok": h.is_home_position_ok,
                "is_armable": h.is_armable,
            }
            return normalize_health_flags(flags)
        return health_status_err("no health samples")
    except Exception as e:
        logger.error("Failed to retrieve health: %s", e)
        return health_status_err(str(e))




@mcp.tool()
async def get_rc_status(ctx: Context) -> dict:
    """
    Get RC link status (availability + signal strength percent).

    Structured dict for MCP agents. Missing stream fails closed.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching RC status telemetry")
    try:
        async for rc in drone.telemetry.rc_status():
            flags = {
                "was_available_once": rc.was_available_once,
                "is_available": rc.is_available,
                "signal_strength_percent": rc.signal_strength_percent,
            }
            return normalize_rc_status(flags)
        return rc_status_err("no rc_status samples")
    except Exception as e:
        logger.error("Failed to retrieve rc_status: %s", e)
        return rc_status_err(str(e))




@mcp.tool()
async def get_altitude(ctx: Context) -> dict:
    """
    Get altitude telemetry (AMSL, relative, optional local/terrain).

    Structured dict for MCP agents. Core AMSL/relative must be finite.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching altitude telemetry")
    try:
        async for alt in drone.telemetry.altitude():
            fields = {
                "altitude_amsl_m": alt.altitude_amsl_m,
                "altitude_local_m": alt.altitude_local_m,
                "altitude_relative_m": alt.altitude_relative_m,
                "altitude_terrain_m": alt.altitude_terrain_m,
            }
            return normalize_altitude(fields)
        return altitude_status_err("no altitude samples")
    except Exception as e:
        logger.error("Failed to retrieve altitude: %s", e)
        return altitude_status_err(str(e))


@mcp.tool()
async def get_landed_state(ctx: Context) -> dict:
    """
    Get landed state telemetry (ON_GROUND / IN_AIR / TAKING_OFF / LANDING / UNKNOWN).

    Structured fail-closed dict for MCP agents (gates arm / takeoff / land).
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching landed_state telemetry")
    try:
        async for state in drone.telemetry.landed_state():
            return normalize_landed_state(state)
        return landed_state_status_err("no landed_state samples")
    except Exception as e:
        logger.error("Failed to retrieve landed_state: %s", e)
        return landed_state_status_err(str(e))


@mcp.tool()
async def get_distance_sensor(ctx: Context) -> dict:
    """
    Get rangefinder / distance sensor sample.

    Structured fail-closed dict: current_distance_m required finite >= 0.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching distance_sensor telemetry")
    try:
        async for ds in drone.telemetry.distance_sensor():
            fields = {
                "minimum_distance_m": getattr(ds, "minimum_distance_m", None),
                "maximum_distance_m": getattr(ds, "maximum_distance_m", None),
                "current_distance_m": getattr(ds, "current_distance_m", None),
            }
            orientation = getattr(ds, "orientation", None)
            if orientation is not None:
                fields["orientation"] = orientation
            return normalize_distance_sensor(fields)
        return distance_sensor_status_err("no distance_sensor samples")
    except Exception as e:
        logger.error("Failed to retrieve distance_sensor: %s", e)
        return distance_sensor_status_err(str(e))




@mcp.tool()
async def get_wind(ctx: Context) -> dict:
    """
    Get wind estimate (NED components when available).

    Structured fail-closed dict; finite wind_*_ned_m_s required when present.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching wind telemetry")
    try:
        async for w in drone.telemetry.wind():
            fields = {
                "wind_x_ned_m_s": getattr(w, "wind_x_ned_m_s", None),
                "wind_y_ned_m_s": getattr(w, "wind_y_ned_m_s", None),
                "wind_z_ned_m_s": getattr(w, "wind_z_ned_m_s", None),
            }
            # Some mavsdk versions expose speed_horizontal / direction
            for alt_src, alt_dst in (
                ("speed_m_s", "speed_m_s"),
                ("direction_deg", "direction_deg"),
                ("wind_speed_m_s", "speed_m_s"),
                ("direction_from_north_deg", "direction_deg"),
            ):
                if hasattr(w, alt_src) and getattr(w, alt_src) is not None:
                    if alt_dst not in fields or fields.get(alt_dst) is None:
                        fields[alt_dst] = getattr(w, alt_src)
            return normalize_wind(fields)
        return wind_status_err("no wind samples")
    except Exception as e:
        logger.error("Failed to retrieve wind: %s", e)
        return wind_status_err(str(e))




@mcp.tool()
async def get_odometry(ctx: Context) -> dict:
    """
    Get vehicle odometry sample (position_body required finite XYZ).

    Structured fail-closed dict for agent/navigation consumers.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    logger.info("Fetching odometry telemetry")
    try:
        async for odom in drone.telemetry.odometry():
            fields = {
                "position_body": getattr(odom, "position_body", None),
                "velocity_body": getattr(odom, "velocity_body", None),
            }
            for key in ("frame_id", "child_frame_id"):
                if hasattr(odom, key):
                    fields[key] = getattr(odom, key)
            return normalize_odometry(fields)
        return odometry_status_err("no odometry samples")
    except Exception as e:
        logger.error("Failed to retrieve odometry: %s", e)
        return odometry_status_err(str(e))



@mcp.tool()
async def get_unix_epoch_time(ctx: Context) -> dict:
    """Get vehicle unix epoch time as a fail-closed dictionary.

    Returns:
        dict: {"status": "success", "unix_epoch_time": {"unix_epoch_s": float}}
              or {"status": "failed", "error": str}
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for sample in drone.telemetry.unix_epoch_time():
            return normalize_unix_epoch_time(sample)
        return unix_epoch_time_status_err("no unix_epoch_time samples")
    except Exception as e:
        logger.error(f"Failed to retrieve unix_epoch_time: {e}")
        return unix_epoch_time_status_err(str(e))



@mcp.tool()
async def get_vtol_state(ctx: Context) -> dict:
    """Get VTOL state (UNDEFINED/TRANSITION_TO_FW/TRANSITION_TO_MC/MC/FW). Fail-closed dict.

    Returns:
        dict: {"status": "success", "vtol_state": str} or {"status": "failed", "error": str}
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        async for state in drone.telemetry.vtol_state():
            return normalize_vtol_state(state)
        return vtol_state_status_err("no vtol_state samples")
    except Exception as e:
        logger.error(f"Failed to retrieve vtol_state: {e}")
        return vtol_state_status_err(str(e))


LOCAL_HOSTS = ("127.0.0.1", "localhost", "::1")


# =====================================================================
# Flight plans: see the ground, draw the path, fly it.
#
# The pipeline is deliberately split so a route is reviewable before it
# reaches the vehicle:
#
#   get_map_view -> create_survey_plan -> render_plan_view
#     -> validate_plan -> preflight_check -> upload_plan
#     -> verify_uploaded_plan -> start_mission
#
# Every authoring tool below works with the link down, so plans can be
# built and costed at a desk and flown later.
# =====================================================================


def _plans_root():
    """Resolve the data root, converting a config error into a tool error."""
    return plan_store.data_root()


def _load_plan_or_err(plan_id, revision=None):
    """Return ``(plan, None)`` or ``(None, error_dict)``."""
    try:
        return plan_store.load_plan(plan_id, revision), None
    except (ValueError, FileNotFoundError) as e:
        return None, tool_err(e)
    except (OSError, json.JSONDecodeError) as e:
        return None, tool_err(f"plan {plan_id!r} is unreadable: {e}")


def _mission_items_from(waypoints):
    """Map stored JSON waypoints onto MAVSDK MissionItems.

    Storage keeps ``camera_action`` as a string and omits unset floats, because
    a plan has to survive a JSON round-trip. MAVSDK wants the enum and NaN, so
    the translation happens here — the one place that knows both vocabularies.
    """
    items = []
    for point in waypoints:
        action_name = point.get("camera_action") or "NONE"
        try:
            camera_action = getattr(MissionItem.CameraAction, action_name)
        except AttributeError as e:
            raise ValueError(f"unsupported camera_action {action_name!r}") from e
        try:
            vehicle_action = getattr(
                MissionItem.VehicleAction, point.get("vehicle_action") or "NONE"
            )
        except AttributeError:
            vehicle_action = MissionItem.VehicleAction.NONE

        def opt(key):
            value = point.get(key)
            return float("nan") if value is None else float(value)

        items.append(
            MissionItem(
                latitude_deg=point["latitude_deg"],
                longitude_deg=point["longitude_deg"],
                relative_altitude_m=point["relative_altitude_m"],
                speed_m_s=point["speed_m_s"],
                is_fly_through=point["is_fly_through"],
                gimbal_pitch_deg=opt("gimbal_pitch_deg"),
                gimbal_yaw_deg=opt("gimbal_yaw_deg"),
                camera_action=camera_action,
                loiter_time_s=opt("loiter_time_s"),
                camera_photo_interval_s=opt("camera_photo_interval_s"),
                acceptance_radius_m=opt("acceptance_radius_m"),
                yaw_deg=opt("yaw_deg"),
                camera_photo_distance_m=opt("camera_photo_distance_m"),
                vehicle_action=vehicle_action,
            )
        )
    return items


async def _current_position(drone):
    """One position sample as ``(lat, lon, rel_alt_m)``, or None."""
    async for position in drone.telemetry.position():
        return (
            position.latitude_deg,
            position.longitude_deg,
            position.relative_altitude_m,
        )
    return None


async def _current_heading(drone):
    """One heading sample in degrees, or None when the vehicle reports none."""
    try:
        async for heading in drone.telemetry.heading():
            return normalize_heading_deg(heading.heading_deg)
    except Exception as e:  # noqa: BLE001 - heading is optional context
        logger.info("heading unavailable for map view: %s", e)
    return None


# -- perception -------------------------------------------------------


@mcp.tool()
async def get_map_view(
    ctx: Context,
    latitude_deg: float | None = None,
    longitude_deg: float | None = None,
    radius_m: float = 300.0,
    orientation: str = "north_up",
    size_px: int = 768,
    grid: bool = True,
) -> Any:
    """Return a georeferenced satellite view as an image, plus its exact transform.

    Omit the coordinates to centre the view on the vehicle (this needs the link);
    supply them to look anywhere, which works with no vehicle at all.

    Identify features in the image and report their positions as **pixels**, then
    convert them with map_transform or hand them straight to create_survey_plan
    as polygon_pixels. Do not estimate latitude and longitude by eye — the
    returned transform is exact and reading coordinates off the picture is not.

    Args:
        latitude_deg, longitude_deg: view centre; omit to use the vehicle.
        radius_m: half-width of the ground area to cover (10..20000).
        orientation: "north_up" (default) or "heading_up", which rotates the
            view so the vehicle's forward direction is up — use it for questions
            like "the field in front of the drone".
        size_px: rendered size, 256..1024. Larger costs proportionally more tokens.
        grid: burn in a labelled pixel grid to read coordinates against.

    Returns:
        list: [image, dict] on success; a fail-closed dict on error.
    """
    drone_position = None
    heading = None
    if latitude_deg is None or longitude_deg is None:
        drone, link_err = resolve_drone(ctx)
        if link_err is not None:
            return link_err
        try:
            sample = await _current_position(drone)
        except Exception as e:
            logger.error("map view position fetch failed: %s", e)
            return tool_err(e)
        if sample is None:
            return tool_err("no position samples; cannot centre a view on the vehicle")
        latitude_deg, longitude_deg, _ = sample
        drone_position = (latitude_deg, longitude_deg)
        heading = await _current_heading(drone)
    else:
        # An explicit centre may still be near the vehicle; show it if we can.
        connector, link_err = resolve_connector(ctx)
        if link_err is None:
            try:
                sample = await _current_position(connector.drone)
                if sample is not None:
                    drone_position = (sample[0], sample[1])
                    heading = await _current_heading(connector.drone)
            except Exception as e:  # noqa: BLE001 - context, not the point of the call
                logger.info("vehicle position unavailable for map view: %s", e)

    try:
        provider = map_source.resolve_provider()
        view, image, stats = await map_view.build_map_view(
            float(latitude_deg),
            float(longitude_deg),
            radius_m=radius_m,
            size_px=size_px,
            orientation=orientation,
            heading_deg=heading,
            provider=provider,
        )
        map_view.annotate(
            view,
            image,
            drone_latlon=drone_position,
            heading_deg=heading,
            grid=bool(grid),
        )
        path = map_view.persist(view, image)
    except ValueError as e:
        logger.error("map view failed: %s", e)
        return tool_err(e)
    except Exception as e:
        logger.error("map view render failed: %s", e)
        return tool_err(e)

    payload = map_view.describe(
        view, stats, drone_latlon=drone_position, heading_deg=heading, image_path=path
    )
    return _image_result(path, tool_ok(payload))


@mcp.tool()
async def map_transform(
    ctx: Context,
    view_id: str,
    pixels: list | None = None,
    coordinates: list | None = None,
) -> dict:
    """Convert between image pixels and WGS84 coordinates on a stored map view.

    Bidirectional: give ``pixels`` ([[x, y], ...]) to get coordinates, or
    ``coordinates`` ([[lat, lon], ...]) to get pixels. This is how a position
    picked out of a map view becomes something safe to fly to.

    Returns:
        dict: {"status": "success", "coordinates"|"pixels": [...]} or an error.
    """
    if (pixels is None) == (coordinates is None):
        return tool_err("give exactly one of pixels or coordinates")
    try:
        view = plan_store.load_view(view_id)
    except (ValueError, FileNotFoundError) as e:
        return tool_err(e)
    except (OSError, json.JSONDecodeError) as e:
        return tool_err(f"map view {view_id!r} is unreadable: {e}")

    try:
        if pixels is not None:
            checked = validate_pixels(pixels, view.width_px, view.height_px)
            coords = view.pixels_to_latlon(checked)
            return tool_ok(
                view_id=view.view_id,
                coordinates=[[lat, lon] for lat, lon in coords],
                polygon=[[lat, lon] for lat, lon in coords],
                meters_per_pixel=round(view.meters_per_pixel, 4),
            )
        ring = [validate_lat_lon(c[0], c[1], f"coordinates[{i}]")
                for i, c in enumerate(coordinates)]
        return tool_ok(
            view_id=view.view_id,
            pixels=[[round(x, 1), round(y, 1)] for x, y in view.latlon_to_pixels(ring)],
            size_px=[view.width_px, view.height_px],
        )
    except (ValueError, TypeError, IndexError) as e:
        return tool_err(e)


@mcp.tool()
async def render_plan_view(
    ctx: Context, plan_id: str, size_px: int = 768, grid: bool = False
) -> Any:
    """Draw a stored plan over satellite imagery, as an image.

    The verification step: it shows whether the generated path actually covers
    the area that was meant, which no amount of reading coordinates will.

    Returns:
        list: [image, dict] on success; a fail-closed dict on error.
    """
    plan, err = _load_plan_or_err(plan_id)
    if err is not None:
        return err
    waypoints = plan.get("waypoints") or []
    if not waypoints:
        return tool_err(f"plan {plan_id!r} has no waypoints to draw")

    coords = [(w["latitude_deg"], w["longitude_deg"]) for w in waypoints]
    polygon = (plan.get("generator") or {}).get("params", {}).get("polygon")
    bounds = geo_helpers.bbox_of(coords + [tuple(p) for p in (polygon or [])])
    centre = (
        (bounds["min_lat"] + bounds["max_lat"]) / 2.0,
        (bounds["min_lon"] + bounds["max_lon"]) / 2.0,
    )
    # Cover the whole plan with a margin, never less than a usable minimum.
    span_m = max(
        geo_helpers.haversine_m((bounds["min_lat"], bounds["min_lon"]),
                                (bounds["max_lat"], bounds["min_lon"])),
        geo_helpers.haversine_m((bounds["min_lat"], bounds["min_lon"]),
                                (bounds["min_lat"], bounds["max_lon"])),
    )
    radius = max(50.0, span_m * 0.62)

    try:
        provider = map_source.resolve_provider()
        view, image, stats = await map_view.build_map_view(
            centre[0], centre[1], radius_m=radius, size_px=size_px, provider=provider
        )
        stats_block = plan.get("stats") or {}
        note = (
            f"{plan.get('name')} rev{plan.get('revision')} [{plan.get('status')}] | "
            f"{stats_block.get('waypoint_count')} wp | "
            f"{(stats_block.get('path_length_m') or 0) / 1000.0:.2f} km"
        )
        map_view.annotate(
            view,
            image,
            polygon_latlon=[tuple(p) for p in polygon] if polygon else None,
            path_latlon=coords,
            grid=bool(grid),
            note=note,
        )
        path = map_view.persist(view, image)
    except ValueError as e:
        logger.error("render_plan_view failed: %s", e)
        return tool_err(e)
    except Exception as e:
        logger.error("render_plan_view render failed: %s", e)
        return tool_err(e)

    payload = map_view.describe(view, stats, image_path=path)
    payload["plan_id"] = plan.get("plan_id")
    payload["revision"] = plan.get("revision")
    return _image_result(path, tool_ok(payload))


@mcp.tool()
async def prefetch_map_area(
    ctx: Context,
    polygon: list | None = None,
    bbox: dict | None = None,
    radius_m: float = 500.0,
    latitude_deg: float | None = None,
    longitude_deg: float | None = None,
    zoom: int | None = None,
) -> dict:
    """Warm the tile cache for an area so later views need no network.

    Use before going somewhere without connectivity: every map view over the
    cached area is then served from disk.

    Returns:
        dict: structured status with tile counts, or an error.
    """
    try:
        provider = map_source.resolve_provider()
        if provider.get("kind") == "blank":
            return tool_err(
                "MAVLINKMCP_MAP_PROVIDER=none serves no tiles; "
                "configure a provider before prefetching"
            )

        if polygon:
            ring = geo_helpers.validate_polygon(polygon)
            bounds = geo_helpers.bbox_of(ring)
        elif bbox:
            bounds = {k: float(bbox[k]) for k in
                      ("min_lat", "min_lon", "max_lat", "max_lon")}
        elif latitude_deg is not None and longitude_deg is not None:
            centre = validate_lat_lon(latitude_deg, longitude_deg, "centre")
            radius = map_view.clamp_radius_m(radius_m)
            north = geo_helpers.destination(centre, 0.0, radius)
            east = geo_helpers.destination(centre, 90.0, radius)
            south = geo_helpers.destination(centre, 180.0, radius)
            west = geo_helpers.destination(centre, 270.0, radius)
            bounds = geo_helpers.bbox_of([north, east, south, west])
        else:
            return tool_err(
                "give a polygon, a bbox, or latitude_deg/longitude_deg with radius_m"
            )

        if zoom is None:
            span = geo_helpers.haversine_m(
                (bounds["min_lat"], bounds["min_lon"]),
                (bounds["max_lat"], bounds["max_lon"]),
            )
            zoom = tile_helpers.zoom_for_radius(
                bounds["min_lat"], max(span / 2.0, 10.0), 1024
            )
        zoom = tile_helpers.validate_zoom(zoom)

        x_min, y_min, x_max, y_max = tile_helpers.tile_range_for_bbox(
            bounds["min_lon"], bounds["min_lat"], bounds["max_lon"], bounds["max_lat"], zoom
        )
        needed = tile_helpers.tile_count(x_min, y_min, x_max, y_max)
        if needed > map_source.MAX_PREFETCH_TILES:
            return tool_err(
                f"that area needs {needed} tiles at zoom {zoom}, over the "
                f"{map_source.MAX_PREFETCH_TILES} limit; use a lower zoom or a smaller area"
            )

        _, stats = await map_source.fetch_tiles(
            provider,
            zoom,
            tile_helpers.tiles_in_range(x_min, y_min, x_max, y_max),
            max_tiles=map_source.MAX_PREFETCH_TILES,
        )
    except ValueError as e:
        return tool_err(e)
    except Exception as e:
        logger.error("prefetch failed: %s", e)
        return tool_err(e)

    return tool_ok(
        zoom=zoom,
        bbox=bounds,
        tiles=stats,
        cache=plan_store.tile_cache_stats(),
    )


# -- authoring --------------------------------------------------------


def _resolve_polygon(polygon, view_id, polygon_pixels):
    """Turn either polygon form into a validated ring plus its provenance."""
    have_direct = polygon is not None
    have_pixels = view_id is not None and polygon_pixels is not None
    if have_direct == have_pixels:
        raise ValueError(
            "give exactly one of polygon, or view_id together with polygon_pixels"
        )
    if have_direct:
        return geo_helpers.validate_polygon(polygon), {}

    view = plan_store.load_view(view_id)
    pixels = validate_pixels(polygon_pixels, view.width_px, view.height_px, min_count=3)
    ring = geo_helpers.validate_polygon(view.pixels_to_latlon(pixels))
    # Provenance: a suspicious polygon can be traced back to the exact image it
    # was drawn on, and the pixels someone pointed at.
    return ring, {
        "view_id": view.view_id,
        "polygon_pixels": [[round(x, 1), round(y, 1)] for x, y in pixels],
        "view_provider": view.provider,
        "view_meters_per_pixel": round(view.meters_per_pixel, 4),
    }


def _generate_and_store(
    name, plan_id, revision, params, source, return_to_launch, derived_from=None
):
    """Run the generator and persist the resulting revision."""
    waypoints, derived = coverage_helpers.generate(
        "lawnmower", params["polygon"], params
    )
    plan = build_plan(
        plan_id=plan_id,
        name=name,
        waypoints=waypoints,
        revision=revision,
        pattern="lawnmower",
        params=params,
        derived=derived,
        source=source,
        return_to_launch=return_to_launch,
        derived_from_revision=derived_from,
    )
    plan_store.save_plan(plan)
    return plan


def _plan_summary(plan, **extra):
    """The compact view of a plan: everything but the waypoint list."""
    derived = (plan.get("generator") or {}).get("derived") or {}
    summary = {
        "plan_id": plan.get("plan_id"),
        "revision": plan.get("revision"),
        "name": plan.get("name"),
        # Named plan_status, not status: tool_ok/tool_err own the "status" key,
        # and a lifecycle value there would read as a failed call.
        "plan_status": plan.get("status"),
        "pattern": (plan.get("generator") or {}).get("pattern"),
        "return_to_launch": plan.get("return_to_launch"),
        "stats": plan.get("stats"),
        "derived": {
            k: (round(v, 3) if isinstance(v, float) else v)
            for k, v in derived.items()
            if k != "centroid"
        },
    }
    for key in ("estimate", "checks", "uploaded"):
        if plan.get(key):
            summary[key] = plan[key]
    summary.update(extra)
    return summary


@mcp.tool()
async def create_survey_plan(
    ctx: Context,
    name: str,
    altitude_m: float,
    speed_m_s: float = 5.0,
    polygon: list | None = None,
    view_id: str | None = None,
    polygon_pixels: list | None = None,
    line_spacing_m: float | None = None,
    camera: dict | None = None,
    sweep_angle_deg: float | None = None,
    margin_m: float = 0.0,
    overshoot_m: float = 0.0,
    return_to_launch: bool = True,
) -> dict:
    """Generate a lawnmower survey over an area and store it as a flight plan.

    Give the area either as ``polygon`` ([[lat, lon], ...]) or, after looking at
    a map view, as ``view_id`` plus ``polygon_pixels`` ([[x, y], ...]) — the
    corners read straight off the image, converted exactly here.

    Line spacing comes from either ``line_spacing_m`` or a ``camera``; supplying
    both is refused rather than silently preferring one. A camera also sets the
    photo trigger distance and reports the ground sample distance.

    The plan is stored as a draft. Run validate_plan before uploading it.

    Args:
        name: human name; the plan id is derived from it.
        altitude_m: relative altitude for every waypoint.
        speed_m_s: ground speed for every waypoint.
        camera: {sensor_width_mm, focal_length_mm, image_width_px,
            image_height_px, front_overlap, side_overlap}.
        sweep_angle_deg: line bearing; omit to use the area's long axis, which
            minimises turns.
        margin_m: inward inset from the boundary.
        overshoot_m: extra distance past each line end for turn-in room.

    Returns:
        dict: structured status with the stored plan summary, or an error.
    """
    try:
        ring, source = _resolve_polygon(polygon, view_id, polygon_pixels)
        params = {
            "polygon": [[lat, lon] for lat, lon in ring],
            "altitude_m": float(altitude_m),
            "speed_m_s": float(speed_m_s),
            "line_spacing_m": line_spacing_m,
            "camera": camera,
            "sweep_angle_deg": sweep_angle_deg,
            "margin_m": float(margin_m or 0.0),
            "overshoot_m": float(overshoot_m or 0.0),
        }
        plan_id = plan_store.unique_plan_id(slugify_plan_id(name))
        plan = _generate_and_store(
            name, plan_id, 1, params, source, bool(return_to_launch)
        )
    except (ValueError, FileNotFoundError) as e:
        logger.error("create_survey_plan failed: %s", e)
        return tool_err(e)
    except (OSError, TypeError) as e:
        logger.error("create_survey_plan storage failed: %s", e)
        return tool_err(e)

    return tool_ok(
        _plan_summary(plan, next_step="validate_plan, or render_plan_view to see it")
    )


@mcp.tool()
async def create_plan_from_waypoints(
    ctx: Context, name: str, waypoints: list, return_to_launch: bool = True
) -> dict:
    """Store an explicit waypoint list as a flight plan.

    For routes that are not a coverage pattern. Waypoints use the same fields as
    initiate_mission: latitude_deg, longitude_deg, relative_altitude_m,
    speed_m_s, is_fly_through.

    Returns:
        dict: structured status with the stored plan summary, or an error.
    """
    try:
        plan_id = plan_store.unique_plan_id(slugify_plan_id(name))
        plan = build_plan(
            plan_id=plan_id,
            name=name,
            waypoints=waypoints,
            revision=1,
            pattern="waypoints",
            params={},
            derived={},
            source={},
            return_to_launch=bool(return_to_launch),
        )
        plan_store.save_plan(plan)
    except (ValueError, FileNotFoundError) as e:
        logger.error("create_plan_from_waypoints failed: %s", e)
        return tool_err(e)
    except (OSError, TypeError) as e:
        return tool_err(e)
    return tool_ok(_plan_summary(plan, next_step="validate_plan"))


@mcp.tool()
async def revise_plan(
    ctx: Context,
    plan_id: str,
    altitude_m: float | None = None,
    speed_m_s: float | None = None,
    line_spacing_m: float | None = None,
    sweep_angle_deg: float | None = None,
    margin_m: float | None = None,
    overshoot_m: float | None = None,
    camera: dict | None = None,
    return_to_launch: bool | None = None,
) -> dict:
    """Regenerate a plan with changed parameters, as a new revision.

    Re-runs the generator rather than editing waypoints, so the stored
    parameters and the stored path can never disagree. The new revision starts
    as a draft: any change invalidates the previous check.

    Returns:
        dict: structured status with the new revision summary, or an error.
    """
    plan, err = _load_plan_or_err(plan_id)
    if err is not None:
        return err

    generator = plan.get("generator") or {}
    if generator.get("pattern") != "lawnmower":
        return tool_err(
            f"plan {plan_id!r} was not generated from parameters "
            f"(pattern {generator.get('pattern')!r}); create a new plan instead"
        )

    params = dict(generator.get("params") or {})
    overrides = {
        "altitude_m": altitude_m,
        "speed_m_s": speed_m_s,
        "line_spacing_m": line_spacing_m,
        "sweep_angle_deg": sweep_angle_deg,
        "margin_m": margin_m,
        "overshoot_m": overshoot_m,
        "camera": camera,
    }
    applied = {k: v for k, v in overrides.items() if v is not None}
    if not applied and return_to_launch is None:
        return tool_err("no changes given; pass at least one parameter to revise")
    params.update(applied)
    # A new spacing and a camera would now be two sources for one number.
    if line_spacing_m is not None and camera is None:
        params["camera"] = None
    if camera is not None and line_spacing_m is None:
        params["line_spacing_m"] = None

    rtl = plan.get("return_to_launch") if return_to_launch is None else bool(return_to_launch)
    try:
        revision = plan_store.next_revision(plan_id)
        new_plan = _generate_and_store(
            plan.get("name"),
            plan.get("plan_id"),
            revision,
            params,
            dict(generator.get("source") or {}),
            rtl,
            derived_from=plan.get("revision"),
        )
    except (ValueError, FileNotFoundError) as e:
        logger.error("revise_plan failed: %s", e)
        return tool_err(e)
    except (OSError, TypeError) as e:
        return tool_err(e)

    return tool_ok(_plan_summary(new_plan, changed=sorted(applied), next_step="validate_plan"))


@mcp.tool()
async def delete_plan(ctx: Context, plan_id: str) -> dict:
    """Delete a stored plan and all of its revisions.

    Returns:
        dict: structured status with the revision count removed, or an error.
    """
    try:
        removed = plan_store.delete_plan(plan_id)
    except (ValueError, FileNotFoundError) as e:
        return tool_err(e)
    except OSError as e:
        return tool_err(f"could not delete plan {plan_id!r}: {e}")
    return tool_ok(plan_id=plan_id, revisions_removed=removed)


# -- inspection -------------------------------------------------------


@mcp.tool()
async def list_plans(ctx: Context) -> dict:
    """List stored flight plans, most recently updated first.

    Returns:
        dict: structured status with plan summaries, or an error.
    """
    try:
        plans = plan_store.list_plans()
    except (ValueError, OSError) as e:
        return tool_err(e)
    return tool_ok(plans=plans, count=len(plans), store=str(_plans_root()))


@mcp.tool()
async def get_plan(
    ctx: Context,
    plan_id: str,
    revision: int | None = None,
    include_waypoints: bool = False,
) -> dict:
    """Read a stored plan.

    Waypoints are omitted unless asked for: a survey is hundreds of points, and
    the summary is what answers most questions.

    Returns:
        dict: structured status with the plan, or an error.
    """
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err
    summary = _plan_summary(plan)
    summary["revisions"] = plan_store.list_revisions(plan_id)
    summary["generator_params"] = (plan.get("generator") or {}).get("params")
    summary["source"] = (plan.get("generator") or {}).get("source")
    if include_waypoints:
        summary["waypoints"] = plan.get("waypoints")
    return tool_ok(summary)


@mcp.tool()
async def preview_plan_geojson(ctx: Context, plan_id: str, revision: int | None = None) -> dict:
    """Return a plan as GeoJSON: the area, the path and the numbered waypoints.

    The machine-readable counterpart to render_plan_view, and pasteable into any
    GeoJSON viewer by a human.

    Returns:
        dict: structured status with a FeatureCollection, or an error.
    """
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err
    try:
        return tool_ok(geojson=plan_to_geojson(plan), plan_id=plan.get("plan_id"))
    except (KeyError, TypeError, ValueError) as e:
        return tool_err(e)


@mcp.tool()
async def estimate_plan(
    ctx: Context,
    plan_id: str,
    battery_capacity_mah: float | None = None,
    cruise_current_a: float | None = None,
    revision: int | None = None,
) -> dict:
    """Estimate distance, duration, photo count and battery use for a plan.

    Battery is reported only when both capacity and cruise current are given;
    otherwise it stays null rather than being invented.

    Returns:
        dict: structured status with the estimate, or an error.
    """
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err

    home = None
    connector, link_err = resolve_connector(ctx)
    if link_err is None:
        try:
            sample = await _current_position(connector.drone)
            if sample is not None:
                home = (sample[0], sample[1])
        except Exception as e:  # noqa: BLE001 - transit legs are a refinement
            logger.info("home position unavailable for estimate: %s", e)

    try:
        estimate = plan_check_helpers.estimate_plan(
            plan,
            battery_capacity_mah=battery_capacity_mah,
            cruise_current_a=cruise_current_a,
            home=home,
        )
        plan_store.update_plan(plan_id, plan.get("revision"), estimate=dict(estimate))
    except (ValueError, OSError) as e:
        return tool_err(e)

    return tool_ok(
        plan_id=plan.get("plan_id"),
        revision=plan.get("revision"),
        estimate=estimate,
        duration_min=round(estimate["duration_s"] / 60.0, 1),
        home_used=home is not None,
    )


# -- checking ---------------------------------------------------------


@mcp.tool()
async def validate_plan(
    ctx: Context,
    plan_id: str,
    home_lat: float | None = None,
    home_lon: float | None = None,
    revision: int | None = None,
) -> dict:
    """Check a plan for safety problems and, if it passes, mark it validated.

    Only a plan in the validated state can be uploaded, so this is the gate
    between authoring and flying. Errors block; warnings inform.

    Home position matters more than anything else here: a polygon drawn on the
    wrong map produces a perfectly well-formed plan somewhere else entirely, and
    the distance from home is what catches it. If home is not given, the live
    vehicle position is used when the link is up.

    Returns:
        dict: structured status with findings and the resulting plan status.
    """
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err

    home = None
    try:
        if home_lat is not None and home_lon is not None:
            home = validate_lat_lon(home_lat, home_lon, "home")
    except ValueError as e:
        return tool_err(e)

    if home is None:
        connector, link_err = resolve_connector(ctx)
        if link_err is None:
            try:
                sample = await _current_position(connector.drone)
                if sample is not None:
                    home = (sample[0], sample[1])
            except Exception as e:  # noqa: BLE001 - absence only weakens the check
                logger.info("home position unavailable for validation: %s", e)

    try:
        estimate = plan.get("estimate") or plan_check_helpers.estimate_plan(plan, home=home)
        findings = plan_check_helpers.check_plan(plan, home=home, estimate=estimate)
        checks = plan_check_helpers.summarize_checks(findings)
        status = STATUS_VALIDATED if checks["passed"] else "draft"
        # A plan that already flew stays uploaded; a failing check still demotes it.
        if plan.get("status") == STATUS_UPLOADED and checks["passed"]:
            status = STATUS_UPLOADED
        plan_store.update_plan(plan_id, plan.get("revision"), checks=checks, status=status)
    except (ValueError, OSError) as e:
        return tool_err(e)

    return tool_ok(
        plan_id=plan.get("plan_id"),
        revision=plan.get("revision"),
        plan_status=status,
        passed=checks["passed"],
        findings=checks["findings"],
        error_count=checks["error_count"],
        warning_count=checks["warning_count"],
        home_checked=home is not None,
        note=(
            "no home position available, so FAR_FROM_HOME was not checked"
            if home is None
            else None
        ),
    )


@mcp.tool()
async def preflight_check(ctx: Context, plan_id: str) -> dict:
    """Live go/no-go for a plan: vehicle readiness plus fit against this route.

    Reads health, GPS, battery and landed state from the vehicle and measures
    home against the plan's first waypoint. Reports blockers separately from
    warnings so the decision is explicit rather than implied.

    Returns:
        dict: structured status with a go/no-go verdict, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    plan, err = _load_plan_or_err(plan_id)
    if err is not None:
        return err

    report = {}
    blockers = []
    warnings = []

    async def _first(stream_name):
        """One sample from a telemetry stream, or None if it is unavailable.

        Each read is isolated: a preflight that cannot read one stream should
        still report everything else it learned, because a partial answer is
        what lets an operator decide. Only a missing position is fatal.
        """
        try:
            stream = getattr(drone.telemetry, stream_name)
            async for sample in stream():
                return sample
        except Exception as e:  # noqa: BLE001 - one stream must not sink the check
            logger.info("preflight: %s unavailable: %s", stream_name, e)
            report[stream_name] = {"status": "failed", "error": str(e)}
            warnings.append(f"{stream_name} telemetry unavailable")
        return None

    health = await _first("health")
    if health is not None:
        flags = normalize_health_flags(
            {
                "is_gyrometer_calibration_ok": health.is_gyrometer_calibration_ok,
                "is_accelerometer_calibration_ok": health.is_accelerometer_calibration_ok,
                "is_magnetometer_calibration_ok": health.is_magnetometer_calibration_ok,
                "is_local_position_ok": health.is_local_position_ok,
                "is_global_position_ok": health.is_global_position_ok,
                "is_home_position_ok": health.is_home_position_ok,
                "is_armable": health.is_armable,
            }
        )
        report["health"] = flags
        for key, label in (
            ("is_global_position_ok", "global position estimate"),
            ("is_home_position_ok", "home position"),
            ("is_armable", "armable"),
        ):
            if flags.get("health", {}).get(key) is False:
                blockers.append(f"{label} not ready")

    gps = await _first("gps_info")
    if gps is not None:
        try:
            report["gps_info"] = format_gps_info(gps)
            fix = report["gps_info"]["fix_type"].upper()
            # Match the good fix types explicitly. Substring tests do not work
            # here: "NO_FIX" contains "FIX", and reading that as a valid fix
            # would let a mission start with no position solution at all.
            if not any(good in fix for good in ("FIX_3D", "DGPS", "RTK")):
                blockers.append(f"GPS fix is {fix}; a mission needs a 3D fix or better")
            satellites = report["gps_info"].get("num_satellites", 0)
            if satellites < 6:
                warnings.append(f"only {satellites} satellites")
        except ValueError as e:
            warnings.append(f"GPS info unreadable: {e}")

    battery = await _first("battery")
    if battery is not None:
        remaining = getattr(battery, "remaining_percent", None)
        voltage = getattr(battery, "voltage_v", None)
        try:
            if remaining is not None:
                value = float(remaining)
                # MAVSDK reports 0-1 on some versions and 0-100 on others.
                percent = value * 100.0 if 0.0 <= value <= 1.0 else value
                report["battery"] = (
                    format_battery(remaining_fraction=value, voltage_v=voltage)
                    if 0.0 <= value <= 1.0
                    else format_battery(remaining_percent=value, voltage_v=voltage)
                )
                if percent < 30.0:
                    blockers.append(f"battery at {percent:.0f}%")
                elif percent < 50.0:
                    warnings.append(f"battery at {percent:.0f}%")
            elif voltage is not None:
                report["battery"] = format_battery(voltage_v=voltage)
        except ValueError as e:
            warnings.append(f"battery reading unreadable: {e}")

    landed = await _first("landed_state")
    if landed is not None:
        report["landed_state"] = normalize_landed_state(landed)

    try:
        position = await _current_position(drone)
    except Exception as e:
        logger.error("preflight position fetch failed: %s", e)
        return tool_err(e)

    home = None
    if position is not None:
        home = (position[0], position[1])
        report["home"] = {"latitude_deg": home[0], "longitude_deg": home[1]}
    else:
        blockers.append("no position samples; the vehicle does not know where it is")

    try:
        estimate = plan_check_helpers.estimate_plan(plan, home=home)
        findings = plan_check_helpers.check_plan(plan, home=home, estimate=estimate)
    except ValueError as e:
        return tool_err(e)

    for item in findings:
        target = blockers if item["severity"] == "error" else warnings
        target.append(f"{item['code']}: {item['message']}")

    if plan.get("status") == "draft":
        warnings.append("plan is still a draft; validate_plan must pass before upload")

    return tool_ok(
        plan_id=plan.get("plan_id"),
        revision=plan.get("revision"),
        verdict="GO" if not blockers else "NO-GO",
        blockers=blockers,
        warnings=warnings,
        estimate=estimate,
        telemetry=report,
    )


# -- execution --------------------------------------------------------


@mcp.tool()
async def upload_plan(ctx: Context, plan_id: str, revision: int | None = None) -> dict:
    """Upload a stored plan to the vehicle. Does not start it.

    Refuses a plan that has not passed validate_plan: uploading is the point
    where a route stops being a document and becomes something the aircraft will
    fly, so the check is not optional.

    Returns:
        dict: structured status with the uploaded waypoint count, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err

    refusal = can_upload(plan)
    if refusal is not None:
        return tool_err(
            refusal, plan_id=plan.get("plan_id"), plan_status=plan.get("status")
        )

    try:
        items = _mission_items_from(plan["waypoints"])
    except (ValueError, KeyError) as e:
        return tool_err(e)

    try:
        await drone.mission.set_return_to_launch_after_mission(
            bool(plan.get("return_to_launch"))
        )
        logger.info("Uploading plan %s rev%s (%s waypoints)",
                    plan.get("plan_id"), plan.get("revision"), len(items))
        await drone.mission.upload_mission(MissionPlan(items))
    except Exception as e:
        logger.error("Plan upload failed: %s", e)
        return tool_err(e)

    try:
        plan_store.update_plan(
            plan_id,
            plan.get("revision"),
            status=STATUS_UPLOADED,
            uploaded={
                "at": utc_now_iso(),
                "revision": plan.get("revision"),
                "waypoint_count": len(items),
                "verified": False,
            },
        )
    except (ValueError, OSError) as e:
        # The vehicle has the mission; failing to record that is not fatal.
        logger.error("could not record upload state: %s", e)

    return tool_ok(
        plan_id=plan.get("plan_id"),
        revision=plan.get("revision"),
        waypoint_count=len(items),
        return_to_launch=bool(plan.get("return_to_launch")),
        plan_status=STATUS_UPLOADED,
        next_step="verify_uploaded_plan, then arm_drone and start_mission",
    )


@mcp.tool()
async def verify_uploaded_plan(ctx: Context, plan_id: str, revision: int | None = None) -> dict:
    """Download the mission from the vehicle and compare it to the stored plan.

    Confirms that what the aircraft will fly is what was reviewed, rather than
    trusting that the upload did what it said.

    Returns:
        dict: structured status with any differences, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    plan, err = _load_plan_or_err(plan_id, revision)
    if err is not None:
        return err

    try:
        onboard = await drone.mission.download_mission()
    except Exception as e:
        logger.error("mission download failed: %s", e)
        return tool_err(e)

    try:
        actual = [
            {
                "latitude_deg": item.latitude_deg,
                "longitude_deg": item.longitude_deg,
                "relative_altitude_m": item.relative_altitude_m,
            }
            for item in getattr(onboard, "mission_items", []) or []
        ]
    except (AttributeError, TypeError) as e:
        return tool_err(f"unexpected mission format from vehicle: {e}")

    diffs = waypoints_equal(plan.get("waypoints") or [], actual)
    matched = not diffs

    if matched:
        try:
            record = dict(plan.get("uploaded") or {})
            record.update({"verified": True, "verified_at": utc_now_iso(),
                           "vehicle_waypoint_count": len(actual)})
            plan_store.update_plan(plan_id, plan.get("revision"), uploaded=record)
        except (ValueError, OSError) as e:
            logger.error("could not record verification: %s", e)

    return tool_ok(
        plan_id=plan.get("plan_id"),
        revision=plan.get("revision"),
        verified=matched,
        vehicle_waypoint_count=len(actual),
        plan_waypoint_count=len(plan.get("waypoints") or []),
        differences=diffs[:20],
    )


@mcp.tool()
async def start_mission(ctx: Context) -> dict:
    """Start the mission loaded on the vehicle, or resume a paused one.

    MAVSDK has no separate resume verb: starting again after pause_mission
    continues from the current item.

    Returns:
        dict: structured status, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        await drone.mission.start_mission()
        return tool_ok(mission_started=True)
    except Exception as e:
        logger.error("start_mission failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def pause_mission(ctx: Context) -> dict:
    """Pause the running mission; the vehicle holds position.

    Returns:
        dict: structured status, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        await drone.mission.pause_mission()
        return tool_ok(mission_paused=True, resume_with="start_mission")
    except Exception as e:
        logger.error("pause_mission failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def clear_mission(ctx: Context) -> dict:
    """Erase the mission stored on the vehicle.

    Returns:
        dict: structured status, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        await drone.mission.clear_mission()
        return tool_ok(mission_cleared=True)
    except Exception as e:
        logger.error("clear_mission failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def set_current_waypoint(ctx: Context, index: int) -> dict:
    """Jump the running mission to a waypoint index (0-based).

    Returns:
        dict: structured status, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        if isinstance(index, bool):
            raise ValueError("index must be an integer, not bool")
        target = int(index)
        if target < 0:
            raise ValueError(f"waypoint index must not be negative: {target}")
    except (TypeError, ValueError) as e:
        return tool_err(e)
    try:
        await drone.mission.set_current_mission_item(target)
        return tool_ok(current_waypoint=target)
    except Exception as e:
        logger.error("set_current_waypoint failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def is_mission_finished(ctx: Context) -> dict:
    """Report whether the vehicle has completed its mission.

    Returns:
        dict: {"status": "success", "mission_finished": bool} or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        finished = await drone.mission.is_mission_finished()
        return tool_ok(mission_finished=bool(finished))
    except Exception as e:
        logger.error("is_mission_finished failed: %s", e)
        return tool_err(e)


@mcp.tool()
async def download_mission_as_plan(ctx: Context, name: str) -> dict:
    """Capture the mission currently on the vehicle as a stored plan.

    Useful for recording what is actually loaded, or for pulling in a mission
    that was uploaded by a ground station.

    Returns:
        dict: structured status with the stored plan summary, or an error.
    """
    drone, link_err = resolve_drone(ctx)
    if link_err is not None:
        return link_err
    try:
        onboard = await drone.mission.download_mission()
    except Exception as e:
        logger.error("mission download failed: %s", e)
        return tool_err(e)

    waypoints = []
    for item in getattr(onboard, "mission_items", []) or []:
        speed = getattr(item, "speed_m_s", float("nan"))
        waypoints.append(
            {
                "latitude_deg": item.latitude_deg,
                "longitude_deg": item.longitude_deg,
                "relative_altitude_m": item.relative_altitude_m,
                # The vehicle reports NaN for "unchanged"; storage needs a number.
                "speed_m_s": 5.0 if speed != speed else speed,
                "is_fly_through": bool(getattr(item, "is_fly_through", True)),
            }
        )
    if not waypoints:
        return tool_err("the vehicle has no mission loaded")

    try:
        plan_id = plan_store.unique_plan_id(slugify_plan_id(name))
        plan = build_plan(
            plan_id=plan_id,
            name=name,
            waypoints=waypoints,
            revision=1,
            pattern="downloaded",
            params={},
            derived={},
            source={"origin": "vehicle"},
            return_to_launch=False,
        )
        plan_store.save_plan(plan)
    except (ValueError, OSError, TypeError) as e:
        return tool_err(e)
    return tool_ok(_plan_summary(plan, next_step="validate_plan"))


def _apply_transport_security(host: str, allowed_hosts, allow_any_host: bool, parser) -> None:
    """Keep Host/Origin checks meaningful when binding off localhost.

    FastMCP auto-enables DNS-rebinding protection only for localhost, and decides
    it at construction time. Mutating ``settings.host`` afterwards would leave the
    localhost allowlist in place and silently reject every remote request, so the
    allowlist is rebuilt here to match the bind that was actually requested.
    """
    # Imported lazily: only the HTTP transports need it, so the stdio path stays
    # importable with a minimal mcp surface.
    from mcp.server.transport_security import TransportSecuritySettings

    if allow_any_host:
        mcp.settings.transport_security = TransportSecuritySettings(
            enable_dns_rebinding_protection=False,
        )
        logger.warning(
            "DNS-rebinding protection disabled: any Host header is accepted. "
            "These tools can arm and fly a vehicle - use a trusted network only."
        )
        return

    if allowed_hosts:
        mcp.settings.transport_security = TransportSecuritySettings(
            enable_dns_rebinding_protection=True,
            allowed_hosts=list(allowed_hosts),
            allowed_origins=[f"http://{h}" for h in allowed_hosts]
            + [f"https://{h}" for h in allowed_hosts],
        )
        return

    if host not in LOCAL_HOSTS:
        parser.error(
            f"--host {host} binds beyond localhost, exposing tools that can arm and fly a "
            "vehicle. Declare the Host headers to accept with --allowed-host (repeatable, "
            "e.g. --allowed-host drone.lan:8000), or pass --allow-any-host to turn the "
            "check off deliberately."
        )


def main(argv: list[str] | None = None) -> None:
    """Console-script entry point for the ``mavlinkmcp`` command.

    Serves MCP over stdio by default, so a chat app can spawn the server by
    absolute path (``<venv>/bin/mavlinkmcp``) from any working directory. The
    HTTP transports expose the same tools over a network socket instead.

    Args:
        argv: Argument list to parse. Defaults to ``sys.argv[1:]``.
    """
    parser = argparse.ArgumentParser(
        prog="mavlinkmcp",
        description="MAVLink MCP server: MAVSDK-backed vehicle tools for LLM agents.",
    )
    parser.add_argument(
        "--transport",
        choices=("stdio", "streamable-http", "sse"),
        default="stdio",
        help="MCP transport (default: stdio). 'sse' is deprecated in the MCP spec; "
        "prefer 'streamable-http' for new clients.",
    )
    parser.add_argument(
        "--host", default=None, metavar="ADDR",
        help="Bind address for the HTTP transports (default: 127.0.0.1).",
    )
    parser.add_argument(
        "--port", type=int, default=None, metavar="PORT",
        help="Bind port for the HTTP transports (default: 8000).",
    )
    parser.add_argument(
        "--path", default=None, metavar="PATH",
        help="Endpoint path for --transport streamable-http (default: /mcp).",
    )
    parser.add_argument(
        "--allowed-host", action="append", default=None, metavar="HOST[:PORT]",
        help="Host header to accept when binding off localhost; repeatable. "
        "Wildcards allowed, e.g. 'drone.lan:*'.",
    )
    parser.add_argument(
        "--allow-any-host", action="store_true",
        help="Disable DNS-rebinding protection entirely. Trusted networks only.",
    )
    args = parser.parse_args(argv)

    if args.transport == "stdio":
        http_only = [
            name
            for name, value in (
                ("--host", args.host),
                ("--port", args.port),
                ("--path", args.path),
                ("--allowed-host", args.allowed_host),
            )
            if value is not None
        ]
        if args.allow_any_host:
            http_only.append("--allow-any-host")
        if http_only:
            parser.error(
                f"{', '.join(http_only)} require{'s' if len(http_only) == 1 else ''} "
                "an HTTP transport (--transport streamable-http)"
            )
        mcp.run(transport="stdio")
        return

    if args.host is not None:
        mcp.settings.host = args.host
    if args.port is not None:
        mcp.settings.port = args.port
    if args.path is not None:
        mcp.settings.streamable_http_path = args.path

    _apply_transport_security(mcp.settings.host, args.allowed_host, args.allow_any_host, parser)

    logger.info(
        "Serving MCP over %s at %s:%s", args.transport, mcp.settings.host, mcp.settings.port
    )
    mcp.run(transport=args.transport)


if __name__ == "__main__":
    main()
