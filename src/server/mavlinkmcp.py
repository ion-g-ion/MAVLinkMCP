# Add lifespan support for startup/shutdown with strong typing
from contextlib import asynccontextmanager
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from mcp.server.fastmcp import Context, FastMCP
from typing import Tuple
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import OffboardError, PositionNedYaw
import asyncio
import os
import logging

# Configure logger
logger = logging.getLogger("MAVLinkMCP")
logger.setLevel(logging.INFO)
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


def tool_ok(**payload):
    return {"status": "success", **payload}


def tool_err(message, **payload):
    return {"status": "error", "message": str(message), **payload}


def validate_mission_points(
    mission_points,
    min_rel_alt_m: float = 0.5,
    max_rel_alt_m: float = 500.0,
    max_speed_m_s: float = 30.0,
):
    """Validate waypoint list for initiate_mission (raise ValueError if invalid).

    Returns a shallow-copied list of point dicts after basic numeric checks.
    """
    if not isinstance(mission_points, list):
        raise ValueError("mission_points must be a list")
    if len(mission_points) == 0:
        raise ValueError("mission_points must be a non-empty list")

    required = (
        "latitude_deg",
        "longitude_deg",
        "relative_altitude_m",
        "speed_m_s",
        "is_fly_through",
    )
    validated = []
    for idx, point in enumerate(mission_points):
        if not isinstance(point, dict):
            raise ValueError(f"mission_points[{idx}] must be a dict")
        for key in required:
            if key not in point:
                raise ValueError(f"Missing required field in mission point: '{key}'")
        try:
            lat = float(point["latitude_deg"])
            lon = float(point["longitude_deg"])
            rel_alt = float(point["relative_altitude_m"])
            speed = float(point["speed_m_s"])
        except (TypeError, ValueError) as e:
            raise ValueError(f"mission_points[{idx}] numeric fields invalid: {e}") from e

        for name, v in (
            ("latitude_deg", lat),
            ("longitude_deg", lon),
            ("relative_altitude_m", rel_alt),
            ("speed_m_s", speed),
        ):
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError(f"mission_points[{idx}].{name} must be finite")

        if not (-90.0 <= lat <= 90.0):
            raise ValueError(
                f"Invalid latitude_deg: {lat}. Must be between -90 and 90."
            )
        if not (-180.0 <= lon <= 180.0):
            raise ValueError(
                f"Invalid longitude_deg: {lon}. Must be between -180 and 180."
            )
        if not (min_rel_alt_m <= rel_alt <= max_rel_alt_m):
            raise ValueError(
                f"relative_altitude_m {rel_alt} outside [{min_rel_alt_m}, {max_rel_alt_m}]"
            )
        if not (0.0 < speed <= max_speed_m_s):
            raise ValueError(
                f"speed_m_s {speed} outside (0, {max_speed_m_s}]"
            )

        cleaned = dict(point)
        cleaned["latitude_deg"] = lat
        cleaned["longitude_deg"] = lon
        cleaned["relative_altitude_m"] = rel_alt
        cleaned["speed_m_s"] = speed
        cleaned["is_fly_through"] = bool(point["is_fly_through"])
        validated.append(cleaned)
    return validated
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


@dataclass
class MAVLinkConnector:
    drone: System
    last_offboard_position: PositionNedYaw = field(default_factory=lambda: PositionNedYaw(0.0, 0.0, 0.0, 0.0))

@asynccontextmanager
async def app_lifespan(server: FastMCP) -> AsyncIterator[MAVLinkConnector]:
    """Manage application lifecycle with type-safe context"""
    # Initialize on startup
    address = os.environ.get("MAVLINK_ADDRESS", "")
    port = os.environ.get("MAVLINK_PORT", "14540")
    drone = System()
    logger.info("Connecting to drone at %s:%s", address, port)
    await drone.connect(system_address=f"udp://{address}:{port}")

    logger.info("Waiting for drone to connect at %s:%s", address, port)
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("Connected to drone at %s:%s!", address, port)
            break

    logger.info("Waiting for drone to have a global position estimate...")
    logger.info(f"{drone.telemetry.health()}")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok or health.is_home_position_ok:
            logger.info(f"Global position {health.is_global_position_ok}, home position {health.is_home_position_ok}")
            break

    try:
        yield MAVLinkConnector(drone=drone)
    finally:
        # Cleanup on shutdown
        logger.info("Disconnecting drone")
        await drone.close()

# Pass lifespan to server
mcp = FastMCP("MAVLink MCP", lifespan=app_lifespan)


# ARM
@mcp.tool()
async def arm_drone(ctx: Context) -> dict:
    """Arm the drone. Returns a structured status dict (fail-closed on errors)."""
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone
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
    connector = ctx.request_context.lifespan_context
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
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone
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
    drone = ctx.request_context.lifespan_context.drone

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
async def get_flight_mode(ctx: Context) -> dict:
    """
    Get the current flight mode of the drone.

    Returns:
        dict: structured status with mode string, or error.
    """
    drone = ctx.request_context.lifespan_context.drone
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



if __name__ == "__main__":
    # Run the server
    mcp.run(transport='stdio')
