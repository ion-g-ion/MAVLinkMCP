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
async def arm_drone(ctx: Context) -> bool:
    """Arm the drone."""
    drone = ctx.request_context.lifespan_context.drone
    logger.info("Arming")
    await drone.action.arm()
    return True


# Get Position
@mcp.tool()
async def get_position(ctx: Context) -> dict:
    """
    Get the position of the drone in latitude/longitude degrees and atittude in meters.
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
        return str({"status": "failed"})

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
async def move_to_relative(ctx: Context, lr: float, fb: float, altitude: float, yaw: float) -> bool:
    """
    Move the drone relative to the current position. The drone must be armed and offboard mode must be active.

    Args:
        ctx (Context): the context.
        lr (float): distance in left/right axis. right is the positive.
        fb (float): distance along front/back axis. front is positive.
        altitude (float): the altitude relative to the current point.
        yaw (float): yaw change.

    Returns:
        bool: success flag.
    """
    connector = ctx.request_context.lifespan_context
    drone = connector.drone

    # Activate offboard mode
    if not await start_offboard_mode(connector):
        return False

    # Update the last offboard position
    connector.last_offboard_position.north_m += fb
    connector.last_offboard_position.east_m += lr
    connector.last_offboard_position.down_m += -altitude
    connector.last_offboard_position.yaw_deg += yaw

    # Send the updated position
    logger.info(f"Sending updated offboard position: {connector.last_offboard_position}")
    await drone.offboard.set_position_ned(connector.last_offboard_position)

    return True

@mcp.tool()
async def takeoff(ctx: Context, takeoff_altitude: float = 3.0) -> bool:
    """Command the drone to initiate takeoff and ascend to a specified altitude. The drone must be armed.

    Args:
        ctx (Context): The context of the request.
        takeoff_altitude (float): The altitude to ascend to after takeoff. Default is 10.0 meters.

    Returns:
        bool: True if the takeoff command was initiated successfully.
    """
    drone = ctx.request_context.lifespan_context.drone
    logger.info("Initiating takeoff")
    await drone.action.set_takeoff_altitude(takeoff_altitude)
    await drone.action.takeoff()
    return True

@mcp.tool()
async def land(ctx: Context) -> bool:
    """Command the drone to initiate landing at its current location.

    Args:
        ctx (Context): The context of the request.

    Returns:
        bool: True if the land command was initiated successfully.
    """
    drone = ctx.request_context.lifespan_context.drone
    logger.info("Initiating landing")
    await drone.action.land()
    return True

@mcp.tool()
async def print_status_text(ctx: Context) -> dict:
    """Print and return status text from the drone."""
    drone = ctx.request_context.lifespan_context.drone
    try:
        async for status_text in drone.telemetry.status_text():
            logger.info(f"Status: {status_text.type}: {status_text.text}")
            return {"type": status_text.type, "text": status_text.text}  # Return a single dict
    except asyncio.CancelledError:
        return {"message": "Failed to retrieve status text"}  # Return a failure message

@mcp.tool()
async def get_imu(ctx: Context, n: int = 1) -> list:
    """Fetch the first n IMU data points from the drone.

    Args:
        ctx (Context): The context of the request.
        n (int): The number of IMU data points to fetch. Default is 1.

    Returns:
        list: A list of dictionaries containing IMU data points.
    """
    drone = ctx.request_context.lifespan_context.drone
    telemetry = drone.telemetry

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
        if count >= n:
            break

    return imu_data

@mcp.tool()
async def print_mission_progress(ctx: Context) -> dict:
    """
    Print and return the current mission progress of the drone.

    Args:
        ctx (Context): The context of the request.

    Returns:
        dict: A dictionary containing the current and total mission progress.
    """
    drone = ctx.request_context.lifespan_context.drone
    async for mission_progress in drone.mission.mission_progress():
        logger.info(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        return {"current": mission_progress.current, "total": mission_progress.total}



@mcp.tool()
async def initiate_mission(ctx: Context, mission_points: list, return_to_launch: bool = True) -> bool:
    """
    Initiate a mission with a list of mission points. The drone must be armed.

    Args:
        ctx (Context): The context of the request.
        mission_points (list): A list of dictionaries representing mission points. Each dictionary must include:
            - latitude_deg (float): Latitude in degrees (range: -90 to +90).
            - longitude_deg (float): Longitude in degrees (range: -180 to +180).
            - relative_altitude_m (float): Altitude relative to the takeoff altitude in meters.
            - speed_m_s (float): Speed in meters per second.
            - is_fly_through (bool): Whether to fly through the point or stop.
            - gimbal_pitch_deg (float): Gimbal pitch angle in degrees (optional).
            - gimbal_yaw_deg (float): Gimbal yaw angle in degrees (optional).
            - camera_action (MissionItem.CameraAction): Camera action at the point (optional).
            - loiter_time_s (float): Loiter time in seconds (optional).
            - camera_photo_interval_s (float): Camera photo interval in seconds (optional).
            - acceptance_radius_m (float): Acceptance radius in meters (optional).
            - yaw_deg (float): Yaw angle in degrees (optional).
            - camera_photo_distance_m (float): Camera photo distance in meters (optional).
            - vehicle_action (MissionItem.VehicleAction): Vehicle action at the point (optional).
        return_to_launch (bool): Whether to return to launch after completing the mission. Default is True.

    Returns:
        bool: True if the mission was successfully initiated.
    """
    drone = ctx.request_context.lifespan_context.drone

    # Validate and construct mission items
    mission_items = []
    for point in mission_points:
        try:
            # Validate latitude and longitude ranges
            if not (-90 <= point["latitude_deg"] <= 90):
                raise ValueError(f"Invalid latitude_deg: {point['latitude_deg']}. Must be between -90 and 90.")
            if not (-180 <= point["longitude_deg"] <= 180):
                raise ValueError(f"Invalid longitude_deg: {point['longitude_deg']}. Must be between -180 and 180.")

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
        except KeyError as e:
            raise ValueError(f"Missing required field in mission point: {e}")

    mission_plan = MissionPlan(mission_items)

    # Set return-to-launch behavior
    await drone.mission.set_return_to_launch_after_mission(return_to_launch)

    logger.info("Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    logger.info("Starting mission")
    await drone.mission.start_mission()

    return True

@mcp.tool()
async def get_flight_mode(ctx: Context) -> str:
    """
    Get the current flight mode of the drone.

    Args:
        ctx (Context): The context of the request.

    Returns:
        str: The current flight mode of the drone.
    """
    drone = ctx.request_context.lifespan_context.drone
    try:
        flight_mode = await drone.telemetry.flight_mode().__anext__()
        logger.info(f"FlightMode: {flight_mode}")
        return str(flight_mode)
    except StopAsyncIteration:
        logger.error("Failed to retrieve flight mode")
        return "Unknown"


if __name__ == "__main__":
    # Run the server
    mcp.run(transport='stdio')