"""Shared stubs for loading ``mavlinkmcp.server`` offline (no drone required).

``server.py`` imports ``mcp`` and ``mavsdk`` at module scope, so reaching the
tools at all means faking both first. The five older test modules each carry
their own copy of this preamble; new tests import it from here instead.
"""
import asyncio
import importlib
import sys
import types


def load_server():
    """Import ``mavlinkmcp.server`` with mcp/mavsdk stubbed out."""
    for name in (
        "mcp",
        "mcp.server",
        "mcp.server.fastmcp",
        "mavsdk",
        "mavsdk.mission",
        "mavsdk.offboard",
    ):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)

    fast = sys.modules["mcp.server.fastmcp"]

    class Context:
        pass

    class FastMCP:
        def __init__(self, *a, **k):
            pass

        def tool(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        def run(self, *a, **k):
            pass

    class Image:
        """Stands in for the SDK type that base64-encodes an image on the wire."""

        def __init__(self, path=None, data=None, format=None):
            self.path = path
            self.data = data
            self.format = format

    fast.Context = Context
    fast.FastMCP = FastMCP
    fast.Image = Image

    mav = sys.modules["mavsdk"]

    class System:
        pass

    mav.System = System

    mis = sys.modules["mavsdk.mission"]

    class MissionItem:
        class CameraAction:
            NONE = "NONE"
            TAKE_PHOTO = "TAKE_PHOTO"
            START_PHOTO_DISTANCE = "START_PHOTO_DISTANCE"
            STOP_PHOTO_DISTANCE = "STOP_PHOTO_DISTANCE"
            START_PHOTO_INTERVAL = "START_PHOTO_INTERVAL"
            STOP_PHOTO_INTERVAL = "STOP_PHOTO_INTERVAL"
            START_VIDEO = "START_VIDEO"
            STOP_VIDEO = "STOP_VIDEO"

        class VehicleAction:
            NONE = "NONE"

        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class MissionPlan:
        def __init__(self, items):
            self.items = items
            self.mission_items = items

    mis.MissionItem = MissionItem
    mis.MissionPlan = MissionPlan

    off = sys.modules["mavsdk.offboard"]

    class OffboardError(Exception):
        def __init__(self):
            self._result = types.SimpleNamespace(result="fail")

    class PositionNedYaw:
        def __init__(self, n=0.0, e=0.0, d=0.0, y=0.0):
            self.north_m, self.east_m, self.down_m, self.yaw_deg = n, e, d, y

    off.OffboardError = OffboardError
    off.PositionNedYaw = PositionNedYaw

    sys.modules.pop("mavlinkmcp.server", None)
    return importlib.import_module("mavlinkmcp.server")


def ctx(connector):
    """Minimal stand-in for the FastMCP Context a tool receives."""
    return types.SimpleNamespace(
        request_context=types.SimpleNamespace(lifespan_context=connector)
    )


def run(coro):
    """Drive one coroutine to completion; tools are all async."""
    return asyncio.run(coro)


class FakeTelemetry:
    """Async-generator telemetry with values a test can dictate."""

    def __init__(self, lat=47.3977, lon=8.5456, heading=72.4, fix="FIX_3D",
                 satellites=14, battery=0.96, armable=True):
        self._lat, self._lon = lat, lon
        self._heading, self._fix = heading, fix
        self._satellites, self._battery = satellites, battery
        self._armable = armable

    async def position(self):
        yield types.SimpleNamespace(
            latitude_deg=self._lat,
            longitude_deg=self._lon,
            absolute_altitude_m=500.0,
            relative_altitude_m=0.0,
        )

    async def heading(self):
        yield types.SimpleNamespace(heading_deg=self._heading)

    async def health(self):
        yield types.SimpleNamespace(
            is_gyrometer_calibration_ok=True,
            is_accelerometer_calibration_ok=True,
            is_magnetometer_calibration_ok=True,
            is_local_position_ok=True,
            is_global_position_ok=True,
            is_home_position_ok=True,
            is_armable=self._armable,
        )

    async def gps_info(self):
        yield types.SimpleNamespace(
            num_satellites=self._satellites, fix_type=self._fix
        )

    async def battery(self):
        yield types.SimpleNamespace(
            voltage_v=16.2, remaining_percent=self._battery
        )

    async def landed_state(self):
        yield "ON_GROUND"


class FakeMission:
    """Records what a tool sent to the vehicle, so tests can assert on it."""

    def __init__(self):
        self.plan = None
        self.return_to_launch = None
        self.started = False
        self.paused = False
        self.cleared = False
        self.current_item = None

    async def set_return_to_launch_after_mission(self, value):
        self.return_to_launch = value

    async def upload_mission(self, plan):
        self.plan = plan

    async def download_mission(self):
        return self.plan

    async def start_mission(self):
        self.started = True

    async def pause_mission(self):
        self.paused = True

    async def clear_mission(self):
        self.cleared = True
        self.plan = None

    async def set_current_mission_item(self, index):
        self.current_item = index

    async def is_mission_finished(self):
        return False


def fake_drone(telemetry=None, mission=None):
    return types.SimpleNamespace(
        telemetry=telemetry or FakeTelemetry(),
        mission=mission or FakeMission(),
    )
