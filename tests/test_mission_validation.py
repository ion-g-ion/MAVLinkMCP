"""Offline unit tests for mission waypoint validation (no drone required)."""
import sys
import types
import unittest
import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load_module():
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

        # Resources and prompts register at import time exactly like tools, so
        # the stub has to accept them or ``server.py`` never finishes loading.
        def resource(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        def prompt(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        def run(self, *a, **k):
            pass

    fast.Context = Context
    fast.FastMCP = FastMCP

    mav = sys.modules["mavsdk"]

    class System:
        pass

    mav.System = System

    mis = sys.modules["mavsdk.mission"]

    class MissionItem:
        class CameraAction:
            NONE = 0

        class VehicleAction:
            NONE = 0

        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class MissionPlan:
        def __init__(self, items):
            self.items = items

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

    # Avoid double-load pollution
    # Import the real package module so its relative imports resolve, forcing a
    # fresh execution against the stubs installed above.
    sys.modules.pop("mavlinkmcp.server", None)
    return importlib.import_module("mavlinkmcp.server")


def _good_point(**overrides):
    p = {
        "latitude_deg": 47.397742,
        "longitude_deg": 8.545594,
        "relative_altitude_m": 10.0,
        "speed_m_s": 5.0,
        "is_fly_through": True,
    }
    p.update(overrides)
    return p


class TestValidateMissionPoints(unittest.TestCase):
    def setUp(self):
        self.m = _load_module()

    def test_empty_rejected(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([])

    def test_non_list_rejected(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points("nope")  # type: ignore

    def test_bad_lat(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([_good_point(latitude_deg=100)])

    def test_bad_lon(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([_good_point(longitude_deg=-200)])

    def test_nan_altitude(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points(
                [_good_point(relative_altitude_m=float("nan"))]
            )

    def test_speed_zero(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([_good_point(speed_m_s=0)])

    def test_speed_over_cap(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([_good_point(speed_m_s=31)])

    def test_alt_under_floor(self):
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([_good_point(relative_altitude_m=0.1)])

    def test_missing_key(self):
        bad = _good_point()
        del bad["is_fly_through"]
        with self.assertRaises(ValueError):
            self.m.validate_mission_points([bad])

    def test_happy_path(self):
        out = self.m.validate_mission_points([_good_point(), _good_point(latitude_deg=47.4)])
        self.assertEqual(len(out), 2)
        self.assertEqual(out[0]["speed_m_s"], 5.0)


if __name__ == "__main__":
    unittest.main()
