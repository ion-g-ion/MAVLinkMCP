"""Offline unit tests for MAVLink MCP fail-closed helpers (no drone required)."""
import sys
import types
import unittest
import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load_helpers():
    """Import the server module, with lightweight stubs when mcp/mavsdk are absent."""
    try:
        import mavlinkmcp.server as m  # type: ignore

        return m
    except Exception:
        pass

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

    # Import the real package module so its relative imports resolve, forcing a
    # fresh execution against the stubs installed above.
    sys.modules.pop("mavlinkmcp.server", None)
    return importlib.import_module("mavlinkmcp.server")


class TestClampTakeoff(unittest.TestCase):
    def setUp(self):
        self.m = _load_helpers()

    def test_default_range(self):
        self.assertEqual(self.m.clamp_takeoff_altitude(3.0), 3.0)

    def test_clamp_low(self):
        self.assertEqual(self.m.clamp_takeoff_altitude(0.1), 0.5)

    def test_clamp_high(self):
        self.assertEqual(self.m.clamp_takeoff_altitude(500.0), 120.0)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            self.m.clamp_takeoff_altitude(float("nan"))

    def test_inf_rejected(self):
        with self.assertRaises(ValueError):
            self.m.clamp_takeoff_altitude(float("inf"))


class TestValidateRelativeMove(unittest.TestCase):
    def setUp(self):
        self.m = _load_helpers()

    def test_ok(self):
        self.m.validate_relative_move(1.0, 2.0, 0.5, 10.0)

    def test_too_large(self):
        with self.assertRaises(ValueError):
            self.m.validate_relative_move(501.0, 0.0, 0.0, 0.0)

    def test_nan(self):
        with self.assertRaises(ValueError):
            self.m.validate_relative_move(float("nan"), 0.0, 0.0, 0.0)


class TestToolShapes(unittest.TestCase):
    def setUp(self):
        self.m = _load_helpers()

    def test_ok_err(self):
        self.assertEqual(self.m.tool_ok(x=1)["status"], "success")
        # positional-dict form (used by disarm_drone / return_to_launch)
        self.assertEqual(self.m.tool_ok({"disarmed": True})["disarmed"], True)
        err = self.m.tool_err("nope")
        self.assertEqual(err["status"], "failed")
        self.assertIn("nope", err["error"])


if __name__ == "__main__":
    unittest.main()
