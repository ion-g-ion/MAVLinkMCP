"""Offline unit tests for telemetry clamp helpers (no drone required)."""
import sys
import types
import unittest
import importlib.util
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

    mod_name = "mavlinkmcp_telemetry_under_test"
    if mod_name in sys.modules:
        del sys.modules[mod_name]

    spec = importlib.util.spec_from_file_location(
        mod_name, ROOT / "src" / "server" / "mavlinkmcp.py"
    )
    m = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[mod_name] = m
    spec.loader.exec_module(m)
    return m


class TestClampImuCount(unittest.TestCase):
    def setUp(self):
        self.m = _load_module()

    def test_default_ok(self):
        self.assertEqual(self.m.clamp_imu_count(1), 1)
        self.assertEqual(self.m.clamp_imu_count(10), 10)

    def test_clamp_low(self):
        self.assertEqual(self.m.clamp_imu_count(0), 1)
        self.assertEqual(self.m.clamp_imu_count(-5), 1)

    def test_clamp_high(self):
        self.assertEqual(self.m.clamp_imu_count(1000), 100)

    def test_bool_rejected(self):
        with self.assertRaises(ValueError):
            self.m.clamp_imu_count(True)  # type: ignore

    def test_non_int_rejected(self):
        with self.assertRaises(ValueError):
            self.m.clamp_imu_count("nope")  # type: ignore


if __name__ == "__main__":
    unittest.main()
