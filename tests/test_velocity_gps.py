"""Offline tests for velocity/GPS format helpers."""
import unittest
from types import SimpleNamespace

from src.server.tool_dicts import (
    format_gps_info,
    format_velocity_ned,
    tool_err,
    tool_ok,
)


class TestVelocityGps(unittest.TestCase):
    def test_format_velocity_ok(self):
        v = SimpleNamespace(north_m_s=1.0, east_m_s=-0.5, down_m_s=0.0)
        self.assertEqual(
            format_velocity_ned(v),
            {"north_m_s": 1.0, "east_m_s": -0.5, "down_m_s": 0.0},
        )

    def test_format_velocity_nonfinite(self):
        v = SimpleNamespace(north_m_s=float("nan"), east_m_s=0.0, down_m_s=0.0)
        with self.assertRaises(ValueError):
            format_velocity_ned(v)

    def test_format_gps_ok(self):
        g = SimpleNamespace(num_satellites=12, fix_type="FIX_3D")
        self.assertEqual(
            format_gps_info(g),
            {"num_satellites": 12, "fix_type": "FIX_3D"},
        )

    def test_format_gps_neg_sats(self):
        g = SimpleNamespace(num_satellites=-1, fix_type="NO_GPS")
        with self.assertRaises(ValueError):
            format_gps_info(g)

    def test_tool_wrappers(self):
        self.assertEqual(tool_ok({"a": 1})["a"], 1)
        self.assertEqual(tool_err("e")["status"], "failed")


if __name__ == "__main__":
    unittest.main()
