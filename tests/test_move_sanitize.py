import math
import unittest

from server.move_sanitize import (
    MAX_ALTITUDE_DELTA_M,
    MAX_HORIZONTAL_M,
    MAX_YAW_DEG,
    sanitize_relative_move,
)


class TestMoveSanitize(unittest.TestCase):
    def test_happy(self):
        ok, p = sanitize_relative_move(1.0, -2.0, 3.0, 10.0)
        self.assertTrue(ok)
        self.assertEqual(p["lr"], 1.0)
        self.assertEqual(p["fb"], -2.0)
        self.assertEqual(p["altitude"], 3.0)
        self.assertEqual(p["yaw"], 10.0)

    def test_bool_rejected(self):
        ok, p = sanitize_relative_move(True, 0, 0, 0)
        self.assertFalse(ok)
        self.assertEqual(p["status"], "failed")

    def test_nan_inf(self):
        ok, p = sanitize_relative_move(float("nan"), 0, 0, 0)
        self.assertFalse(ok)
        ok2, _ = sanitize_relative_move(0, float("inf"), 0, 0)
        self.assertFalse(ok2)

    def test_caps(self):
        ok, p = sanitize_relative_move(MAX_HORIZONTAL_M + 1, 0, 0, 0)
        self.assertFalse(ok)
        ok2, p2 = sanitize_relative_move(0, 0, MAX_ALTITUDE_DELTA_M + 0.1, 0)
        self.assertFalse(ok2)
        ok3, p3 = sanitize_relative_move(0, 0, 0, MAX_YAW_DEG + 1)
        self.assertFalse(ok3)


if __name__ == "__main__":
    unittest.main()
