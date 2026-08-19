import math
import unittest
from home_position_helpers import normalize_home_position, status_err


class TestHomePositionHelpers(unittest.TestCase):
    def test_ok(self):
        d = normalize_home_position(37.4, -122.1, 10.5)
        self.assertEqual(d["status"], "success")
        self.assertAlmostEqual(d["home"]["latitude_deg"], 37.4)
        self.assertAlmostEqual(d["home"]["absolute_altitude_m"], 10.5)

    def test_lat_range(self):
        self.assertEqual(normalize_home_position(91, 0, 0)["status"], "failed")
        self.assertEqual(normalize_home_position(-91, 0, 0)["status"], "failed")

    def test_lon_range(self):
        self.assertEqual(normalize_home_position(0, 181, 0)["status"], "failed")
        self.assertEqual(normalize_home_position(0, -181, 0)["status"], "failed")

    def test_non_finite(self):
        self.assertEqual(normalize_home_position(float("nan"), 0, 0)["status"], "failed")
        self.assertEqual(normalize_home_position(0, float("inf"), 0)["status"], "failed")

    def test_bad_types(self):
        self.assertEqual(normalize_home_position("x", 0, 0)["status"], "failed")
        self.assertEqual(status_err("e")["error"], "e")


if __name__ == "__main__":
    unittest.main()
