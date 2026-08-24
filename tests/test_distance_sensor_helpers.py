import unittest
from types import SimpleNamespace

from mavlinkmcp.distance_sensor_helpers import (
    distance_sensor_status_err,
    normalize_distance_sensor,
)


class TestDistanceSensorHelpers(unittest.TestCase):
    def test_success_current_only(self):
        r = normalize_distance_sensor({"current_distance_m": 1.25})
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["distance_sensor"]["current_distance_m"], 1.25)
        self.assertNotIn("minimum_distance_m", r["distance_sensor"])
        self.assertNotIn("orientation", r["distance_sensor"])

    def test_success_full(self):
        r = normalize_distance_sensor(
            {
                "current_distance_m": 2.0,
                "minimum_distance_m": 0.2,
                "maximum_distance_m": 40.0,
                "orientation": 25,
            }
        )
        self.assertEqual(r["status"], "success")
        ds = r["distance_sensor"]
        self.assertEqual(ds["current_distance_m"], 2.0)
        self.assertEqual(ds["minimum_distance_m"], 0.2)
        self.assertEqual(ds["maximum_distance_m"], 40.0)
        self.assertEqual(ds["orientation"], 25)

    def test_orientation_enum_value(self):
        r = normalize_distance_sensor(
            {
                "current_distance_m": 1.0,
                "orientation": SimpleNamespace(value=12),
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["distance_sensor"]["orientation"], 12)

    def test_omit_none_orientation(self):
        r = normalize_distance_sensor(
            {"current_distance_m": 3.0, "orientation": None}
        )
        self.assertEqual(r["status"], "success")
        self.assertNotIn("orientation", r["distance_sensor"])

    def test_nan_current_failed(self):
        r = normalize_distance_sensor({"current_distance_m": float("nan")})
        self.assertEqual(r["status"], "failed")

    def test_negative_current_failed(self):
        r = normalize_distance_sensor({"current_distance_m": -0.1})
        self.assertEqual(r["status"], "failed")

    def test_min_gt_max_failed(self):
        r = normalize_distance_sensor(
            {
                "current_distance_m": 1.0,
                "minimum_distance_m": 10.0,
                "maximum_distance_m": 5.0,
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_non_mapping_failed(self):
        r = normalize_distance_sensor([1, 2, 3])  # type: ignore[arg-type]
        self.assertEqual(r["status"], "failed")

    def test_bad_orientation_type(self):
        r = normalize_distance_sensor(
            {"current_distance_m": 1.0, "orientation": "forward"}
        )
        self.assertEqual(r["status"], "failed")

    def test_orientation_oor(self):
        r = normalize_distance_sensor(
            {"current_distance_m": 1.0, "orientation": 99}
        )
        self.assertEqual(r["status"], "failed")

    def test_err_helper(self):
        e = distance_sensor_status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
