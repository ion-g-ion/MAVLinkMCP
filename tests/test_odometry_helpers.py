import unittest
from types import SimpleNamespace
import math

from odometry_helpers import normalize_odometry, odometry_status_err


class TestOdometryHelpers(unittest.TestCase):
    def test_success_position_only(self):
        r = normalize_odometry(
            {"position_body": {"x_m": 1.0, "y_m": 2.0, "z_m": -3.0}}
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["odometry"]["position_body"]["x_m"], 1.0)
        self.assertNotIn("velocity_body", r["odometry"])

    def test_success_with_velocity(self):
        r = normalize_odometry(
            {
                "position_body": {"x": 0.0, "y": 0.0, "z": 1.0},
                "velocity_body": {"x": 0.1, "y": -0.2, "z": 0.0},
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["odometry"]["velocity_body"]["y_m"], -0.2)

    def test_namespace_position(self):
        r = normalize_odometry(
            {"position_body": SimpleNamespace(x_m=4.0, y_m=5.0, z_m=6.0)}
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["odometry"]["position_body"]["z_m"], 6.0)

    def test_frames(self):
        r = normalize_odometry(
            {
                "position_body": {"x_m": 0, "y_m": 0, "z_m": 0},
                "frame_id": "odom",
                "child_frame_id": "base_link",
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["odometry"]["frame_id"], "odom")

    def test_empty_frame_failed(self):
        r = normalize_odometry(
            {
                "position_body": {"x_m": 0, "y_m": 0, "z_m": 0},
                "frame_id": "  ",
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_nan_position_failed(self):
        r = normalize_odometry(
            {"position_body": {"x_m": float("nan"), "y_m": 0, "z_m": 0}}
        )
        self.assertEqual(r["status"], "failed")

    def test_missing_position(self):
        r = normalize_odometry({})
        self.assertEqual(r["status"], "failed")

    def test_non_mapping(self):
        r = normalize_odometry("nope")  # type: ignore[arg-type]
        self.assertEqual(r["status"], "failed")

    def test_bad_velocity(self):
        r = normalize_odometry(
            {
                "position_body": {"x_m": 1, "y_m": 2, "z_m": 3},
                "velocity_body": {"x_m": math.inf, "y_m": 0, "z_m": 0},
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_flat_keys(self):
        r = normalize_odometry({"x_m": 1.0, "y_m": 2.0, "z_m": 3.0})
        self.assertEqual(r["status"], "success")

    def test_err_helper(self):
        e = odometry_status_err("z")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "z")


if __name__ == "__main__":
    unittest.main()
