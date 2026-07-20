import math
import unittest

from server.attitude_helpers import normalize_attitude_euler, status_err


class TestAttitudeHelpers(unittest.TestCase):
    def test_success(self):
        r = normalize_attitude_euler(1.5, -2.0, 90.0)
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["attitude"]["yaw_deg"], 90.0)
        self.assertEqual(r["attitude"]["roll_deg"], 1.5)

    def test_nan_rejected(self):
        r = normalize_attitude_euler(float("nan"), 0.0, 0.0)
        self.assertEqual(r["status"], "failed")
        self.assertIn("finite", r["error"])

    def test_inf_rejected(self):
        r = normalize_attitude_euler(0.0, float("inf"), 0.0)
        self.assertEqual(r["status"], "failed")

    def test_none_rejected(self):
        r = normalize_attitude_euler(None, 0.0, 0.0)
        self.assertEqual(r["status"], "failed")

    def test_bool_rejected(self):
        r = normalize_attitude_euler(True, 0.0, 0.0)
        self.assertEqual(r["status"], "failed")

    def test_status_err_shape(self):
        e = status_err("x", code=1)
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "x")
        self.assertEqual(e["code"], 1)


if __name__ == "__main__":
    unittest.main()
