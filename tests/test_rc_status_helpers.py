import unittest

from mavlinkmcp.rc_status_helpers import normalize_rc_status, rc_status_err


class TestRcStatusHelpers(unittest.TestCase):
    def test_success_available(self):
        r = normalize_rc_status(
            {
                "was_available_once": True,
                "is_available": True,
                "signal_strength_percent": 87.5,
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertTrue(r["rc_status"]["is_available"])
        self.assertEqual(r["rc_status"]["signal_strength_percent"], 87.5)

    def test_success_unavailable_no_signal(self):
        r = normalize_rc_status(
            {"was_available_once": False, "is_available": False}
        )
        self.assertEqual(r["status"], "success")
        self.assertFalse(r["rc_status"]["is_available"])

    def test_missing_signal_when_available(self):
        r = normalize_rc_status(
            {"was_available_once": True, "is_available": True}
        )
        self.assertEqual(r["status"], "failed")

    def test_nan_signal(self):
        r = normalize_rc_status(
            {
                "was_available_once": True,
                "is_available": True,
                "signal_strength_percent": float("nan"),
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_out_of_range(self):
        r = normalize_rc_status(
            {
                "was_available_once": True,
                "is_available": True,
                "signal_strength_percent": 120,
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_err_helper(self):
        e = rc_status_err("x")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "x")


if __name__ == "__main__":
    unittest.main()
