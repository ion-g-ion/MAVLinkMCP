import unittest
from mavlinkmcp.armed_air_helpers import normalize_in_air, normalize_is_armed, status_err


class TestArmedAirHelpers(unittest.TestCase):
    def test_status_err(self):
        d = status_err("boom", code=1)
        self.assertEqual(d["status"], "failed")
        self.assertEqual(d["error"], "boom")
        self.assertEqual(d["code"], 1)

    def test_armed_true_false(self):
        self.assertEqual(normalize_is_armed(True)["is_armed"], True)
        self.assertEqual(normalize_is_armed(False)["status"], "success")
        self.assertFalse(normalize_is_armed(False)["is_armed"])

    def test_armed_reject_non_bool(self):
        self.assertEqual(normalize_is_armed(None)["status"], "failed")
        self.assertEqual(normalize_is_armed("true")["status"], "failed")
        self.assertEqual(normalize_is_armed(1)["status"], "failed")

    def test_in_air(self):
        self.assertTrue(normalize_in_air(True)["in_air"])
        self.assertFalse(normalize_in_air(False)["in_air"])
        self.assertEqual(normalize_in_air(None)["status"], "failed")
        self.assertEqual(normalize_in_air(0)["status"], "failed")


if __name__ == "__main__":
    unittest.main()
