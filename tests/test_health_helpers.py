import unittest

from health_helpers import normalize_health_flags, status_err


class TestHealthHelpers(unittest.TestCase):
    def test_success_subset(self):
        r = normalize_health_flags(
            {
                "is_global_position_ok": True,
                "is_home_position_ok": False,
                "ignored": True,
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(
            r["health"],
            {
                "is_global_position_ok": True,
                "is_home_position_ok": False,
            },
        )

    def test_empty_fails(self):
        r = normalize_health_flags({})
        self.assertEqual(r["status"], "failed")

    def test_non_mapping(self):
        r = normalize_health_flags([1, 2])
        self.assertEqual(r["status"], "failed")

    def test_non_bool(self):
        r = normalize_health_flags({"is_armable": 1})
        self.assertEqual(r["status"], "failed")

    def test_status_err(self):
        e = status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
