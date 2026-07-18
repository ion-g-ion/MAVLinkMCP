import unittest

from server.status_helpers import (
    normalize_mission_progress,
    normalize_status_text,
    status_err,
    status_ok,
)


class TestStatusHelpers(unittest.TestCase):
    def test_status_ok_and_err(self):
        self.assertEqual(status_ok(a=1)["status"], "success")
        self.assertEqual(status_ok(a=1)["a"], 1)
        err = status_err("boom")
        self.assertEqual(err["status"], "failed")
        self.assertEqual(err["error"], "boom")

    def test_normalize_status_text(self):
        d = normalize_status_text("INFO", "hello")
        self.assertEqual(d["status"], "success")
        self.assertEqual(d["type"], "INFO")
        self.assertEqual(d["text"], "hello")

        class T:
            name = "WARNING"

        d2 = normalize_status_text(T(), 123)
        self.assertEqual(d2["type"], "WARNING")
        self.assertEqual(d2["text"], "123")

    def test_normalize_mission_progress(self):
        ok = normalize_mission_progress(2, 10)
        self.assertEqual(ok, {"status": "success", "current": 2, "total": 10})
        bad = normalize_mission_progress("x", 1)
        self.assertEqual(bad["status"], "failed")
        self.assertIn("invalid_mission_progress", bad["error"])
        bool_bad = normalize_mission_progress(True, 3)
        self.assertEqual(bool_bad["status"], "failed")


if __name__ == "__main__":
    unittest.main()
