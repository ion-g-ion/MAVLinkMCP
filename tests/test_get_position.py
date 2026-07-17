"""Offline tests for get_position helpers (no MAVSDK connection)."""
import unittest

from src.server.tool_dicts import tool_err, tool_ok


class TestToolDicts(unittest.TestCase):
    def test_tool_ok_empty(self):
        self.assertEqual(tool_ok(), {"status": "success"})

    def test_tool_ok_payload(self):
        d = tool_ok({"position": {"latitude_deg": 1.0}})
        self.assertEqual(d["status"], "success")
        self.assertEqual(d["position"]["latitude_deg"], 1.0)

    def test_tool_err_str(self):
        d = tool_err("boom")
        self.assertEqual(d, {"status": "failed", "error": "boom"})

    def test_tool_err_exception(self):
        d = tool_err(ValueError("x"))
        self.assertEqual(d["status"], "failed")
        self.assertIn("x", d["error"])

    def test_never_stringifies_failure(self):
        d = tool_err("no")
        self.assertIsInstance(d, dict)
        self.assertNotIsInstance(d, str)


if __name__ == "__main__":
    unittest.main()
