"""Offline unit tests for MAVSDK system_address construction."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "server"))

from endpoint import build_system_address  # noqa: E402


class TestBuildSystemAddress(unittest.TestCase):
    def test_defaults_local_sitl(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(build_system_address("", "14540"), "udp://127.0.0.1:14540")
            self.assertEqual(build_system_address(None, None), "udp://127.0.0.1:14540")

    def test_custom_host_port(self):
        self.assertEqual(build_system_address("10.0.0.2", 14550), "udp://10.0.0.2:14550")

    def test_port_out_of_range(self):
        with self.assertRaises(ValueError):
            build_system_address("127.0.0.1", 0)
        with self.assertRaises(ValueError):
            build_system_address("127.0.0.1", 65536)

    def test_port_bool_rejected(self):
        with self.assertRaises(ValueError):
            build_system_address("127.0.0.1", True)

    def test_port_non_int(self):
        with self.assertRaises(ValueError):
            build_system_address("127.0.0.1", "abc")

    def test_whitespace_host(self):
        with self.assertRaises(ValueError):
            build_system_address("bad host", 14540)

    def test_scheme_injection_rejected(self):
        with self.assertRaises(ValueError):
            build_system_address("udp://evil", 14540)


if __name__ == "__main__":
    unittest.main()
