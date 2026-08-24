"""Offline unit tests for MAVSDK system_address construction."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest import mock

from mavlinkmcp.endpoint import build_system_address


class TestBuildSystemAddress(unittest.TestCase):
    def test_defaults_local_sitl(self):
        # No address means "listen": PX4 SITL sends to 14540 and expects a
        # bound peer, so the default must be the incoming form.
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(build_system_address("", "14540"), "udpin://0.0.0.0:14540")
            self.assertEqual(build_system_address(None, None), "udpin://0.0.0.0:14540")

    def test_custom_host_port(self):
        # An explicit host means "send to that peer".
        self.assertEqual(build_system_address("10.0.0.2", 14550), "udpout://10.0.0.2:14550")

    def test_never_emits_deprecated_bare_udp_scheme(self):
        # Bare udp:// is deprecated by MAVSDK and resolves to the outgoing
        # form, which silently never connects to SITL and hangs the lifespan.
        with mock.patch.dict(os.environ, {}, clear=True):
            for built in (
                build_system_address(None, None),
                build_system_address("", 14540),
                build_system_address("10.0.0.2", 14550),
            ):
                self.assertFalse(built.startswith("udp://"), built)
                self.assertRegex(built, r"^udp(in|out)://")

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
