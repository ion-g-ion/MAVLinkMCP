"""Offline unit tests for the MAVLink link guard (no drone required).

These cover the fail-closed path that keeps an unreachable vehicle from
wedging the MCP ``initialize`` handshake.
"""
import importlib
import os
import sys
import types
import unittest
from unittest import mock


def _load_module():
    for name in (
        "mcp",
        "mcp.server",
        "mcp.server.fastmcp",
        "mavsdk",
        "mavsdk.mission",
        "mavsdk.offboard",
    ):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)

    fast = sys.modules["mcp.server.fastmcp"]

    class Context:
        pass

    class FastMCP:
        def __init__(self, *a, **k):
            pass

        def tool(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        # Resources and prompts register at import time exactly like tools, so
        # the stub has to accept them or ``server.py`` never finishes loading.
        def resource(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        def prompt(self, *a, **k):
            def deco(fn):
                return fn

            return deco

        def run(self, *a, **k):
            pass

    fast.Context = Context
    fast.FastMCP = FastMCP

    mav = sys.modules["mavsdk"]

    class System:
        pass

    mav.System = System

    mis = sys.modules["mavsdk.mission"]

    class MissionItem:
        class CameraAction:
            NONE = 0

        class VehicleAction:
            NONE = 0

        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class MissionPlan:
        def __init__(self, items):
            self.items = items

    mis.MissionItem = MissionItem
    mis.MissionPlan = MissionPlan

    off = sys.modules["mavsdk.offboard"]

    class OffboardError(Exception):
        def __init__(self):
            self._result = types.SimpleNamespace(result="fail")

    class PositionNedYaw:
        def __init__(self, n=0.0, e=0.0, d=0.0, y=0.0):
            self.north_m, self.east_m, self.down_m, self.yaw_deg = n, e, d, y

    off.OffboardError = OffboardError
    off.PositionNedYaw = PositionNedYaw

    sys.modules.pop("mavlinkmcp.server", None)
    return importlib.import_module("mavlinkmcp.server")


def _ctx(connector):
    """Minimal stand-in for the FastMCP Context a tool receives."""
    return types.SimpleNamespace(
        request_context=types.SimpleNamespace(lifespan_context=connector)
    )


class TestConnectTimeout(unittest.TestCase):
    def setUp(self):
        self.m = _load_module()

    def test_default(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(self.m.connect_timeout_s(), 60.0)

    def test_env_override(self):
        with mock.patch.dict(os.environ, {"MAVLINK_CONNECT_TIMEOUT": "2.5"}, clear=True):
            self.assertEqual(self.m.connect_timeout_s(), 2.5)

    def test_explicit_value_wins(self):
        with mock.patch.dict(os.environ, {"MAVLINK_CONNECT_TIMEOUT": "99"}, clear=True):
            self.assertEqual(self.m.connect_timeout_s(3), 3.0)

    def test_non_positive_rejected(self):
        for bad in (0, -1, "-0.5"):
            with self.assertRaises(ValueError):
                self.m.connect_timeout_s(bad)

    def test_non_numeric_rejected(self):
        with self.assertRaises(ValueError):
            self.m.connect_timeout_s("soon")

    def test_non_finite_rejected(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError):
                self.m.connect_timeout_s(bad)


class TestLinkResolvers(unittest.TestCase):
    def setUp(self):
        self.m = _load_module()
        self.drone = object()

    def test_connecting_is_not_yet_linked(self):
        # A fresh connector is still dialling; tools must not run yet.
        conn = self.m.MAVLinkConnector(drone=self.drone)
        self.assertEqual(conn.link_state, self.m.LINK_CONNECTING)
        self.assertFalse(conn.is_linked)

        drone, err = self.m.resolve_drone(_ctx(conn))
        self.assertIsNone(drone)
        self.assertEqual(err["status"], "failed")
        self.assertEqual(err["link_state"], self.m.LINK_CONNECTING)
        self.assertIn("still coming up", err["error"])

    def test_linked_connector_passes_through(self):
        conn = self.m.MAVLinkConnector(drone=self.drone, link_state=self.m.LINK_READY)
        self.assertTrue(conn.is_linked)
        self.assertEqual(conn.link_failure(), "")

        got, err = self.m.resolve_connector(_ctx(conn))
        self.assertIs(got, conn)
        self.assertIsNone(err)

        drone, err = self.m.resolve_drone(_ctx(conn))
        self.assertIs(drone, self.drone)
        self.assertIsNone(err)

    def test_broken_link_fails_closed(self):
        conn = self.m.MAVLinkConnector(
            drone=self.drone,
            link_state=self.m.LINK_FAILED,
            link_error="no vehicle on udpin://0.0.0.0:14540",
        )
        self.assertFalse(conn.is_linked)

        got, err = self.m.resolve_connector(_ctx(conn))
        self.assertIsNone(got)
        self.assertEqual(err["status"], "failed")
        self.assertFalse(err["connected"])
        self.assertIn("no vehicle on udpin://0.0.0.0:14540", err["error"])

    def test_broken_link_withholds_drone(self):
        conn = self.m.MAVLinkConnector(
            drone=self.drone, link_state=self.m.LINK_FAILED, link_error="link down"
        )
        drone, err = self.m.resolve_drone(_ctx(conn))
        # The drone handle must never leak when the link is down: a tool that
        # got it would block inside MAVSDK instead of returning.
        self.assertIsNone(drone)
        self.assertEqual(err["status"], "failed")


if __name__ == "__main__":
    unittest.main()
