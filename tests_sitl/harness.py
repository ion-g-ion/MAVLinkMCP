r"""Shared setup for the SITL integration tests.

These are the opposite of ``tests/``: they import the real ``mavsdk``, open a
real MAVLink link, and drive the tool functions against a PX4 that is actually
running. What they cover is precisely what the offline suite cannot -- the
MAVSDK handshake, the connection guard, and the mission protocol round-trip.

Start a vehicle first (see ``sitl/``)::

    docker run -d --rm --name px4 --network host \
      -e PX4_HOME_LAT=473977420 -e PX4_HOME_LON=85455940 \
      px4-sih:v1.14.3

With no vehicle listening every test here skips rather than fails, so
``python -m unittest discover`` stays meaningful on a machine without Docker.
"""
from __future__ import annotations

import asyncio
import atexit
import importlib
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

from mavlinkmcp.endpoint import build_system_address

# Bound by _load_real_server() on first use; see the note there for why this is
# not a plain module-level import.
server = None

# Home position baked into the SITL image, in decimal degrees. PX4 wants these
# as scaled int32 (see sitl/Dockerfile); telemetry reports them like this.
HOME_LAT = 47.397742
HOME_LON = 8.545594

# Boot, EKF convergence and GPS lock take a while on a cold container.
CONNECT_TIMEOUT_S = float(os.environ.get("MAVLINKMCP_SITL_TIMEOUT", "90"))

# How long to listen for a heartbeat before deciding nothing is there. Short,
# because this is the cost the offline suite pays: `unittest discover` at the
# repo root walks into this package too, and waiting out CONNECT_TIMEOUT_S to
# discover the obvious would turn a one-second run into a ninety-second one.
PROBE_TIMEOUT_S = float(os.environ.get("MAVLINKMCP_SITL_PROBE_TIMEOUT", "1.5"))

# How long mavsdk_server gets to open its gRPC port.
SERVER_START_TIMEOUT_S = 20.0

_loop: asyncio.AbstractEventLoop | None = None
_connector = None
_skip_reason: str | None = None


def _get_loop() -> asyncio.AbstractEventLoop:
    global _loop
    if _loop is None:
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
    return _loop


def run(coro):
    """Drive a coroutine on the suite's one persistent event loop.

    The offline tests can use ``asyncio.run`` per call because their drone is a
    namespace. A real MAVSDK ``System`` opens its gRPC channel on whichever loop
    was running when it connected, so a fresh loop per call would break the link
    after the first tool. One loop for the whole process instead.
    """
    return _get_loop().run_until_complete(coro)


# Everything the offline suite fakes. It installs its stubs by *assignment*
# onto whatever sys.modules holds -- so when this package has already imported
# the genuine article, the real mavsdk.mission.MissionItem is replaced by a stub
# with a much smaller CameraAction. Re-importing from scratch undoes that.
_FAKED_MODULES = (
    "mavlinkmcp.server",
    "mavsdk", "mavsdk.mission", "mavsdk.offboard",
    "mcp", "mcp.server", "mcp.server.fastmcp",
)


def _load_real_server():
    """Import ``mavlinkmcp.server`` bound to the genuine MAVSDK types.

    Running ``python -m unittest discover`` from the repository root puts the
    offline suite and this one in a single process, and the offline suite runs
    first. Without this, mission upload here fails on a stubbed enum rather than
    on anything the vehicle said.
    """
    global server
    for name in _FAKED_MODULES:
        sys.modules.pop(name, None)
    server = importlib.import_module("mavlinkmcp.server")
    return server


async def _wait_until_ready(drone) -> None:
    """Block until the vehicle can actually fly, not merely until it is alive.

    ``server.wait_for_position_estimate`` returns as soon as *either* the global
    or the home estimate is up. On a cold SITL that is home, seconds before the
    EKF has a global fix -- enough for a readiness assertion to see
    is_global_position_ok False and for arming to be refused.
    """
    async for health in drone.telemetry.health():
        if (
            health.is_global_position_ok
            and health.is_home_position_ok
            and health.is_armable
        ):
            return


def _free_port() -> int:
    """An unused TCP port for this run's mavsdk_server.

    MAVSDK defaults its gRPC server to 50051. Two runs back to back collide
    there -- the second server cannot bind, and the client waits on plugins that
    will never come up -- so each run gets its own port instead.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _spawn_mavsdk_server(address: str) -> int:
    """Run mavsdk_server ourselves and return its gRPC port.

    Letting ``System()`` spawn it is the obvious route, but MAVSDK then attaches
    a non-daemon thread to the server's stdout. Python joins non-daemon threads
    *before* running atexit handlers, so that thread blocks interpreter exit
    until the server dies, and the handler that would kill the server never
    runs: the process hangs after the last test and only a signal ends it --
    which in turn skips the cleanup and leaks a server holding udp/14540.

    Owning the subprocess avoids the thread entirely.
    """
    import mavsdk

    binary = pathlib.Path(mavsdk.__file__).parent / "bin" / "mavsdk_server"
    port = _free_port()
    proc = subprocess.Popen(
        [str(binary), "-p", str(port), address],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    atexit.register(_kill, proc)

    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"mavsdk_server exited early (rc={proc.returncode})")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(0.2)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                return port
    raise RuntimeError(f"mavsdk_server did not open port {port}")


def _release() -> None:
    """Drop the System while the import system still works.

    ``System.__del__`` imports subprocess, so letting it run during interpreter
    teardown prints an ignored ImportError over the test output. Releasing the
    last reference from an atexit handler keeps the shutdown quiet.
    """
    global _connector
    _connector = None
    if _loop is not None and not _loop.is_closed():
        _loop.close()


def _kill(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        proc.kill()
        proc.wait(timeout=5)


def _probe_vehicle() -> str | None:
    """None if a vehicle is talking to us, else why we are skipping.

    PX4 pushes heartbeats without being asked, so a passive listen settles the
    question far faster than letting the MAVSDK connect time out.
    """
    port = int(build_system_address().rsplit(":", 1)[1])
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.settimeout(PROBE_TIMEOUT_S)
        try:
            probe.bind(("0.0.0.0", port))
        except OSError as e:
            # Distinct from "nothing is there": a stale mavsdk_server from an
            # interrupted run holds this port and swallows the telemetry, and
            # the fix is to kill it rather than to start another vehicle.
            return (
                f"udp/{port} is already bound ({e}); a previous run's "
                f"mavsdk_server is probably still alive -- pkill mavsdk_server"
            )
        try:
            probe.recv(1)
        except (TimeoutError, OSError):
            return f"nothing sending MAVLink to udp/{port} after {PROBE_TIMEOUT_S}s"
    finally:
        probe.close()
    return None


def connector():
    """A connected ``MAVLinkConnector``, or skip if no SITL is listening.

    Connecting is slow and the vehicle is stateless between tests, so the link
    is built once and shared. A failure is cached too -- without that, every
    test in the suite would pay the full timeout before skipping.
    """
    global _connector, _skip_reason
    if _skip_reason is not None:
        raise unittest.SkipTest(_skip_reason)
    if _connector is not None:
        return _connector

    address = build_system_address()
    problem = _probe_vehicle()
    if problem is not None:
        _skip_reason = (
            f"no PX4 SITL on {address}: {problem}. Start one with "
            f"sitl/docker-compose.yml (see tests_sitl/harness.py)"
        )
        raise unittest.SkipTest(_skip_reason)

    mod = _load_real_server()
    # The server binds the MAVLink address; System then only speaks gRPC to it,
    # so connect_vehicle's system_address argument is a no-op here. The link
    # bring-up path it exercises is still the real one.
    port = _spawn_mavsdk_server(address)
    conn = mod.MAVLinkConnector(
        drone=mod.System(mavsdk_server_address="127.0.0.1", port=port))
    try:
        # The real bring-up path, not a shortcut around it.
        run(asyncio.wait_for(mod.connect_vehicle(conn.drone, address), CONNECT_TIMEOUT_S))
        run(asyncio.wait_for(mod.wait_for_position_estimate(conn.drone), CONNECT_TIMEOUT_S))
        run(asyncio.wait_for(_wait_until_ready(conn.drone), CONNECT_TIMEOUT_S))
    except Exception as e:  # noqa: BLE001 - any failure here means "no vehicle"
        _skip_reason = (
            f"no PX4 SITL reachable on {address} ({type(e).__name__}: {e}); "
            f"start one with sitl/docker-compose.yml"
        )
        raise unittest.SkipTest(_skip_reason) from None

    conn.link_state = mod.LINK_READY
    _connector = conn
    # LIFO: this runs before the _kill registered by _spawn_mavsdk_server, so
    # the client goes away before the server it is talking to.
    atexit.register(_release)
    return conn


def ctx(conn):
    """Minimal stand-in for the FastMCP Context a tool receives."""
    return types.SimpleNamespace(
        request_context=types.SimpleNamespace(lifespan_context=conn)
    )


class SitlTestCase(unittest.TestCase):
    """Base case: a live link plus a plan store that does not touch real data."""

    def setUp(self):
        self.connector = connector()
        # Always reach the tools through here, never a module-level import: the
        # module object is only trustworthy after connector() has rebuilt it.
        self.server = server
        self.ctx = ctx(self.connector)

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        env = mock.patch.dict(
            os.environ,
            {
                "MAVLINKMCP_PLANS_DIR": tmp.name,
                # Nothing here may reach a tile server.
                "MAVLINKMCP_MAP_PROVIDER": "none",
            },
        )
        env.start()
        self.addCleanup(env.stop)

    def ok(self, result, what="tool"):
        """Assert the structured envelope reports success, and return it."""
        self.assertEqual(
            result.get("status"), "success",
            f"{what} failed: {result.get('error')!r} ({result})",
        )
        return result
