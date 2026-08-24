"""Pure helpers for MAVSDK system address construction (offline-testable)."""

from __future__ import annotations

import os


def build_system_address(address: str | None = None, port: str | int | None = None) -> str:
    """Build MAVSDK ``system_address`` for UDP (SITL-friendly defaults).

    The direction of the link depends on whether an address is given:

    * Empty/missing address -> ``udpin://0.0.0.0:<port>``. The server *binds*
      the port and waits for the autopilot to talk to it. This is what PX4 SITL
      needs: SITL sends to 14540 and expects the peer to be listening.
    * Explicit address -> ``udpout://<host>:<port>``. The server *sends* to a
      peer that is already listening there (e.g. a companion link on real
      hardware).

    Bare ``udp://`` is deliberately not emitted: MAVSDK deprecates it, and it
    silently resolves to the outgoing form, which never connects to SITL.

    Port must be an integer in 1..65535 (default from env ``MAVLINK_PORT`` or 14540).
    """
    if address is None:
        address = os.environ.get("MAVLINK_ADDRESS", "")
    if port is None:
        port = os.environ.get("MAVLINK_PORT", "14540")

    host = (address or "").strip()
    if host:
        if any(ch.isspace() for ch in host):
            raise ValueError(f"MAVLINK_ADDRESS must not contain whitespace: {address!r}")
        if "/" in host or ":" in host:
            # Reject path/injection and bare IPv6/forms with colon for this simple UDP helper
            raise ValueError(
                f"MAVLINK_ADDRESS must be a hostname or IPv4 without scheme/port: {host!r}"
            )

    try:
        if isinstance(port, bool):
            raise ValueError("port must be an integer, not bool")
        p = int(str(port).strip())
    except (TypeError, ValueError) as e:
        raise ValueError(f"MAVLINK_PORT must be an integer 1..65535: {port!r}") from e
    if p < 1 or p > 65535:
        raise ValueError(f"MAVLINK_PORT out of range 1..65535: {p}")

    if not host:
        return f"udpin://0.0.0.0:{p}"
    return f"udpout://{host}:{p}"
