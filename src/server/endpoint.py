"""Pure helpers for MAVSDK system address construction (offline-testable)."""

from __future__ import annotations

import os


def build_system_address(address: str | None = None, port: str | int | None = None) -> str:
    """Build MAVSDK ``system_address`` for UDP (SITL-friendly defaults).

    Empty/missing address defaults to ``127.0.0.1`` (local PX4 SITL).
    Port must be an integer in 1..65535 (default from env ``MAVLINK_PORT`` or 14540).
    """
    if address is None:
        address = os.environ.get("MAVLINK_ADDRESS", "")
    if port is None:
        port = os.environ.get("MAVLINK_PORT", "14540")

    host = (address or "").strip() or "127.0.0.1"
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

    return f"udp://{host}:{p}"
