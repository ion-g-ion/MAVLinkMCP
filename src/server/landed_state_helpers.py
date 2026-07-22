"""Pure helpers for landed_state telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

from typing import Any, Dict, Optional

KNOWN_STATES = (
    "UNKNOWN",
    "ON_GROUND",
    "IN_AIR",
    "TAKING_OFF",
    "LANDING",
)

_KNOWN_SET = set(KNOWN_STATES)


def landed_state_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _canonical_state_name(token: str) -> Optional[str]:
    """Map a free-form token to a KNOWN_STATES value, or None."""
    if not token:
        return None
    # Strip common enum prefixes: "LandedState.ON_GROUND", "Telemetry.LandedState.IN_AIR"
    parts = token.replace(" ", "").split(".")
    leaf = parts[-1] if parts else token
    upper = leaf.upper()
    if upper in _KNOWN_SET:
        return upper
    return None


def normalize_landed_state(value: Any) -> Dict[str, Any]:
    """
    Coerce telemetry.landed_state samples into a fail-closed structured dict.

    Accepts:
      - str matching a known state (case-insensitive)
      - enum-like objects with ``.name`` or useful ``str()`` form
    """
    if value is None:
        return landed_state_status_err("landed_state missing")

    # Prefer enum .name when present
    name_attr = getattr(value, "name", None)
    if isinstance(name_attr, str):
        canon = _canonical_state_name(name_attr)
        if canon is not None:
            return {"status": "success", "landed_state": canon}
        return landed_state_status_err(
            f"unknown landed_state name: {name_attr!r}"
        )

    if isinstance(value, str):
        if not value.strip():
            return landed_state_status_err("landed_state empty")
        canon = _canonical_state_name(value.strip())
        if canon is not None:
            return {"status": "success", "landed_state": canon}
        return landed_state_status_err(f"unknown landed_state: {value!r}")

    # Last resort: str(value) (e.g. "LandedState.ON_GROUND")
    as_str = str(value).strip()
    if as_str:
        canon = _canonical_state_name(as_str)
        if canon is not None:
            return {"status": "success", "landed_state": canon}

    return landed_state_status_err(
        "landed_state must be a known state name",
        # keep payload minimal/JSON-safe — error string only already
    )
