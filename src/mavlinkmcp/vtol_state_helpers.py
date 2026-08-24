"""Pure helpers for vtol_state telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

from typing import Any, Dict, Optional

KNOWN_STATES = (
    "UNDEFINED",
    "TRANSITION_TO_FW",
    "TRANSITION_TO_MC",
    "MC",
    "FW",
)

_KNOWN_SET = set(KNOWN_STATES)

# Friendly aliases → canonical
_ALIASES = {
    "MULTI_COPTER": "MC",
    "MULTICOPTER": "MC",
    "FIXED_WING": "FW",
    "FIXEDWING": "FW",
    "VTOL_STATE_UNDEFINED": "UNDEFINED",
    "VTOL_STATE_TRANSITION_TO_FW": "TRANSITION_TO_FW",
    "VTOL_STATE_TRANSITION_TO_MC": "TRANSITION_TO_MC",
    "VTOL_STATE_MC": "MC",
    "VTOL_STATE_FW": "FW",
}


def vtol_state_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _canonical_state_name(token: str) -> Optional[str]:
    if not token:
        return None
    parts = token.replace(" ", "").split(".")
    leaf = parts[-1] if parts else token
    upper = leaf.upper()
    if upper in _KNOWN_SET:
        return upper
    if upper in _ALIASES:
        return _ALIASES[upper]
    return None


def normalize_vtol_state(value: Any) -> Dict[str, Any]:
    """
    Coerce telemetry.vtol_state samples into a fail-closed structured dict.

    Accepts str, enum-like .name, or useful str() forms.
    """
    if value is None:
        return vtol_state_status_err("vtol_state missing")

    name_attr = getattr(value, "name", None)
    if isinstance(name_attr, str):
        canon = _canonical_state_name(name_attr)
        if canon is not None:
            return {"status": "success", "vtol_state": canon}
        return vtol_state_status_err(f"unknown vtol_state name: {name_attr!r}")

    if isinstance(value, str):
        if not value.strip():
            return vtol_state_status_err("vtol_state empty")
        canon = _canonical_state_name(value.strip())
        if canon is not None:
            return {"status": "success", "vtol_state": canon}
        return vtol_state_status_err(f"unknown vtol_state: {value!r}")

    as_str = str(value).strip()
    if as_str:
        canon = _canonical_state_name(as_str)
        if canon is not None:
            return {"status": "success", "vtol_state": canon}

    return vtol_state_status_err("vtol_state must be a known state name")
