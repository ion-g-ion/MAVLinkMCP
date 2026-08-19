"""Pure helpers for RC status telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def rc_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _as_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return bool(value)
    if isinstance(value, str):
        low = value.strip().lower()
        if low in ("true", "1", "yes", "on"):
            return True
        if low in ("false", "0", "no", "off"):
            return False
    return None


def _finite_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return f


def normalize_rc_status(flags: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Normalize RC status fields.

    Required:
      - was_available_once: bool
      - is_available: bool
      - signal_strength_percent: finite float in [0, 100] when available
        (if is_available is False, signal may be omitted or 0)
    """
    if not isinstance(flags, Mapping):
        return rc_status_err("rc_status flags must be a mapping")

    was = _as_bool(flags.get("was_available_once"))
    avail = _as_bool(flags.get("is_available"))
    if was is None:
        return rc_status_err("was_available_once missing or invalid")
    if avail is None:
        return rc_status_err("is_available missing or invalid")

    out: Dict[str, Any] = {
        "status": "success",
        "rc_status": {
            "was_available_once": was,
            "is_available": avail,
        },
    }

    if "signal_strength_percent" in flags and flags.get("signal_strength_percent") is not None:
        sig = _finite_float(flags.get("signal_strength_percent"))
        if sig is None:
            return rc_status_err("signal_strength_percent not finite")
        # clamp display range without inventing signal when out of bounds hard
        if sig < 0.0 or sig > 100.0:
            return rc_status_err("signal_strength_percent out of range [0, 100]")
        out["rc_status"]["signal_strength_percent"] = sig
    elif avail:
        return rc_status_err("signal_strength_percent required when is_available")

    return out
