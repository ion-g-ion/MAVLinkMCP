"""Pure helpers for unix_epoch_time telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def unix_epoch_time_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _finite_number(value: Any) -> Optional[float]:
    """Return float if value is a real finite number (reject bool impostors)."""
    if type(value) is bool:
        return None
    if type(value) is int:
        return float(value)
    if type(value) is float:
        if not math.isfinite(value):
            return None
        return value
    # other numeric types (e.g. numpy scalars) via float() with finiteness check
    try:
        if hasattr(value, "__float__") and type(value) is not str:
            f = float(value)
        else:
            return None
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return f


def _epoch_seconds_from_us(us: Any) -> Optional[float]:
    f = _finite_number(us)
    if f is None:
        return None
    if f < 0.0:
        return None
    return f / 1_000_000.0


def normalize_unix_epoch_time(value: Any) -> Dict[str, Any]:
    """
    Coerce telemetry.unix_epoch_time samples into a fail-closed structured dict.

    Accepts bare int/float seconds, objects with time_us / time_utc_us,
    or mappings with unix_epoch_s / time_utc_us / value.
    """
    if value is None:
        return unix_epoch_time_status_err("unix_epoch_time missing")

    if type(value) is bool:
        return unix_epoch_time_status_err("unix_epoch_time must be a number (not bool)")

    # Bare numeric → treat as seconds
    if type(value) is int or type(value) is float:
        secs = _finite_number(value)
        if secs is None:
            return unix_epoch_time_status_err("unix_epoch_time not finite")
        if secs < 0.0:
            return unix_epoch_time_status_err("unix_epoch_time must be >= 0")
        return {"status": "success", "unix_epoch_time": {"unix_epoch_s": float(secs)}}

    # Mapping forms
    if isinstance(value, Mapping):
        if "unix_epoch_s" in value and value.get("unix_epoch_s") is not None:
            secs = _finite_number(value.get("unix_epoch_s"))
            if secs is None:
                return unix_epoch_time_status_err("unix_epoch_s not finite")
            if secs < 0.0:
                return unix_epoch_time_status_err("unix_epoch_s must be >= 0")
            return {
                "status": "success",
                "unix_epoch_time": {"unix_epoch_s": float(secs)},
            }
        if "time_utc_us" in value and value.get("time_utc_us") is not None:
            secs = _epoch_seconds_from_us(value.get("time_utc_us"))
            if secs is None:
                return unix_epoch_time_status_err("time_utc_us invalid")
            return {
                "status": "success",
                "unix_epoch_time": {"unix_epoch_s": float(secs)},
            }
        if "time_us" in value and value.get("time_us") is not None:
            secs = _epoch_seconds_from_us(value.get("time_us"))
            if secs is None:
                return unix_epoch_time_status_err("time_us invalid")
            return {
                "status": "success",
                "unix_epoch_time": {"unix_epoch_s": float(secs)},
            }
        if "value" in value and value.get("value") is not None:
            return normalize_unix_epoch_time(value.get("value"))
        return unix_epoch_time_status_err("unix_epoch_time mapping missing known keys")

    # Object forms: time_utc_us / time_us attributes
    time_utc_us = getattr(value, "time_utc_us", None)
    if time_utc_us is not None:
        secs = _epoch_seconds_from_us(time_utc_us)
        if secs is None:
            return unix_epoch_time_status_err("time_utc_us invalid")
        return {
            "status": "success",
            "unix_epoch_time": {"unix_epoch_s": float(secs)},
        }

    time_us = getattr(value, "time_us", None)
    if time_us is not None:
        secs = _epoch_seconds_from_us(time_us)
        if secs is None:
            return unix_epoch_time_status_err("time_us invalid")
        return {
            "status": "success",
            "unix_epoch_time": {"unix_epoch_s": float(secs)},
        }

    # Some bindings expose .seconds
    seconds_attr = getattr(value, "seconds", None)
    if seconds_attr is not None:
        secs = _finite_number(seconds_attr)
        if secs is None:
            return unix_epoch_time_status_err("seconds not finite")
        if secs < 0.0:
            return unix_epoch_time_status_err("seconds must be >= 0")
        return {
            "status": "success",
            "unix_epoch_time": {"unix_epoch_s": float(secs)},
        }

    return unix_epoch_time_status_err("unix_epoch_time unrecognized sample shape")
