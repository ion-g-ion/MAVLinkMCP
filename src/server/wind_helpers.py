"""Pure helpers for wind telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def wind_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _finite_float(value: Any, field: str) -> tuple[Optional[float], Optional[str]]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None, f"{field} missing or not a number"
    if not math.isfinite(f):
        return None, f"{field} not finite"
    return f, None


def normalize_wind(fields: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Normalize MAVSDK Wind-like fields into a fail-closed structured dict.

    Requires finite wind_x_ned_m_s, wind_y_ned_m_s, wind_z_ned_m_s when using
    NED components. Alternatively accepts speed_m_s + direction_deg.

    Prefer NED component fields if any of wind_*_ned_m_s is present; else
    speed/direction pair.
    """
    if not isinstance(fields, Mapping):
        return wind_status_err("wind fields must be a mapping")

    ned_keys = ("wind_x_ned_m_s", "wind_y_ned_m_s", "wind_z_ned_m_s")
    has_ned = any(k in fields and fields.get(k) is not None for k in ned_keys)

    if has_ned:
        body: Dict[str, Any] = {}
        for k in ned_keys:
            if k not in fields or fields.get(k) is None:
                return wind_status_err(f"{k} required when NED wind is used")
            v, err = _finite_float(fields.get(k), k)
            if err is not None or v is None:
                return wind_status_err(err or f"{k} invalid")
            body[k] = float(v)
        # Optional horizontal speed derived check only when speed provided
        if "horizontal_speed_m_s" in fields and fields.get("horizontal_speed_m_s") is not None:
            hs, err = _finite_float(fields.get("horizontal_speed_m_s"), "horizontal_speed_m_s")
            if err is not None or hs is None:
                return wind_status_err(err or "horizontal_speed_m_s invalid")
            if hs < 0.0:
                return wind_status_err("horizontal_speed_m_s must be >= 0")
            body["horizontal_speed_m_s"] = float(hs)
        return {"status": "success", "wind": body}

    # Speed + direction path
    if "speed_m_s" not in fields and "direction_deg" not in fields:
        return wind_status_err("no recognized wind fields")

    body2: Dict[str, Any] = {}
    if "speed_m_s" in fields and fields.get("speed_m_s") is not None:
        sp, err = _finite_float(fields.get("speed_m_s"), "speed_m_s")
        if err is not None or sp is None:
            return wind_status_err(err or "speed_m_s invalid")
        if sp < 0.0:
            return wind_status_err("speed_m_s must be >= 0")
        body2["speed_m_s"] = float(sp)
    else:
        return wind_status_err("speed_m_s required without NED components")

    if "direction_deg" in fields and fields.get("direction_deg") is not None:
        d, err = _finite_float(fields.get("direction_deg"), "direction_deg")
        if err is not None or d is None:
            return wind_status_err(err or "direction_deg invalid")
        if abs(d) > 720.0:
            return wind_status_err("direction_deg out of range")
        body2["direction_deg"] = float(d)

    if "vertical_speed_m_s" in fields and fields.get("vertical_speed_m_s") is not None:
        vs, err = _finite_float(fields.get("vertical_speed_m_s"), "vertical_speed_m_s")
        if err is not None or vs is None:
            return wind_status_err(err or "vertical_speed_m_s invalid")
        body2["vertical_speed_m_s"] = float(vs)

    return {"status": "success", "wind": body2}
