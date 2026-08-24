"""Pure helpers for altitude telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def altitude_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _finite_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f):
        return None
    return f


def normalize_altitude(fields: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Normalize altitude telemetry.

    Requires finite altitude_amsl_m and altitude_relative_m.
    Includes altitude_local_m / altitude_terrain_m only when finite.
    """
    if not isinstance(fields, Mapping):
        return altitude_status_err("altitude fields must be a mapping")

    amsl = _finite_float(fields.get("altitude_amsl_m"))
    rel = _finite_float(fields.get("altitude_relative_m"))
    if amsl is None:
        return altitude_status_err("altitude_amsl_m missing or not finite")
    if rel is None:
        return altitude_status_err("altitude_relative_m missing or not finite")

    body: Dict[str, Any] = {
        "altitude_amsl_m": amsl,
        "altitude_relative_m": rel,
    }

    local = _finite_float(fields.get("altitude_local_m"))
    if fields.get("altitude_local_m") is not None and local is None:
        return altitude_status_err("altitude_local_m not finite")
    if local is not None:
        body["altitude_local_m"] = local

    terrain = _finite_float(fields.get("altitude_terrain_m"))
    # MAVSDK often uses NaN for unknown terrain — treat non-finite as omit
    if terrain is not None:
        body["altitude_terrain_m"] = terrain

    return {"status": "success", "altitude": body}
