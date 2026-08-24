"""Pure helpers for home-position telemetry (offline unit-testable)."""
from __future__ import annotations

import math
from typing import Any, Dict


def status_err(message: str, **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "failed", "error": str(message)}
    out.update(extra)
    return out


def _as_finite_float(value: Any, name: str) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{name} not numeric") from e
    if not math.isfinite(f):
        raise ValueError(f"{name} not finite")
    return f


def normalize_home_position(lat: Any, lon: Any, abs_alt_m: Any) -> Dict[str, Any]:
    """Validate WGS84 home and return a structured success/failure dict."""
    try:
        latitude = _as_finite_float(lat, "latitude_deg")
        longitude = _as_finite_float(lon, "longitude_deg")
        alt = _as_finite_float(abs_alt_m, "absolute_altitude_m")
    except ValueError as e:
        return status_err(str(e))

    if not (-90.0 <= latitude <= 90.0):
        return status_err("latitude_deg out of range", latitude_deg=latitude)
    if not (-180.0 <= longitude <= 180.0):
        return status_err("longitude_deg out of range", longitude_deg=longitude)

    return {
        "status": "success",
        "home": {
            "latitude_deg": latitude,
            "longitude_deg": longitude,
            "absolute_altitude_m": alt,
        },
    }
