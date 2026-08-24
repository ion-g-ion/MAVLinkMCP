"""Pure helpers for distance_sensor telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def distance_sensor_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _finite_nonneg_float(value: Any, field: str) -> tuple[Optional[float], Optional[str]]:
    """Return (float, None) on success or (None, error_message)."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None, f"{field} missing or not a number"
    if not math.isfinite(f):
        return None, f"{field} not finite"
    if f < 0.0:
        return None, f"{field} must be >= 0"
    return f, None


def normalize_distance_sensor(fields: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Normalize rangefinder / distance sensor telemetry.

    Requires finite non-negative current_distance_m.
    Includes min/max/orientation only when provided and valid.
    """
    if not isinstance(fields, Mapping):
        return distance_sensor_status_err("distance_sensor fields must be a mapping")

    current, err = _finite_nonneg_float(fields.get("current_distance_m"), "current_distance_m")
    if err is not None or current is None:
        return distance_sensor_status_err(err or "current_distance_m missing or not a number")

    body: Dict[str, Any] = {"current_distance_m": float(current)}
    minimum_f: Optional[float] = None
    maximum_f: Optional[float] = None

    if "minimum_distance_m" in fields and fields.get("minimum_distance_m") is not None:
        minimum_f, err = _finite_nonneg_float(
            fields.get("minimum_distance_m"), "minimum_distance_m"
        )
        if err is not None or minimum_f is None:
            return distance_sensor_status_err(err or "minimum_distance_m missing or not a number")
        body["minimum_distance_m"] = float(minimum_f)

    if "maximum_distance_m" in fields and fields.get("maximum_distance_m") is not None:
        maximum_f, err = _finite_nonneg_float(
            fields.get("maximum_distance_m"), "maximum_distance_m"
        )
        if err is not None or maximum_f is None:
            return distance_sensor_status_err(err or "maximum_distance_m missing or not a number")
        body["maximum_distance_m"] = float(maximum_f)

    if minimum_f is not None and maximum_f is not None and minimum_f > maximum_f:
        return distance_sensor_status_err(
            "minimum_distance_m must be <= maximum_distance_m"
        )

    if "orientation" in fields and fields.get("orientation") is not None:
        ori: Any = fields.get("orientation")
        # Allow int-like enums with .value
        if type(ori) is not int and hasattr(ori, "value"):
            ori = getattr(ori, "value")
        if type(ori) is not int:
            return distance_sensor_status_err("orientation must be int")
        if ori < 0 or ori > 40:
            return distance_sensor_status_err("orientation out of range 0-40")
        body["orientation"] = ori

    return {"status": "success", "distance_sensor": body}
