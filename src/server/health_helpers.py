"""Pure helpers for telemetry.health flags (unit-testable offline)."""
from __future__ import annotations

from typing import Any, Dict, Mapping

KNOWN_FLAGS = (
    "is_gyrometer_calibration_ok",
    "is_accelerometer_calibration_ok",
    "is_magnetometer_calibration_ok",
    "is_local_position_ok",
    "is_global_position_ok",
    "is_home_position_ok",
    "is_armable",
)


def status_err(message: str, **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "failed", "error": str(message)}
    out.update(extra)
    return out


def normalize_health_flags(mapping: Any) -> Dict[str, Any]:
    """
    Coerce a mapping of health flag names → values into a structured dict.

    Only known flag names are kept. Each present value must be a real bool.
    Empty / non-mapping input fails closed.
    """
    if not isinstance(mapping, Mapping):
        return status_err("health flags must be a mapping", value_type=type(mapping).__name__)

    health: Dict[str, bool] = {}
    for key in KNOWN_FLAGS:
        if key not in mapping:
            continue
        val = mapping[key]
        if type(val) is not bool:
            return status_err(f"{key} must be bool", value_type=type(val).__name__)
        health[key] = val

    if not health:
        return status_err("no recognized health flags")

    return {"status": "success", "health": health}
