"""Pure helpers for attitude_euler telemetry (unit-testable offline)."""
from __future__ import annotations

import math
from typing import Any, Dict


def status_err(message: str, **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "failed", "error": str(message)}
    out.update(extra)
    return out


def _finite_float(name: str, value: Any) -> float | Dict[str, Any]:
    if isinstance(value, bool) or value is None:
        return status_err(f"{name} must be a finite number", value=value)
    try:
        f = float(value)
    except (TypeError, ValueError):
        return status_err(f"{name} must be a finite number", value_type=type(value).__name__)
    if not math.isfinite(f):
        return status_err(f"{name} must be finite", value=f)
    return f


def normalize_attitude_euler(roll_deg: Any, pitch_deg: Any, yaw_deg: Any) -> Dict[str, Any]:
    """Validate roll/pitch/yaw degrees and return a structured MCP dict."""
    roll = _finite_float("roll_deg", roll_deg)
    if isinstance(roll, dict):
        return roll
    pitch = _finite_float("pitch_deg", pitch_deg)
    if isinstance(pitch, dict):
        return pitch
    yaw = _finite_float("yaw_deg", yaw_deg)
    if isinstance(yaw, dict):
        return yaw
    return {
        "status": "success",
        "attitude": {
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        },
    }
