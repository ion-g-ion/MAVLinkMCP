"""Pure relative-move argument sanitization (no mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Tuple, Union

# Soft safety caps — reject (fail-closed), do not silent-clamp.
MAX_HORIZONTAL_M = 50.0
MAX_ALTITUDE_DELTA_M = 30.0
MAX_YAW_DEG = 180.0


def _reject(msg: str) -> Tuple[bool, Dict[str, str]]:
    return False, {"status": "failed", "error": msg}


def sanitize_relative_move(
    lr: Any, fb: Any, altitude: Any, yaw: Any
) -> Tuple[bool, Union[Dict[str, float], Dict[str, str]]]:
    """
    Validate relative offboard deltas.

    Returns (True, {lr, fb, altitude, yaw}) or (False, {status, error}).
    """
    names = ("lr", "fb", "altitude", "yaw")
    raw = (lr, fb, altitude, yaw)
    out: Dict[str, float] = {}
    for name, val in zip(names, raw):
        if isinstance(val, bool) or val is None:
            return _reject(f"{name}: bool/None not allowed")
        try:
            f = float(val)
        except (TypeError, ValueError):
            return _reject(f"{name}: not a finite number")
        if not math.isfinite(f):
            return _reject(f"{name}: non-finite")
        out[name] = f

    if abs(out["lr"]) > MAX_HORIZONTAL_M or abs(out["fb"]) > MAX_HORIZONTAL_M:
        return _reject(
            f"horizontal move exceeds ±{MAX_HORIZONTAL_M:g} m "
            f"(lr={out['lr']}, fb={out['fb']})"
        )
    if abs(out["altitude"]) > MAX_ALTITUDE_DELTA_M:
        return _reject(
            f"altitude delta exceeds ±{MAX_ALTITUDE_DELTA_M:g} m "
            f"(altitude={out['altitude']})"
        )
    if abs(out["yaw"]) > MAX_YAW_DEG:
        return _reject(
            f"yaw exceeds ±{MAX_YAW_DEG:g} deg (yaw={out['yaw']})"
        )
    return True, out
