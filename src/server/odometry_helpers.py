"""Pure helpers for odometry telemetry (unit-testable without mavsdk)."""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


def odometry_status_err(message: str) -> Dict[str, Any]:
    return {"status": "failed", "error": str(message)}


def _finite_float(value: Any, field: str) -> tuple[Optional[float], Optional[str]]:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None, f"{field} missing or not a number"
    if not math.isfinite(f):
        return None, f"{field} not finite"
    return f, None


def _extract_xyz(obj: Any, prefix: str) -> tuple[Optional[Dict[str, float]], Optional[str]]:
    """
    Accept either a mapping with x/y/z keys or an object with x,y,z attrs.
    prefix used only in error messages.
    """
    if obj is None:
        return None, f"{prefix} missing"
    if isinstance(obj, Mapping):
        raw = obj
        get = raw.get
    else:
        get = lambda k, default=None: getattr(obj, k, default)

    out: Dict[str, float] = {}
    for axis in ("x_m", "y_m", "z_m"):
        # also allow short x/y/z
        val = get(axis)
        if val is None:
            short = axis[0]  # x, y, z
            val = get(short)
        v, err = _finite_float(val, f"{prefix}.{axis}")
        if err is not None or v is None:
            return None, err or f"{prefix}.{axis} invalid"
        # canonicalize to x_m style
        out[axis] = float(v)
    return out, None


def normalize_odometry(fields: Mapping[str, Any]) -> Dict[str, Any]:
    """
    Normalize a subset of MAVSDK Odometry-like fields.

    Requires position_body with finite x/y/z meters.
    Optional velocity_body with the same shape.
    Optional frame_id / child_frame_id non-empty strings when provided.
    """
    if not isinstance(fields, Mapping):
        return odometry_status_err("odometry fields must be a mapping")

    pos_src = fields.get("position_body")
    if pos_src is None:
        # flat keys fallback
        if any(k in fields for k in ("x_m", "y_m", "z_m", "x", "y", "z")):
            pos_src = fields
        else:
            return odometry_status_err("position_body required")

    pos, err = _extract_xyz(pos_src, "position_body")
    if err is not None or pos is None:
        return odometry_status_err(err or "position_body invalid")

    body: Dict[str, Any] = {"position_body": pos}

    if "velocity_body" in fields and fields.get("velocity_body") is not None:
        vel, err = _extract_xyz(fields.get("velocity_body"), "velocity_body")
        if err is not None or vel is None:
            return odometry_status_err(err or "velocity_body invalid")
        body["velocity_body"] = vel

    for key in ("frame_id", "child_frame_id"):
        if key in fields and fields.get(key) is not None:
            v = fields.get(key)
            if type(v) is not str or not v.strip():
                return odometry_status_err(f"{key} must be non-empty str")
            body[key] = v.strip()

    return {"status": "success", "odometry": body}
