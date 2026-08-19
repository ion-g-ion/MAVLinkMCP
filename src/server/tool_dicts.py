"""Pure MCP tool helpers (no MAVSDK import). Offline-testable."""
import math
from types import SimpleNamespace


def _finite_float(value, name):
    try:
        f = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{name} is not a float: {value!r}") from e
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite, got {f!r}")
    return f


def format_velocity_ned(velocity):
    """Map MAVSDK VelocityNed or namespace → serializable dict."""
    return {
        "north_m_s": _finite_float(getattr(velocity, "north_m_s"), "north_m_s"),
        "east_m_s": _finite_float(getattr(velocity, "east_m_s"), "east_m_s"),
        "down_m_s": _finite_float(getattr(velocity, "down_m_s"), "down_m_s"),
    }


def format_gps_info(info):
    """Map MAVSDK GpsInfo or namespace → serializable dict."""
    num = getattr(info, "num_satellites", None)
    fix = getattr(info, "fix_type", None)
    try:
        n = int(num)
    except (TypeError, ValueError) as e:
        raise ValueError(f"num_satellites invalid: {num!r}") from e
    if n < 0:
        raise ValueError(f"num_satellites must be >= 0, got {n}")
    return {
        "num_satellites": n,
        "fix_type": str(fix) if fix is not None else "UNKNOWN",
    }
