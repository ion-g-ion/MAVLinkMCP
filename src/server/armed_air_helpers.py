"""Pure helpers for armed / in-air telemetry (unit-testable offline)."""
from __future__ import annotations

from typing import Any, Dict


def status_err(message: str, **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "failed", "error": str(message)}
    out.update(extra)
    return out


def normalize_is_armed(value: Any) -> Dict[str, Any]:
    """Coerce a telemetry is_armed flag into a structured dict."""
    if isinstance(value, bool):
        return {"status": "success", "is_armed": value}
    if value in (0, 1) and not isinstance(value, bool):
        # Reject bare ints that are not bool — prefer explicit bools from SDK
        pass
    if type(value) is int and value in (0, 1):
        return status_err("is_armed must be bool", value=value)
    if value is None:
        return status_err("is_armed missing")
    try:
        # Explicit bool only; refuse strings like "true"
        if type(value) is bool:
            return {"status": "success", "is_armed": value}
    except Exception as e:  # pragma: no cover
        return status_err(str(e))
    return status_err("is_armed must be bool", value_type=type(value).__name__)


def normalize_in_air(value: Any) -> Dict[str, Any]:
    """Coerce a telemetry in_air flag into a structured dict."""
    if type(value) is bool:
        return {"status": "success", "in_air": value}
    if value is None:
        return status_err("in_air missing")
    return status_err("in_air must be bool", value_type=type(value).__name__)
