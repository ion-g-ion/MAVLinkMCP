"""Pure status/mission-progress result helpers (no mavsdk)."""
from __future__ import annotations

from typing import Any, Dict, Mapping, MutableMapping, Optional, Union


def status_ok(payload: Optional[Mapping[str, Any]] = None, **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "success"}
    if payload:
        out.update(dict(payload))
    if extra:
        out.update(extra)
    return out


def status_err(message: Union[str, BaseException], **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"status": "failed", "error": str(message)}
    if extra:
        out.update(extra)
    return out


def normalize_status_text(type_val: Any, text: Any) -> Dict[str, Any]:
    return status_ok(type=type_val if not hasattr(type_val, "name") else getattr(type_val, "name", str(type_val)), text=str(text))


def normalize_mission_progress(current: Any, total: Any) -> Dict[str, Any]:
    if isinstance(current, bool) or isinstance(total, bool):
        return status_err("invalid_mission_progress: bool not allowed")
    try:
        cur_i = int(current)
        tot_i = int(total)
    except (TypeError, ValueError) as exc:
        return status_err(f"invalid_mission_progress: {exc}")
    return status_ok(current=cur_i, total=tot_i)
