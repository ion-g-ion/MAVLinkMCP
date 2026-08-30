"""Pure estimation and safety checks for a flight plan (no mavsdk, no vehicle).

Two jobs, deliberately kept together because the second consumes the first:

* ``estimate_plan`` answers "how long, how far, how many photos, how much
  battery" so a plan can be costed and iterated at a desk.
* ``check_plan`` answers "is there anything wrong with this route" and produces
  findings. Only an ``error`` blocks; warnings inform.

The highest-value check by far is FAR_FROM_HOME. A polygon read off the wrong
map, or a transposed digit in a latitude, produces a perfectly well-formed plan
on the other side of the planet — and nothing else in the pipeline would notice.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .geo_helpers import haversine_m
from .plan_helpers import utc_now_iso, waypoint_coords

SEVERITY_ERROR = "error"
SEVERITY_WARNING = "warning"
SEVERITY_INFO = "info"

DEFAULT_LIMITS: Dict[str, float] = {
    # A leg longer than this is nearly always a coordinate typo rather than an
    # intentional transit, and it is the shape a fly-away takes.
    "max_leg_m": 2000.0,
    "min_altitude_m": 0.5,
    "max_altitude_m": 500.0,
    "max_speed_m_s": 30.0,
    "warn_waypoint_count": 500.0,
    "max_waypoint_count": 2000.0,
    "warn_home_distance_m": 2000.0,
    "max_home_distance_m": 20000.0,
    # Vertical speed used to price the initial climb.
    "climb_rate_m_s": 2.5,
    # Seconds lost to each direction change sharper than turn_angle_deg.
    "turn_time_s": 3.0,
    "turn_angle_deg": 30.0,
}


def check_status_err(message: Any) -> dict:
    """Structured failure payload for plan checks (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def finding(
    severity: str, code: str, message: str, waypoint_index: Optional[int] = None
) -> dict:
    """One check result."""
    return {
        "severity": severity,
        "code": code,
        "message": message,
        "waypoint_index": waypoint_index,
    }


def _limits(overrides: Optional[Mapping[str, Any]] = None) -> Dict[str, float]:
    merged = dict(DEFAULT_LIMITS)
    for key, value in (overrides or {}).items():
        if key in merged and value is not None:
            merged[key] = float(value)
    return merged


def _turn_angle_deg(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    """Course change at ``b`` when flying a -> b -> c, in degrees."""
    from .geo_helpers import bearing_deg

    delta = abs(bearing_deg(b, c) - bearing_deg(a, b)) % 360.0
    return 360.0 - delta if delta > 180.0 else delta


def estimate_plan(
    plan: Mapping[str, Any],
    battery_capacity_mah: Optional[float] = None,
    cruise_current_a: Optional[float] = None,
    home: Optional[Tuple[float, float]] = None,
    limits: Optional[Mapping[str, Any]] = None,
) -> dict:
    """Distance, duration, photo count and (when priceable) battery draw."""
    lim = _limits(limits)
    waypoints = list(plan.get("waypoints") or [])
    if not waypoints:
        return {
            "distance_m": 0.0,
            "duration_s": 0.0,
            "photo_count": 0,
            "battery_pct": None,
            "battery_basis": "no waypoints",
        }

    coords = waypoint_coords(waypoints)
    speeds = [float(w.get("speed_m_s") or 0.0) for w in waypoints]
    altitudes = [float(w.get("relative_altitude_m") or 0.0) for w in waypoints]

    distance = 0.0
    duration = 0.0
    for i in range(len(coords) - 1):
        leg = haversine_m(coords[i], coords[i + 1])
        distance += leg
        # MAVSDK applies a mission item's speed on the way *to* that item.
        speed = speeds[i + 1] or speeds[i]
        if speed > 0.0:
            duration += leg / speed

    # Transit to the first waypoint, and home again when RTL is set.
    transit_m = 0.0
    if home is not None:
        transit_m += haversine_m(home, coords[0])
        if plan.get("return_to_launch"):
            transit_m += haversine_m(coords[-1], home)
        cruise = max([s for s in speeds if s > 0.0], default=0.0)
        if cruise > 0.0:
            duration += transit_m / cruise

    turns = 0
    for i in range(1, len(coords) - 1):
        if _turn_angle_deg(coords[i - 1], coords[i], coords[i + 1]) >= lim["turn_angle_deg"]:
            turns += 1
    duration += turns * lim["turn_time_s"]

    if lim["climb_rate_m_s"] > 0.0 and altitudes:
        duration += max(altitudes[0], 0.0) / lim["climb_rate_m_s"]

    derived = (plan.get("generator") or {}).get("derived") or {}
    trigger = derived.get("trigger_distance_m")
    photos = 0
    if trigger:
        try:
            photos = int(math.floor(distance / float(trigger))) + 1
        except (TypeError, ValueError, ZeroDivisionError):
            photos = 0

    battery_pct: Optional[float] = None
    basis = "battery_capacity_mah and cruise_current_a not supplied"
    if battery_capacity_mah and cruise_current_a:
        try:
            endurance_s = (float(battery_capacity_mah) / 1000.0) / float(cruise_current_a) * 3600.0
            if endurance_s > 0.0:
                battery_pct = round(duration / endurance_s * 100.0, 1)
                basis = f"{endurance_s / 60.0:.1f} min endurance at {cruise_current_a} A"
        except (TypeError, ValueError, ZeroDivisionError):
            battery_pct = None
            basis = "invalid battery parameters"

    return {
        "distance_m": round(distance, 2),
        "transit_m": round(transit_m, 2),
        "duration_s": round(duration, 1),
        "turn_count": turns,
        "photo_count": photos,
        "battery_pct": battery_pct,
        "battery_basis": basis,
    }


def check_plan(
    plan: Mapping[str, Any],
    home: Optional[Tuple[float, float]] = None,
    limits: Optional[Mapping[str, Any]] = None,
    estimate: Optional[Mapping[str, Any]] = None,
) -> List[dict]:
    """Static safety findings for a plan. An empty list means nothing to report."""
    lim = _limits(limits)
    findings: List[dict] = []
    waypoints = list(plan.get("waypoints") or [])

    if not waypoints:
        findings.append(
            finding(SEVERITY_ERROR, "EMPTY_PLAN", "plan contains no waypoints")
        )
        return findings

    count = len(waypoints)
    if count > lim["max_waypoint_count"]:
        findings.append(
            finding(
                SEVERITY_ERROR,
                "WAYPOINT_COUNT_EXCEEDS_LIMIT",
                f"{count} waypoints exceeds the hard limit of {int(lim['max_waypoint_count'])}",
            )
        )
    elif count > lim["warn_waypoint_count"]:
        findings.append(
            finding(
                SEVERITY_WARNING,
                "WAYPOINT_COUNT_EXCEEDS_LIMIT",
                f"{count} waypoints is large; some autopilots limit mission storage",
            )
        )

    for idx, wp in enumerate(waypoints):
        alt = float(wp.get("relative_altitude_m", 0.0))
        if not (lim["min_altitude_m"] <= alt <= lim["max_altitude_m"]):
            findings.append(
                finding(
                    SEVERITY_ERROR,
                    "ALTITUDE_OUT_OF_RANGE",
                    f"relative_altitude_m {alt} outside "
                    f"[{lim['min_altitude_m']}, {lim['max_altitude_m']}]",
                    idx,
                )
            )
        speed = float(wp.get("speed_m_s", 0.0))
        if not (0.0 < speed <= lim["max_speed_m_s"]):
            findings.append(
                finding(
                    SEVERITY_ERROR,
                    "SPEED_OUT_OF_RANGE",
                    f"speed_m_s {speed} outside (0, {lim['max_speed_m_s']}]",
                    idx,
                )
            )

    coords = waypoint_coords(waypoints)
    for i in range(len(coords) - 1):
        leg = haversine_m(coords[i], coords[i + 1])
        if leg > lim["max_leg_m"]:
            findings.append(
                finding(
                    SEVERITY_ERROR,
                    "LEG_TOO_LONG",
                    f"leg {i}->{i + 1} is {leg / 1000.0:.2f} km, over the "
                    f"{lim['max_leg_m'] / 1000.0:.2f} km limit; check for a coordinate typo",
                    i,
                )
            )
        elif leg == 0.0:
            findings.append(
                finding(
                    SEVERITY_WARNING,
                    "DUPLICATE_WAYPOINT",
                    f"waypoints {i} and {i + 1} are at the same position",
                    i,
                )
            )

    if home is not None:
        distance = haversine_m(home, coords[0])
        if distance > lim["max_home_distance_m"]:
            findings.append(
                finding(
                    SEVERITY_ERROR,
                    "FAR_FROM_HOME",
                    f"first waypoint is {distance / 1000.0:.1f} km from home, over the "
                    f"{lim['max_home_distance_m'] / 1000.0:.1f} km limit; "
                    "the plan may have been drawn on the wrong location",
                    0,
                )
            )
        elif distance > lim["warn_home_distance_m"]:
            findings.append(
                finding(
                    SEVERITY_WARNING,
                    "FAR_FROM_HOME",
                    f"first waypoint is {distance / 1000.0:.1f} km from home",
                    0,
                )
            )

    if estimate and estimate.get("battery_pct") is not None:
        pct = float(estimate["battery_pct"])
        if pct >= 100.0:
            findings.append(
                finding(
                    SEVERITY_ERROR,
                    "EXCEEDS_ENDURANCE",
                    f"estimated {pct:.0f}% of battery needed; the plan does not fit one charge",
                )
            )
        elif pct >= 80.0:
            findings.append(
                finding(
                    SEVERITY_WARNING,
                    "EXCEEDS_ENDURANCE",
                    f"estimated {pct:.0f}% of battery needed, leaving little reserve",
                )
            )

    return findings


def findings_passed(findings: Sequence[Mapping[str, Any]]) -> bool:
    """True when no finding is an error."""
    return not any(f.get("severity") == SEVERITY_ERROR for f in findings)


def summarize_checks(findings: Sequence[Mapping[str, Any]]) -> dict:
    """The ``checks`` block stored on a plan revision."""
    return {
        "checked_at": utc_now_iso(),
        "passed": findings_passed(findings),
        "error_count": sum(1 for f in findings if f.get("severity") == SEVERITY_ERROR),
        "warning_count": sum(1 for f in findings if f.get("severity") == SEVERITY_WARNING),
        "findings": list(findings),
    }
