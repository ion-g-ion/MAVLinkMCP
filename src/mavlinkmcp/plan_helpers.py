"""Pure helpers for the flight-plan document (unit-testable without mavsdk).

A plan is a JSON document: what was asked for, what was generated from it, and
what has been checked. Revisions are immutable, so a revision is the unit of
review — and ``generator.params`` is a complete input, so any path can be
regenerated from the parameters that produced it rather than trusted blindly.

This module owns the document's shape and its status machine. Reading and
writing it is ``plan_store``; deciding whether it is safe to fly is
``plan_check_helpers``.
"""

from __future__ import annotations

import math
import re
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence

from .geo_helpers import bbox_of, haversine_m, path_length_m

SCHEMA_VERSION = 1

STATUS_DRAFT = "draft"
STATUS_VALIDATED = "validated"
STATUS_UPLOADED = "uploaded"
STATUSES = (STATUS_DRAFT, STATUS_VALIDATED, STATUS_UPLOADED)

# Ids reach the filesystem and originate from a model, so the accepted shape is
# narrow by construction rather than by blocklist.
PLAN_ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]{0,63}$")

# MAVSDK MissionItem.CameraAction names, kept as strings in storage so a plan
# stays JSON. server.py maps these onto the enum at upload time.
CAMERA_ACTIONS = (
    "NONE",
    "TAKE_PHOTO",
    "START_PHOTO_INTERVAL",
    "STOP_PHOTO_INTERVAL",
    "START_VIDEO",
    "STOP_VIDEO",
    "START_PHOTO_DISTANCE",
    "STOP_PHOTO_DISTANCE",
)

# Optional MissionItem fields a stored waypoint may carry through to upload.
OPTIONAL_WAYPOINT_FIELDS = (
    "gimbal_pitch_deg",
    "gimbal_yaw_deg",
    "camera_action",
    "loiter_time_s",
    "camera_photo_interval_s",
    "acceptance_radius_m",
    "yaw_deg",
    "camera_photo_distance_m",
    "vehicle_action",
)


def plan_status_err(message: Any) -> dict:
    """Structured failure payload for plan helpers (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def utc_now_iso() -> str:
    """Timestamp for plan metadata, second-resolution UTC."""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def validate_mission_points(
    mission_points,
    min_rel_alt_m: float = 0.5,
    max_rel_alt_m: float = 500.0,
    max_speed_m_s: float = 30.0,
):
    """Validate waypoint list for initiate_mission (raise ValueError if invalid).

    Returns a shallow-copied list of point dicts after basic numeric checks.
    """
    if not isinstance(mission_points, list):
        raise ValueError("mission_points must be a list")
    if len(mission_points) == 0:
        raise ValueError("mission_points must be a non-empty list")

    required = (
        "latitude_deg",
        "longitude_deg",
        "relative_altitude_m",
        "speed_m_s",
        "is_fly_through",
    )
    validated = []
    for idx, point in enumerate(mission_points):
        if not isinstance(point, dict):
            raise ValueError(f"mission_points[{idx}] must be a dict")
        for key in required:
            if key not in point:
                raise ValueError(f"Missing required field in mission point: '{key}'")
        try:
            lat = float(point["latitude_deg"])
            lon = float(point["longitude_deg"])
            rel_alt = float(point["relative_altitude_m"])
            speed = float(point["speed_m_s"])
        except (TypeError, ValueError) as e:
            raise ValueError(f"mission_points[{idx}] numeric fields invalid: {e}") from e

        for name, v in (
            ("latitude_deg", lat),
            ("longitude_deg", lon),
            ("relative_altitude_m", rel_alt),
            ("speed_m_s", speed),
        ):
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError(f"mission_points[{idx}].{name} must be finite")

        if not (-90.0 <= lat <= 90.0):
            raise ValueError(
                f"Invalid latitude_deg: {lat}. Must be between -90 and 90."
            )
        if not (-180.0 <= lon <= 180.0):
            raise ValueError(
                f"Invalid longitude_deg: {lon}. Must be between -180 and 180."
            )
        if not (min_rel_alt_m <= rel_alt <= max_rel_alt_m):
            raise ValueError(
                f"relative_altitude_m {rel_alt} outside [{min_rel_alt_m}, {max_rel_alt_m}]"
            )
        if not (0.0 < speed <= max_speed_m_s):
            raise ValueError(
                f"speed_m_s {speed} outside (0, {max_speed_m_s}]"
            )

        cleaned = dict(point)
        cleaned["latitude_deg"] = lat
        cleaned["longitude_deg"] = lon
        cleaned["relative_altitude_m"] = rel_alt
        cleaned["speed_m_s"] = speed
        cleaned["is_fly_through"] = bool(point["is_fly_through"])
        validated.append(cleaned)
    return validated


def normalize_camera_action(value: Any) -> str:
    """Coerce a camera action to a known MAVSDK name."""
    name = str(value or "NONE").strip().upper()
    if name not in CAMERA_ACTIONS:
        raise ValueError(
            f"unknown camera_action {value!r}; expected one of {CAMERA_ACTIONS}"
        )
    return name


def sanitize_waypoints(waypoints: Any) -> List[dict]:
    """Validate waypoints and reduce them to JSON-storable dicts.

    Anything MAVSDK would represent as NaN is stored as ``null`` instead, so the
    document round-trips through JSON without losing meaning. Unknown keys are
    dropped rather than carried, which keeps a stored plan a closed vocabulary.
    """
    points = validate_mission_points(waypoints)
    out: List[dict] = []
    for idx, point in enumerate(points):
        wp: Dict[str, Any] = {
            "latitude_deg": point["latitude_deg"],
            "longitude_deg": point["longitude_deg"],
            "relative_altitude_m": point["relative_altitude_m"],
            "speed_m_s": point["speed_m_s"],
            "is_fly_through": point["is_fly_through"],
        }
        for key in OPTIONAL_WAYPOINT_FIELDS:
            if key not in point or point[key] is None:
                continue
            value = point[key]
            if key == "camera_action":
                wp[key] = normalize_camera_action(value)
                continue
            if key == "vehicle_action":
                wp[key] = str(value).strip().upper()
                continue
            try:
                num = float(value)
            except (TypeError, ValueError) as e:
                raise ValueError(f"waypoints[{idx}].{key} must be a number: {value!r}") from e
            if num != num or num in (float("inf"), float("-inf")):
                # NaN is MAVSDK's "unset"; store it as absent so the JSON is valid.
                continue
            wp[key] = num
        out.append(wp)
    return out


def waypoint_coords(waypoints: Sequence[Mapping[str, Any]]) -> List[tuple]:
    """Extract ``(lat, lon)`` pairs in flight order."""
    return [(w["latitude_deg"], w["longitude_deg"]) for w in waypoints]


def plan_stats(waypoints: Sequence[Mapping[str, Any]], area_m2: Optional[float] = None) -> dict:
    """Distance, extent and leg statistics for a waypoint list."""
    coords = waypoint_coords(waypoints)
    legs = [haversine_m(coords[i], coords[i + 1]) for i in range(len(coords) - 1)]
    stats = {
        "waypoint_count": len(coords),
        "path_length_m": round(path_length_m(coords), 2),
        "max_leg_m": round(max(legs), 2) if legs else 0.0,
        "bbox": bbox_of(coords) if coords else None,
    }
    if area_m2 is not None:
        stats["area_m2"] = round(float(area_m2), 2)
    return stats


def slugify_plan_id(name: Any) -> str:
    """Turn a human name into a filesystem-safe plan id.

    Rejects anything that does not round-trip: the id is used as a path segment
    and arrives from a model, so ``..``, separators and empty results must not
    survive by accident.
    """
    raw = str(name or "").strip().lower()
    slug = re.sub(r"[^a-z0-9]+", "-", raw).strip("-")
    slug = re.sub(r"-{2,}", "-", slug)[:64].strip("-")
    if not slug or not PLAN_ID_RE.match(slug):
        raise ValueError(
            f"cannot derive a plan id from {name!r}; "
            "use letters, digits, spaces or hyphens"
        )
    return slug


def validate_plan_id(plan_id: Any) -> str:
    """Accept an existing plan id only if it matches the generated shape exactly."""
    candidate = str(plan_id or "").strip()
    if not PLAN_ID_RE.match(candidate):
        raise ValueError(
            f"invalid plan_id {plan_id!r}; expected {PLAN_ID_RE.pattern}"
        )
    return candidate


def build_plan(
    plan_id: str,
    name: str,
    waypoints: Sequence[Mapping[str, Any]],
    revision: int = 1,
    pattern: str = "waypoints",
    params: Optional[Mapping[str, Any]] = None,
    derived: Optional[Mapping[str, Any]] = None,
    source: Optional[Mapping[str, Any]] = None,
    return_to_launch: bool = True,
    derived_from_revision: Optional[int] = None,
) -> dict:
    """Assemble a plan revision document. Always starts at ``draft``."""
    stored = sanitize_waypoints(list(waypoints))
    area = None
    if derived and derived.get("inset_area_m2") is not None:
        area = derived.get("inset_area_m2")
    elif derived:
        area = derived.get("area_m2")
    return {
        "schema_version": SCHEMA_VERSION,
        "plan_id": plan_id,
        "revision": int(revision),
        "name": str(name),
        "status": STATUS_DRAFT,
        "created_at": utc_now_iso(),
        "derived_from_revision": derived_from_revision,
        "generator": {
            "pattern": pattern,
            "params": dict(params or {}),
            "derived": dict(derived or {}),
            "source": dict(source or {}),
        },
        "waypoints": stored,
        "return_to_launch": bool(return_to_launch),
        "stats": plan_stats(stored, area_m2=area),
        "estimate": None,
        "checks": None,
        "uploaded": None,
    }


def can_upload(plan: Mapping[str, Any]) -> Optional[str]:
    """Return None when the plan may be uploaded, else why it may not."""
    status = plan.get("status")
    if status == STATUS_DRAFT:
        return (
            "plan is a draft; run validate_plan first so the route is checked "
            "before it reaches the vehicle"
        )
    if status not in (STATUS_VALIDATED, STATUS_UPLOADED):
        return f"plan status {status!r} is not uploadable"
    if not plan.get("waypoints"):
        return "plan has no waypoints"
    return None


def waypoints_equal(
    a: Sequence[Mapping[str, Any]],
    b: Sequence[Mapping[str, Any]],
    tol_deg: float = 1e-6,
    tol_m: float = 0.5,
) -> List[str]:
    """Compare two waypoint lists, returning human-readable differences.

    Tolerances are loose enough for the float32 round-trip a mission makes
    through MAVLink and tight enough that a genuinely different route shows up.
    """
    diffs: List[str] = []
    if len(a) != len(b):
        diffs.append(f"waypoint count differs: {len(a)} vs {len(b)}")
        return diffs
    for idx, (left, right) in enumerate(zip(a, b)):
        for key, tol in (
            ("latitude_deg", tol_deg),
            ("longitude_deg", tol_deg),
            ("relative_altitude_m", tol_m),
        ):
            lv = float(left.get(key, float("nan")))
            rv = float(right.get(key, float("nan")))
            if math.isnan(lv) or math.isnan(rv) or abs(lv - rv) > tol:
                diffs.append(f"waypoint[{idx}].{key}: {lv} vs {rv}")
    return diffs


def plan_to_geojson(plan: Mapping[str, Any]) -> dict:
    """Render a plan as GeoJSON: the area, the path, and the numbered waypoints."""
    features: List[dict] = []

    polygon = (plan.get("generator") or {}).get("params", {}).get("polygon")
    if polygon:
        ring = [[float(p[1]), float(p[0])] for p in polygon]
        if ring and ring[0] != ring[-1]:
            ring.append(ring[0])
        features.append(
            {
                "type": "Feature",
                "properties": {"role": "area_of_interest", "name": plan.get("name")},
                "geometry": {"type": "Polygon", "coordinates": [ring]},
            }
        )

    waypoints = plan.get("waypoints") or []
    if len(waypoints) >= 2:
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "role": "flight_path",
                    "waypoint_count": len(waypoints),
                    "path_length_m": (plan.get("stats") or {}).get("path_length_m"),
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [w["longitude_deg"], w["latitude_deg"]] for w in waypoints
                    ],
                },
            }
        )

    for idx, wp in enumerate(waypoints):
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "role": "waypoint",
                    "index": idx,
                    "relative_altitude_m": wp.get("relative_altitude_m"),
                    "speed_m_s": wp.get("speed_m_s"),
                    "camera_action": wp.get("camera_action"),
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [wp["longitude_deg"], wp["latitude_deg"]],
                },
            }
        )

    return {"type": "FeatureCollection", "features": features}
