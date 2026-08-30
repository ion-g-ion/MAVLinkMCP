"""Pure coverage-pattern generators (unit-testable without mavsdk or a vehicle).

A generator turns intent — a polygon, an altitude, a camera — into an ordered
waypoint list. Everything happens in a local metric plane from ``geo_helpers``,
because survey geometry in degrees is wrong by a factor of cos(latitude) in one
axis and unreadable in both.

Generators are ``(ring_latlon, params) -> (waypoints, derived)``: pure, no I/O,
no clock, no randomness. Adding a pattern is one function plus one line in
``PATTERNS``.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Mapping, Sequence, Tuple

from shapely import affinity
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon

from . import camera_helpers
from .geo_helpers import LatLon, local_crs, ring_centroid, to_local, to_wgs84, validate_polygon

# A survey that needs more waypoints than this is almost always a mistake — a
# spacing typo or an overlap of 0.99 — and PX4's mission storage would refuse it
# anyway. Fail loudly with a fixable message instead of uploading nonsense.
MAX_WAYPOINTS = 2000

MIN_LINE_SPACING_M = 0.5
MAX_LINE_SPACING_M = 5000.0


def coverage_status_err(message: Any) -> dict:
    """Structured failure payload for coverage generators (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not bool")
    try:
        out = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{name} must be a number: {value!r}") from e
    if out != out or out in (float("inf"), float("-inf")):
        raise ValueError(f"{name} must be finite")
    return out


def _largest_polygon(geom: Any, context: str) -> Polygon:
    """Reduce a possibly-multipart geometry to its largest polygon part."""
    if isinstance(geom, Polygon):
        if geom.is_empty:
            raise ValueError(f"{context} produced an empty area")
        return geom
    if isinstance(geom, MultiPolygon):
        parts = [p for p in geom.geoms if not p.is_empty]
        if not parts:
            raise ValueError(f"{context} produced an empty area")
        # A concave area inset past its waist splits in two; the largest part is
        # the only defensible automatic choice, and the caller is told.
        return max(parts, key=lambda p: p.area)
    raise ValueError(f"{context} did not produce a polygon (got {geom.geom_type})")


def build_local_polygon(ring: Sequence[LatLon]) -> Tuple[Polygon, Any, Any, LatLon]:
    """Project a WGS84 ring into a local metric plane and validate the polygon."""
    centroid = ring_centroid(ring)
    fwd, inv = local_crs(centroid[0], centroid[1])
    poly = Polygon(to_local(ring, fwd))
    if not poly.is_valid:
        # buffer(0) is the standard shapely repair for a ring that touches or
        # crosses itself once; anything it cannot fix is a real error.
        poly = poly.buffer(0)
        if not isinstance(poly, (Polygon, MultiPolygon)) or poly.is_empty:
            raise ValueError("polygon is self-intersecting and could not be repaired")
        poly = _largest_polygon(poly, "polygon repair")
    if poly.area <= 0.0:
        raise ValueError("polygon has zero area")
    return poly, fwd, inv, centroid


def long_edge_bearing_deg(poly: Polygon) -> float:
    """Compass bearing of the longest edge of the polygon's minimum bounding box.

    Sweeping along this direction minimises the number of turns, which is where
    survey time and battery actually go.
    """
    mrr = poly.minimum_rotated_rectangle
    if not isinstance(mrr, Polygon):
        return 0.0
    coords = list(mrr.exterior.coords)[:-1]
    if len(coords) < 4:
        return 0.0
    best_len = -1.0
    best = (0.0, 0.0)
    for i in range(len(coords)):
        x1, y1 = coords[i]
        x2, y2 = coords[(i + 1) % len(coords)]
        d = math.hypot(x2 - x1, y2 - y1)
        if d > best_len:
            best_len = d
            best = (x2 - x1, y2 - y1)
    # Local plane is x=east, y=north, so atan2(east, north) is a compass bearing.
    # A sweep line has no direction, so fold onto [0, 180).
    return math.degrees(math.atan2(best[0], best[1])) % 180.0


def _sweep_segments(
    poly: Polygon, spacing_m: float, overshoot_m: float
) -> List[List[Tuple[float, float]]]:
    """Boustrophedon sweep segments of an axis-aligned polygon, in flight order."""
    minx, miny, maxx, maxy = poly.bounds
    height = maxy - miny
    # The epsilon keeps an exact multiple (a 100 m field at 25 m spacing) from
    # flapping to an extra, entirely redundant line on floating-point noise.
    n_lines = max(1, int(math.ceil(height / spacing_m - 1e-9)))
    span = (n_lines - 1) * spacing_m
    # Centre the line set in the band so the first and last lines sit inside the
    # area rather than hugging one edge.
    y0 = miny + (height - span) / 2.0

    segments: List[List[Tuple[float, float]]] = []
    for i in range(n_lines):
        y = y0 + i * spacing_m
        cutter = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
        hit = cutter.intersection(poly)
        if hit.is_empty:
            continue

        parts: List[LineString] = []
        if isinstance(hit, LineString):
            parts = [hit]
        elif isinstance(hit, MultiLineString):
            parts = [g for g in hit.geoms if isinstance(g, LineString)]
        # A line grazing a single vertex yields a Point; there is nothing to fly.

        row: List[List[Tuple[float, float]]] = []
        for part in parts:
            xs = [c[0] for c in part.coords]
            if not xs:
                continue
            x_lo, x_hi = min(xs), max(xs)
            if x_hi - x_lo <= 0.0:
                continue
            row.append([(x_lo - overshoot_m, y), (x_hi + overshoot_m, y)])

        row.sort(key=lambda seg: seg[0][0])
        if i % 2 == 1:
            # Reverse both the order of the segments and each segment, so the
            # aircraft ends every line adjacent to the start of the next one.
            row.reverse()
            row = [[seg[1], seg[0]] for seg in row]
        segments.extend(row)
    return segments


def generate_lawnmower(
    ring: Sequence[LatLon], params: Mapping[str, Any]
) -> Tuple[List[dict], Dict[str, Any]]:
    """Parallel-sweep (boustrophedon) coverage of a polygon.

    Required params: ``altitude_m``, ``speed_m_s``, and a spacing — either
    ``line_spacing_m`` directly or a ``camera`` to derive it from.
    Optional: ``sweep_angle_deg`` (default: the area's long axis), ``margin_m``
    (inward inset), ``overshoot_m`` (turn-in room past each line end).
    """
    altitude_m = _finite_float(params.get("altitude_m"), "altitude_m")
    speed_m_s = _finite_float(params.get("speed_m_s"), "speed_m_s")
    margin_m = _finite_float(params.get("margin_m", 0.0) or 0.0, "margin_m")
    overshoot_m = _finite_float(params.get("overshoot_m", 0.0) or 0.0, "overshoot_m")
    if margin_m < 0.0:
        raise ValueError(f"margin_m must not be negative, got {margin_m}")
    if overshoot_m < 0.0:
        raise ValueError(f"overshoot_m must not be negative, got {overshoot_m}")

    camera = params.get("camera")
    spacing_param = params.get("line_spacing_m")
    if spacing_param is not None and camera is not None:
        raise ValueError(
            "give either line_spacing_m or camera, not both — "
            "two sources for the same number is a silent-mismatch waiting to happen"
        )

    derived: Dict[str, Any] = {}
    trigger_distance: float | None = None
    if camera is not None:
        geometry = camera_helpers.survey_geometry(camera, altitude_m)
        spacing = geometry["line_spacing_m"]
        trigger_distance = geometry["trigger_distance_m"]
        derived.update(geometry)
    elif spacing_param is not None:
        spacing = _finite_float(spacing_param, "line_spacing_m")
    else:
        raise ValueError("need either line_spacing_m or camera to derive line spacing")

    if not (MIN_LINE_SPACING_M <= spacing <= MAX_LINE_SPACING_M):
        raise ValueError(
            f"line spacing {spacing:.2f} m outside "
            f"[{MIN_LINE_SPACING_M}, {MAX_LINE_SPACING_M}] m"
        )

    poly, fwd, inv, centroid = build_local_polygon(ring)
    derived["area_m2"] = poly.area

    if margin_m > 0.0:
        inset = poly.buffer(-margin_m)
        if inset.is_empty:
            raise ValueError(
                f"margin_m {margin_m} m leaves no area to survey; "
                "reduce the margin or enlarge the polygon"
            )
        poly = _largest_polygon(inset, f"inset by margin_m {margin_m}")
        derived["inset_area_m2"] = poly.area

    sweep_raw = params.get("sweep_angle_deg")
    if sweep_raw is None:
        sweep_angle = long_edge_bearing_deg(poly)
        derived["sweep_angle_source"] = "long_axis"
    else:
        sweep_angle = _finite_float(sweep_raw, "sweep_angle_deg") % 180.0
        derived["sweep_angle_source"] = "explicit"
    derived["sweep_angle_deg"] = sweep_angle
    derived["line_spacing_m"] = spacing

    # Rotate so the sweep direction lies along +x. A compass bearing b points
    # along (sin b, cos b) in the east/north plane, whose math angle is 90 - b,
    # so rotating by (b - 90) brings it to 0.
    origin = (poly.centroid.x, poly.centroid.y)
    rotated = affinity.rotate(poly, sweep_angle - 90.0, origin=origin)
    segments = _sweep_segments(rotated, spacing, overshoot_m)
    if not segments:
        raise ValueError("no survey lines fit inside the polygon at this spacing")
    derived["line_count"] = len(segments)

    flat: List[Tuple[float, float]] = []
    boundaries: List[int] = []  # index of each segment's first point
    for seg in segments:
        boundaries.append(len(flat))
        flat.extend(seg)

    unrotated = affinity.rotate(
        LineString(flat) if len(flat) > 1 else LineString([flat[0], flat[0]]),
        90.0 - sweep_angle,
        origin=origin,
    )
    local_points = list(unrotated.coords)[: len(flat)]
    coords = to_wgs84(local_points, inv)

    if len(coords) > MAX_WAYPOINTS:
        raise ValueError(
            f"pattern needs {len(coords)} waypoints, over the {MAX_WAYPOINTS} limit; "
            "raise the altitude, widen line_spacing_m, or reduce the area"
        )

    starts = set(boundaries)
    ends = {i - 1 for i in boundaries[1:]} | {len(coords) - 1}

    waypoints: List[dict] = []
    for idx, (lat, lon) in enumerate(coords):
        wp: Dict[str, Any] = {
            "latitude_deg": lat,
            "longitude_deg": lon,
            "relative_altitude_m": altitude_m,
            "speed_m_s": speed_m_s,
            "is_fly_through": True,
        }
        if trigger_distance is not None:
            if idx in starts:
                wp["camera_action"] = "START_PHOTO_DISTANCE"
                wp["camera_photo_distance_m"] = trigger_distance
            elif idx in ends:
                wp["camera_action"] = "STOP_PHOTO_DISTANCE"
        waypoints.append(wp)

    derived["centroid"] = {"latitude_deg": centroid[0], "longitude_deg": centroid[1]}
    return waypoints, derived


PATTERNS: Dict[str, Callable[[Sequence[LatLon], Mapping[str, Any]], Tuple[List[dict], Dict[str, Any]]]] = {
    "lawnmower": generate_lawnmower,
}


def generate(
    pattern: str, polygon: Any, params: Mapping[str, Any]
) -> Tuple[List[dict], Dict[str, Any]]:
    """Validate a polygon and run the named pattern generator."""
    name = str(pattern or "").strip().lower()
    if name not in PATTERNS:
        raise ValueError(
            f"unknown pattern {pattern!r}; available: {sorted(PATTERNS)}"
        )
    ring = validate_polygon(polygon)
    return PATTERNS[name](ring, params)
