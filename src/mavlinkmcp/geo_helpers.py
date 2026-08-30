"""Pure helpers for WGS84 geometry and local metric projection (no mavsdk).

Coverage patterns are computed in metres, not degrees, so every generator starts
by projecting its area of interest onto a local tangent plane. The plane is an
azimuthal-equidistant (AEQD) CRS centred on the area itself rather than UTM:
distances and areas are accurate to well under a metre at survey scale, and there
are no zone-edge cases to handle when a field happens to straddle one.

Distances and bearings are geodesic on the WGS84 ellipsoid, matching the plane
the generators work in, so a measured path and a generated one never disagree.

Coordinates are ``(latitude_deg, longitude_deg)`` pairs everywhere in this
module's public surface. Projected coordinates are ``(east_m, north_m)``.
"""

from __future__ import annotations

from functools import lru_cache
from typing import Any, Iterable, List, Sequence, Tuple

from pyproj import CRS, Geod, Transformer

# Distances are geodesic on WGS84, not spherical. The coverage generators work
# in an ellipsoidal AEQD plane, so a spherical haversine here would disagree with
# the geometry it is measuring by ~0.3% -- 9 m on a 3 km survey. pyproj is already
# a dependency; there is no reason to carry the approximation.
GEOD = Geod(ellps="WGS84")

LatLon = Tuple[float, float]
EastNorth = Tuple[float, float]


def geo_status_err(message: Any) -> dict:
    """Structured failure payload for geometry helpers (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def _finite_float(value: Any, name: str) -> float:
    """Coerce to a finite float or raise ValueError naming the field."""
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not bool")
    try:
        out = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{name} must be a number: {value!r}") from e
    if out != out:
        raise ValueError(f"{name} must be finite (got NaN)")
    if out in (float("inf"), float("-inf")):
        raise ValueError(f"{name} must be finite")
    return out


def validate_lat_lon(lat: Any, lon: Any, name: str = "coordinate") -> LatLon:
    """Validate one WGS84 pair, returning finite floats in range."""
    lat_f = _finite_float(lat, f"{name} latitude_deg")
    lon_f = _finite_float(lon, f"{name} longitude_deg")
    if not (-90.0 <= lat_f <= 90.0):
        raise ValueError(f"{name} latitude_deg {lat_f} outside [-90, 90]")
    if not (-180.0 <= lon_f <= 180.0):
        raise ValueError(f"{name} longitude_deg {lon_f} outside [-180, 180]")
    return (lat_f, lon_f)


def validate_polygon(coords: Any, min_vertices: int = 3) -> List[LatLon]:
    """Validate a polygon ring given as ``[[lat, lon], ...]``.

    Accepts an explicitly closed ring (last vertex equal to the first) and drops
    the duplicate, since shapely closes rings itself. Raises ValueError with a
    message naming the offending vertex; callers convert that to a tool error.
    """
    if isinstance(coords, (str, bytes)) or not isinstance(coords, Iterable):
        raise ValueError("polygon must be a list of [latitude_deg, longitude_deg] pairs")

    ring: List[LatLon] = []
    for idx, point in enumerate(coords):
        if isinstance(point, dict):
            pair = (point.get("latitude_deg"), point.get("longitude_deg"))
        elif isinstance(point, (list, tuple)) and len(point) == 2:
            pair = (point[0], point[1])
        else:
            raise ValueError(
                f"polygon[{idx}] must be [latitude_deg, longitude_deg] or "
                "{'latitude_deg': .., 'longitude_deg': ..}"
            )
        ring.append(validate_lat_lon(pair[0], pair[1], name=f"polygon[{idx}]"))

    # An explicitly closed ring carries the first vertex twice; shapely closes
    # rings on its own, so the duplicate would only skew the vertex count.
    if len(ring) >= 2 and ring[0] == ring[-1]:
        ring = ring[:-1]

    if len(ring) < min_vertices:
        raise ValueError(
            f"polygon needs at least {min_vertices} distinct vertices, got {len(ring)}"
        )
    if len(set(ring)) != len(ring):
        raise ValueError("polygon has duplicate vertices")
    return ring


@lru_cache(maxsize=64)
def _transformers(lat0: float, lon0: float) -> Tuple[Transformer, Transformer]:
    crs = CRS.from_proj4(
        f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} +datum=WGS84 +units=m +no_defs"
    )
    wgs84 = CRS.from_epsg(4326)
    return (
        Transformer.from_crs(wgs84, crs, always_xy=True),
        Transformer.from_crs(crs, wgs84, always_xy=True),
    )


def local_crs(lat0: float, lon0: float) -> Tuple[Transformer, Transformer]:
    """Return ``(to_local, to_wgs84)`` for an AEQD plane centred on (lat0, lon0).

    Both transformers are ``always_xy``, so they speak ``(lon, lat)`` on the
    geographic side and ``(east_m, north_m)`` on the projected side — matching
    shapely's x/y convention.

    The origin is rounded to ~1 m before caching: the plane's centre only needs
    to be near the work area, and rounding makes the cache actually hit.
    """
    lat0 = _finite_float(lat0, "lat0")
    lon0 = _finite_float(lon0, "lon0")
    return _transformers(round(lat0, 5), round(lon0, 5))


def ring_centroid(ring: Sequence[LatLon]) -> LatLon:
    """Mean of the ring's vertices — good enough to centre a projection on."""
    if not ring:
        raise ValueError("ring is empty")
    return (
        sum(p[0] for p in ring) / len(ring),
        sum(p[1] for p in ring) / len(ring),
    )


def to_local(ring: Sequence[LatLon], transformer: Transformer) -> List[EastNorth]:
    """Project ``(lat, lon)`` pairs to ``(east_m, north_m)``."""
    return [tuple(transformer.transform(lon, lat)) for lat, lon in ring]  # type: ignore[misc]


def to_wgs84(points: Sequence[EastNorth], transformer: Transformer) -> List[LatLon]:
    """Unproject ``(east_m, north_m)`` pairs back to ``(lat, lon)``."""
    out: List[LatLon] = []
    for east, north in points:
        lon, lat = transformer.transform(east, north)
        out.append((lat, lon))
    return out


def distance_m(a: LatLon, b: LatLon) -> float:
    """Geodesic distance in metres between two ``(lat, lon)`` pairs."""
    if a == b:
        return 0.0
    return float(GEOD.inv(a[1], a[0], b[1], b[0])[2])


# Retained name for readability at call sites that predate the ellipsoidal switch.
haversine_m = distance_m


def bearing_deg(a: LatLon, b: LatLon) -> float:
    """Initial geodesic bearing from ``a`` to ``b``, degrees clockwise from north."""
    if a == b:
        return 0.0
    return float(GEOD.inv(a[1], a[0], b[1], b[0])[0]) % 360.0


def path_length_m(coords: Sequence[LatLon]) -> float:
    """Total length of an open polyline of ``(lat, lon)`` pairs."""
    return sum(
        distance_m(coords[i], coords[i + 1]) for i in range(len(coords) - 1)
    )


def bbox_of(coords: Sequence[LatLon]) -> dict:
    """Axis-aligned WGS84 bounds of a coordinate list."""
    if not coords:
        raise ValueError("cannot compute bbox of an empty coordinate list")
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    return {
        "min_lat": min(lats),
        "min_lon": min(lons),
        "max_lat": max(lats),
        "max_lon": max(lons),
    }


def destination(origin: LatLon, bearing_degrees: float, distance: float) -> LatLon:
    """Point reached by travelling ``distance`` metres from ``origin`` on a bearing."""
    lon, lat, _ = GEOD.fwd(origin[1], origin[0], float(bearing_degrees), float(distance))
    return (lat, lon)
