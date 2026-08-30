"""Pure slippy-map (XYZ / Web Mercator) tile math (no network, no mavsdk).

Every raster tile provider worth using — Esri, Mapbox, MapTiler, OSM — speaks the
same 256 px Web Mercator XYZ scheme, so this module is provider-agnostic. It only
computes *which* tiles cover an area and where they sit; fetching them is
``map_source`` and drawing them is ``map_render``.
"""

from __future__ import annotations

import math
from typing import Any, List, Tuple

TILE_SIZE_PX = 256

# Web Mercator cannot represent the poles; this is the latitude where the
# projection is clipped to a square world.
MAX_MERCATOR_LAT = 85.0511287798066

# Ground resolution in metres per pixel at zoom 0 on the equator, for 256 px
# tiles: earth circumference (2*pi*6378137) / 256.
EQUATOR_MPP_Z0 = 156543.03392804097

MIN_ZOOM = 0
MAX_ZOOM = 22


def tile_status_err(message: Any) -> dict:
    """Structured failure payload for tile math (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def clamp_mercator_lat(lat: float) -> float:
    """Clamp a latitude into the range Web Mercator can represent."""
    return max(-MAX_MERCATOR_LAT, min(MAX_MERCATOR_LAT, lat))


def validate_zoom(zoom: Any) -> int:
    """Coerce to an integer zoom in [MIN_ZOOM, MAX_ZOOM] or raise."""
    if isinstance(zoom, bool):
        raise ValueError("zoom must be an integer, not bool")
    try:
        z = int(zoom)
    except (TypeError, ValueError) as e:
        raise ValueError(f"zoom must be an integer: {zoom!r}") from e
    if not (MIN_ZOOM <= z <= MAX_ZOOM):
        raise ValueError(f"zoom {z} outside [{MIN_ZOOM}, {MAX_ZOOM}]")
    return z


def lonlat_to_tile(lon: float, lat: float, zoom: int) -> Tuple[float, float]:
    """Fractional tile coordinates for a position.

    Fractional rather than integer on purpose: the fraction is what places a
    point within a tile, which is how the view's pixel origin is computed.
    """
    z = validate_zoom(zoom)
    n = 2.0**z
    lat_rad = math.radians(clamp_mercator_lat(lat))
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return (x, y)


def tile_to_lonlat(x: float, y: float, zoom: int) -> Tuple[float, float]:
    """Longitude/latitude of a (fractional) tile coordinate's north-west corner."""
    z = validate_zoom(zoom)
    n = 2.0**z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return (lon, lat)


def meters_per_pixel(lat: float, zoom: int) -> float:
    """Ground resolution at a latitude and zoom, for 256 px tiles."""
    z = validate_zoom(zoom)
    return EQUATOR_MPP_Z0 * math.cos(math.radians(clamp_mercator_lat(lat))) / (2.0**z)


def zoom_for_meters_per_pixel(lat: float, target_mpp: float) -> int:
    """Lowest zoom that is at least as detailed as ``target_mpp``.

    Resolution improves as zoom rises, so this rounds *up* — a view is never
    coarser than asked for, only finer.
    """
    if not (target_mpp > 0.0) or target_mpp != target_mpp:
        raise ValueError(f"target metres-per-pixel must be positive: {target_mpp!r}")
    ratio = EQUATOR_MPP_Z0 * math.cos(math.radians(clamp_mercator_lat(lat))) / target_mpp
    if ratio <= 1.0:
        return MIN_ZOOM
    return max(MIN_ZOOM, min(MAX_ZOOM, math.ceil(math.log2(ratio))))


def zoom_for_radius(lat: float, radius_m: float, size_px: int) -> int:
    """Zoom that fits a ``2 * radius_m`` square into ``size_px`` pixels."""
    if not (radius_m > 0.0):
        raise ValueError(f"radius_m must be positive: {radius_m!r}")
    if not (size_px > 0):
        raise ValueError(f"size_px must be positive: {size_px!r}")
    return zoom_for_meters_per_pixel(lat, (2.0 * radius_m) / float(size_px))


def tile_range_for_bbox(
    min_lon: float, min_lat: float, max_lon: float, max_lat: float, zoom: int
) -> Tuple[int, int, int, int]:
    """Inclusive integer tile range ``(x_min, y_min, x_max, y_max)`` covering a bbox."""
    z = validate_zoom(zoom)
    x0, y0 = lonlat_to_tile(min_lon, max_lat, z)  # NW corner
    x1, y1 = lonlat_to_tile(max_lon, min_lat, z)  # SE corner
    n = int(2**z)
    x_min = max(0, min(n - 1, math.floor(x0)))
    x_max = max(0, min(n - 1, math.floor(x1)))
    y_min = max(0, min(n - 1, math.floor(y0)))
    y_max = max(0, min(n - 1, math.floor(y1)))
    return (x_min, y_min, x_max, y_max)


def tiles_in_range(x_min: int, y_min: int, x_max: int, y_max: int) -> List[Tuple[int, int]]:
    """Every (x, y) tile in an inclusive range, row-major."""
    return [
        (x, y) for y in range(y_min, y_max + 1) for x in range(x_min, x_max + 1)
    ]


def tile_count(x_min: int, y_min: int, x_max: int, y_max: int) -> int:
    """How many tiles an inclusive range covers, without materialising them."""
    return max(0, x_max - x_min + 1) * max(0, y_max - y_min + 1)
