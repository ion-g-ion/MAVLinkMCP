"""Pure georeferencing for a rendered map view (no network, no Pillow, no mavsdk).

This is the piece that lets a vision model contribute coordinates safely. The
model is good at pointing at a field in an image and bad at inventing latitudes,
so it answers in **pixels** and this module converts. The transform is stored
alongside the rendered image, which means a polygon can always be traced back to
the exact picture it was drawn on.

A view is a square-ish window of Web Mercator at one zoom level, centred on a
position, optionally rotated so a chosen compass bearing points up.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Sequence, Tuple

from .tile_helpers import (
    TILE_SIZE_PX,
    lonlat_to_tile,
    meters_per_pixel,
    tile_to_lonlat,
    validate_zoom,
)

ORIENTATIONS = ("north_up", "heading_up")


def view_status_err(message: Any) -> dict:
    """Structured failure payload for view/transform helpers (fail-closed)."""
    return {"status": "failed", "error": str(message)}


@dataclass
class MapView:
    """A rendered view and everything needed to georeference it.

    ``rotation_deg`` is the compass bearing that appears *up* in the image: 0 for
    a north-up view, the vehicle heading for a heading-up one. Both cases go
    through the same rotation, so there is only one code path to get wrong.
    """

    view_id: str
    center_lat: float
    center_lon: float
    zoom: int
    width_px: int
    height_px: int
    rotation_deg: float = 0.0
    orientation: str = "north_up"
    provider: str = "none"
    attribution: str = ""
    created_at: str = ""
    tiles_total: int = 0
    tiles_missing: int = 0
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def meters_per_pixel(self) -> float:
        return meters_per_pixel(self.center_lat, self.zoom)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MapView":
        known = {k: v for k, v in data.items() if k in cls.__dataclass_fields__}
        return cls(**known)

    # -- geometry ---------------------------------------------------------

    def _center_world_px(self) -> Tuple[float, float]:
        tx, ty = lonlat_to_tile(self.center_lon, self.center_lat, self.zoom)
        return (tx * TILE_SIZE_PX, ty * TILE_SIZE_PX)

    def pixel_to_lonlat(self, px: float, py: float) -> Tuple[float, float]:
        """Image pixel -> ``(lon, lat)``. Origin is the image's top-left corner."""
        cwx, cwy = self._center_world_px()
        dx = px - self.width_px / 2.0
        dy = py - self.height_px / 2.0
        th = math.radians(self.rotation_deg)
        cos_t, sin_t = math.cos(th), math.sin(th)
        # Rotate the image-space offset into world-pixel space. World pixel y
        # grows southward, matching image y, so north-up is the identity.
        wx = cwx + dx * cos_t - dy * sin_t
        wy = cwy + dx * sin_t + dy * cos_t
        return tile_to_lonlat(wx / TILE_SIZE_PX, wy / TILE_SIZE_PX, self.zoom)

    def lonlat_to_pixel(self, lon: float, lat: float) -> Tuple[float, float]:
        """``(lon, lat)`` -> image pixel. The exact inverse of ``pixel_to_lonlat``."""
        cwx, cwy = self._center_world_px()
        tx, ty = lonlat_to_tile(lon, lat, self.zoom)
        wx_off = tx * TILE_SIZE_PX - cwx
        wy_off = ty * TILE_SIZE_PX - cwy
        th = math.radians(self.rotation_deg)
        cos_t, sin_t = math.cos(th), math.sin(th)
        dx = wx_off * cos_t + wy_off * sin_t
        dy = -wx_off * sin_t + wy_off * cos_t
        return (dx + self.width_px / 2.0, dy + self.height_px / 2.0)

    def pixels_to_latlon(self, pixels: Sequence[Sequence[float]]) -> List[Tuple[float, float]]:
        """Convert many pixels to ``(lat, lon)`` pairs, the order the rest of the code uses."""
        out: List[Tuple[float, float]] = []
        for px, py in pixels:
            lon, lat = self.pixel_to_lonlat(px, py)
            out.append((lat, lon))
        return out

    def latlon_to_pixels(self, coords: Sequence[Sequence[float]]) -> List[Tuple[float, float]]:
        """Convert many ``(lat, lon)`` pairs to pixels."""
        return [self.lonlat_to_pixel(lon, lat) for lat, lon in coords]

    def corners_latlon(self) -> List[Tuple[float, float]]:
        """The four image corners as ``(lat, lon)``, clockwise from top-left."""
        pts = [
            (0.0, 0.0),
            (float(self.width_px), 0.0),
            (float(self.width_px), float(self.height_px)),
            (0.0, float(self.height_px)),
        ]
        return self.pixels_to_latlon(pts)

    def bbox(self) -> dict:
        """Axis-aligned WGS84 bounds of the view, including any rotation."""
        corners = self.corners_latlon()
        lats = [c[0] for c in corners]
        lons = [c[1] for c in corners]
        return {
            "min_lat": min(lats),
            "min_lon": min(lons),
            "max_lat": max(lats),
            "max_lon": max(lons),
        }


def validate_orientation(orientation: Any) -> str:
    """Coerce to a known orientation name or raise."""
    name = str(orientation or "north_up").strip().lower()
    if name not in ORIENTATIONS:
        raise ValueError(f"orientation must be one of {ORIENTATIONS}, got {orientation!r}")
    return name


def validate_pixels(
    pixels: Any, width_px: int, height_px: int, min_count: int = 1
) -> List[Tuple[float, float]]:
    """Validate pixel coordinates coming from a model.

    Out-of-bounds pixels are rejected rather than clamped: a model pointing off
    the edge of the image has misread it, and silently pulling the point back to
    the border would produce a plausible-looking polygon in the wrong place.
    """
    if isinstance(pixels, (str, bytes)) or not isinstance(pixels, (list, tuple)):
        raise ValueError("pixels must be a list of [x, y] pairs")
    if len(pixels) < min_count:
        raise ValueError(f"need at least {min_count} pixel pair(s), got {len(pixels)}")

    out: List[Tuple[float, float]] = []
    for idx, point in enumerate(pixels):
        if isinstance(point, dict):
            pair = (point.get("x"), point.get("y"))
        elif isinstance(point, (list, tuple)) and len(point) == 2:
            pair = (point[0], point[1])
        else:
            raise ValueError(f"pixels[{idx}] must be [x, y] or {{'x': .., 'y': ..}}")

        vals: List[float] = []
        for name, raw, limit in (("x", pair[0], width_px), ("y", pair[1], height_px)):
            if isinstance(raw, bool):
                raise ValueError(f"pixels[{idx}].{name} must be a number, not bool")
            try:
                v = float(raw)
            except (TypeError, ValueError) as e:
                raise ValueError(f"pixels[{idx}].{name} must be a number: {raw!r}") from e
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError(f"pixels[{idx}].{name} must be finite")
            if not (0.0 <= v <= float(limit)):
                raise ValueError(
                    f"pixels[{idx}].{name} = {v} outside the image (0..{limit}); "
                    "the point must lie on the view it was read from"
                )
            vals.append(v)
        out.append((vals[0], vals[1]))
    return out


def build_view(
    view_id: str,
    center_lat: float,
    center_lon: float,
    zoom: Any,
    width_px: int,
    height_px: int,
    orientation: str = "north_up",
    heading_deg: float = 0.0,
    **extra: Any,
) -> MapView:
    """Assemble a MapView, resolving orientation into a concrete rotation."""
    name = validate_orientation(orientation)
    rotation = float(heading_deg) % 360.0 if name == "heading_up" else 0.0
    return MapView(
        view_id=view_id,
        center_lat=float(center_lat),
        center_lon=float(center_lon),
        zoom=validate_zoom(zoom),
        width_px=int(width_px),
        height_px=int(height_px),
        rotation_deg=rotation,
        orientation=name,
        **extra,
    )
