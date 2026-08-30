"""Orchestration for map views: fetch tiles, render, georeference, persist.

Ties ``tile_helpers`` (which tiles), ``map_source`` (get them), ``map_render``
(draw them), ``map_transform`` (where things are) and ``plan_store`` (keep them)
into the two operations the tools need: build a view, and draw a plan on one.

Kept out of ``server.py`` so the MCP wrappers stay as thin as every other tool
in this codebase.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from . import map_render, map_source, plan_store
from .map_transform import MapView, build_view, validate_orientation
from .tile_helpers import (
    TILE_SIZE_PX,
    lonlat_to_tile,
    meters_per_pixel,
    tile_range_for_bbox,
    tile_count,
    tiles_in_range,
    zoom_for_radius,
)

DEFAULT_SIZE_PX = 768
MAX_SIZE_PX = 1024
MIN_SIZE_PX = 256
DEFAULT_RADIUS_M = 300.0
MAX_RADIUS_M = 20000.0
MIN_RADIUS_M = 10.0


def map_view_status_err(message: Any) -> dict:
    """Structured failure payload for map views (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def clamp_size_px(value: Any) -> int:
    """Clamp the rendered size.

    The ceiling is about tokens, not bytes: image cost scales with pixel area
    (~w*h/750), so 1024 px is about 1400 tokens and 768 px about 790. A survey
    conversation renders many views.
    """
    if value is None:
        return DEFAULT_SIZE_PX
    if isinstance(value, bool):
        raise ValueError("size_px must be an integer, not bool")
    try:
        size = int(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"size_px must be an integer: {value!r}") from e
    return max(MIN_SIZE_PX, min(MAX_SIZE_PX, size))


def clamp_radius_m(value: Any) -> float:
    if value is None:
        return DEFAULT_RADIUS_M
    try:
        radius = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"radius_m must be a number: {value!r}") from e
    if radius != radius or radius in (float("inf"), float("-inf")):
        raise ValueError("radius_m must be finite")
    return max(MIN_RADIUS_M, min(MAX_RADIUS_M, radius))


def _tile_window(view: MapView) -> Tuple[int, int, int, int]:
    """Inclusive tile range covering the view, including rotation overhang.

    A rotated view's corners reach outside its axis-aligned footprint, so the
    window is computed from the actual corner coordinates rather than from the
    centre and size.
    """
    corners = view.corners_latlon()
    lats = [c[0] for c in corners]
    lons = [c[1] for c in corners]
    return tile_range_for_bbox(min(lons), min(lats), max(lons), max(lats), view.zoom)


async def build_map_view(
    center_lat: float,
    center_lon: float,
    radius_m: float = DEFAULT_RADIUS_M,
    size_px: int = DEFAULT_SIZE_PX,
    orientation: str = "north_up",
    heading_deg: Optional[float] = None,
    provider: Optional[Mapping[str, Any]] = None,
    root: Optional[Any] = None,
    fetcher: Optional[Any] = None,
) -> Tuple[MapView, Any, Dict[str, Any]]:
    """Fetch and compose a view. Returns ``(view, PIL image, fetch stats)``."""
    size = clamp_size_px(size_px)
    radius = clamp_radius_m(radius_m)
    name = validate_orientation(orientation)
    if name == "heading_up" and heading_deg is None:
        raise ValueError(
            "orientation='heading_up' needs a heading; the vehicle is not "
            "reporting one, so use 'north_up'"
        )

    resolved = dict(provider) if provider is not None else map_source.resolve_provider()
    zoom = zoom_for_radius(center_lat, radius, size)

    view = build_view(
        view_id=plan_store.new_view_id(),
        center_lat=center_lat,
        center_lon=center_lon,
        zoom=zoom,
        width_px=size,
        height_px=size,
        orientation=name,
        heading_deg=heading_deg or 0.0,
        provider=str(resolved.get("name", "none")),
        attribution=str(resolved.get("attribution", "")),
        created_at=plan_store.utc_now_iso(),
    )

    x_min, y_min, x_max, y_max = _tile_window(view)
    needed = tile_count(x_min, y_min, x_max, y_max)
    if needed > map_source.MAX_TILES_PER_REQUEST:
        raise ValueError(
            f"this view needs {needed} tiles, over the "
            f"{map_source.MAX_TILES_PER_REQUEST} limit; reduce radius_m or size_px"
        )

    tiles: Dict[Tuple[int, int], bytes] = {}
    stats: Dict[str, Any] = {
        "requested": needed,
        "from_cache": 0,
        "fetched": 0,
        "missing": needed,
        "provider": resolved.get("name"),
        "errors": [],
    }
    if resolved.get("kind") != "blank":
        tiles, stats = await map_source.fetch_tiles(
            resolved,
            view.zoom,
            tiles_in_range(x_min, y_min, x_max, y_max),
            root=root,
            fetcher=fetcher,
        )

    view.tiles_total = needed
    view.tiles_missing = int(stats.get("missing", needed))

    if tiles:
        mosaic = map_render.compose_mosaic(tiles, x_min, y_min, x_max, y_max)
        center_world = (
            lonlat_to_tile(view.center_lon, view.center_lat, view.zoom)[0] * TILE_SIZE_PX,
            lonlat_to_tile(view.center_lon, view.center_lat, view.zoom)[1] * TILE_SIZE_PX,
        )
        origin = (x_min * TILE_SIZE_PX, y_min * TILE_SIZE_PX)
        image = map_render.warp_to_view(mosaic, view, center_world, origin)
    else:
        image = map_render.blank_canvas(view)

    return view, image, stats


def annotate(
    view: MapView,
    image: Any,
    polygon_latlon: Optional[Sequence[Sequence[float]]] = None,
    path_latlon: Optional[Sequence[Sequence[float]]] = None,
    drone_latlon: Optional[Sequence[float]] = None,
    heading_deg: Optional[float] = None,
    grid: bool = True,
    note: str = "",
) -> Any:
    """Draw overlays onto a rendered view, in back-to-front order."""
    if grid:
        map_render.draw_grid(image)
    if polygon_latlon:
        map_render.draw_polygon(image, view.latlon_to_pixels(polygon_latlon))
    if path_latlon:
        map_render.draw_path(image, view.latlon_to_pixels(path_latlon))
    if drone_latlon is not None:
        map_render.draw_drone(
            image,
            view.lonlat_to_pixel(drone_latlon[1], drone_latlon[0]),
            heading_deg,
            view.rotation_deg,
        )
    map_render.draw_chrome(image, view, note=note)
    return image


def persist(view: MapView, image: Any, root: Optional[Any] = None) -> str:
    """Save the rendered view and its geotransform; returns the image path."""
    return str(plan_store.save_view(view, map_render.to_jpeg(image), root))


def describe(
    view: MapView,
    stats: Optional[Mapping[str, Any]] = None,
    drone_latlon: Optional[Sequence[float]] = None,
    heading_deg: Optional[float] = None,
    image_path: Optional[str] = None,
) -> dict:
    """The numbers that make the image usable.

    Without this a model has a pretty picture and no way to say anything true
    about it. With it, the model can point at a feature in pixels and hand those
    pixels back for exact conversion.
    """
    mpp = view.meters_per_pixel
    up = "north" if view.orientation == "north_up" else f"heading {view.rotation_deg:.0f} deg"
    note = f"Up in this image is {up}. Report positions as [x, y] pixels."

    payload: Dict[str, Any] = {
        "view_id": view.view_id,
        "center": {"latitude_deg": view.center_lat, "longitude_deg": view.center_lon},
        "zoom": view.zoom,
        "size_px": [view.width_px, view.height_px],
        "meters_per_pixel": round(mpp, 4),
        "coverage_m": [round(mpp * view.width_px, 1), round(mpp * view.height_px, 1)],
        "orientation": view.orientation,
        "rotation_deg": round(view.rotation_deg, 2),
        "bbox": view.bbox(),
        "provider": view.provider,
        "attribution": view.attribution,
        "tiles_total": view.tiles_total,
        "tiles_missing": view.tiles_missing,
        "note": note,
    }
    if drone_latlon is not None:
        px, py = view.lonlat_to_pixel(drone_latlon[1], drone_latlon[0])
        payload["drone"] = {
            "latitude_deg": drone_latlon[0],
            "longitude_deg": drone_latlon[1],
            "pixel": [round(px, 1), round(py, 1)],
            "heading_deg": None if heading_deg is None else round(float(heading_deg), 1),
        }
    if view.provider == "none":
        payload["imagery_warning"] = (
            "MAVLINKMCP_MAP_PROVIDER=none, so this view has no basemap behind it. "
            "Geometry and overlays are exact, but there is nothing here to identify "
            "ground features from"
        )
    elif view.tiles_missing:
        payload["imagery_warning"] = (
            f"{view.tiles_missing} of {view.tiles_total} tiles are missing and render "
            "as flat grey; the georeferencing is still exact"
        )
    if stats and stats.get("errors"):
        payload["tile_errors"] = list(stats["errors"])[:5]
    if image_path:
        payload["image_path"] = image_path
    return payload
