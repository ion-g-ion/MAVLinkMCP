"""Payload builders for the read-only map resources.

An MCP *resource* answers "what is already here" without running anything: which
tile provider is configured, what the map layer will and will not do, which views
have been rendered, and exactly where each one sits on the ground. Tools change
the world; these only describe it, which is why none of them touch the network or
the vehicle.

Kept out of ``server.py`` so the MCP wrappers stay as thin as every tool there,
and separate from ``map_view`` because nothing here renders.

**Redaction.** Every client on a session can read every resource, and a tile URL
can carry an operator's API key inline. So the provider block reports whether a
key is configured and never what it is, and tile templates lose their query
string on the way out.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from . import map_source, map_view, plan_store, tile_helpers
from .map_transform import ORIENTATIONS

# Enough views to see what a session has been looking at without turning the
# resource into a dump of the whole store.
DEFAULT_VIEW_LIMIT = 50


def redact_tile_url(url: Any) -> str:
    """A tile template with its query string removed.

    ``MAVLINKMCP_MAP_TILE_URL`` can carry a key inline (``?key=abc``), so the
    query is dropped even when it holds nothing but a ``{key}`` placeholder --
    telling the two apart from here is guesswork, and guessing wrong publishes a
    credential.
    """
    text = str(url or "")
    if not text:
        return ""
    base, separator, _query = text.partition("?")
    return f"{base}?<redacted>" if separator else base


def _safe_paths() -> Dict[str, Any]:
    """Where this server keeps things, or why it cannot say."""
    try:
        root = plan_store.data_root()
    except ValueError as e:
        # A relative MAVLINKMCP_PLANS_DIR is a configuration error the operator
        # needs to see; it is not a reason to fail the whole resource.
        return {"error": str(e)}
    return {
        "root": str(root),
        "plans": str(plan_store.plans_dir(root)),
        "views": str(plan_store.views_dir(root)),
        "tiles": str(plan_store.tiles_dir(root)),
    }


def provider_catalog() -> dict:
    """Every provider this server knows, plus the one actually configured.

    Reading this is how a client finds out the map layer is misconfigured
    *before* asking for a view, so a bad provider is reported in the payload
    rather than raised.
    """
    catalog: Dict[str, Any] = {
        name: {
            "kind": spec["kind"],
            "needs_key": spec["needs_key"],
            "attribution": spec["attribution"],
            "tile_url": redact_tile_url(spec["url"]),
        }
        for name, spec in map_source.PROVIDERS.items()
    }
    catalog["custom"] = {
        "kind": "custom",
        "needs_key": False,
        "attribution": "whatever MAVLINKMCP_MAP_ATTRIBUTION says",
        "tile_url": "MAVLINKMCP_MAP_TILE_URL, an XYZ template containing {z}, {x} and {y}",
    }

    payload: Dict[str, Any] = {
        "providers": catalog,
        "default": map_source.DEFAULT_PROVIDER,
        "configured_by": {
            "MAVLINKMCP_MAP_PROVIDER": "which provider to use",
            "MAVLINKMCP_MAP_API_KEY": "key for the providers that need one",
            "MAVLINKMCP_MAP_TILE_URL": "XYZ template when the provider is 'custom'",
            "MAVLINKMCP_MAP_ATTRIBUTION": "attribution text for 'custom'",
        },
        "attribution_note": (
            "Provider terms require the notice to be shown. It is drawn onto every "
            "rendered view and repeated in the tool payload; do not strip it."
        ),
    }

    try:
        active = map_source.resolve_provider()
    except ValueError as e:
        payload["active"] = {
            "error": str(e),
            "consequence": "map tools will fail closed until this is fixed",
        }
        return payload

    payload["active"] = {
        "name": active["name"],
        "kind": active["kind"],
        "host": active["host"],
        "attribution": active["attribution"],
        "tile_url": redact_tile_url(active["url"]),
        # The key itself never leaves this process.
        "api_key_configured": bool(active.get("api_key")),
        "serves_tiles": bool(active.get("url")),
    }
    if active["kind"] == "blank":
        payload["active"]["note"] = (
            "provider 'none' disables all outbound network access; views render as a "
            "correctly georeferenced blank canvas with nothing to identify features from"
        )
    return payload


def map_limits() -> dict:
    """The numbers that decide whether a map call will be accepted.

    Worth reading before choosing ``radius_m`` and ``size_px``: both feed the
    tile count, and a view over the per-request tile cap is refused outright.
    """
    return {
        "view": {
            "size_px": {
                "min": map_view.MIN_SIZE_PX,
                "max": map_view.MAX_SIZE_PX,
                "default": map_view.DEFAULT_SIZE_PX,
                "note": (
                    "the ceiling is about tokens, not bytes: image cost scales with "
                    "pixel area, so a 768 px view is roughly 790 tokens and a 1024 px "
                    "one about 1400"
                ),
            },
            "radius_m": {
                "min": map_view.MIN_RADIUS_M,
                "max": map_view.MAX_RADIUS_M,
                "default": map_view.DEFAULT_RADIUS_M,
                "note": "half-width of the ground area covered; values outside are clamped",
            },
            "orientations": list(ORIENTATIONS),
        },
        "zoom": {
            "min": tile_helpers.MIN_ZOOM,
            "max": tile_helpers.MAX_ZOOM,
            "note": "chosen automatically from radius_m and size_px; never passed to get_map_view",
        },
        "tiles": {
            "size_px": tile_helpers.TILE_SIZE_PX,
            "max_per_view": map_source.MAX_TILES_PER_REQUEST,
            "max_per_prefetch": map_source.MAX_PREFETCH_TILES,
            "max_tile_bytes": plan_store.MAX_TILE_BYTES,
            "max_total_bytes": map_source.MAX_TOTAL_BYTES,
            "fetch_timeout_s": map_source.FETCH_TIMEOUT_S,
            "fetch_concurrency": map_source.FETCH_CONCURRENCY,
            "note": (
                "a tile that cannot be fetched renders as flat grey and the response "
                "says so; the georeferencing stays exact either way"
            ),
        },
        "store": {
            "max_plans": plan_store.MAX_PLANS,
            "max_revisions_per_plan": plan_store.MAX_REVISIONS_PER_PLAN,
            "max_views": plan_store.MAX_VIEWS,
            "max_plan_bytes": plan_store.MAX_PLAN_BYTES,
        },
    }


def cache_report() -> dict:
    """What the tile cache and view store currently hold, and where."""
    payload: Dict[str, Any] = {"paths": _safe_paths()}

    try:
        tiles = plan_store.tile_cache_stats()
    except (OSError, ValueError) as e:
        # ValueError comes from data_root() when MAVLINKMCP_PLANS_DIR is relative:
        # the same configuration error _safe_paths() already reported above.
        tiles = {"error": str(e)}
    if "bytes" in tiles:
        tiles = {**tiles, "megabytes": round(tiles["bytes"] / (1024 * 1024), 2)}
    payload["tiles"] = tiles

    try:
        directory = plan_store.views_dir()
        stored = len(list(directory.glob("v*.json"))) if directory.is_dir() else 0
        payload["views"] = {"count": stored, "max_retained": plan_store.MAX_VIEWS}
    except (OSError, ValueError) as e:
        payload["views"] = {"error": str(e)}

    payload["note"] = (
        "prefetch_map_area warms this cache for an area, so later views over it need "
        "no network. Views past max_retained are pruned oldest first."
    )
    return payload


def _view_summary(view: Any, root: Optional[Any] = None) -> dict:
    """The compact description of one stored view."""
    return {
        "view_id": view.view_id,
        "center": {"latitude_deg": view.center_lat, "longitude_deg": view.center_lon},
        "zoom": view.zoom,
        "size_px": [view.width_px, view.height_px],
        "meters_per_pixel": round(view.meters_per_pixel, 4),
        "orientation": view.orientation,
        "rotation_deg": round(view.rotation_deg, 2),
        "provider": view.provider,
        "created_at": view.created_at,
        "tiles_missing": view.tiles_missing,
        "has_image": plan_store.view_image_path(view.view_id, root).is_file(),
    }


def view_index(limit: int = DEFAULT_VIEW_LIMIT, root: Optional[Any] = None) -> dict:
    """Every map view still on disk, newest first.

    These are the ``view_id`` values ``map_transform`` and ``create_survey_plan``
    accept, so this is how a client picks up work from an earlier session.
    """
    try:
        views = [_view_summary(v, root) for v in plan_store.list_views(limit, root)]
    except (OSError, ValueError) as e:
        return {"error": str(e), "views": [], "count": 0}
    return {
        "views": views,
        "count": len(views),
        "max_retained": plan_store.MAX_VIEWS,
        "note": (
            "pass a view_id to map_transform to convert pixels read off that image "
            "into coordinates, or to create_survey_plan with polygon_pixels"
        ),
    }


def view_detail(view_id: str, root: Optional[Any] = None) -> dict:
    """One view's full georeference: centre, scale, rotation and ground footprint.

    Raises FileNotFoundError or ValueError when the view is gone or unreadable --
    there is nothing partial worth returning for a view that does not exist.
    """
    view = plan_store.load_view(view_id, root)
    corners = view.corners_latlon()
    mpp = view.meters_per_pixel
    payload = _view_summary(view, root)
    payload.update(
        {
            "attribution": view.attribution,
            "tiles_total": view.tiles_total,
            "coverage_m": [round(mpp * view.width_px, 1), round(mpp * view.height_px, 1)],
            "bbox": view.bbox(),
            # Clockwise from the image's top-left, matching corners_latlon().
            "corners_latlon": {
                "top_left": list(corners[0]),
                "top_right": list(corners[1]),
                "bottom_right": list(corners[2]),
                "bottom_left": list(corners[3]),
            },
            "up_is": (
                "north"
                if view.orientation == "north_up"
                else f"compass bearing {view.rotation_deg:.0f} deg"
            ),
            "image_path": str(plan_store.view_image_path(view.view_id, root)),
            "note": (
                "report positions on this view as [x, y] pixels and convert them with "
                "map_transform; reading latitude and longitude off the picture by eye "
                "is not accurate and this transform is"
            ),
        }
    )
    return payload


def view_image(view_id: str, root: Optional[Any] = None) -> bytes:
    """The rendered JPEG for a stored view.

    Lets a client look at a view again without re-rendering it, which would cost
    another round of tile fetches for a picture that already exists.
    """
    path = plan_store.view_image_path(view_id, root)
    if not path.is_file():
        raise FileNotFoundError(
            f"no rendered image for map view {view_id!r}; it may have been pruned -- "
            "call get_map_view again"
        )
    return path.read_bytes()
