"""Map tile providers and fetching — the only outbound network I/O in the server.

The server has no imagery of its own and neither does the drone, so answering
"what is the field in front of me" means fetching an aerial basemap from a tile
provider. That is the sole reason this module exists; nothing else here talks to
the network.

Adding outbound HTTP to something that flies aircraft is a real change in
posture, so the surface is deliberately narrow:

* URLs come from a fixed provider table or one operator-configured template —
  never from a tool argument.
* Requests are capped in count, size, concurrency and time.
* A tile that fails becomes a grey placeholder rather than a failed call: the
  georeferencing is still exact, and the caller is told the imagery is partial.
* ``MAVLINKMCP_MAP_PROVIDER=none`` disables the network entirely and still
  produces a correctly georeferenced (blank) view.
"""

from __future__ import annotations

import asyncio
import os
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple
from urllib.parse import urlparse

from . import plan_store

# A view is at most 8x8 tiles: ~2048 px of source imagery, plenty for a 1024 px
# render, and a hard ceiling on what a single call can pull.
MAX_TILES_PER_REQUEST = 64
MAX_PREFETCH_TILES = 512
FETCH_TIMEOUT_S = 10.0
FETCH_CONCURRENCY = 6
MAX_TOTAL_BYTES = 32 * 1024 * 1024

USER_AGENT = "MAVLinkMCP/0.1 (+https://github.com/mavlinkmcp)"

PROVIDERS: Dict[str, Dict[str, Any]] = {
    "esri": {
        "url": (
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
        "attribution": "Imagery (c) Esri, Maxar, Earthstar Geographics",
        "needs_key": False,
        "kind": "satellite",
    },
    "osm": {
        "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "attribution": "(c) OpenStreetMap contributors",
        "needs_key": False,
        "kind": "street",
    },
    "mapbox": {
        "url": (
            "https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90"
            "?access_token={key}"
        ),
        "attribution": "(c) Mapbox (c) Maxar",
        "needs_key": True,
        "kind": "satellite",
    },
    "maptiler": {
        "url": "https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key={key}",
        "attribution": "(c) MapTiler (c) OpenStreetMap contributors",
        "needs_key": True,
        "kind": "satellite",
    },
    "none": {
        "url": "",
        "attribution": "",
        "needs_key": False,
        "kind": "blank",
    },
}

DEFAULT_PROVIDER = "esri"


def source_status_err(message: Any) -> dict:
    """Structured failure payload for the tile source (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def resolve_provider(
    name: Optional[str] = None,
    api_key: Optional[str] = None,
    template: Optional[str] = None,
) -> Dict[str, Any]:
    """Resolve provider configuration from arguments or environment.

    Validates rather than silently degrading, in the style of ``endpoint.py``: a
    provider that needs a key and has none is a configuration error, not a
    reason to quietly hand back a blank map.
    """
    if name is None:
        name = os.environ.get("MAVLINKMCP_MAP_PROVIDER", DEFAULT_PROVIDER)
    key = api_key if api_key is not None else os.environ.get("MAVLINKMCP_MAP_API_KEY", "")
    custom = template if template is not None else os.environ.get("MAVLINKMCP_MAP_TILE_URL", "")

    provider = str(name or DEFAULT_PROVIDER).strip().lower()
    if provider == "custom":
        url = (custom or "").strip()
        if not url:
            raise ValueError(
                "MAVLINKMCP_MAP_PROVIDER=custom requires MAVLINKMCP_MAP_TILE_URL"
            )
        parsed = urlparse(url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc:
            raise ValueError(f"MAVLINKMCP_MAP_TILE_URL must be an http(s) URL: {url!r}")
        for token in ("{z}", "{x}", "{y}"):
            if token not in url:
                raise ValueError(f"MAVLINKMCP_MAP_TILE_URL must contain {token}")
        return {
            "name": "custom",
            "url": url,
            "attribution": os.environ.get(
                "MAVLINKMCP_MAP_ATTRIBUTION", "custom tile source"
            ),
            "api_key": (key or "").strip(),
            "host": parsed.netloc,
            "kind": "custom",
        }

    if provider not in PROVIDERS:
        raise ValueError(
            f"unknown MAVLINKMCP_MAP_PROVIDER {name!r}; "
            f"expected one of {sorted(PROVIDERS) + ['custom']}"
        )
    spec = PROVIDERS[provider]
    if spec["needs_key"] and not (key or "").strip():
        raise ValueError(f"map provider {provider!r} requires MAVLINKMCP_MAP_API_KEY")
    host = urlparse(spec["url"]).netloc if spec["url"] else ""
    return {
        "name": provider,
        "url": spec["url"],
        "attribution": spec["attribution"],
        "api_key": (key or "").strip(),
        "host": host,
        "kind": spec["kind"],
    }


def tile_url(provider: Mapping[str, Any], zoom: int, x: int, y: int) -> str:
    """Build one tile URL from a resolved provider."""
    if not provider.get("url"):
        raise ValueError(f"provider {provider.get('name')!r} serves no tiles")
    return (
        provider["url"]
        .replace("{z}", str(int(zoom)))
        .replace("{x}", str(int(x)))
        .replace("{y}", str(int(y)))
        .replace("{key}", provider.get("api_key", ""))
    )


def check_host(provider: Mapping[str, Any], url: str) -> None:
    """Refuse any host other than the configured provider's.

    Guards against a malformed template or a redirect pulling tiles from
    somewhere the operator never approved.
    """
    host = urlparse(url).netloc
    if not provider.get("host") or host != provider["host"]:
        raise ValueError(
            f"refusing tile request to {host!r}; provider "
            f"{provider.get('name')!r} is pinned to {provider.get('host')!r}"
        )


async def fetch_tiles(
    provider: Mapping[str, Any],
    zoom: int,
    tiles: Iterable[Tuple[int, int]],
    root: Optional[Any] = None,
    max_tiles: int = MAX_TILES_PER_REQUEST,
    timeout_s: float = FETCH_TIMEOUT_S,
    fetcher: Optional[Any] = None,
) -> Tuple[Dict[Tuple[int, int], bytes], Dict[str, Any]]:
    """Return ``{(x, y): image_bytes}`` for the tiles that could be obtained.

    Cache first, network second. ``fetcher`` is an injection point so tests can
    exercise every path without a network: an async callable ``(url) -> bytes``.
    """
    wanted = list(tiles)
    stats: Dict[str, Any] = {
        "requested": len(wanted),
        "from_cache": 0,
        "fetched": 0,
        "missing": 0,
        "bytes": 0,
        "provider": provider.get("name"),
        "errors": [],
    }
    if len(wanted) > max_tiles:
        raise ValueError(
            f"{len(wanted)} tiles requested, over the {max_tiles} limit; "
            "reduce radius_m or size_px"
        )

    out: Dict[Tuple[int, int], bytes] = {}
    pending: List[Tuple[int, int]] = []
    for x, y in wanted:
        cached = plan_store.read_tile(provider["name"], zoom, x, y, root)
        if cached:
            out[(x, y)] = cached
            stats["from_cache"] += 1
            stats["bytes"] += len(cached)
        else:
            pending.append((x, y))

    if not pending or provider.get("kind") == "blank" or not provider.get("url"):
        stats["missing"] = len(wanted) - len(out)
        return out, stats

    if fetcher is None:
        fetcher = _httpx_fetcher(timeout_s)

    semaphore = asyncio.Semaphore(FETCH_CONCURRENCY)
    budget = {"bytes": stats["bytes"]}

    async def one(x: int, y: int) -> None:
        try:
            url = tile_url(provider, zoom, x, y)
            check_host(provider, url)
        except ValueError as e:
            if len(stats["errors"]) < 5:
                stats["errors"].append(str(e))
            return
        async with semaphore:
            if budget["bytes"] >= MAX_TOTAL_BYTES:
                return
            try:
                payload = await fetcher(url)
            except Exception as e:  # noqa: BLE001 - one bad tile must not fail the view
                if len(stats["errors"]) < 5:
                    stats["errors"].append(f"{x},{y}: {e}")
                return
            if not payload:
                return
            budget["bytes"] += len(payload)
            out[(x, y)] = payload
            stats["fetched"] += 1
            try:
                plan_store.write_tile(provider["name"], zoom, x, y, payload, root)
            except (OSError, ValueError):
                # An uncacheable tile is still a usable tile.
                pass

    await asyncio.gather(*(one(x, y) for x, y in pending))
    stats["bytes"] = budget["bytes"]
    stats["missing"] = len(wanted) - len(out)
    return out, stats


def _httpx_fetcher(timeout_s: float):
    """Build the default async tile fetcher."""

    async def fetch(url: str) -> Optional[bytes]:
        import httpx

        async with httpx.AsyncClient(
            timeout=timeout_s,
            follow_redirects=True,
            headers={"User-Agent": USER_AGENT},
        ) as client:
            response = await client.get(url)
            response.raise_for_status()
            payload = response.content
            if len(payload) > plan_store.MAX_TILE_BYTES:
                raise ValueError(f"tile too large: {len(payload)} bytes")
            return payload

    return fetch
