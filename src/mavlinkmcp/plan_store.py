"""On-disk persistence for flight plans, map views and map tiles.

The only module in the plan layer that touches the filesystem. Everything it
stores is plain JSON or an image file, on purpose: a plan that decides where an
aircraft flies should be readable, diffable and editable by a human without this
server running.

**What is immutable.** A revision's *generated content* — its waypoints and the
generator block that produced them — never changes; revising a plan writes a new
revision. The lifecycle annotations (``status``, ``checks``, ``estimate``,
``uploaded``) do change in place, because they record what has happened to that
revision rather than what it is.

Ids arrive from a model and become path segments, so every id is re-validated
against a narrow pattern here rather than trusted from the caller.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import uuid
from pathlib import Path
from typing import Any, List, Mapping, Optional

from .map_transform import MapView
from .plan_helpers import STATUSES, utc_now_iso, validate_plan_id

VIEW_ID_RE = re.compile(r"^v[a-f0-9]{12}$")

MAX_PLANS = 200
MAX_REVISIONS_PER_PLAN = 200
MAX_PLAN_BYTES = 4 * 1024 * 1024
MAX_VIEWS = 200
MAX_TILE_BYTES = 2 * 1024 * 1024

# Lifecycle annotations may be rewritten on an existing revision; nothing else.
MUTABLE_PLAN_FIELDS = ("status", "checks", "estimate", "uploaded", "name")


def store_status_err(message: Any) -> dict:
    """Structured failure payload for the plan store (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def data_root(override: Optional[str] = None) -> Path:
    """Root directory for everything this server persists.

    ``MAVLINKMCP_PLANS_DIR`` wins; otherwise the XDG data location. Follows
    ``endpoint.py``'s convention of validating configuration rather than
    silently falling back.
    """
    raw = override if override is not None else os.environ.get("MAVLINKMCP_PLANS_DIR", "")
    raw = (raw or "").strip()
    if raw:
        path = Path(raw).expanduser()
        if not path.is_absolute():
            raise ValueError(
                f"MAVLINKMCP_PLANS_DIR must be an absolute path: {raw!r}"
            )
        return path
    xdg = os.environ.get("XDG_DATA_HOME", "").strip()
    base = Path(xdg).expanduser() if xdg else Path.home() / ".local" / "share"
    return base / "mavlinkmcp"


def plans_dir(root: Optional[Path] = None) -> Path:
    return (root or data_root()) / "plans"


def views_dir(root: Optional[Path] = None) -> Path:
    return (root or data_root()) / "views"


def tiles_dir(root: Optional[Path] = None) -> Path:
    return (root or data_root()) / "tiles"


def _ensure(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    """Write via a temp file in the same directory, then rename.

    A half-written plan is worse than a missing one: the next session would load
    it and fly it.
    """
    _ensure(path.parent)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=".tmp-", suffix=path.suffix)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(payload)
        os.replace(tmp, path)
    except BaseException:
        with_suppress = Path(tmp)
        if with_suppress.exists():
            with_suppress.unlink()
        raise


def _atomic_write_json(path: Path, obj: Any, max_bytes: int = MAX_PLAN_BYTES) -> None:
    payload = json.dumps(obj, indent=2, sort_keys=False, allow_nan=False).encode("utf-8")
    if len(payload) > max_bytes:
        raise ValueError(
            f"document is {len(payload)} bytes, over the {max_bytes} byte limit"
        )
    _atomic_write_bytes(path, payload)


def _read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


# -- plans ----------------------------------------------------------------


def plan_dir(plan_id: str, root: Optional[Path] = None) -> Path:
    """Directory for one plan, with the id re-validated on the way in."""
    return plans_dir(root) / validate_plan_id(plan_id)


def _revision_path(plan_id: str, revision: int, root: Optional[Path] = None) -> Path:
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError(f"revision must be a positive integer, got {revision!r}")
    return plan_dir(plan_id, root) / f"rev-{revision:03d}.json"


def plan_exists(plan_id: str, root: Optional[Path] = None) -> bool:
    try:
        return (plan_dir(plan_id, root) / "meta.json").is_file()
    except ValueError:
        return False


def read_meta(plan_id: str, root: Optional[Path] = None) -> dict:
    path = plan_dir(plan_id, root) / "meta.json"
    if not path.is_file():
        raise FileNotFoundError(f"no plan named {plan_id!r}")
    return _read_json(path)


def list_revisions(plan_id: str, root: Optional[Path] = None) -> List[int]:
    directory = plan_dir(plan_id, root)
    if not directory.is_dir():
        return []
    out = []
    for child in directory.glob("rev-*.json"):
        try:
            out.append(int(child.stem.split("-", 1)[1]))
        except (IndexError, ValueError):
            continue
    return sorted(out)


def save_plan(plan: Mapping[str, Any], root: Optional[Path] = None) -> dict:
    """Write a plan revision and point the head at it."""
    plan_id = validate_plan_id(plan.get("plan_id"))
    revision = int(plan.get("revision", 1))
    directory = plan_dir(plan_id, root)

    existing = list_revisions(plan_id, root)
    if revision not in existing and len(existing) >= MAX_REVISIONS_PER_PLAN:
        raise ValueError(
            f"plan {plan_id!r} already has {len(existing)} revisions "
            f"(limit {MAX_REVISIONS_PER_PLAN})"
        )
    if not directory.exists():
        current = list_plans(root)
        if len(current) >= MAX_PLANS:
            raise ValueError(
                f"plan store already holds {len(current)} plans (limit {MAX_PLANS}); "
                "delete some before creating more"
            )

    _ensure(directory)
    _atomic_write_json(_revision_path(plan_id, revision, root), dict(plan))

    meta = {
        "plan_id": plan_id,
        "name": plan.get("name", plan_id),
        "head": revision,
        "created_at": plan.get("created_at", utc_now_iso()),
        "updated_at": utc_now_iso(),
    }
    if (directory / "meta.json").is_file():
        try:
            previous = _read_json(directory / "meta.json")
            meta["created_at"] = previous.get("created_at", meta["created_at"])
        except (OSError, json.JSONDecodeError):
            pass
    _atomic_write_json(directory / "meta.json", meta)
    return meta


def load_plan(
    plan_id: str, revision: Optional[int] = None, root: Optional[Path] = None
) -> dict:
    """Load a revision, defaulting to the head."""
    meta = read_meta(plan_id, root)
    if revision is None:
        rev = int(meta.get("head", 1))
    elif isinstance(revision, bool) or not isinstance(revision, (int, str)):
        raise ValueError(f"revision must be a positive integer, got {revision!r}")
    else:
        try:
            rev = int(revision)
        except (TypeError, ValueError) as e:
            raise ValueError(f"revision must be a positive integer, got {revision!r}") from e
    path = _revision_path(plan_id, rev, root)
    if not path.is_file():
        available = list_revisions(plan_id, root)
        raise FileNotFoundError(
            f"plan {plan_id!r} has no revision {rev}; available: {available}"
        )
    return _read_json(path)


def update_plan(
    plan_id: str,
    revision: Optional[int] = None,
    root: Optional[Path] = None,
    **fields: Any,
) -> dict:
    """Rewrite the lifecycle annotations of an existing revision.

    Refuses any field outside ``MUTABLE_PLAN_FIELDS``: waypoints and the
    generator block are what a reviewer approved, and must not drift afterwards.
    """
    # ``revision`` is a selector, not a field: it names which revision to
    # annotate. Everything in **fields is checked against the mutable set.
    unknown = [k for k in fields if k not in MUTABLE_PLAN_FIELDS]
    if unknown:
        raise ValueError(
            f"cannot modify {unknown} on a stored revision; "
            f"only {list(MUTABLE_PLAN_FIELDS)} are mutable — revise the plan instead"
        )
    if "status" in fields and fields["status"] not in STATUSES:
        raise ValueError(f"status must be one of {STATUSES}, got {fields['status']!r}")

    plan = load_plan(plan_id, revision, root)
    plan.update(fields)
    _atomic_write_json(
        _revision_path(plan_id, int(plan["revision"]), root), plan
    )
    directory = plan_dir(plan_id, root)
    if (directory / "meta.json").is_file():
        meta = _read_json(directory / "meta.json")
        meta["updated_at"] = utc_now_iso()
        if "name" in fields:
            meta["name"] = fields["name"]
        _atomic_write_json(directory / "meta.json", meta)
    return plan


def list_plans(root: Optional[Path] = None) -> List[dict]:
    """Summarise every stored plan, newest update first."""
    directory = plans_dir(root)
    if not directory.is_dir():
        return []
    out: List[dict] = []
    for child in sorted(directory.iterdir()):
        if not child.is_dir() or not (child / "meta.json").is_file():
            continue
        try:
            meta = _read_json(child / "meta.json")
            plan = load_plan(meta["plan_id"], root=root)
        except (OSError, json.JSONDecodeError, KeyError, FileNotFoundError, ValueError):
            continue
        out.append(
            {
                "plan_id": meta.get("plan_id"),
                "name": meta.get("name"),
                "revision": plan.get("revision"),
                "plan_status": plan.get("status"),
                "pattern": (plan.get("generator") or {}).get("pattern"),
                "waypoint_count": (plan.get("stats") or {}).get("waypoint_count"),
                "path_length_m": (plan.get("stats") or {}).get("path_length_m"),
                "updated_at": meta.get("updated_at"),
                "revisions": list_revisions(meta.get("plan_id", ""), root),
            }
        )
    out.sort(key=lambda p: p.get("updated_at") or "", reverse=True)
    return out


def delete_plan(plan_id: str, root: Optional[Path] = None) -> int:
    """Remove a plan and all its revisions. Returns how many revisions went."""
    directory = plan_dir(plan_id, root)
    if not directory.is_dir():
        raise FileNotFoundError(f"no plan named {plan_id!r}")
    count = len(list_revisions(plan_id, root))
    shutil.rmtree(directory)
    return count


def next_revision(plan_id: str, root: Optional[Path] = None) -> int:
    existing = list_revisions(plan_id, root)
    return (max(existing) + 1) if existing else 1


def unique_plan_id(slug: str, root: Optional[Path] = None) -> str:
    """A free plan id near ``slug``, suffixing ``-2``, ``-3``… if taken."""
    base = validate_plan_id(slug)
    if not plan_exists(base, root):
        return base
    for n in range(2, 100):
        candidate = f"{base[:60]}-{n}"
        if not plan_exists(candidate, root):
            return candidate
    raise ValueError(f"too many plans named like {slug!r}; delete some first")


# -- map views ------------------------------------------------------------


def new_view_id() -> str:
    """Opaque, filesystem-safe id for a rendered view."""
    return "v" + uuid.uuid4().hex[:12]


def validate_view_id(view_id: Any) -> str:
    candidate = str(view_id or "").strip()
    if not VIEW_ID_RE.match(candidate):
        raise ValueError(f"invalid view_id {view_id!r}; expected {VIEW_ID_RE.pattern}")
    return candidate


def view_image_path(view_id: str, root: Optional[Path] = None) -> Path:
    return views_dir(root) / f"{validate_view_id(view_id)}.jpg"


def view_meta_path(view_id: str, root: Optional[Path] = None) -> Path:
    return views_dir(root) / f"{validate_view_id(view_id)}.json"


def save_view(view: MapView, image_bytes: bytes, root: Optional[Path] = None) -> Path:
    """Persist a rendered view and its geotransform side by side."""
    _ensure(views_dir(root))
    path = view_image_path(view.view_id, root)
    _atomic_write_bytes(path, image_bytes)
    _atomic_write_json(view_meta_path(view.view_id, root), view.to_dict())
    prune_views(root=root)
    return path


def load_view(view_id: str, root: Optional[Path] = None) -> MapView:
    path = view_meta_path(view_id, root)
    if not path.is_file():
        raise FileNotFoundError(
            f"no map view {view_id!r}; it may have expired — call get_map_view again"
        )
    return MapView.from_dict(_read_json(path))


def prune_views(max_views: int = MAX_VIEWS, root: Optional[Path] = None) -> int:
    """Drop the oldest views past the cap. Returns how many were removed."""
    directory = views_dir(root)
    if not directory.is_dir():
        return 0
    metas = sorted(directory.glob("v*.json"), key=lambda p: p.stat().st_mtime)
    excess = len(metas) - max_views
    removed = 0
    for meta in metas[: max(0, excess)]:
        image = meta.with_suffix(".jpg")
        for target in (meta, image):
            if target.exists():
                target.unlink()
        removed += 1
    return removed


# -- tile cache -----------------------------------------------------------


def tile_path(provider: str, zoom: int, x: int, y: int, root: Optional[Path] = None) -> Path:
    """Cache location for one tile. Provider names are constrained, not trusted."""
    name = re.sub(r"[^a-z0-9_-]+", "", str(provider).lower())
    if not name:
        raise ValueError(f"invalid tile provider name: {provider!r}")
    return tiles_dir(root) / name / str(int(zoom)) / str(int(x)) / f"{int(y)}.png"


def read_tile(provider: str, zoom: int, x: int, y: int, root: Optional[Path] = None) -> Optional[bytes]:
    path = tile_path(provider, zoom, x, y, root)
    if not path.is_file():
        return None
    try:
        return path.read_bytes()
    except OSError:
        return None


def write_tile(
    provider: str, zoom: int, x: int, y: int, payload: bytes, root: Optional[Path] = None
) -> None:
    if len(payload) > MAX_TILE_BYTES:
        raise ValueError(f"tile is {len(payload)} bytes, over the {MAX_TILE_BYTES} limit")
    _atomic_write_bytes(tile_path(provider, zoom, x, y, root), payload)


def tile_cache_stats(root: Optional[Path] = None) -> dict:
    """Tile count and total bytes on disk, for reporting to an operator."""
    directory = tiles_dir(root)
    if not directory.is_dir():
        return {"tile_count": 0, "bytes": 0}
    count = 0
    total = 0
    for path in directory.rglob("*.png"):
        try:
            total += path.stat().st_size
            count += 1
        except OSError:
            continue
    return {"tile_count": count, "bytes": total}
