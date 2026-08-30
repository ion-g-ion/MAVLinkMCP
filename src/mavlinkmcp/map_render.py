"""Pillow rendering of a georeferenced map view (no network, no mavsdk).

Composites fetched tiles into one image, warps it to the view's window and
orientation, and draws the overlays that make the picture *readable* by a model:
where the vehicle is, which way it faces, what the plan covers, and a pixel grid
to report coordinates against.

The warp uses the same rotation convention as ``map_transform``, so a pixel
drawn here and a pixel converted there refer to the same place on the ground.
That shared convention is the whole safety argument for letting a model point at
a field, so it lives in exactly one formula.
"""

from __future__ import annotations

import io
import math
from typing import Any, Mapping, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont

from .map_transform import MapView
from .tile_helpers import TILE_SIZE_PX

BACKGROUND = (34, 38, 43)
MISSING_TILE = (58, 62, 68)

COLOR_AOI = (255, 214, 10)
COLOR_PATH = (0, 229, 255)
COLOR_WAYPOINT = (255, 255, 255)
COLOR_DRONE = (255, 82, 82)
COLOR_GRID = (255, 255, 255, 60)
COLOR_TEXT = (255, 255, 255)
COLOR_SHADOW = (0, 0, 0)

GRID_STEP_PX = 128
SCALE_STEPS_M = (
    5, 10, 20, 25, 50, 100, 200, 250, 500,
    1000, 2000, 2500, 5000, 10000, 20000, 50000,
)


def render_status_err(message: Any) -> dict:
    """Structured failure payload for rendering (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def _font(size: int = 13):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10 has no size argument
        return ImageFont.load_default()


def _text(draw: ImageDraw.ImageDraw, xy: Tuple[float, float], text: str, font, fill=COLOR_TEXT) -> None:
    """Draw text with a 1 px shadow so it survives both bright and dark imagery."""
    x, y = xy
    draw.text((x + 1, y + 1), text, font=font, fill=COLOR_SHADOW)
    draw.text((x, y), text, font=font, fill=fill)


def compose_mosaic(
    tiles: Mapping[Tuple[int, int], bytes],
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
) -> Image.Image:
    """Stitch tiles into one image covering the inclusive tile range.

    A tile that is absent becomes a flat grey block rather than a hole: the
    geometry stays exact and the gap is visible for what it is.
    """
    cols = x_max - x_min + 1
    rows = y_max - y_min + 1
    mosaic = Image.new("RGB", (cols * TILE_SIZE_PX, rows * TILE_SIZE_PX), MISSING_TILE)
    for (x, y), payload in tiles.items():
        if not payload:
            continue
        try:
            tile = Image.open(io.BytesIO(payload)).convert("RGB")
        except Exception:  # noqa: BLE001 - a corrupt tile is a missing tile
            continue
        if tile.size != (TILE_SIZE_PX, TILE_SIZE_PX):
            tile = tile.resize((TILE_SIZE_PX, TILE_SIZE_PX), Image.Resampling.BILINEAR)
        mosaic.paste(tile, ((x - x_min) * TILE_SIZE_PX, (y - y_min) * TILE_SIZE_PX))
    return mosaic


def warp_to_view(
    mosaic: Image.Image,
    view: MapView,
    center_world_px: Tuple[float, float],
    mosaic_origin_px: Tuple[float, float],
) -> Image.Image:
    """Cut the view's window out of the mosaic, applying any rotation.

    The affine maps *output* pixels back to *input* pixels, which is exactly the
    inverse direction of ``MapView.pixel_to_lonlat`` — the same rotation, read
    the other way.
    """
    cx = center_world_px[0] - mosaic_origin_px[0]
    cy = center_world_px[1] - mosaic_origin_px[1]
    w, h = view.width_px, view.height_px
    th = math.radians(view.rotation_deg)
    cos_t, sin_t = math.cos(th), math.sin(th)

    a, b = cos_t, -sin_t
    c = cx - (w / 2.0) * cos_t + (h / 2.0) * sin_t
    d, e = sin_t, cos_t
    f = cy - (w / 2.0) * sin_t - (h / 2.0) * cos_t

    return mosaic.transform(
        (w, h),
        Image.Transform.AFFINE,
        (a, b, c, d, e, f),
        resample=Image.Resampling.BICUBIC,
        fillcolor=MISSING_TILE,
    )


def _nice_scale_length(mpp: float, width_px: int) -> Tuple[int, float]:
    """Pick a round distance whose bar is roughly a fifth of the image."""
    target_m = mpp * width_px / 5.0
    best = SCALE_STEPS_M[0]
    for step in SCALE_STEPS_M:
        if step <= target_m:
            best = step
    return best, best / mpp


def draw_grid(image: Image.Image, step: int = GRID_STEP_PX) -> None:
    """Faint pixel ruler so a model can report coordinates it can actually see."""
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    for x in range(step, image.width, step):
        draw.line([(x, 0), (x, image.height)], fill=COLOR_GRID, width=1)
    for y in range(step, image.height, step):
        draw.line([(0, y), (image.width, y)], fill=COLOR_GRID, width=1)
    image.paste(Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB"))

    font = _font(11)
    draw2 = ImageDraw.Draw(image)
    for x in range(step, image.width, step):
        _text(draw2, (x + 3, 3), str(x), font)
    for y in range(step, image.height, step):
        _text(draw2, (3, y + 3), str(y), font)


def draw_polygon(image: Image.Image, points: Sequence[Tuple[float, float]], color=COLOR_AOI) -> None:
    """Outline an area of interest."""
    if len(points) < 3:
        return
    draw = ImageDraw.Draw(image)
    ring = list(points) + [points[0]]
    draw.line([(p[0], p[1]) for p in ring], fill=COLOR_SHADOW, width=5, joint="curve")
    draw.line([(p[0], p[1]) for p in ring], fill=color, width=3, joint="curve")


def draw_path(
    image: Image.Image,
    points: Sequence[Tuple[float, float]],
    color=COLOR_PATH,
    number_every: int = 0,
) -> None:
    """Draw the flight path, optionally numbering waypoints."""
    if len(points) < 2:
        return
    draw = ImageDraw.Draw(image)
    coords = [(p[0], p[1]) for p in points]
    draw.line(coords, fill=COLOR_SHADOW, width=5, joint="curve")
    draw.line(coords, fill=color, width=2, joint="curve")

    for idx, (x, y) in enumerate(coords):
        r = 3
        draw.ellipse([x - r - 1, y - r - 1, x + r + 1, y + r + 1], fill=COLOR_SHADOW)
        draw.ellipse([x - r, y - r, x + r, y + r], fill=COLOR_WAYPOINT)

    # Start and end are what an operator looks for first.
    font = _font(12)
    _text(draw, (coords[0][0] + 6, coords[0][1] - 14), "start", font, COLOR_PATH)
    _text(draw, (coords[-1][0] + 6, coords[-1][1] + 4), "end", font, COLOR_PATH)

    if number_every > 0:
        for idx in range(0, len(coords), number_every):
            _text(draw, (coords[idx][0] + 5, coords[idx][1] + 4), str(idx), font)


def draw_drone(
    image: Image.Image,
    pixel: Tuple[float, float],
    heading_deg: Optional[float],
    rotation_deg: float = 0.0,
    wedge_deg: float = 60.0,
    wedge_px: float = 130.0,
) -> None:
    """Draw the vehicle as a triangle pointing along its heading.

    The translucent wedge is what makes "the field in front of the drone" a
    question about the picture rather than about compass arithmetic.
    """
    x, y = pixel
    draw = ImageDraw.Draw(image)

    if heading_deg is not None:
        # Screen angle: compass heading, minus whatever bearing is already "up".
        screen_deg = (float(heading_deg) - float(rotation_deg)) % 360.0
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        odraw = ImageDraw.Draw(overlay)
        # PIL pieslice measures clockwise from 3 o'clock; screen "up" is -90.
        start = screen_deg - 90.0 - wedge_deg / 2.0
        end = screen_deg - 90.0 + wedge_deg / 2.0
        odraw.pieslice(
            [x - wedge_px, y - wedge_px, x + wedge_px, y + wedge_px],
            start,
            end,
            fill=(255, 82, 82, 46),
        )
        merged = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
        image.paste(merged)
        draw = ImageDraw.Draw(image)

        th = math.radians(screen_deg)
        nose = (x + 13 * math.sin(th), y - 13 * math.cos(th))
        left = (x + 9 * math.sin(th + 2.5), y - 9 * math.cos(th + 2.5))
        right = (x + 9 * math.sin(th - 2.5), y - 9 * math.cos(th - 2.5))
        draw.polygon([nose, left, right], fill=COLOR_DRONE, outline=COLOR_SHADOW)
    else:
        draw.ellipse([x - 7, y - 7, x + 7, y + 7], fill=COLOR_DRONE, outline=COLOR_SHADOW)

    draw.ellipse([x - 2, y - 2, x + 2, y + 2], fill=COLOR_TEXT)


def draw_chrome(image: Image.Image, view: MapView, note: str = "") -> None:
    """North arrow, scale bar and attribution — the frame around the picture."""
    draw = ImageDraw.Draw(image)
    font = _font(12)
    w, h = image.size

    # North arrow, rotated by whatever bearing is up.
    cx, cy = w - 34, 34
    th = math.radians(-view.rotation_deg)
    tip = (cx + 16 * math.sin(th), cy - 16 * math.cos(th))
    tail = (cx - 12 * math.sin(th), cy + 12 * math.cos(th))
    left = (cx + 7 * math.sin(th + 2.4), cy - 7 * math.cos(th + 2.4))
    right = (cx + 7 * math.sin(th - 2.4), cy - 7 * math.cos(th - 2.4))
    draw.line([tail, tip], fill=COLOR_SHADOW, width=4)
    draw.line([tail, tip], fill=COLOR_TEXT, width=2)
    draw.polygon([tip, left, right], fill=COLOR_TEXT, outline=COLOR_SHADOW)
    _text(draw, (cx - 4, cy + 14), "N", font)

    # Scale bar.
    mpp = view.meters_per_pixel
    metres, bar_px = _nice_scale_length(mpp, w)
    x0, y0 = 12, h - 26
    draw.line([(x0, y0), (x0 + bar_px, y0)], fill=COLOR_SHADOW, width=6)
    draw.line([(x0, y0), (x0 + bar_px, y0)], fill=COLOR_TEXT, width=3)
    for tick in (x0, x0 + bar_px):
        draw.line([(tick, y0 - 5), (tick, y0 + 5)], fill=COLOR_TEXT, width=3)
    label = f"{metres} m" if metres < 1000 else f"{metres / 1000:g} km"
    _text(draw, (x0, y0 - 20), label, font)

    if view.attribution:
        _text(draw, (12, h - 14), view.attribution, _font(11))
    if note:
        _text(draw, (12, 10), note, font)


def to_jpeg(image: Image.Image, quality: int = 82) -> bytes:
    """Encode for transport. JPEG because imagery is photographic."""
    buffer = io.BytesIO()
    image.convert("RGB").save(buffer, format="JPEG", quality=quality, optimize=True)
    return buffer.getvalue()


def blank_canvas(view: MapView) -> Image.Image:
    """A correctly georeferenced view with no imagery behind it."""
    return Image.new("RGB", (view.width_px, view.height_px), BACKGROUND)
