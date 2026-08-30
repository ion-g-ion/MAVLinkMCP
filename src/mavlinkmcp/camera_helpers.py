"""Pure photogrammetry helpers for survey planning (unit-testable without mavsdk).

Given a camera and a flight altitude these produce the two numbers a coverage
pattern actually needs — how far apart to fly the lines, and how often to trigger
the shutter — plus the ground sample distance, which is the number that tells an
operator whether the survey is worth flying at all.

Convention: the sensor's **width** lies across-track (a landscape-mounted camera
flying along the long axis of the frame). So sensor width sets line spacing and
sensor height sets trigger spacing.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping

# Above this, spacing collapses toward zero and waypoint counts explode. 95% is
# already far beyond what any real survey uses (75-80% front is typical).
MAX_OVERLAP = 0.95

REQUIRED_CAMERA_FIELDS = (
    "sensor_width_mm",
    "focal_length_mm",
    "image_width_px",
    "image_height_px",
)


def camera_status_err(message: Any) -> dict:
    """Structured failure payload for camera helpers (fail-closed)."""
    return {"status": "failed", "error": str(message)}


def _positive_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"camera.{name} must be a number, not bool")
    try:
        out = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"camera.{name} must be a number: {value!r}") from e
    if out != out or out in (float("inf"), float("-inf")):
        raise ValueError(f"camera.{name} must be finite")
    if out <= 0.0:
        raise ValueError(f"camera.{name} must be positive, got {out}")
    return out


def _overlap(value: Any, name: str, default: float) -> float:
    if value is None:
        value = default
    if isinstance(value, bool):
        raise ValueError(f"camera.{name} must be a number, not bool")
    try:
        out = float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"camera.{name} must be a number: {value!r}") from e
    if out != out or out in (float("inf"), float("-inf")):
        raise ValueError(f"camera.{name} must be finite")
    # Accept a percentage as well as a fraction: 75 clearly means 0.75, and a
    # model that writes one when it meant the other should not fly a survey with
    # 7500% overlap.
    if out > 1.0:
        out = out / 100.0
    if not (0.0 <= out <= MAX_OVERLAP):
        raise ValueError(
            f"camera.{name} {out} outside [0, {MAX_OVERLAP}] "
            "(as a fraction; percentages are accepted and divided by 100)"
        )
    return out


def validate_camera(camera: Any) -> Dict[str, float]:
    """Validate a camera spec dict, filling in default overlaps.

    Raises ValueError naming the offending field; callers convert to a tool error.
    """
    if not isinstance(camera, Mapping):
        raise ValueError("camera must be a dict of sensor/lens parameters")
    for key in REQUIRED_CAMERA_FIELDS:
        if key not in camera:
            raise ValueError(f"missing required camera field: '{key}'")

    out: Dict[str, float] = {
        "sensor_width_mm": _positive_float(camera["sensor_width_mm"], "sensor_width_mm"),
        "focal_length_mm": _positive_float(camera["focal_length_mm"], "focal_length_mm"),
        "image_width_px": _positive_float(camera["image_width_px"], "image_width_px"),
        "image_height_px": _positive_float(camera["image_height_px"], "image_height_px"),
        "front_overlap": _overlap(camera.get("front_overlap"), "front_overlap", 0.75),
        "side_overlap": _overlap(camera.get("side_overlap"), "side_overlap", 0.65),
    }
    if camera.get("sensor_height_mm") is not None:
        out["sensor_height_mm"] = _positive_float(
            camera["sensor_height_mm"], "sensor_height_mm"
        )
    else:
        # Square pixels: the sensor's aspect ratio matches the image's.
        out["sensor_height_mm"] = (
            out["sensor_width_mm"] * out["image_height_px"] / out["image_width_px"]
        )
    return out


def ground_sample_distance_cm_px(camera: Mapping[str, float], altitude_m: float) -> float:
    """Centimetres of ground per image pixel at a given altitude."""
    alt = float(altitude_m)
    if not (alt > 0.0) or alt != alt:
        raise ValueError(f"altitude_m must be positive: {altitude_m!r}")
    gsd_m = (camera["sensor_width_mm"] * alt) / (
        camera["focal_length_mm"] * camera["image_width_px"]
    )
    return gsd_m * 100.0


def footprint_m(camera: Mapping[str, float], altitude_m: float) -> Dict[str, float]:
    """Ground footprint of one frame: across-track width and along-track height."""
    alt = float(altitude_m)
    if not (alt > 0.0) or alt != alt:
        raise ValueError(f"altitude_m must be positive: {altitude_m!r}")
    focal = camera["focal_length_mm"]
    return {
        "width_m": camera["sensor_width_mm"] * alt / focal,
        "height_m": camera["sensor_height_mm"] * alt / focal,
    }


def line_spacing_m(camera: Mapping[str, float], altitude_m: float) -> float:
    """Distance between adjacent survey lines for the configured side overlap."""
    width = footprint_m(camera, altitude_m)["width_m"]
    spacing = width * (1.0 - camera["side_overlap"])
    if not (spacing > 0.0):
        raise ValueError(
            "derived line spacing is not positive; reduce side_overlap or raise altitude"
        )
    return spacing


def trigger_distance_m(camera: Mapping[str, float], altitude_m: float) -> float:
    """Distance between shutter triggers for the configured front overlap."""
    height = footprint_m(camera, altitude_m)["height_m"]
    distance = height * (1.0 - camera["front_overlap"])
    if not (distance > 0.0):
        raise ValueError(
            "derived trigger distance is not positive; reduce front_overlap or raise altitude"
        )
    return distance


def photo_count(path_length_m: float, trigger_distance: float) -> int:
    """How many frames a path of this length produces at this trigger spacing."""
    if not (trigger_distance > 0.0):
        raise ValueError("trigger distance must be positive")
    if path_length_m <= 0.0:
        return 0
    return int(math.floor(path_length_m / trigger_distance)) + 1


def survey_geometry(camera: Any, altitude_m: float) -> Dict[str, float]:
    """Everything a coverage pattern needs from a camera, in one call."""
    cam = validate_camera(camera)
    fp = footprint_m(cam, altitude_m)
    return {
        "gsd_cm_px": ground_sample_distance_cm_px(cam, altitude_m),
        "footprint_width_m": fp["width_m"],
        "footprint_height_m": fp["height_m"],
        "line_spacing_m": line_spacing_m(cam, altitude_m),
        "trigger_distance_m": trigger_distance_m(cam, altitude_m),
        "front_overlap": cam["front_overlap"],
        "side_overlap": cam["side_overlap"],
    }
