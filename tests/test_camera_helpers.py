"""Offline unit tests for photogrammetry helpers (no drone required)."""
import unittest

from mavlinkmcp.camera_helpers import (
    MAX_OVERLAP,
    camera_status_err,
    footprint_m,
    ground_sample_distance_cm_px,
    line_spacing_m,
    photo_count,
    survey_geometry,
    trigger_distance_m,
    validate_camera,
)

# A 1" sensor with an 8.8 mm lens: the common survey quadcopter camera.
CAM = {
    "sensor_width_mm": 13.2,
    "focal_length_mm": 8.8,
    "image_width_px": 5472,
    "image_height_px": 3648,
    "front_overlap": 0.75,
    "side_overlap": 0.65,
}


class TestValidation(unittest.TestCase):
    def test_derives_sensor_height_from_aspect_ratio(self):
        cam = validate_camera(CAM)
        # Square pixels: 13.2 * 3648/5472 = 8.8
        self.assertAlmostEqual(cam["sensor_height_mm"], 8.8, places=6)

    def test_explicit_sensor_height_wins(self):
        cam = validate_camera({**CAM, "sensor_height_mm": 9.9})
        self.assertEqual(cam["sensor_height_mm"], 9.9)

    def test_defaults_for_absent_overlaps(self):
        cam = validate_camera(
            {k: v for k, v in CAM.items() if "overlap" not in k}
        )
        self.assertEqual(cam["front_overlap"], 0.75)
        self.assertEqual(cam["side_overlap"], 0.65)

    def test_percentages_are_accepted_as_percentages(self):
        # A model that writes 75 rather than 0.75 must not fly a survey with
        # 7500% overlap.
        cam = validate_camera({**CAM, "front_overlap": 75, "side_overlap": 65})
        self.assertAlmostEqual(cam["front_overlap"], 0.75)
        self.assertAlmostEqual(cam["side_overlap"], 0.65)

    def test_rejects_missing_fields(self):
        for key in ("sensor_width_mm", "focal_length_mm", "image_width_px"):
            bad = {k: v for k, v in CAM.items() if k != key}
            with self.assertRaises(ValueError):
                validate_camera(bad)

    def test_rejects_nonsense_values(self):
        for bad in ({"focal_length_mm": 0}, {"sensor_width_mm": -1},
                    {"image_width_px": float("nan")}, {"focal_length_mm": True}):
            with self.assertRaises(ValueError):
                validate_camera({**CAM, **bad})

    def test_rejects_overlap_beyond_the_cap(self):
        with self.assertRaises(ValueError):
            validate_camera({**CAM, "side_overlap": 0.99})
        with self.assertRaises(ValueError):
            validate_camera({**CAM, "side_overlap": -0.1})

    def test_rejects_non_mapping(self):
        with self.assertRaises(ValueError):
            validate_camera("a camera")


class TestGeometry(unittest.TestCase):
    def setUp(self):
        self.cam = validate_camera(CAM)

    def test_gsd_matches_the_textbook_formula(self):
        # GSD = sensor_width * altitude / (focal * image_width), in cm.
        # 13.2 * 40 / (8.8 * 5472) = 1.0965e-2 m = 1.0965 cm
        self.assertAlmostEqual(
            ground_sample_distance_cm_px(self.cam, 40.0), 1.0965, places=4
        )

    def test_gsd_scales_linearly_with_altitude(self):
        low = ground_sample_distance_cm_px(self.cam, 40.0)
        high = ground_sample_distance_cm_px(self.cam, 80.0)
        self.assertAlmostEqual(high, low * 2.0, places=9)

    def test_footprint_is_gsd_times_pixels(self):
        fp = footprint_m(self.cam, 40.0)
        self.assertAlmostEqual(fp["width_m"], 60.0, places=6)
        self.assertAlmostEqual(fp["height_m"], 40.0, places=6)

    def test_spacing_and_trigger_follow_the_overlaps(self):
        # 60 m wide footprint at 65% side overlap leaves 21 m between lines.
        self.assertAlmostEqual(line_spacing_m(self.cam, 40.0), 21.0, places=6)
        # 40 m tall footprint at 75% front overlap triggers every 10 m.
        self.assertAlmostEqual(trigger_distance_m(self.cam, 40.0), 10.0, places=6)

    def test_more_overlap_means_tighter_spacing(self):
        loose = validate_camera({**CAM, "side_overlap": 0.4})
        tight = validate_camera({**CAM, "side_overlap": 0.8})
        self.assertGreater(line_spacing_m(loose, 40.0), line_spacing_m(tight, 40.0))

    def test_rejects_non_positive_altitude(self):
        for bad in (0.0, -10.0, float("nan")):
            with self.assertRaises(ValueError):
                footprint_m(self.cam, bad)

    def test_photo_count_includes_the_first_frame(self):
        self.assertEqual(photo_count(100.0, 10.0), 11)
        self.assertEqual(photo_count(0.0, 10.0), 0)
        with self.assertRaises(ValueError):
            photo_count(100.0, 0.0)

    def test_survey_geometry_bundles_everything(self):
        g = survey_geometry(CAM, 40.0)
        self.assertAlmostEqual(g["line_spacing_m"], 21.0, places=6)
        self.assertAlmostEqual(g["trigger_distance_m"], 10.0, places=6)
        self.assertAlmostEqual(g["gsd_cm_px"], 1.0965, places=4)
        self.assertEqual(g["side_overlap"], 0.65)

    def test_status_err_is_fail_closed(self):
        self.assertEqual(camera_status_err("x"), {"status": "failed", "error": "x"})


if __name__ == "__main__":
    unittest.main()
