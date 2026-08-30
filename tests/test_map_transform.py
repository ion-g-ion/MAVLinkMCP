"""Offline unit tests for map-view georeferencing (no drone, no network).

This transform is the safety argument for letting a vision model contribute
coordinates: the model points in pixels and the server converts. If it drifts,
a plan is generated somewhere other than where the model was looking.
"""
import unittest

from mavlinkmcp.geo_helpers import distance_m
from mavlinkmcp.map_transform import (
    ORIENTATIONS,
    MapView,
    build_view,
    validate_orientation,
    validate_pixels,
    view_status_err,
)

CENTER = (47.3977, 8.5456)


def view(orientation="north_up", heading=0.0, size=768, zoom=18):
    return build_view(
        view_id="v0123456789ab",
        center_lat=CENTER[0],
        center_lon=CENTER[1],
        zoom=zoom,
        width_px=size,
        height_px=size,
        orientation=orientation,
        heading_deg=heading,
    )


class TestRoundTrip(unittest.TestCase):
    PIXELS = [(0, 0), (768, 0), (768, 768), (0, 768), (384, 384), (120, 655), (7, 401)]

    def test_north_up_round_trips_exactly(self):
        v = view()
        for px, py in self.PIXELS:
            lon, lat = v.pixel_to_lonlat(px, py)
            bx, by = v.lonlat_to_pixel(lon, lat)
            self.assertAlmostEqual(bx, px, places=6)
            self.assertAlmostEqual(by, py, places=6)

    def test_heading_up_round_trips_exactly(self):
        for heading in (0.0, 45.0, 72.4, 180.0, 271.9, 359.5):
            v = view("heading_up", heading)
            for px, py in self.PIXELS:
                lon, lat = v.pixel_to_lonlat(px, py)
                bx, by = v.lonlat_to_pixel(lon, lat)
                self.assertAlmostEqual(bx, px, places=6)
                self.assertAlmostEqual(by, py, places=6)

    def test_centre_pixel_is_the_centre_coordinate(self):
        for v in (view(), view("heading_up", 72.4)):
            lon, lat = v.pixel_to_lonlat(v.width_px / 2, v.height_px / 2)
            self.assertAlmostEqual(lat, v.center_lat, places=9)
            self.assertAlmostEqual(lon, v.center_lon, places=9)


class TestOrientation(unittest.TestCase):
    def test_north_up_puts_north_at_the_top(self):
        v = view()
        top_lat, _ = v.pixels_to_latlon([(384, 0)])[0]
        bottom_lat, _ = v.pixels_to_latlon([(384, 768)])[0]
        self.assertGreater(top_lat, v.center_lat)
        self.assertLess(bottom_lat, v.center_lat)

    def test_north_up_puts_east_on_the_right(self):
        v = view()
        _, right_lon = v.pixels_to_latlon([(768, 384)])[0]
        _, left_lon = v.pixels_to_latlon([(0, 384)])[0]
        self.assertGreater(right_lon, v.center_lon)
        self.assertLess(left_lon, v.center_lon)

    def test_heading_up_puts_the_heading_at_the_top(self):
        # Looking east: "up" in the image must be east of centre.
        v = view("heading_up", 90.0)
        top_lat, top_lon = v.pixels_to_latlon([(384, 0)])[0]
        self.assertGreater(top_lon, v.center_lon)
        self.assertAlmostEqual(top_lat, v.center_lat, places=4)

    def test_heading_up_at_zero_matches_north_up(self):
        a = view("heading_up", 0.0)
        b = view()
        self.assertEqual(a.rotation_deg, b.rotation_deg)
        self.assertEqual(a.pixel_to_lonlat(100, 200), b.pixel_to_lonlat(100, 200))

    def test_north_up_ignores_a_supplied_heading(self):
        self.assertEqual(view("north_up", 137.0).rotation_deg, 0.0)

    def test_validate_orientation(self):
        self.assertEqual(validate_orientation("HEADING_UP"), "heading_up")
        self.assertEqual(validate_orientation(None), "north_up")
        for bad in ("sideways", "up", 3):
            with self.assertRaises(ValueError):
                validate_orientation(bad)
        self.assertEqual(ORIENTATIONS, ("north_up", "heading_up"))


class TestScaleAndBounds(unittest.TestCase):
    def test_meters_per_pixel_matches_measured_ground_distance(self):
        v = view()
        a = v.pixels_to_latlon([(100, 384)])[0]
        b = v.pixels_to_latlon([(500, 384)])[0]
        self.assertAlmostEqual(
            distance_m(a, b) / 400.0, v.meters_per_pixel, delta=0.01
        )

    def test_bbox_contains_every_corner(self):
        for v in (view(), view("heading_up", 33.0)):
            box = v.bbox()
            for lat, lon in v.corners_latlon():
                self.assertGreaterEqual(lat, box["min_lat"] - 1e-9)
                self.assertLessEqual(lat, box["max_lat"] + 1e-9)
                self.assertGreaterEqual(lon, box["min_lon"] - 1e-9)
                self.assertLessEqual(lon, box["max_lon"] + 1e-9)

    def test_rotated_bbox_is_larger_than_the_aligned_one(self):
        # A rotated square's corners reach outside its axis-aligned footprint;
        # the tile window has to account for that or the view renders short.
        straight = view().bbox()
        turned = view("heading_up", 45.0).bbox()
        self.assertGreater(
            turned["max_lat"] - turned["min_lat"],
            straight["max_lat"] - straight["min_lat"],
        )

    def test_corners_are_clockwise_from_top_left(self):
        corners = view().corners_latlon()
        self.assertEqual(len(corners), 4)
        self.assertGreater(corners[0][0], corners[3][0])   # top above bottom
        self.assertGreater(corners[1][1], corners[0][1])   # right east of left


class TestSerialization(unittest.TestCase):
    def test_round_trips_through_a_dict(self):
        v = view("heading_up", 72.4)
        restored = MapView.from_dict(v.to_dict())
        self.assertEqual(restored.to_dict(), v.to_dict())
        self.assertEqual(restored.pixel_to_lonlat(10, 20), v.pixel_to_lonlat(10, 20))

    def test_unknown_keys_are_ignored(self):
        data = view().to_dict()
        data["something_new"] = 1
        self.assertEqual(MapView.from_dict(data).view_id, "v0123456789ab")


class TestPixelValidation(unittest.TestCase):
    def test_accepts_pairs_and_dicts(self):
        got = validate_pixels([[1, 2], (3.5, 4.5), {"x": 5, "y": 6}], 768, 768)
        self.assertEqual(got, [(1.0, 2.0), (3.5, 4.5), (5.0, 6.0)])

    def test_edges_are_inside(self):
        self.assertEqual(
            validate_pixels([[0, 0], [768, 768]], 768, 768), [(0.0, 0.0), (768.0, 768.0)]
        )

    def test_out_of_bounds_is_rejected_not_clamped(self):
        # Clamping would produce a plausible-looking polygon in the wrong place.
        for bad in ([[769, 10]], [[-1, 10]], [[10, 900]]):
            with self.assertRaises(ValueError) as ctx:
                validate_pixels(bad, 768, 768)
            self.assertIn("outside the image", str(ctx.exception))

    def test_rejects_malformed_and_non_finite(self):
        for bad in ("nope", [[1]], [[1, 2, 3]], [[float("nan"), 1]], [[True, 1]]):
            with self.assertRaises(ValueError):
                validate_pixels(bad, 768, 768)

    def test_enforces_a_minimum_count(self):
        with self.assertRaises(ValueError):
            validate_pixels([[1, 2], [3, 4]], 768, 768, min_count=3)

    def test_status_err_is_fail_closed(self):
        self.assertEqual(view_status_err("x"), {"status": "failed", "error": "x"})


if __name__ == "__main__":
    unittest.main()
