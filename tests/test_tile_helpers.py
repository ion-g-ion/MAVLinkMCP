"""Offline unit tests for slippy-map tile math (no drone, no network)."""
import math
import unittest

from mavlinkmcp.tile_helpers import (
    EQUATOR_MPP_Z0,
    MAX_MERCATOR_LAT,
    clamp_mercator_lat,
    lonlat_to_tile,
    meters_per_pixel,
    tile_count,
    tile_range_for_bbox,
    tile_status_err,
    tile_to_lonlat,
    tiles_in_range,
    validate_zoom,
    zoom_for_meters_per_pixel,
    zoom_for_radius,
)


class TestKnownTileAnswers(unittest.TestCase):
    def test_null_island_at_zoom_one(self):
        # At z=1 the world is 2x2 tiles and (0,0) sits on the centre corner.
        self.assertEqual(lonlat_to_tile(0.0, 0.0, 1), (1.0, 1.0))

    def test_whole_world_is_one_tile_at_zoom_zero(self):
        x, y = lonlat_to_tile(0.0, 0.0, 0)
        self.assertEqual((x, y), (0.5, 0.5))
        self.assertEqual(tile_range_for_bbox(-180, -85, 180, 85, 0), (0, 0, 0, 0))

    def test_corners_of_the_world(self):
        self.assertEqual(lonlat_to_tile(-180.0, MAX_MERCATOR_LAT, 2)[0], 0.0)
        self.assertAlmostEqual(lonlat_to_tile(-180.0, MAX_MERCATOR_LAT, 2)[1], 0.0, places=6)
        self.assertEqual(lonlat_to_tile(180.0, 0.0, 2)[0], 4.0)

    def test_tile_to_lonlat_inverts_lonlat_to_tile(self):
        for lon, lat, z in ((8.5456, 47.3977, 18), (-122.4, 37.8, 14), (151.2, -33.9, 12)):
            x, y = lonlat_to_tile(lon, lat, z)
            back_lon, back_lat = tile_to_lonlat(x, y, z)
            self.assertAlmostEqual(back_lon, lon, places=9)
            self.assertAlmostEqual(back_lat, lat, places=9)


class TestResolution(unittest.TestCase):
    def test_equator_zoom_zero_is_the_reference_constant(self):
        self.assertAlmostEqual(meters_per_pixel(0.0, 0), EQUATOR_MPP_Z0, places=6)

    def test_resolution_halves_each_zoom(self):
        for z in range(0, 20):
            self.assertAlmostEqual(
                meters_per_pixel(0.0, z) / 2.0, meters_per_pixel(0.0, z + 1), places=9
            )

    def test_resolution_shrinks_with_latitude(self):
        self.assertLess(meters_per_pixel(60.0, 15), meters_per_pixel(0.0, 15))

    def test_zoom_choice_is_never_coarser_than_asked(self):
        for lat in (0.0, 47.4, -33.9, 70.0):
            for target in (0.05, 0.4, 1.0, 5.0, 40.0):
                z = zoom_for_meters_per_pixel(lat, target)
                self.assertLessEqual(meters_per_pixel(lat, z), target + 1e-9)
                if z > 0:
                    # ...and not needlessly finer than asked, either.
                    self.assertGreater(meters_per_pixel(lat, z - 1), target)

    def test_zoom_for_radius_fits_the_ground_area(self):
        z = zoom_for_radius(47.4, 300.0, 768)
        self.assertLessEqual(meters_per_pixel(47.4, z) * 768, 600.0 + 1e-6)

    def test_rejects_nonsense_targets(self):
        for bad in (0.0, -1.0, float("nan")):
            with self.assertRaises(ValueError):
                zoom_for_meters_per_pixel(0.0, bad)
        with self.assertRaises(ValueError):
            zoom_for_radius(0.0, -5.0, 768)
        with self.assertRaises(ValueError):
            zoom_for_radius(0.0, 100.0, 0)


class TestRanges(unittest.TestCase):
    def test_range_covers_the_bbox_inclusively(self):
        x_min, y_min, x_max, y_max = tile_range_for_bbox(8.54, 47.39, 8.55, 47.40, 16)
        self.assertLessEqual(x_min, x_max)
        self.assertLessEqual(y_min, y_max)
        self.assertEqual(
            tile_count(x_min, y_min, x_max, y_max),
            len(tiles_in_range(x_min, y_min, x_max, y_max)),
        )

    def test_range_is_clamped_to_the_world(self):
        x_min, y_min, x_max, y_max = tile_range_for_bbox(-180, -90, 180, 90, 3)
        self.assertEqual((x_min, y_min), (0, 0))
        self.assertEqual((x_max, y_max), (7, 7))

    def test_tile_count_of_an_empty_range_is_zero(self):
        self.assertEqual(tile_count(5, 5, 4, 4), 0)


class TestValidation(unittest.TestCase):
    def test_zoom_bounds(self):
        self.assertEqual(validate_zoom("12"), 12)
        for bad in (-1, 99, True, "abc", None):
            with self.assertRaises(ValueError):
                validate_zoom(bad)

    def test_latitude_clamped_to_mercator_limits(self):
        self.assertEqual(clamp_mercator_lat(90.0), MAX_MERCATOR_LAT)
        self.assertEqual(clamp_mercator_lat(-90.0), -MAX_MERCATOR_LAT)
        self.assertEqual(clamp_mercator_lat(10.0), 10.0)

    def test_status_err_is_fail_closed(self):
        self.assertEqual(tile_status_err("x"), {"status": "failed", "error": "x"})


if __name__ == "__main__":
    unittest.main()
