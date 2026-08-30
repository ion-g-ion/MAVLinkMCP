"""Offline unit tests for WGS84 geometry and local projection (no drone required)."""
import math
import unittest

from mavlinkmcp.geo_helpers import (
    bbox_of,
    bearing_deg,
    destination,
    distance_m,
    geo_status_err,
    haversine_m,
    local_crs,
    path_length_m,
    ring_centroid,
    to_local,
    to_wgs84,
    validate_lat_lon,
    validate_polygon,
)

ZURICH = (47.3977, 8.5456)


class TestDistanceAndBearing(unittest.TestCase):
    def test_known_distance(self):
        # One degree of latitude at the equator is 110.574 km on WGS84. The
        # spherical figure (111.19 km) is wrong here: the ellipsoid is flattest
        # at the equator, and the generators work on the ellipsoid.
        self.assertAlmostEqual(distance_m((0.0, 0.0), (1.0, 0.0)), 110574.4, delta=1.0)
        # One degree of longitude at the equator is a touch longer.
        self.assertAlmostEqual(distance_m((0.0, 0.0), (0.0, 1.0)), 111319.5, delta=1.0)

    def test_haversine_m_is_the_same_function(self):
        self.assertIs(haversine_m, distance_m)

    def test_zero_distance(self):
        self.assertEqual(haversine_m(ZURICH, ZURICH), 0.0)

    def test_bearings_are_compass_referenced(self):
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (1.0, 0.0)), 0.0, places=6)
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (0.0, 1.0)), 90.0, places=6)
        self.assertAlmostEqual(bearing_deg((1.0, 0.0), (0.0, 0.0)), 180.0, places=6)
        self.assertAlmostEqual(bearing_deg((0.0, 1.0), (0.0, 0.0)), 270.0, places=6)

    def test_destination_round_trips(self):
        for brg in (0.0, 45.0, 137.5, 270.0, 359.0):
            end = destination(ZURICH, brg, 500.0)
            self.assertAlmostEqual(haversine_m(ZURICH, end), 500.0, delta=0.01)
            self.assertAlmostEqual(bearing_deg(ZURICH, end), brg, delta=0.01)

    def test_destination_wraps_the_antimeridian(self):
        end = destination((0.0, 179.99), 90.0, 5000.0)
        self.assertTrue(-180.0 <= end[1] <= 180.0)
        self.assertLess(end[1], 0.0)

    def test_path_length_sums_legs(self):
        a = ZURICH
        b = destination(a, 90.0, 100.0)
        c = destination(b, 0.0, 100.0)
        self.assertAlmostEqual(path_length_m([a, b, c]), 200.0, delta=0.02)
        self.assertEqual(path_length_m([a]), 0.0)


class TestProjection(unittest.TestCase):
    def test_local_plane_round_trips_under_a_decimetre(self):
        fwd, inv = local_crs(*ZURICH)
        points = [
            ZURICH,
            destination(ZURICH, 45.0, 5000.0),
            destination(ZURICH, 200.0, 2500.0),
            destination(ZURICH, 310.0, 100.0),
        ]
        back = to_wgs84(to_local(points, fwd), inv)
        for original, restored in zip(points, back):
            self.assertLess(haversine_m(original, restored), 0.1)

    def test_origin_projects_to_zero(self):
        fwd, _ = local_crs(*ZURICH)
        east, north = to_local([ZURICH], fwd)[0]
        self.assertAlmostEqual(east, 0.0, places=6)
        self.assertAlmostEqual(north, 0.0, places=6)

    def test_axes_are_east_and_north_in_metres(self):
        fwd, _ = local_crs(*ZURICH)
        east, north = to_local([destination(ZURICH, 90.0, 250.0)], fwd)[0]
        self.assertAlmostEqual(east, 250.0, delta=0.1)
        self.assertAlmostEqual(north, 0.0, delta=0.1)
        east, north = to_local([destination(ZURICH, 0.0, 250.0)], fwd)[0]
        self.assertAlmostEqual(east, 0.0, delta=0.1)
        self.assertAlmostEqual(north, 250.0, delta=0.1)

    def test_transformers_are_cached_per_origin(self):
        self.assertIs(local_crs(*ZURICH)[0], local_crs(ZURICH[0], ZURICH[1])[0])


class TestValidation(unittest.TestCase):
    def test_accepts_pairs_and_dicts(self):
        ring = validate_polygon(
            [[0.0, 0.0], (0.0, 1.0), {"latitude_deg": 1.0, "longitude_deg": 1.0}]
        )
        self.assertEqual(len(ring), 3)
        self.assertEqual(ring[2], (1.0, 1.0))

    def test_drops_an_explicit_closing_vertex(self):
        ring = validate_polygon([[0, 0], [0, 1], [1, 1], [0, 0]])
        self.assertEqual(len(ring), 3)

    def test_rejects_too_few_vertices(self):
        with self.assertRaises(ValueError):
            validate_polygon([[0, 0], [0, 1]])

    def test_rejects_duplicate_vertices(self):
        with self.assertRaises(ValueError):
            validate_polygon([[0, 0], [0, 1], [0, 1]])

    def test_rejects_out_of_range_and_non_finite(self):
        for bad in ([[91.0, 0], [0, 1], [1, 1]],
                    [[0, 181.0], [0, 1], [1, 1]],
                    [[float("nan"), 0], [0, 1], [1, 1]],
                    [[float("inf"), 0], [0, 1], [1, 1]]):
            with self.assertRaises(ValueError):
                validate_polygon(bad)

    def test_rejects_malformed_entries(self):
        for bad in ("not a polygon", [[0, 0, 0], [0, 1], [1, 1]], [1, 2, 3]):
            with self.assertRaises(ValueError):
                validate_polygon(bad)

    def test_lat_lon_rejects_bool(self):
        with self.assertRaises(ValueError):
            validate_lat_lon(True, 0.0)

    def test_bbox_and_centroid(self):
        ring = [(1.0, 2.0), (3.0, 4.0), (-1.0, 0.0)]
        self.assertEqual(
            bbox_of(ring),
            {"min_lat": -1.0, "min_lon": 0.0, "max_lat": 3.0, "max_lon": 4.0},
        )
        self.assertEqual(ring_centroid(ring), (1.0, 2.0))
        with self.assertRaises(ValueError):
            bbox_of([])

    def test_status_err_is_fail_closed(self):
        self.assertEqual(
            geo_status_err("bad"), {"status": "failed", "error": "bad"}
        )


if __name__ == "__main__":
    unittest.main()
