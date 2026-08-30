"""Offline unit tests for coverage-pattern generation (no drone required).

The assertions are deliberately arithmetic rather than "it returned something":
a survey that silently leaves gaps still looks like a valid waypoint list.
"""
import math
import unittest

from mavlinkmcp.coverage_helpers import (
    MAX_WAYPOINTS,
    PATTERNS,
    build_local_polygon,
    coverage_status_err,
    generate,
    generate_lawnmower,
    long_edge_bearing_deg,
)
from mavlinkmcp.geo_helpers import destination, distance_m, path_length_m

SW = (47.3977, 8.5456)
CAM = {
    "sensor_width_mm": 13.2,
    "focal_length_mm": 8.8,
    "image_width_px": 5472,
    "image_height_px": 3648,
    "front_overlap": 0.75,
    "side_overlap": 0.65,
}


def square(side_m=100.0, origin=SW):
    """An exact side_m x side_m box, built geodesically."""
    se = destination(origin, 90.0, side_m)
    ne = destination(se, 0.0, side_m)
    nw = destination(origin, 0.0, side_m)
    return [list(origin), list(se), list(ne), list(nw)]


def rectangle(width_m, height_m, origin=SW):
    se = destination(origin, 90.0, width_m)
    ne = destination(se, 0.0, height_m)
    nw = destination(origin, 0.0, height_m)
    return [list(origin), list(se), list(ne), list(nw)]


def coords_of(waypoints):
    return [(w["latitude_deg"], w["longitude_deg"]) for w in waypoints]


class TestLawnmowerGeometry(unittest.TestCase):
    def test_square_north_south_sweep_has_known_length(self):
        # A 100 m box at 25 m spacing needs exactly four lines: each covers a
        # 25 m swath, so 4 x 25 m tiles the field with no gap and no waste. A
        # fifth line would be pure overlap, and getting this wrong by one line
        # is a whole extra leg of battery on every survey.
        wps, derived = generate(
            "lawnmower", square(100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
             "sweep_angle_deg": 0.0, "overshoot_m": 0.0},
        )
        self.assertEqual(derived["line_count"], 4)
        self.assertEqual(len(wps), 8)
        # Four 100 m lines joined by three 25 m connectors.
        self.assertAlmostEqual(path_length_m(coords_of(wps)), 475.0, delta=1.0)

    def test_line_count_covers_the_area_without_gaps(self):
        # Each line covers one spacing of width, so n lines must span the field.
        for side, spacing in ((100.0, 25.0), (100.0, 30.0), (250.0, 40.0), (60.0, 25.0)):
            _, derived = generate(
                "lawnmower", square(side),
                {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": spacing,
                 "sweep_angle_deg": 0.0, "overshoot_m": 0.0},
            )
            covered = derived["line_count"] * spacing
            self.assertGreaterEqual(covered, side - 0.5, f"{side}m at {spacing}m leaves a gap")
            # ...and never more than one redundant line.
            self.assertLess(covered - side, spacing)

    def test_each_sweep_line_spans_the_area(self):
        wps, _ = generate(
            "lawnmower", square(100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
             "sweep_angle_deg": 0.0, "overshoot_m": 0.0},
        )
        pts = coords_of(wps)
        for i in range(0, len(pts), 2):
            self.assertAlmostEqual(distance_m(pts[i], pts[i + 1]), 100.0, delta=1.0)

    def test_lines_alternate_direction(self):
        wps, _ = generate(
            "lawnmower", square(100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
             "sweep_angle_deg": 0.0, "overshoot_m": 0.0},
        )
        pts = coords_of(wps)
        # Boustrophedon: consecutive lines run opposite ways, so the sign of the
        # latitude change flips every line. Anything else means the aircraft
        # flies back to the start of each line, doubling the transit.
        signs = [
            math.copysign(1.0, pts[i + 1][0] - pts[i][0]) for i in range(0, len(pts), 2)
        ]
        for a, b in zip(signs, signs[1:]):
            self.assertNotEqual(a, b)

    def test_connectors_are_one_spacing_long(self):
        wps, _ = generate(
            "lawnmower", square(100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
             "sweep_angle_deg": 0.0, "overshoot_m": 0.0},
        )
        pts = coords_of(wps)
        for i in range(1, len(pts) - 1, 2):
            self.assertAlmostEqual(distance_m(pts[i], pts[i + 1]), 25.0, delta=0.5)

    def test_overshoot_extends_the_lines(self):
        params = {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
                  "sweep_angle_deg": 0.0}
        plain, _ = generate("lawnmower", square(100.0), {**params, "overshoot_m": 0.0})
        over, _ = generate("lawnmower", square(100.0), {**params, "overshoot_m": 10.0})
        self.assertEqual(len(plain), len(over))
        first_plain = distance_m(*coords_of(plain)[:2])
        first_over = distance_m(*coords_of(over)[:2])
        self.assertAlmostEqual(first_over - first_plain, 20.0, delta=0.5)

    def test_auto_sweep_angle_follows_the_long_axis(self):
        # A 400 x 100 m field should be swept along its length, minimising turns.
        wide, _ = generate(
            "lawnmower", rectangle(400.0, 100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0},
        )
        across, derived = generate(
            "lawnmower", rectangle(400.0, 100.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
             "sweep_angle_deg": 0.0},
        )
        self.assertLess(len(wide), len(across))
        self.assertAlmostEqual(
            generate("lawnmower", rectangle(400.0, 100.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0})[1][
                "sweep_angle_deg"],
            90.0, delta=1.0,
        )

    def test_sweep_angle_is_recorded_with_its_source(self):
        _, auto = generate("lawnmower", square(100.0),
                           {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0})
        _, explicit = generate("lawnmower", square(100.0),
                               {"altitude_m": 40, "speed_m_s": 5,
                                "line_spacing_m": 25.0, "sweep_angle_deg": 30.0})
        self.assertEqual(auto["sweep_angle_source"], "long_axis")
        self.assertEqual(explicit["sweep_angle_source"], "explicit")
        self.assertAlmostEqual(explicit["sweep_angle_deg"], 30.0)

    def test_area_is_reported_accurately(self):
        _, derived = generate("lawnmower", square(100.0),
                              {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0})
        self.assertAlmostEqual(derived["area_m2"], 10000.0, delta=60.0)


class TestConcaveAndInset(unittest.TestCase):
    def _l_shape(self):
        o = SW
        return [
            list(o),
            list(destination(o, 90.0, 200.0)),
            list(destination(destination(o, 90.0, 200.0), 0.0, 100.0)),
            list(destination(destination(o, 90.0, 100.0), 0.0, 100.0)),
            list(destination(destination(o, 90.0, 100.0), 0.0, 200.0)),
            list(destination(o, 0.0, 200.0)),
        ]

    def test_concave_area_keeps_waypoints_inside(self):
        from shapely.geometry import Point

        ring = self._l_shape()
        wps, _ = generate("lawnmower", ring,
                          {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 20.0,
                           "sweep_angle_deg": 0.0, "overshoot_m": 0.0})
        poly, fwd, _, _ = build_local_polygon(
            [(p[0], p[1]) for p in ring]
        )
        from mavlinkmcp.geo_helpers import to_local

        for east, north in to_local(coords_of(wps), fwd):
            self.assertTrue(
                poly.buffer(0.5).contains(Point(east, north)),
                f"waypoint ({east:.1f}, {north:.1f}) fell outside the area",
            )

    def test_margin_insets_the_area(self):
        plain, _ = generate("lawnmower", square(200.0),
                            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
                             "sweep_angle_deg": 0.0, "overshoot_m": 0.0})
        inset, derived = generate("lawnmower", square(200.0),
                                  {"altitude_m": 40, "speed_m_s": 5,
                                   "line_spacing_m": 25.0, "sweep_angle_deg": 0.0,
                                   "margin_m": 30.0, "overshoot_m": 0.0})
        self.assertLess(path_length_m(coords_of(inset)),
                        path_length_m(coords_of(plain)))
        self.assertLess(derived["inset_area_m2"], derived["area_m2"])

    def test_margin_that_consumes_the_area_is_an_error(self):
        with self.assertRaises(ValueError) as ctx:
            generate("lawnmower", square(50.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 10.0,
                      "margin_m": 40.0})
        self.assertIn("margin", str(ctx.exception))

    def test_self_intersecting_polygon_is_repaired_or_refused(self):
        o = SW
        bowtie = [
            list(o),
            list(destination(destination(o, 90.0, 100.0), 0.0, 100.0)),
            list(destination(o, 90.0, 100.0)),
            list(destination(o, 0.0, 100.0)),
        ]
        try:
            wps, _ = generate("lawnmower", bowtie,
                              {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 20.0})
            self.assertGreater(len(wps), 0)
        except ValueError as e:
            self.assertIn("self-intersecting", str(e))


class TestCameraDrivenSpacing(unittest.TestCase):
    def test_camera_sets_spacing_and_trigger(self):
        wps, derived = generate(
            "lawnmower", rectangle(300.0, 200.0),
            {"altitude_m": 40, "speed_m_s": 5, "camera": CAM},
        )
        self.assertAlmostEqual(derived["line_spacing_m"], 21.0, places=4)
        self.assertAlmostEqual(derived["trigger_distance_m"], 10.0, places=4)
        self.assertAlmostEqual(derived["gsd_cm_px"], 1.0965, places=3)

    def test_camera_actions_bracket_each_line(self):
        wps, derived = generate(
            "lawnmower", rectangle(300.0, 200.0),
            {"altitude_m": 40, "speed_m_s": 5, "camera": CAM},
        )
        starts = [w for w in wps if w.get("camera_action") == "START_PHOTO_DISTANCE"]
        stops = [w for w in wps if w.get("camera_action") == "STOP_PHOTO_DISTANCE"]
        self.assertEqual(len(starts), derived["line_count"])
        self.assertEqual(len(stops), derived["line_count"])
        for wp in starts:
            self.assertAlmostEqual(wp["camera_photo_distance_m"], 10.0, places=4)

    def test_no_camera_means_no_camera_actions(self):
        wps, _ = generate("lawnmower", square(100.0),
                          {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0})
        self.assertTrue(all("camera_action" not in w for w in wps))

    def test_spacing_and_camera_together_are_refused(self):
        with self.assertRaises(ValueError) as ctx:
            generate("lawnmower", square(100.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0,
                      "camera": CAM})
        self.assertIn("not both", str(ctx.exception))

    def test_neither_spacing_nor_camera_is_refused(self):
        with self.assertRaises(ValueError):
            generate("lawnmower", square(100.0), {"altitude_m": 40, "speed_m_s": 5})


class TestGuards(unittest.TestCase):
    def test_waypoints_carry_the_required_mission_fields(self):
        wps, _ = generate("lawnmower", square(100.0),
                          {"altitude_m": 42.5, "speed_m_s": 6.5, "line_spacing_m": 25.0})
        for wp in wps:
            self.assertEqual(wp["relative_altitude_m"], 42.5)
            self.assertEqual(wp["speed_m_s"], 6.5)
            self.assertTrue(wp["is_fly_through"])
            self.assertIn("latitude_deg", wp)
            self.assertIn("longitude_deg", wp)

    def test_absurd_spacing_is_refused_before_generating(self):
        with self.assertRaises(ValueError):
            generate("lawnmower", square(100.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 0.01})
        with self.assertRaises(ValueError):
            generate("lawnmower", square(100.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 99999.0})

    def test_runaway_waypoint_count_is_refused(self):
        with self.assertRaises(ValueError) as ctx:
            generate("lawnmower", square(20000.0),
                     {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 5.0})
        self.assertIn(str(MAX_WAYPOINTS), str(ctx.exception))

    def test_negative_margin_and_overshoot_are_refused(self):
        for bad in ({"margin_m": -1.0}, {"overshoot_m": -1.0}):
            with self.assertRaises(ValueError):
                generate("lawnmower", square(100.0),
                         {"altitude_m": 40, "speed_m_s": 5,
                          "line_spacing_m": 25.0, **bad})

    def test_unknown_pattern_names_the_alternatives(self):
        with self.assertRaises(ValueError) as ctx:
            generate("spiral", square(100.0), {"altitude_m": 40, "speed_m_s": 5})
        self.assertIn("lawnmower", str(ctx.exception))

    def test_registry_holds_the_generator(self):
        self.assertIs(PATTERNS["lawnmower"], generate_lawnmower)

    def test_long_edge_bearing_of_a_wide_box(self):
        poly, _, _, _ = build_local_polygon(
            [(p[0], p[1]) for p in rectangle(400.0, 100.0)]
        )
        self.assertAlmostEqual(long_edge_bearing_deg(poly), 90.0, delta=1.0)

    def test_status_err_is_fail_closed(self):
        self.assertEqual(coverage_status_err("x"), {"status": "failed", "error": "x"})


class TestGroundCoverage(unittest.TestCase):
    """The assertion that actually matters: does the sweep cover the ground?

    A survey with gaps produces a valid-looking waypoint list, flies without
    complaint, and comes back with holes in the map. Waypoint counts and path
    lengths cannot detect that; measuring swept area can.
    """

    def _coverage_fraction(self, ring, params):
        from shapely.geometry import LineString, MultiLineString

        from mavlinkmcp.geo_helpers import to_local

        waypoints, derived = generate("lawnmower", ring, params)
        poly, fwd, _, _ = build_local_polygon([(p[0], p[1]) for p in ring])
        if params.get("margin_m"):
            poly = poly.buffer(-params["margin_m"])
        points = to_local(coords_of(waypoints), fwd)
        lines = [
            LineString([points[i], points[i + 1]]) for i in range(0, len(points) - 1, 2)
        ]
        swept = MultiLineString(lines).buffer(derived["line_spacing_m"] / 2.0)
        return poly.intersection(swept).area / poly.area

    def test_rectangles_are_fully_covered(self):
        for ring, spacing in ((square(200.0), 25.0), (rectangle(400.0, 150.0), 30.0),
                              (square(100.0), 25.0), (rectangle(150.0, 320.0), 18.0)):
            fraction = self._coverage_fraction(
                ring, {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": spacing}
            )
            self.assertGreater(fraction, 0.9999, f"{fraction:.4%} covered")

    def test_irregular_polygon_is_covered_but_for_corner_tips(self):
        # A straight sweep buffered by half a swath cannot reach into an acute
        # corner; every survey planner has this residue.
        ring = [
            list(SW),
            list(destination(SW, 78.0, 260.0)),
            list(destination(destination(SW, 78.0, 260.0), 8.0, 250.0)),
            list(destination(SW, 355.0, 235.0)),
        ]
        fraction = self._coverage_fraction(
            ring, {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 34.0}
        )
        self.assertGreater(fraction, 0.999, f"{fraction:.4%} covered")

    def test_camera_derived_spacing_covers_the_ground(self):
        fraction = self._coverage_fraction(
            rectangle(300.0, 220.0), {"altitude_m": 65, "speed_m_s": 5, "camera": CAM}
        )
        self.assertGreater(fraction, 0.9999, f"{fraction:.4%} covered")

    def test_inset_area_is_covered_after_the_margin(self):
        fraction = self._coverage_fraction(
            square(200.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 25.0, "margin_m": 20.0},
        )
        self.assertGreater(fraction, 0.9999, f"{fraction:.4%} covered")

    def test_a_wider_spacing_than_the_swath_would_leave_gaps(self):
        # Sanity check on the measurement itself: if it cannot detect a gap it
        # cannot vouch for the absence of one.
        from shapely.geometry import LineString, MultiLineString

        from mavlinkmcp.geo_helpers import to_local

        waypoints, derived = generate(
            "lawnmower", square(200.0),
            {"altitude_m": 40, "speed_m_s": 5, "line_spacing_m": 40.0,
             "sweep_angle_deg": 0.0},
        )
        poly, fwd, _, _ = build_local_polygon([(p[0], p[1]) for p in square(200.0)])
        points = to_local(coords_of(waypoints), fwd)
        lines = [
            LineString([points[i], points[i + 1]]) for i in range(0, len(points) - 1, 2)
        ]
        # Pretend the camera only covers 20 m of the 40 m spacing.
        swept = MultiLineString(lines).buffer(10.0)
        self.assertLess(poly.intersection(swept).area / poly.area, 0.6)


if __name__ == "__main__":
    unittest.main()
