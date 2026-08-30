"""Offline unit tests for the flight-plan document (no drone required)."""
import json
import math
import unittest

from mavlinkmcp.plan_helpers import (
    CAMERA_ACTIONS,
    STATUS_DRAFT,
    STATUS_UPLOADED,
    STATUS_VALIDATED,
    build_plan,
    can_upload,
    normalize_camera_action,
    plan_status_err,
    plan_stats,
    plan_to_geojson,
    sanitize_waypoints,
    slugify_plan_id,
    validate_mission_points,
    validate_plan_id,
    waypoint_coords,
    waypoints_equal,
)


def wp(lat, lon, alt=40.0, speed=5.0, **extra):
    return {
        "latitude_deg": lat,
        "longitude_deg": lon,
        "relative_altitude_m": alt,
        "speed_m_s": speed,
        "is_fly_through": True,
        **extra,
    }


POINTS = [wp(47.3977, 8.5456), wp(47.3987, 8.5456), wp(47.3987, 8.5476)]


class TestSlugs(unittest.TestCase):
    def test_slugifies_human_names(self):
        self.assertEqual(slugify_plan_id("North Field Survey"), "north-field-survey")
        self.assertEqual(slugify_plan_id("  Field #3 (west) "), "field-3-west")
        self.assertEqual(slugify_plan_id("A---B"), "a-b")

    def test_truncates_to_the_id_limit(self):
        self.assertLessEqual(len(slugify_plan_id("x" * 200)), 64)

    def test_refuses_names_that_yield_nothing_usable(self):
        for bad in ("", "   ", "...", "///", "..", None, "###"):
            with self.assertRaises(ValueError):
                slugify_plan_id(bad)

    def test_validate_plan_id_blocks_traversal(self):
        # These become path segments, and they arrive from a model.
        for bad in ("../../etc/passwd", "a/b", "..", "", "A", "-x", "x" * 65, "a b"):
            with self.assertRaises(ValueError):
                validate_plan_id(bad)
        self.assertEqual(validate_plan_id("north-field-2"), "north-field-2")


class TestWaypointSanitizing(unittest.TestCase):
    def test_keeps_the_required_fields(self):
        got = sanitize_waypoints(POINTS)
        self.assertEqual(len(got), 3)
        self.assertEqual(got[0]["latitude_deg"], 47.3977)
        self.assertIs(got[0]["is_fly_through"], True)

    def test_drops_nan_optionals_so_the_document_stays_json(self):
        # MAVSDK uses NaN for "unset"; json.dumps(allow_nan=False) would refuse it.
        got = sanitize_waypoints([wp(47.0, 8.0, gimbal_pitch_deg=float("nan"),
                                     yaw_deg=12.0)])
        self.assertNotIn("gimbal_pitch_deg", got[0])
        self.assertEqual(got[0]["yaw_deg"], 12.0)
        json.dumps(got, allow_nan=False)

    def test_drops_unknown_keys(self):
        got = sanitize_waypoints([wp(47.0, 8.0, wild_field="x")])
        self.assertNotIn("wild_field", got[0])

    def test_normalizes_camera_actions(self):
        got = sanitize_waypoints(
            [wp(47.0, 8.0, camera_action="start_photo_distance",
                camera_photo_distance_m=10.0)]
        )
        self.assertEqual(got[0]["camera_action"], "START_PHOTO_DISTANCE")

    def test_rejects_unknown_camera_actions(self):
        with self.assertRaises(ValueError):
            sanitize_waypoints([wp(47.0, 8.0, camera_action="PANORAMA")])
        for name in CAMERA_ACTIONS:
            self.assertEqual(normalize_camera_action(name.lower()), name)

    def test_inherits_the_mission_point_bounds(self):
        for bad in (wp(91.0, 8.0), wp(47.0, 181.0), wp(47.0, 8.0, alt=900.0),
                    wp(47.0, 8.0, speed=0.0), wp(47.0, 8.0, speed=99.0)):
            with self.assertRaises(ValueError):
                sanitize_waypoints([bad])

    def test_validate_mission_points_still_lives_here(self):
        # server.py imports it from this module; the old tests reach it via
        # the server module, so both paths must keep working.
        self.assertEqual(len(validate_mission_points(POINTS)), 3)


class TestStats(unittest.TestCase):
    def test_counts_distance_and_extent(self):
        stats = plan_stats(sanitize_waypoints(POINTS))
        self.assertEqual(stats["waypoint_count"], 3)
        self.assertGreater(stats["path_length_m"], 0.0)
        self.assertGreater(stats["max_leg_m"], 0.0)
        self.assertEqual(stats["bbox"]["min_lat"], 47.3977)

    def test_single_waypoint_has_no_legs(self):
        stats = plan_stats(sanitize_waypoints([wp(47.0, 8.0)]))
        self.assertEqual(stats["path_length_m"], 0.0)
        self.assertEqual(stats["max_leg_m"], 0.0)

    def test_area_is_included_when_known(self):
        self.assertEqual(
            plan_stats(sanitize_waypoints(POINTS), area_m2=1234.5)["area_m2"], 1234.5
        )

    def test_waypoint_coords_preserve_order(self):
        self.assertEqual(waypoint_coords(POINTS)[0], (47.3977, 8.5456))


class TestBuildAndStatus(unittest.TestCase):
    def test_new_plans_start_as_drafts(self):
        plan = build_plan("north-field", "North Field", POINTS)
        self.assertEqual(plan["status"], STATUS_DRAFT)
        self.assertEqual(plan["revision"], 1)
        self.assertEqual(plan["stats"]["waypoint_count"], 3)
        self.assertIsNone(plan["checks"])

    def test_generator_block_records_provenance(self):
        plan = build_plan(
            "p", "P", POINTS, pattern="lawnmower",
            params={"altitude_m": 40}, derived={"line_count": 4},
            source={"view_id": "v0123456789ab"},
        )
        self.assertEqual(plan["generator"]["pattern"], "lawnmower")
        self.assertEqual(plan["generator"]["source"]["view_id"], "v0123456789ab")
        self.assertEqual(plan["generator"]["derived"]["line_count"], 4)

    def test_inset_area_wins_over_gross_area(self):
        plan = build_plan("p", "P", POINTS,
                          derived={"area_m2": 1000.0, "inset_area_m2": 600.0})
        self.assertEqual(plan["stats"]["area_m2"], 600.0)

    def test_a_draft_cannot_be_uploaded(self):
        plan = build_plan("p", "P", POINTS)
        reason = can_upload(plan)
        self.assertIsNotNone(reason)
        self.assertIn("validate_plan", reason)

    def test_a_validated_plan_can_be_uploaded(self):
        plan = build_plan("p", "P", POINTS)
        plan["status"] = STATUS_VALIDATED
        self.assertIsNone(can_upload(plan))

    def test_an_uploaded_plan_can_be_re_uploaded(self):
        plan = build_plan("p", "P", POINTS)
        plan["status"] = STATUS_UPLOADED
        self.assertIsNone(can_upload(plan))

    def test_an_empty_plan_cannot_be_uploaded(self):
        plan = build_plan("p", "P", POINTS)
        plan["status"] = STATUS_VALIDATED
        plan["waypoints"] = []
        self.assertIn("no waypoints", can_upload(plan))

    def test_unknown_status_is_refused(self):
        plan = build_plan("p", "P", POINTS)
        plan["status"] = "flying"
        self.assertIsNotNone(can_upload(plan))


class TestComparison(unittest.TestCase):
    def test_identical_lists_match(self):
        self.assertEqual(waypoints_equal(POINTS, POINTS), [])

    def test_count_mismatch_is_reported_first(self):
        diffs = waypoints_equal(POINTS, POINTS[:2])
        self.assertEqual(len(diffs), 1)
        self.assertIn("count differs", diffs[0])

    def test_float32_rounding_is_tolerated(self):
        # A mission round-trips through float32 on the wire; that must not read
        # as a different route.
        nudged = [dict(p, latitude_deg=p["latitude_deg"] + 1e-8) for p in POINTS]
        self.assertEqual(waypoints_equal(POINTS, nudged), [])

    def test_a_real_difference_is_caught(self):
        moved = [dict(p) for p in POINTS]
        moved[1]["latitude_deg"] += 0.001
        diffs = waypoints_equal(POINTS, moved)
        self.assertTrue(any("waypoint[1].latitude_deg" in d for d in diffs))

    def test_altitude_differences_are_caught(self):
        raised = [dict(p) for p in POINTS]
        raised[0]["relative_altitude_m"] = 60.0
        self.assertTrue(
            any("relative_altitude_m" in d for d in waypoints_equal(POINTS, raised))
        )

    def test_missing_field_is_a_difference(self):
        stripped = [{k: v for k, v in p.items() if k != "latitude_deg"} for p in POINTS]
        self.assertTrue(waypoints_equal(POINTS, stripped))


class TestGeoJSON(unittest.TestCase):
    def setUp(self):
        self.plan = build_plan(
            "p", "P", POINTS, pattern="lawnmower",
            params={"polygon": [[47.39, 8.54], [47.40, 8.54], [47.40, 8.55]]},
        )

    def test_emits_area_path_and_waypoints(self):
        gj = plan_to_geojson(self.plan)
        roles = [f["properties"]["role"] for f in gj["features"]]
        self.assertEqual(gj["type"], "FeatureCollection")
        self.assertEqual(roles.count("area_of_interest"), 1)
        self.assertEqual(roles.count("flight_path"), 1)
        self.assertEqual(roles.count("waypoint"), 3)

    def test_coordinates_are_lon_lat_per_the_spec(self):
        gj = plan_to_geojson(self.plan)
        path = next(f for f in gj["features"] if f["properties"]["role"] == "flight_path")
        self.assertEqual(path["geometry"]["coordinates"][0], [8.5456, 47.3977])

    def test_area_ring_is_closed(self):
        gj = plan_to_geojson(self.plan)
        ring = next(
            f for f in gj["features"] if f["properties"]["role"] == "area_of_interest"
        )["geometry"]["coordinates"][0]
        self.assertEqual(ring[0], ring[-1])

    def test_plan_without_a_polygon_still_renders(self):
        gj = plan_to_geojson(build_plan("p", "P", POINTS))
        roles = [f["properties"]["role"] for f in gj["features"]]
        self.assertNotIn("area_of_interest", roles)
        self.assertIn("flight_path", roles)

    def test_it_is_json_serializable(self):
        json.dumps(plan_to_geojson(self.plan), allow_nan=False)


class TestFailClosed(unittest.TestCase):
    def test_status_err(self):
        self.assertEqual(plan_status_err("x"), {"status": "failed", "error": "x"})


if __name__ == "__main__":
    unittest.main()
