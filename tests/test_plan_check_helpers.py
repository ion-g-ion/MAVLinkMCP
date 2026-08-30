"""Offline unit tests for plan estimation and safety checks (no drone required)."""
import unittest

from mavlinkmcp.geo_helpers import destination
from mavlinkmcp.plan_check_helpers import (
    DEFAULT_LIMITS,
    SEVERITY_ERROR,
    SEVERITY_WARNING,
    check_plan,
    check_status_err,
    estimate_plan,
    finding,
    findings_passed,
    summarize_checks,
)
from mavlinkmcp.plan_helpers import build_plan

HOME = (47.3977, 8.5456)


def wp(lat, lon, alt=40.0, speed=5.0):
    return {
        "latitude_deg": lat,
        "longitude_deg": lon,
        "relative_altitude_m": alt,
        "speed_m_s": speed,
        "is_fly_through": True,
    }


def line_plan(length_m=500.0, speed=5.0, rtl=True, **kw):
    end = destination(HOME, 90.0, length_m)
    plan = build_plan("p", "P", [wp(*HOME, speed=speed), wp(*end, speed=speed)],
                      return_to_launch=rtl, **kw)
    return plan


def codes(findings):
    return [f["code"] for f in findings]


class TestEstimate(unittest.TestCase):
    def test_distance_and_duration_from_speed(self):
        est = estimate_plan(line_plan(500.0, speed=5.0))
        self.assertAlmostEqual(est["distance_m"], 500.0, delta=1.0)
        # 500 m at 5 m/s is 100 s, plus a 40 m climb at 2.5 m/s.
        self.assertAlmostEqual(est["duration_s"], 100.0 + 16.0, delta=1.0)

    def test_faster_speed_means_shorter_duration(self):
        slow = estimate_plan(line_plan(500.0, speed=2.0))["duration_s"]
        fast = estimate_plan(line_plan(500.0, speed=10.0))["duration_s"]
        self.assertGreater(slow, fast)

    def test_home_adds_transit_and_a_return_leg(self):
        plan = line_plan(500.0)
        away = destination(HOME, 270.0, 300.0)
        without = estimate_plan(plan)
        with_home = estimate_plan(plan, home=away)
        self.assertEqual(without["transit_m"], 0.0)
        # 300 m out to the start, then 800 m back from the far end.
        self.assertAlmostEqual(with_home["transit_m"], 1100.0, delta=2.0)
        self.assertGreater(with_home["duration_s"], without["duration_s"])

    def test_no_rtl_only_charges_the_outbound_transit(self):
        away = destination(HOME, 270.0, 300.0)
        est = estimate_plan(line_plan(500.0, rtl=False), home=away)
        self.assertAlmostEqual(est["transit_m"], 300.0, delta=2.0)

    def test_turns_cost_time(self):
        square = build_plan("p", "P", [
            wp(*HOME), wp(*destination(HOME, 90.0, 100.0)),
            wp(*destination(destination(HOME, 90.0, 100.0), 0.0, 100.0)),
            wp(*destination(HOME, 0.0, 100.0)),
        ])
        est = estimate_plan(square)
        self.assertEqual(est["turn_count"], 2)

    def test_straight_line_has_no_turns(self):
        self.assertEqual(estimate_plan(line_plan())["turn_count"], 0)

    def test_photo_count_uses_the_trigger_distance(self):
        plan = line_plan(500.0)
        plan["generator"]["derived"]["trigger_distance_m"] = 10.0
        self.assertEqual(estimate_plan(plan)["photo_count"], 51)

    def test_no_camera_means_no_photo_count(self):
        self.assertEqual(estimate_plan(line_plan())["photo_count"], 0)

    def test_battery_is_null_without_both_parameters(self):
        # Inventing a battery figure would be worse than admitting ignorance.
        for kwargs in ({}, {"battery_capacity_mah": 5200}, {"cruise_current_a": 18.0}):
            est = estimate_plan(line_plan(), **kwargs)
            self.assertIsNone(est["battery_pct"])
            self.assertIn("not supplied", est["battery_basis"])

    def test_battery_percentage_from_capacity_and_current(self):
        # 5200 mAh at 18 A is 17.33 min of endurance.
        est = estimate_plan(line_plan(3000.0, speed=5.0),
                            battery_capacity_mah=5200, cruise_current_a=18.0)
        self.assertIsNotNone(est["battery_pct"])
        self.assertAlmostEqual(est["battery_pct"], 616.0 / 1040.0 * 100.0, delta=3.0)

    def test_empty_plan_estimates_to_nothing(self):
        plan = build_plan("p", "P", [wp(*HOME)])
        plan["waypoints"] = []
        est = estimate_plan(plan)
        self.assertEqual(est["distance_m"], 0.0)
        self.assertEqual(est["duration_s"], 0.0)


class TestChecks(unittest.TestCase):
    def test_a_sane_plan_produces_nothing(self):
        self.assertEqual(check_plan(line_plan(), home=HOME), [])

    def test_empty_plan_is_an_error(self):
        plan = line_plan()
        plan["waypoints"] = []
        found = check_plan(plan)
        self.assertEqual(codes(found), ["EMPTY_PLAN"])
        self.assertFalse(findings_passed(found))

    def test_far_from_home_is_an_error(self):
        # The check that catches a polygon drawn on the wrong part of the world.
        far = destination(HOME, 90.0, 400000.0)
        found = check_plan(line_plan(), home=far)
        self.assertIn("FAR_FROM_HOME", codes(found))
        entry = next(f for f in found if f["code"] == "FAR_FROM_HOME")
        self.assertEqual(entry["severity"], SEVERITY_ERROR)
        self.assertFalse(findings_passed(found))

    def test_moderately_far_from_home_is_only_a_warning(self):
        found = check_plan(line_plan(), home=destination(HOME, 90.0, 5000.0))
        entry = next(f for f in found if f["code"] == "FAR_FROM_HOME")
        self.assertEqual(entry["severity"], SEVERITY_WARNING)
        self.assertTrue(findings_passed(found))

    def test_no_home_means_no_home_check(self):
        self.assertNotIn("FAR_FROM_HOME", codes(check_plan(line_plan())))

    def test_long_leg_is_an_error(self):
        found = check_plan(line_plan(5000.0), home=HOME)
        self.assertIn("LEG_TOO_LONG", codes(found))
        self.assertFalse(findings_passed(found))

    def test_duplicate_waypoint_is_a_warning(self):
        plan = build_plan("p", "P", [wp(*HOME), wp(*HOME)])
        found = check_plan(plan)
        self.assertIn("DUPLICATE_WAYPOINT", codes(found))
        self.assertTrue(findings_passed(found))

    def test_altitude_and_speed_bounds(self):
        plan = build_plan("p", "P", [wp(*HOME, alt=40.0), wp(*destination(HOME, 90, 100))])
        plan["waypoints"][0]["relative_altitude_m"] = 900.0
        plan["waypoints"][1]["speed_m_s"] = 0.0
        found = check_plan(plan)
        self.assertIn("ALTITUDE_OUT_OF_RANGE", codes(found))
        self.assertIn("SPEED_OUT_OF_RANGE", codes(found))
        self.assertEqual(found[0]["waypoint_index"], 0)

    def test_waypoint_count_warns_then_errors(self):
        plan = line_plan()
        many = [wp(*destination(HOME, 90.0, i * 5.0)) for i in range(600)]
        plan["waypoints"] = many
        self.assertIn("WAYPOINT_COUNT_EXCEEDS_LIMIT", codes(check_plan(plan)))
        self.assertTrue(findings_passed(check_plan(plan)))

        plan["waypoints"] = [wp(*destination(HOME, 90.0, i * 5.0)) for i in range(2100)]
        found = check_plan(plan)
        self.assertFalse(findings_passed(found))

    def test_endurance_findings_track_the_estimate(self):
        plan = line_plan()
        warn = check_plan(plan, estimate={"battery_pct": 85.0})
        self.assertIn("EXCEEDS_ENDURANCE", codes(warn))
        self.assertTrue(findings_passed(warn))

        over = check_plan(plan, estimate={"battery_pct": 120.0})
        self.assertIn("EXCEEDS_ENDURANCE", codes(over))
        self.assertFalse(findings_passed(over))

    def test_unknown_battery_raises_no_endurance_finding(self):
        self.assertNotIn(
            "EXCEEDS_ENDURANCE", codes(check_plan(line_plan(), estimate={"battery_pct": None}))
        )

    def test_limits_are_overridable(self):
        self.assertEqual(check_plan(line_plan(5000.0)), 
                         check_plan(line_plan(5000.0)))
        relaxed = check_plan(line_plan(5000.0), limits={"max_leg_m": 10000.0})
        self.assertNotIn("LEG_TOO_LONG", codes(relaxed))

    def test_default_limits_are_present(self):
        for key in ("max_leg_m", "max_altitude_m", "max_home_distance_m"):
            self.assertIn(key, DEFAULT_LIMITS)


class TestSummary(unittest.TestCase):
    def test_counts_and_verdict(self):
        found = [
            finding(SEVERITY_ERROR, "A", "a"),
            finding(SEVERITY_WARNING, "B", "b"),
            finding(SEVERITY_WARNING, "C", "c"),
        ]
        summary = summarize_checks(found)
        self.assertFalse(summary["passed"])
        self.assertEqual(summary["error_count"], 1)
        self.assertEqual(summary["warning_count"], 2)
        self.assertTrue(summary["checked_at"])
        self.assertEqual(len(summary["findings"]), 3)

    def test_warnings_alone_still_pass(self):
        self.assertTrue(summarize_checks([finding(SEVERITY_WARNING, "B", "b")])["passed"])

    def test_no_findings_passes(self):
        self.assertTrue(summarize_checks([])["passed"])

    def test_status_err_is_fail_closed(self):
        self.assertEqual(check_status_err("x"), {"status": "failed", "error": "x"})


if __name__ == "__main__":
    unittest.main()
