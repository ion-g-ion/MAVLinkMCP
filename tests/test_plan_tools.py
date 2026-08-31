"""Offline unit tests for the flight-plan MCP tools (no drone, no network).

Exercises the tool wrappers themselves: the link guard, the upload gate, and the
shape of what goes back to a client.
"""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tests._stubs import FakeMission, FakeTelemetry, ctx, fake_drone, load_server, run


class ToolTestCase(unittest.TestCase):
    def setUp(self):
        self.m = load_server()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self._env = mock.patch.dict(
            os.environ,
            {
                "MAVLINKMCP_PLANS_DIR": self._tmp.name,
                # No test may reach the network; the blank provider renders a
                # correctly georeferenced view with no basemap behind it.
                "MAVLINKMCP_MAP_PROVIDER": "none",
                "MAVLINKMCP_MAP_IMAGE_MODE": "image",
            },
        )
        self._env.start()
        self.addCleanup(self._env.stop)
        self.mission = FakeMission()
        self.drone = fake_drone(mission=self.mission)

    def live(self, telemetry=None):
        drone = fake_drone(telemetry=telemetry, mission=self.mission) if telemetry else self.drone
        return ctx(self.m.MAVLinkConnector(drone=drone, link_state=self.m.LINK_READY))

    def dead(self):
        return ctx(
            self.m.MAVLinkConnector(
                drone=self.drone,
                link_state=self.m.LINK_FAILED,
                link_error="no vehicle on udpin://0.0.0.0:14540",
            )
        )

    def make_plan(self, name="North Field", **kw):
        view = run(self.m.get_map_view(self.live(), radius_m=250, size_px=512))[1]
        params = dict(
            name=name, altitude_m=40.0, speed_m_s=5.0, view_id=view["view_id"],
            polygon_pixels=[[150, 120], [360, 140], [340, 350], [140, 320]],
            line_spacing_m=25.0,
        )
        params.update(kw)
        return run(self.m.create_survey_plan(self.dead(), **params)), view


class TestFailClosed(ToolTestCase):
    def test_every_vehicle_tool_refuses_a_dead_link(self):
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        calls = [
            self.m.upload_plan(self.dead(), pid),
            self.m.verify_uploaded_plan(self.dead(), pid),
            self.m.preflight_check(self.dead(), pid),
            self.m.start_mission(self.dead()),
            self.m.pause_mission(self.dead()),
            self.m.clear_mission(self.dead()),
            self.m.set_current_waypoint(self.dead(), 0),
            self.m.is_mission_finished(self.dead()),
            self.m.download_mission_as_plan(self.dead(), "x"),
            self.m.get_map_view(self.dead()),
        ]
        for result in calls:
            got = run(result)
            self.assertEqual(got["status"], "failed")
            self.assertFalse(got["connected"])
            self.assertIn("no vehicle", got["error"])

    def test_authoring_tools_work_without_a_vehicle(self):
        # Plans are built and costed at a desk and flown later.
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        for result in (
            self.m.list_plans(self.dead()),
            self.m.get_plan(self.dead(), pid),
            self.m.preview_plan_geojson(self.dead(), pid),
            self.m.estimate_plan(self.dead(), pid),
            self.m.validate_plan(self.dead(), pid),
            self.m.render_plan_view(self.dead(), pid, size_px=256),
        ):
            got = run(result)
            payload = got[1] if isinstance(got, list) else got
            self.assertEqual(payload["status"], "success")


class TestUploadGate(ToolTestCase):
    def test_a_draft_never_reaches_the_vehicle(self):
        plan, _ = self.make_plan()
        self.assertEqual(plan["plan_status"], "draft")
        got = run(self.m.upload_plan(self.live(), plan["plan_id"]))
        self.assertEqual(got["status"], "failed")
        self.assertIn("validate_plan", got["error"])
        self.assertIsNone(self.mission.plan)

    def test_lifecycle_status_never_masquerades_as_the_envelope(self):
        # tool_ok/tool_err own "status"; a plan state there would read as a
        # failed call to any client checking the envelope.
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        self.assertEqual(plan["status"], "success")
        self.assertEqual(plan["plan_status"], "draft")

        refused = run(self.m.upload_plan(self.live(), pid))
        self.assertEqual(refused["status"], "failed")
        self.assertEqual(refused["plan_status"], "draft")

        validated = run(self.m.validate_plan(self.live(), pid))
        self.assertEqual(validated["status"], "success")
        self.assertEqual(validated["plan_status"], "validated")

        uploaded = run(self.m.upload_plan(self.live(), pid))
        self.assertEqual(uploaded["status"], "success")
        self.assertEqual(uploaded["plan_status"], "uploaded")

    def test_validated_plan_uploads_and_verifies(self):
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        run(self.m.validate_plan(self.live(), pid))
        got = run(self.m.upload_plan(self.live(), pid))
        self.assertEqual(got["status"], "success")
        self.assertEqual(got["waypoint_count"], plan["stats"]["waypoint_count"])
        self.assertIsNotNone(self.mission.plan)
        self.assertTrue(self.mission.return_to_launch)

        verified = run(self.m.verify_uploaded_plan(self.live(), pid))
        self.assertTrue(verified["verified"])
        self.assertEqual(verified["differences"], [])

    def test_verification_catches_a_mismatched_mission(self):
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        run(self.m.validate_plan(self.live(), pid))
        run(self.m.upload_plan(self.live(), pid))
        # Something else changed what is loaded on the vehicle.
        self.mission.plan.mission_items = self.mission.plan.mission_items[:-2]
        got = run(self.m.verify_uploaded_plan(self.live(), pid))
        self.assertFalse(got["verified"])
        self.assertTrue(got["differences"])

    def test_a_failing_check_demotes_a_validated_plan(self):
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        run(self.m.validate_plan(self.live(), pid))
        # Home on the far side of the planet: the plan is now in the wrong place.
        got = run(self.m.validate_plan(self.live(), pid, home_lat=-33.9, home_lon=151.2))
        self.assertFalse(got["passed"])
        self.assertEqual(got["plan_status"], "draft")
        self.assertIn("FAR_FROM_HOME", [f["code"] for f in got["findings"]])
        self.assertEqual(run(self.m.upload_plan(self.live(), pid))["status"], "failed")


class TestImageResults(ToolTestCase):
    def test_map_view_returns_an_image_then_the_numbers(self):
        got = run(self.m.get_map_view(self.live(), radius_m=200, size_px=512))
        self.assertIsInstance(got, list)
        self.assertEqual(len(got), 2)
        self.assertEqual(type(got[0]).__name__, "Image")
        self.assertEqual(got[1]["status"], "success")

    def test_no_base64_ever_lands_in_the_json_payload(self):
        # Base64 belongs in the ImageContent block the SDK builds. In a text
        # block it costs ~50x the tokens and most clients will not render it.
        image, payload = run(self.m.get_map_view(self.live(), size_px=512))
        blob = json.dumps(payload)
        self.assertNotIn("base64", blob)
        self.assertLess(len(blob), 4000)
        self.assertTrue(str(image.path).endswith(".jpg"))

    def test_payload_carries_what_makes_the_image_usable(self):
        _, payload = run(self.m.get_map_view(self.live(), size_px=512))
        for key in ("view_id", "center", "size_px", "meters_per_pixel",
                    "orientation", "bbox", "note", "drone"):
            self.assertIn(key, payload)
        self.assertEqual(payload["drone"]["pixel"], [256.0, 256.0])

    def test_path_mode_degrades_instead_of_dumping_base64(self):
        with mock.patch.dict(os.environ, {"MAVLINKMCP_MAP_IMAGE_MODE": "path"}):
            got = run(self.m.get_map_view(self.live(), size_px=512))
        self.assertIsInstance(got, dict)
        self.assertEqual(got["image_mode"], "path")
        self.assertTrue(got["image_path"].endswith(".jpg"))

    def test_heading_up_rotates_the_view(self):
        _, payload = run(
            self.m.get_map_view(self.live(), orientation="heading_up", size_px=512)
        )
        self.assertEqual(payload["orientation"], "heading_up")
        self.assertAlmostEqual(payload["rotation_deg"], 72.4, places=1)

    def test_heading_up_without_a_heading_is_refused(self):
        class NoHeading(FakeTelemetry):
            async def heading(self):
                raise RuntimeError("no heading")
                yield  # pragma: no cover

        got = run(
            self.m.get_map_view(self.live(NoHeading()), orientation="heading_up")
        )
        self.assertEqual(got["status"], "failed")
        self.assertIn("north_up", got["error"])

    def test_explicit_coordinates_need_no_vehicle(self):
        got = run(
            self.m.get_map_view(self.dead(), latitude_deg=47.3977, longitude_deg=8.5456)
        )
        self.assertIsInstance(got, list)
        self.assertNotIn("drone", got[1])

    def test_blank_provider_says_there_is_no_basemap(self):
        _, payload = run(self.m.get_map_view(self.live(), size_px=512))
        self.assertIn("imagery_warning", payload)
        self.assertIn("no basemap", payload["imagery_warning"])


class TestMapTransform(ToolTestCase):
    def test_pixels_convert_to_coordinates_and_back(self):
        _, view = run(self.m.get_map_view(self.live(), size_px=512))
        pixels = [[100, 120], [300, 140], [280, 320]]
        forward = run(self.m.map_transform(self.dead(), view["view_id"], pixels=pixels))
        self.assertEqual(forward["status"], "success")
        self.assertEqual(len(forward["coordinates"]), 3)
        # The polygon alias exists so the result can go straight into a plan.
        self.assertEqual(forward["polygon"], forward["coordinates"])

        back = run(
            self.m.map_transform(
                self.dead(), view["view_id"], coordinates=forward["coordinates"]
            )
        )
        for original, restored in zip(pixels, back["pixels"]):
            self.assertAlmostEqual(restored[0], original[0], places=0)
            self.assertAlmostEqual(restored[1], original[1], places=0)

    def test_exactly_one_direction_at_a_time(self):
        _, view = run(self.m.get_map_view(self.live(), size_px=512))
        for kwargs in ({}, {"pixels": [[1, 1]], "coordinates": [[47.0, 8.0]]}):
            got = run(self.m.map_transform(self.dead(), view["view_id"], **kwargs))
            self.assertEqual(got["status"], "failed")
            self.assertIn("exactly one", got["error"])

    def test_out_of_bounds_pixels_are_refused(self):
        _, view = run(self.m.get_map_view(self.live(), size_px=512))
        got = run(self.m.map_transform(self.dead(), view["view_id"], pixels=[[9999, 1]]))
        self.assertEqual(got["status"], "failed")
        self.assertIn("outside the image", got["error"])

    def test_unknown_view_explains_how_to_recover(self):
        got = run(self.m.map_transform(self.dead(), "v0123456789ab", pixels=[[1, 1]]))
        self.assertEqual(got["status"], "failed")
        self.assertIn("get_map_view", got["error"])

    def test_a_traversal_view_id_is_refused(self):
        got = run(self.m.map_transform(self.dead(), "../../etc/passwd", pixels=[[1, 1]]))
        self.assertEqual(got["status"], "failed")


class TestAuthoring(ToolTestCase):
    def test_plan_records_the_view_it_was_drawn_on(self):
        plan, view = self.make_plan()
        stored = run(self.m.get_plan(self.dead(), plan["plan_id"]))
        self.assertEqual(stored["source"]["view_id"], view["view_id"])
        self.assertEqual(len(stored["source"]["polygon_pixels"]), 4)

    def test_polygon_and_pixels_are_mutually_exclusive(self):
        _, view = run(self.m.get_map_view(self.live(), size_px=512))
        got = run(self.m.create_survey_plan(
            self.dead(), name="X", altitude_m=40, line_spacing_m=25.0,
            polygon=[[47.39, 8.54], [47.40, 8.54], [47.40, 8.55]],
            view_id=view["view_id"], polygon_pixels=[[1, 1], [2, 2], [3, 3]],
        ))
        self.assertEqual(got["status"], "failed")
        self.assertIn("exactly one", got["error"])

    def test_a_plain_polygon_works_with_no_view(self):
        got = run(self.m.create_survey_plan(
            self.dead(), name="Plain", altitude_m=40, line_spacing_m=25.0,
            polygon=[[47.3977, 8.5456], [47.3997, 8.5456], [47.3997, 8.5486]],
        ))
        self.assertEqual(got["status"], "success")
        self.assertGreater(got["stats"]["waypoint_count"], 0)

    def test_camera_derives_spacing_and_reports_gsd(self):
        plan, _ = self.make_plan(
            line_spacing_m=None,
            camera={"sensor_width_mm": 13.2, "focal_length_mm": 8.8,
                    "image_width_px": 5472, "image_height_px": 3648,
                    "front_overlap": 0.75, "side_overlap": 0.65},
        )
        self.assertAlmostEqual(plan["derived"]["line_spacing_m"], 21.0, places=2)
        self.assertAlmostEqual(plan["derived"]["gsd_cm_px"], 1.096, places=2)

    def test_names_collide_into_distinct_ids(self):
        first, _ = self.make_plan("Same Name")
        second, _ = self.make_plan("Same Name")
        self.assertNotEqual(first["plan_id"], second["plan_id"])
        self.assertEqual(len(run(self.m.list_plans(self.dead()))["plans"]), 2)

    def test_revise_makes_a_new_draft_revision(self):
        plan, _ = self.make_plan()
        pid = plan["plan_id"]
        run(self.m.validate_plan(self.live(), pid))
        revised = run(self.m.revise_plan(self.dead(), pid, altitude_m=60.0))
        self.assertEqual(revised["revision"], 2)
        # Any change invalidates the previous check.
        self.assertEqual(revised["plan_status"], "draft")
        self.assertEqual(revised["changed"], ["altitude_m"])
        self.assertEqual(
            run(self.m.get_plan(self.dead(), pid, revision=1))["plan_status"], "validated"
        )

    def test_revise_needs_something_to_change(self):
        plan, _ = self.make_plan()
        got = run(self.m.revise_plan(self.dead(), plan["plan_id"]))
        self.assertEqual(got["status"], "failed")

    def test_revise_swaps_spacing_for_a_camera_cleanly(self):
        plan, _ = self.make_plan()
        got = run(self.m.revise_plan(
            self.dead(), plan["plan_id"],
            camera={"sensor_width_mm": 13.2, "focal_length_mm": 8.8,
                    "image_width_px": 5472, "image_height_px": 3648},
        ))
        self.assertEqual(got["status"], "success")
        self.assertIn("gsd_cm_px", got["derived"])

    def test_hand_written_waypoints_can_be_stored(self):
        got = run(self.m.create_plan_from_waypoints(self.dead(), "Manual", [
            {"latitude_deg": 47.3977, "longitude_deg": 8.5456,
             "relative_altitude_m": 40.0, "speed_m_s": 5.0, "is_fly_through": True},
            {"latitude_deg": 47.3987, "longitude_deg": 8.5466,
             "relative_altitude_m": 40.0, "speed_m_s": 5.0, "is_fly_through": True},
        ]))
        self.assertEqual(got["status"], "success")
        self.assertEqual(got["stats"]["waypoint_count"], 2)

    def test_bad_waypoints_are_refused(self):
        got = run(self.m.create_plan_from_waypoints(self.dead(), "Bad", [
            {"latitude_deg": 991.0, "longitude_deg": 8.5456,
             "relative_altitude_m": 40.0, "speed_m_s": 5.0, "is_fly_through": True},
        ]))
        self.assertEqual(got["status"], "failed")

    def test_delete_removes_the_plan(self):
        plan, _ = self.make_plan()
        got = run(self.m.delete_plan(self.dead(), plan["plan_id"]))
        self.assertEqual(got["status"], "success")
        self.assertEqual(got["revisions_removed"], 1)
        self.assertEqual(
            run(self.m.get_plan(self.dead(), plan["plan_id"]))["status"], "failed"
        )

    def test_waypoints_are_omitted_from_get_plan_by_default(self):
        plan, _ = self.make_plan()
        without = run(self.m.get_plan(self.dead(), plan["plan_id"]))
        with_them = run(self.m.get_plan(self.dead(), plan["plan_id"], include_waypoints=True))
        self.assertNotIn("waypoints", without)
        self.assertIn("waypoints", with_them)


class TestPreflight(ToolTestCase):
    def test_go_when_everything_is_ready(self):
        plan, _ = self.make_plan()
        run(self.m.validate_plan(self.live(), plan["plan_id"]))
        got = run(self.m.preflight_check(self.live(), plan["plan_id"]))
        self.assertEqual(got["verdict"], "GO")
        self.assertEqual(got["blockers"], [])

    def test_no_go_without_a_three_dimensional_fix(self):
        # "NO_FIX" contains the substring "FIX"; the gate must not read that as
        # a usable fix.
        plan, _ = self.make_plan()
        run(self.m.validate_plan(self.live(), plan["plan_id"]))
        for fix in ("NO_FIX", "NO_GPS", "FIX_2D"):
            got = run(self.m.preflight_check(
                self.live(FakeTelemetry(fix=fix)), plan["plan_id"]
            ))
            self.assertEqual(got["verdict"], "NO-GO", fix)
            self.assertTrue(any("GPS fix" in b for b in got["blockers"]))

    def test_rtk_and_dgps_fixes_are_accepted(self):
        plan, _ = self.make_plan()
        run(self.m.validate_plan(self.live(), plan["plan_id"]))
        for fix in ("FIX_3D", "RTK_FIXED", "RTK_FLOAT", "FIX_DGPS"):
            got = run(self.m.preflight_check(
                self.live(FakeTelemetry(fix=fix)), plan["plan_id"]
            ))
            self.assertEqual(got["verdict"], "GO", fix)

    def test_low_battery_blocks(self):
        plan, _ = self.make_plan()
        got = run(self.m.preflight_check(
            self.live(FakeTelemetry(battery=0.22)), plan["plan_id"]
        ))
        self.assertEqual(got["verdict"], "NO-GO")
        self.assertTrue(any("battery" in b for b in got["blockers"]))

    def test_unarmable_vehicle_blocks(self):
        plan, _ = self.make_plan()
        got = run(self.m.preflight_check(
            self.live(FakeTelemetry(armable=False)), plan["plan_id"]
        ))
        self.assertEqual(got["verdict"], "NO-GO")

    def test_one_broken_stream_degrades_to_a_warning(self):
        # A preflight that cannot read one stream should still report the rest;
        # a partial answer is what lets an operator decide.
        class NoBattery(FakeTelemetry):
            async def battery(self):
                raise RuntimeError("no battery telemetry")
                yield  # pragma: no cover

        plan, _ = self.make_plan()
        got = run(self.m.preflight_check(self.live(NoBattery()), plan["plan_id"]))
        self.assertEqual(got["status"], "success")
        self.assertEqual(got["verdict"], "GO")
        self.assertTrue(any("battery" in w for w in got["warnings"]))

    def test_a_draft_plan_warns(self):
        plan, _ = self.make_plan()
        got = run(self.m.preflight_check(self.live(), plan["plan_id"]))
        self.assertTrue(any("draft" in w for w in got["warnings"]))


class TestMissionControl(ToolTestCase):
    def test_start_pause_clear_and_jump(self):
        live = self.live()
        self.assertEqual(run(self.m.start_mission(live))["status"], "command_sent")
        self.assertTrue(self.mission.started)
        self.assertEqual(run(self.m.pause_mission(live))["status"], "command_sent")
        self.assertTrue(self.mission.paused)
        self.assertEqual(run(self.m.set_current_waypoint(live, 3))["status"], "success")
        self.assertEqual(self.mission.current_item, 3)
        self.assertEqual(run(self.m.clear_mission(live))["status"], "success")
        self.assertTrue(self.mission.cleared)

    def test_waypoint_index_is_validated(self):
        for bad in (-1, True, "x"):
            got = run(self.m.set_current_waypoint(self.live(), bad))
            self.assertEqual(got["status"], "failed")

    def test_download_captures_the_loaded_mission(self):
        plan, _ = self.make_plan()
        run(self.m.validate_plan(self.live(), plan["plan_id"]))
        run(self.m.upload_plan(self.live(), plan["plan_id"]))
        got = run(self.m.download_mission_as_plan(self.live(), "Captured"))
        self.assertEqual(got["status"], "success")
        self.assertEqual(
            got["stats"]["waypoint_count"], plan["stats"]["waypoint_count"]
        )

    def test_download_with_nothing_loaded(self):
        got = run(self.m.download_mission_as_plan(self.live(), "Empty"))
        self.assertEqual(got["status"], "failed")
        self.assertIn("no mission", got["error"])


class TestGeoJSONTool(ToolTestCase):
    def test_emits_a_feature_collection(self):
        plan, _ = self.make_plan()
        got = run(self.m.preview_plan_geojson(self.dead(), plan["plan_id"]))
        self.assertEqual(got["geojson"]["type"], "FeatureCollection")
        roles = [f["properties"]["role"] for f in got["geojson"]["features"]]
        self.assertIn("area_of_interest", roles)
        self.assertIn("flight_path", roles)


class TestPrefetch(ToolTestCase):
    def test_blank_provider_serves_no_tiles(self):
        got = run(self.m.prefetch_map_area(
            self.dead(), latitude_deg=47.3977, longitude_deg=8.5456, radius_m=200
        ))
        self.assertEqual(got["status"], "failed")
        self.assertIn("serves no tiles", got["error"])

    def test_needs_an_area(self):
        got = run(self.m.prefetch_map_area(self.dead()))
        self.assertEqual(got["status"], "failed")


if __name__ == "__main__":
    unittest.main()
