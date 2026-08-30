"""The plan lifecycle against a real mission protocol.

create -> validate -> upload -> verify -> download. The offline suite stubs
``drone.mission``, so it can only check that the tools call it; whether PX4
accepts the items, and whether what comes back is what went out, is decided
here. Mission upload is also where a field-name or unit mistake shows up as a
route in the wrong place rather than an exception.
"""
from tests_sitl.harness import HOME_LAT, HOME_LON, SitlTestCase, run

# A short box near home. Roughly 100 m per side: far enough to be a real route,
# close enough that the distance-from-home check passes.
WAYPOINTS = [
    {"latitude_deg": HOME_LAT + 0.0009, "longitude_deg": HOME_LON,
     "relative_altitude_m": 30.0, "speed_m_s": 5.0, "is_fly_through": True},
    {"latitude_deg": HOME_LAT + 0.0009, "longitude_deg": HOME_LON + 0.0013,
     "relative_altitude_m": 30.0, "speed_m_s": 5.0, "is_fly_through": True},
    {"latitude_deg": HOME_LAT, "longitude_deg": HOME_LON + 0.0013,
     "relative_altitude_m": 30.0, "speed_m_s": 5.0, "is_fly_through": False},
]


class TestMissionRoundTrip(SitlTestCase):
    def setUp(self):
        super().setUp()
        # Leave no mission behind for the next test; the vehicle outlives it.
        self.addCleanup(lambda: run(self.server.clear_mission(self.ctx)))

    def stored_plan(self, name="Round Trip"):
        """A validated plan, ready to upload."""
        created = self.ok(
            run(self.server.create_plan_from_waypoints(
                self.ctx, name=name, waypoints=WAYPOINTS, return_to_launch=True)),
            "create_plan_from_waypoints",
        )
        plan_id = created["plan_id"]
        validated = self.ok(
            run(self.server.validate_plan(self.ctx, plan_id=plan_id)), "validate_plan")
        self.assertEqual(validated["plan_status"], "validated", validated)
        return plan_id

    def test_upload_gate_refuses_a_draft(self):
        """The gate has to hold against a real vehicle, not just a stub.

        An unvalidated plan reaching the aircraft is the failure this whole
        lifecycle exists to prevent.
        """
        created = self.ok(
            run(self.server.create_plan_from_waypoints(
                self.ctx, name="Unchecked", waypoints=WAYPOINTS)),
            "create_plan_from_waypoints",
        )
        result = run(self.server.upload_plan(self.ctx, plan_id=created["plan_id"]))
        self.assertEqual(result["status"], "failed", result)
        self.assertIn("validate_plan", result["error"])

    def test_upload_then_verify_reports_a_match(self):
        plan_id = self.stored_plan()

        uploaded = self.ok(
            run(self.server.upload_plan(self.ctx, plan_id=plan_id)), "upload_plan")
        self.assertEqual(uploaded["waypoint_count"], len(WAYPOINTS), uploaded)

        verified = self.ok(
            run(self.server.verify_uploaded_plan(self.ctx, plan_id=plan_id)),
            "verify_uploaded_plan",
        )
        self.assertTrue(verified["verified"], verified)
        self.assertEqual(
            verified["vehicle_waypoint_count"], verified["plan_waypoint_count"], verified)
        self.assertEqual(verified["differences"], [], verified)

    def test_downloaded_mission_keeps_the_coordinates(self):
        """What the vehicle gives back must be the route that was sent.

        Guards the translation in _mission_items_from, where stored JSON becomes
        MAVSDK MissionItems -- a swapped lat/lon or a dropped altitude survives
        an upload without error and only shows up on the way back.
        """
        plan_id = self.stored_plan(name="Downloaded")
        self.ok(run(self.server.upload_plan(self.ctx, plan_id=plan_id)), "upload_plan")

        downloaded = self.ok(
            run(self.server.download_mission_as_plan(self.ctx, name="From Vehicle")),
            "download_mission_as_plan",
        )
        self.assertEqual(
            downloaded["stats"]["waypoint_count"], len(WAYPOINTS), downloaded)

        plan = self.ok(
            run(self.server.get_plan(
                self.ctx, plan_id=downloaded["plan_id"], include_waypoints=True)),
            "get_plan",
        )
        actual = plan["waypoints"]
        for sent, got in zip(WAYPOINTS, actual):
            self.assertAlmostEqual(got["latitude_deg"], sent["latitude_deg"], places=5)
            self.assertAlmostEqual(got["longitude_deg"], sent["longitude_deg"], places=5)
            self.assertAlmostEqual(
                got["relative_altitude_m"], sent["relative_altitude_m"], places=1)

    def test_clear_mission_empties_the_vehicle(self):
        plan_id = self.stored_plan(name="Cleared")
        self.ok(run(self.server.upload_plan(self.ctx, plan_id=plan_id)), "upload_plan")
        self.ok(run(self.server.clear_mission(self.ctx)), "clear_mission")

        result = run(self.server.download_mission_as_plan(self.ctx, name="After Clear"))
        self.assertEqual(result["status"], "failed", result)
        self.assertIn("no mission", result["error"].lower())
