"""Telemetry tools against a live PX4.

The offline suite proves these helpers format correctly when handed a namespace.
What is unproven there is that MAVSDK hands them anything at all, and that the
field names still line up with what a real autopilot publishes.
"""
from tests_sitl.harness import HOME_LAT, HOME_LON, SitlTestCase, run


class TestLink(SitlTestCase):
    def test_link_reports_ready(self):
        self.assertTrue(self.connector.is_linked)
        self.assertEqual(self.connector.link_failure(), "")

    def test_position_is_the_configured_home(self):
        """Catches a SITL started without the scaled-int home coordinates.

        Passing decimal degrees to PX4_HOME_LAT truncates 47.397742 to 47, which
        puts the vehicle ~150 km away -- close enough to look plausible in a log,
        far enough to invalidate every distance-from-home check.
        """
        result = self.ok(run(self.server.get_position(self.ctx)), "get_position")
        position = result["position"]
        self.assertAlmostEqual(position["latitude_deg"], HOME_LAT, places=2)
        self.assertAlmostEqual(position["longitude_deg"], HOME_LON, places=2)

    def test_health_is_ready_to_fly(self):
        health = self.ok(run(self.server.get_health(self.ctx)), "get_health")["health"]
        self.assertTrue(health["is_global_position_ok"], health)
        self.assertTrue(health["is_home_position_ok"], health)
        self.assertTrue(health["is_armable"], health)

    def test_gps_has_a_fix(self):
        info = self.ok(run(self.server.get_gps_info(self.ctx)), "get_gps_info")["gps_info"]
        self.assertEqual(info["fix_type"], "FIX_3D", info)
        self.assertGreater(info["num_satellites"], 0, info)

    def test_battery_is_reported(self):
        battery = self.ok(run(self.server.get_battery(self.ctx)), "get_battery")["battery"]
        self.assertGreater(battery["voltage_v"], 0.0, battery)

    def test_starts_disarmed_on_the_ground(self):
        armed = self.ok(run(self.server.get_is_armed(self.ctx)), "get_is_armed")
        self.assertFalse(armed["is_armed"], armed)
        state = self.ok(run(self.server.get_landed_state(self.ctx)), "get_landed_state")
        self.assertEqual(state["landed_state"], "ON_GROUND", state)

    def test_home_position_matches_configured_home(self):
        home = self.ok(run(self.server.get_home_position(self.ctx)), "get_home_position")["home"]
        self.assertAlmostEqual(home["latitude_deg"], HOME_LAT, places=2)
        self.assertAlmostEqual(home["longitude_deg"], HOME_LON, places=2)
