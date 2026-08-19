import unittest

from altitude_helpers import altitude_status_err, normalize_altitude


class TestAltitudeHelpers(unittest.TestCase):
    def test_success_core(self):
        r = normalize_altitude(
            {"altitude_amsl_m": 120.0, "altitude_relative_m": 12.5}
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["altitude"]["altitude_amsl_m"], 120.0)
        self.assertEqual(r["altitude"]["altitude_relative_m"], 12.5)
        self.assertNotIn("altitude_terrain_m", r["altitude"])

    def test_success_with_local_terrain(self):
        r = normalize_altitude(
            {
                "altitude_amsl_m": 100.0,
                "altitude_relative_m": 5.0,
                "altitude_local_m": -5.0,
                "altitude_terrain_m": 4.5,
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["altitude"]["altitude_local_m"], -5.0)
        self.assertEqual(r["altitude"]["altitude_terrain_m"], 4.5)

    def test_omit_nan_terrain(self):
        r = normalize_altitude(
            {
                "altitude_amsl_m": 10.0,
                "altitude_relative_m": 3.0,
                "altitude_terrain_m": float("nan"),
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertNotIn("altitude_terrain_m", r["altitude"])

    def test_bad_local_nan(self):
        r = normalize_altitude(
            {
                "altitude_amsl_m": 10.0,
                "altitude_relative_m": 3.0,
                "altitude_local_m": float("nan"),
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_missing_rel(self):
        r = normalize_altitude({"altitude_amsl_m": 1.0})
        self.assertEqual(r["status"], "failed")

    def test_err_helper(self):
        e = altitude_status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
