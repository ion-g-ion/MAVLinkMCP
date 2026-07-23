import unittest
import math

from src.server.wind_helpers import normalize_wind, wind_status_err


class TestWindHelpers(unittest.TestCase):
    def test_ned_success(self):
        r = normalize_wind(
            {
                "wind_x_ned_m_s": 1.0,
                "wind_y_ned_m_s": -2.0,
                "wind_z_ned_m_s": 0.1,
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["wind"]["wind_x_ned_m_s"], 1.0)
        self.assertEqual(r["wind"]["wind_y_ned_m_s"], -2.0)

    def test_ned_with_horizontal(self):
        r = normalize_wind(
            {
                "wind_x_ned_m_s": 3.0,
                "wind_y_ned_m_s": 4.0,
                "wind_z_ned_m_s": 0.0,
                "horizontal_speed_m_s": 5.0,
            }
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["wind"]["horizontal_speed_m_s"], 5.0)

    def test_speed_direction(self):
        r = normalize_wind({"speed_m_s": 4.5, "direction_deg": 180.0})
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["wind"]["speed_m_s"], 4.5)
        self.assertEqual(r["wind"]["direction_deg"], 180.0)

    def test_speed_with_vertical(self):
        r = normalize_wind(
            {"speed_m_s": 1.0, "direction_deg": 0.0, "vertical_speed_m_s": -0.2}
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["wind"]["vertical_speed_m_s"], -0.2)

    def test_nan_ned_failed(self):
        r = normalize_wind(
            {
                "wind_x_ned_m_s": float("nan"),
                "wind_y_ned_m_s": 0.0,
                "wind_z_ned_m_s": 0.0,
            }
        )
        self.assertEqual(r["status"], "failed")

    def test_negative_speed_failed(self):
        r = normalize_wind({"speed_m_s": -1.0, "direction_deg": 10.0})
        self.assertEqual(r["status"], "failed")

    def test_direction_oor_failed(self):
        r = normalize_wind({"speed_m_s": 1.0, "direction_deg": 1000.0})
        self.assertEqual(r["status"], "failed")

    def test_missing_ned_component(self):
        r = normalize_wind({"wind_x_ned_m_s": 1.0, "wind_y_ned_m_s": 0.0})
        self.assertEqual(r["status"], "failed")

    def test_non_mapping(self):
        r = normalize_wind([1, 2])  # type: ignore[arg-type]
        self.assertEqual(r["status"], "failed")

    def test_empty_failed(self):
        r = normalize_wind({})
        self.assertEqual(r["status"], "failed")

    def test_err_helper(self):
        e = wind_status_err("x")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "x")

    def test_inf_speed(self):
        r = normalize_wind({"speed_m_s": math.inf, "direction_deg": 0.0})
        self.assertEqual(r["status"], "failed")


if __name__ == "__main__":
    unittest.main()
