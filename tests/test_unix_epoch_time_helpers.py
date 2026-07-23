import math
import unittest
from types import SimpleNamespace

from src.server.unix_epoch_time_helpers import (
    normalize_unix_epoch_time,
    unix_epoch_time_status_err,
)


class TestUnixEpochTimeHelpers(unittest.TestCase):
    def test_success_int_seconds(self):
        r = normalize_unix_epoch_time(1_700_000_000)
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 1_700_000_000.0)

    def test_success_float_seconds(self):
        r = normalize_unix_epoch_time(1_700_000_000.5)
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 1_700_000_000.5)

    def test_success_time_us_object(self):
        # 1_700_000_000_000_000 us = 1_700_000_000 s
        r = normalize_unix_epoch_time(
            SimpleNamespace(time_us=1_700_000_000_000_000)
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 1_700_000_000.0)

    def test_success_time_utc_us_object(self):
        r = normalize_unix_epoch_time(
            SimpleNamespace(time_utc_us=2_000_000)
        )
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 2.0)

    def test_success_mapping_unix_epoch_s(self):
        r = normalize_unix_epoch_time({"unix_epoch_s": 1_700_000_000})
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 1_700_000_000.0)

    def test_success_mapping_time_utc_us(self):
        r = normalize_unix_epoch_time({"time_utc_us": 5_000_000})
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 5.0)

    def test_success_mapping_value_nested(self):
        r = normalize_unix_epoch_time({"value": 42})
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["unix_epoch_time"]["unix_epoch_s"], 42.0)

    def test_none_failed(self):
        r = normalize_unix_epoch_time(None)
        self.assertEqual(r["status"], "failed")

    def test_bool_failed(self):
        r = normalize_unix_epoch_time(True)
        self.assertEqual(r["status"], "failed")

    def test_negative_failed(self):
        r = normalize_unix_epoch_time(-1)
        self.assertEqual(r["status"], "failed")

    def test_nan_failed(self):
        r = normalize_unix_epoch_time(float("nan"))
        self.assertEqual(r["status"], "failed")

    def test_inf_failed(self):
        r = normalize_unix_epoch_time(float("inf"))
        self.assertEqual(r["status"], "failed")

    def test_str_failed(self):
        r = normalize_unix_epoch_time("now")
        self.assertEqual(r["status"], "failed")

    def test_empty_mapping_failed(self):
        r = normalize_unix_epoch_time({})
        self.assertEqual(r["status"], "failed")

    def test_err_helper(self):
        e = unix_epoch_time_status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
