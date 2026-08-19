import unittest
from types import SimpleNamespace

from landed_state_helpers import (
    KNOWN_STATES,
    landed_state_status_err,
    normalize_landed_state,
)


class TestLandedStateHelpers(unittest.TestCase):
    def test_success_each_known_str(self):
        for state in KNOWN_STATES:
            with self.subTest(state=state):
                r = normalize_landed_state(state)
                self.assertEqual(r["status"], "success")
                self.assertEqual(r["landed_state"], state)

    def test_case_insensitive(self):
        r = normalize_landed_state("in_air")
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["landed_state"], "IN_AIR")

    def test_enum_name_attr(self):
        r = normalize_landed_state(SimpleNamespace(name="ON_GROUND"))
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["landed_state"], "ON_GROUND")

    def test_str_enum_form(self):
        class _FakeEnum:
            def __str__(self) -> str:
                return "LandedState.TAKING_OFF"

        r = normalize_landed_state(_FakeEnum())
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["landed_state"], "TAKING_OFF")

    def test_none_failed(self):
        r = normalize_landed_state(None)
        self.assertEqual(r["status"], "failed")

    def test_empty_str_failed(self):
        r = normalize_landed_state("")
        self.assertEqual(r["status"], "failed")

    def test_int_failed(self):
        r = normalize_landed_state(1)
        self.assertEqual(r["status"], "failed")

    def test_unknown_name_failed(self):
        r = normalize_landed_state("FLYING")
        self.assertEqual(r["status"], "failed")
        r2 = normalize_landed_state(SimpleNamespace(name="FLYING"))
        self.assertEqual(r2["status"], "failed")

    def test_err_helper(self):
        e = landed_state_status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
