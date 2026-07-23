import unittest
from types import SimpleNamespace

from src.server.vtol_state_helpers import (
    KNOWN_STATES,
    normalize_vtol_state,
    vtol_state_status_err,
)


class TestVtolStateHelpers(unittest.TestCase):
    def test_success_each_known_str(self):
        for state in KNOWN_STATES:
            with self.subTest(state=state):
                r = normalize_vtol_state(state)
                self.assertEqual(r["status"], "success")
                self.assertEqual(r["vtol_state"], state)

    def test_case_insensitive(self):
        r = normalize_vtol_state("mc")
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["vtol_state"], "MC")

    def test_aliases(self):
        self.assertEqual(normalize_vtol_state("multicopter")["vtol_state"], "MC")
        self.assertEqual(normalize_vtol_state("MULTI_COPTER")["vtol_state"], "MC")
        self.assertEqual(normalize_vtol_state("fixed_wing")["vtol_state"], "FW")
        self.assertEqual(normalize_vtol_state("FIXEDWING")["vtol_state"], "FW")

    def test_enum_name_attr(self):
        r = normalize_vtol_state(SimpleNamespace(name="TRANSITION_TO_MC"))
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["vtol_state"], "TRANSITION_TO_MC")

    def test_str_enum_form(self):
        class _FakeEnum:
            def __str__(self) -> str:
                return "VtolState.TRANSITION_TO_FW"

        r = normalize_vtol_state(_FakeEnum())
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["vtol_state"], "TRANSITION_TO_FW")

    def test_vtol_state_prefix_alias(self):
        r = normalize_vtol_state("VTOL_STATE_FW")
        self.assertEqual(r["status"], "success")
        self.assertEqual(r["vtol_state"], "FW")

    def test_none_failed(self):
        self.assertEqual(normalize_vtol_state(None)["status"], "failed")

    def test_empty_str_failed(self):
        self.assertEqual(normalize_vtol_state("")["status"], "failed")

    def test_int_failed(self):
        self.assertEqual(normalize_vtol_state(1)["status"], "failed")

    def test_unknown_failed(self):
        self.assertEqual(normalize_vtol_state("HOVER")["status"], "failed")
        self.assertEqual(
            normalize_vtol_state(SimpleNamespace(name="HOVER"))["status"], "failed"
        )

    def test_err_helper(self):
        e = vtol_state_status_err("nope")
        self.assertEqual(e["status"], "failed")
        self.assertEqual(e["error"], "nope")


if __name__ == "__main__":
    unittest.main()
