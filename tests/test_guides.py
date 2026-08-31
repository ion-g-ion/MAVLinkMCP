"""Offline unit tests for the prompt text (no drone, no network).

Prose cannot be unit-tested for being *good*, but it can be tested for being
*wrong*. These check the two ways this text goes stale: a limit quoted in a
briefing drifting away from the limit the code enforces, and a briefing dropping
one of the rules it exists to state.
"""
import unittest

from mavlinkmcp import guides, map_source, map_view
from mavlinkmcp.plan_helpers import STATUSES

ALL_GUIDES = (
    guides.server_instructions,
    guides.overview,
    guides.telemetry_guide,
    guides.map_view_guide,
    guides.plan_lifecycle_guide,
)


class TestEveryGuide(unittest.TestCase):
    def test_returns_substantial_text(self):
        for fn in ALL_GUIDES:
            with self.subTest(guide=fn.__name__):
                text = fn()
                self.assertIsInstance(text, str)
                self.assertGreater(len(text), 500)

    def test_no_unresolved_interpolations(self):
        # A guide written without the f prefix still returns plausible-looking
        # prose, with `{map_view.MAX_SIZE_PX}` sitting in it as literal text. The
        # braces this text legitimately contains ({z}/{x}/{y}, {plan_id}, the
        # camera dict) never name a module, so look for the module names.
        modules = ("map_view.", "map_source.", "plan_store.", "tile_helpers.", "STATUSES")
        for fn in ALL_GUIDES + (guides.survey_walkthrough, guides.preflight_briefing):
            text = fn("x") if fn is guides.preflight_briefing else fn()
            for module in modules:
                with self.subTest(guide=fn.__name__, module=module):
                    self.assertNotIn("{" + module, text)


class TestOverview(unittest.TestCase):
    def test_states_the_call_envelope(self):
        text = guides.overview()
        self.assertIn("plan_status", text)
        self.assertIn("fail-closed", text)

    def test_explains_the_background_link(self):
        text = guides.overview()
        self.assertIn("link_state", text)
        self.assertIn("MAVLINK_CONNECT_TIMEOUT", text)

    def test_lists_every_plan_status(self):
        text = guides.overview()
        for status in STATUSES:
            self.assertIn(status, text)


class TestTelemetryGuide(unittest.TestCase):
    def test_forbids_treating_a_failed_read_as_zero(self):
        text = guides.telemetry_guide()
        self.assertIn("unknown is not the ground", text)
        self.assertIn("fail-closed", text)


class TestMapViewGuide(unittest.TestCase):
    def test_carries_the_pixel_rule(self):
        self.assertIn(guides.PIXEL_RULE, guides.map_view_guide())

    def test_quotes_the_limits_actually_enforced(self):
        text = guides.map_view_guide()
        self.assertIn(str(map_view.MAX_SIZE_PX), text)
        self.assertIn(str(map_view.DEFAULT_SIZE_PX), text)
        self.assertIn(str(map_source.MAX_TILES_PER_REQUEST), text)

    def test_names_every_provider(self):
        text = guides.map_view_guide()
        for name in map_source.PROVIDERS:
            self.assertIn(name, text)

    def test_warns_about_the_blank_provider(self):
        self.assertIn("MAVLINKMCP_MAP_PROVIDER=none", guides.map_view_guide())

    def test_keeps_attribution_non_optional(self):
        self.assertIn("Attribution is not optional", guides.map_view_guide())


class TestPlanLifecycleGuide(unittest.TestCase):
    def test_states_the_upload_gate(self):
        text = guides.plan_lifecycle_guide()
        self.assertIn("upload_plan refuses", text)
        self.assertIn("FAR_FROM_HOME", text)

    def test_quotes_the_store_limits(self):
        text = guides.plan_lifecycle_guide()
        self.assertIn(str(guides.plan_store.MAX_PLANS), text)
        self.assertIn(str(guides.plan_store.MAX_REVISIONS_PER_PLAN), text)


class TestSurveyWalkthrough(unittest.TestCase):
    def test_works_with_no_arguments(self):
        text = guides.survey_walkthrough()
        self.assertIn(guides.PIPELINE, text)
        self.assertNotIn("The operator wants to survey", text)
        self.assertNotIn("Requested altitude", text)

    def test_folds_in_the_operator_request(self):
        text = guides.survey_walkthrough("the paddock north of the barn", "45")
        self.assertIn("the paddock north of the barn", text)
        self.assertIn("Requested altitude: 45 m", text)

    def test_blank_arguments_are_treated_as_absent(self):
        text = guides.survey_walkthrough("   ", "  ")
        self.assertNotIn("The operator wants to survey", text)
        self.assertNotIn("Requested altitude", text)

    def test_carries_the_pixel_rule(self):
        self.assertIn(guides.PIXEL_RULE, guides.survey_walkthrough())

    def test_refuses_to_let_the_review_steps_be_skipped(self):
        text = guides.survey_walkthrough()
        self.assertIn("render_plan_view", text)
        self.assertIn("preflight_check", text)
        self.assertIn("Never skip", text)


class TestPreflightBriefing(unittest.TestCase):
    def test_embeds_the_plan_id_in_every_call(self):
        text = guides.preflight_briefing("north-field")
        for tool in (
            "get_plan",
            "render_plan_view",
            "estimate_plan",
            "validate_plan",
            "preflight_check",
        ):
            self.assertIn(f'{tool}("north-field")', text)

    def test_does_not_start_the_mission(self):
        text = guides.preflight_briefing("north-field")
        self.assertIn("Do not start the mission", text)
        self.assertNotIn("start_mission(", text)

    def test_missing_id_leaves_a_visible_placeholder(self):
        # Better an obvious <plan_id> than a briefing that reads as if it had one.
        self.assertIn("<plan_id>", guides.preflight_briefing(""))

    def test_upholds_the_upload_gate(self):
        self.assertIn("upload_plan will refuse", guides.preflight_briefing("x"))


class TestServerInstructions(unittest.TestCase):
    """The handshake text is the only guidance a client cannot decline to load."""

    def test_states_the_three_rules(self):
        text = guides.server_instructions()
        self.assertIn("command_sent", text)
        self.assertIn("verify_with", text)
        self.assertIn("Altitudes are relative", text)
        self.assertIn("When you are not sure, ask", text)

    def test_names_every_command_that_only_acknowledges(self):
        text = guides.server_instructions()
        for tool in (
            "arm_drone",
            "takeoff",
            "land",
            "return_to_launch",
            "disarm_drone",
            "move_to_relative",
            "start_mission",
            "pause_mission",
        ):
            with self.subTest(tool=tool):
                self.assertIn(tool, text)

    def test_shares_its_prose_with_the_overview_prompt(self):
        # Two hand-maintained copies would drift; the overview must reuse the
        # constants rather than paraphrase them.
        overview = guides.overview()
        for rule in (guides.COMMAND_SENT_RULE, guides.ALTITUDE_RULE, guides.ASK_RULE):
            with self.subTest(rule=rule[:40]):
                self.assertIn(rule, overview)
                self.assertIn(rule, guides.server_instructions())


if __name__ == "__main__":
    unittest.main()
