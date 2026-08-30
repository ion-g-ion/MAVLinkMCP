"""Offline unit tests for plan/view/tile persistence (no drone, no network).

Every test runs against a temporary directory, so the suite never touches the
operator's real plan store.
"""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from mavlinkmcp import plan_store
from mavlinkmcp.map_transform import build_view
from mavlinkmcp.plan_helpers import STATUS_UPLOADED, STATUS_VALIDATED, build_plan


def wp(lat, lon):
    return {
        "latitude_deg": lat,
        "longitude_deg": lon,
        "relative_altitude_m": 40.0,
        "speed_m_s": 5.0,
        "is_fly_through": True,
    }


POINTS = [wp(47.3977, 8.5456), wp(47.3987, 8.5466)]


class StoreTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def save(self, plan_id="north-field", name="North Field", revision=1, **kw):
        plan = build_plan(plan_id, name, POINTS, revision=revision, **kw)
        plan_store.save_plan(plan, self.root)
        return plan


class TestRoot(StoreTestCase):
    def test_env_override_must_be_absolute(self):
        with mock.patch.dict(
            os.environ, {"MAVLINKMCP_PLANS_DIR": "relative/path"}
        ):
            with self.assertRaises(ValueError):
                plan_store.data_root()

    def test_env_override_is_used(self):
        with mock.patch.dict(
            os.environ, {"MAVLINKMCP_PLANS_DIR": "/tmp/somewhere"}
        ):
            self.assertEqual(plan_store.data_root(), Path("/tmp/somewhere"))

    def test_falls_back_to_xdg(self):
        with mock.patch.dict(
            os.environ, {"MAVLINKMCP_PLANS_DIR": "", "XDG_DATA_HOME": "/tmp/xdg"}
        ):
            self.assertEqual(plan_store.data_root(), Path("/tmp/xdg/mavlinkmcp"))

    def test_subdirectories_hang_off_the_root(self):
        self.assertEqual(plan_store.plans_dir(self.root), self.root / "plans")
        self.assertEqual(plan_store.views_dir(self.root), self.root / "views")
        self.assertEqual(plan_store.tiles_dir(self.root), self.root / "tiles")


class TestPlanRoundTrip(StoreTestCase):
    def test_saves_and_loads(self):
        self.save()
        loaded = plan_store.load_plan("north-field", root=self.root)
        self.assertEqual(loaded["name"], "North Field")
        self.assertEqual(len(loaded["waypoints"]), 2)

    def test_meta_points_at_the_head(self):
        self.save()
        self.save(revision=2)
        self.assertEqual(plan_store.read_meta("north-field", self.root)["head"], 2)
        self.assertEqual(
            plan_store.load_plan("north-field", root=self.root)["revision"], 2
        )

    def test_an_older_revision_is_still_readable(self):
        self.save()
        self.save(revision=2)
        self.assertEqual(
            plan_store.load_plan("north-field", 1, self.root)["revision"], 1
        )
        self.assertEqual(plan_store.list_revisions("north-field", self.root), [1, 2])

    def test_next_revision_counts_up(self):
        self.assertEqual(plan_store.next_revision("north-field", self.root), 1)
        self.save()
        self.assertEqual(plan_store.next_revision("north-field", self.root), 2)

    def test_missing_plan_and_revision_are_distinguished(self):
        with self.assertRaises(FileNotFoundError):
            plan_store.load_plan("nope", root=self.root)
        self.save()
        with self.assertRaises(FileNotFoundError) as ctx:
            plan_store.load_plan("north-field", 7, self.root)
        self.assertIn("available", str(ctx.exception))

    def test_created_at_survives_a_rewrite(self):
        first = self.save()
        self.save(revision=2)
        meta = plan_store.read_meta("north-field", self.root)
        self.assertEqual(meta["created_at"], first["created_at"])

    def test_stored_file_is_plain_readable_json(self):
        self.save()
        path = self.root / "plans" / "north-field" / "rev-001.json"
        self.assertTrue(path.is_file())
        self.assertEqual(json.loads(path.read_text())["plan_id"], "north-field")


class TestMutability(StoreTestCase):
    def test_lifecycle_fields_can_change(self):
        self.save()
        updated = plan_store.update_plan(
            "north-field", root=self.root, status=STATUS_VALIDATED,
            checks={"passed": True}, estimate={"duration_s": 100.0},
        )
        self.assertEqual(updated["status"], STATUS_VALIDATED)
        self.assertEqual(
            plan_store.load_plan("north-field", root=self.root)["checks"]["passed"], True
        )

    def test_generated_content_cannot_change(self):
        # Waypoints are what a reviewer approved; they must not drift afterwards.
        self.save()
        for field in ("waypoints", "generator", "stats", "created_at", "schema_version"):
            with self.assertRaises(ValueError) as ctx:
                plan_store.update_plan("north-field", root=self.root, **{field: {}})
            self.assertIn("revise the plan instead", str(ctx.exception))

    def test_revision_is_a_selector_not_a_field(self):
        # update_plan(plan_id, revision=2, ...) means "annotate revision 2",
        # so it is bound as a parameter rather than treated as a plan field.
        self.save()
        self.save(revision=2)
        plan_store.update_plan("north-field", 1, self.root, status=STATUS_VALIDATED)
        self.assertEqual(
            plan_store.load_plan("north-field", 1, self.root)["status"], STATUS_VALIDATED
        )
        self.assertNotEqual(
            plan_store.load_plan("north-field", 2, self.root)["status"], STATUS_VALIDATED
        )

    def test_a_nonsense_revision_is_refused_clearly(self):
        self.save()
        for bad in ({}, [], True, "abc", 0, -1):
            with self.assertRaises(ValueError):
                plan_store.load_plan("north-field", bad, self.root)

    def test_unknown_status_is_refused(self):
        self.save()
        with self.assertRaises(ValueError):
            plan_store.update_plan("north-field", root=self.root, status="flying")

    def test_updating_the_name_updates_the_meta(self):
        self.save()
        plan_store.update_plan("north-field", root=self.root, name="Renamed")
        self.assertEqual(plan_store.read_meta("north-field", self.root)["name"], "Renamed")


class TestListingAndDeletion(StoreTestCase):
    def test_lists_summaries_without_waypoints(self):
        self.save()
        listed = plan_store.list_plans(self.root)
        self.assertEqual(len(listed), 1)
        self.assertNotIn("waypoints", listed[0])
        self.assertEqual(listed[0]["waypoint_count"], 2)

    def test_lifecycle_status_is_not_called_status(self):
        # tool_ok/tool_err own the "status" key on the wire; a lifecycle value
        # there would read as a failed call.
        self.save()
        self.assertIn("plan_status", plan_store.list_plans(self.root)[0])
        self.assertNotIn("status", plan_store.list_plans(self.root)[0])

    def test_empty_store_lists_nothing(self):
        self.assertEqual(plan_store.list_plans(self.root), [])

    def test_deletes_every_revision(self):
        self.save()
        self.save(revision=2)
        self.assertEqual(plan_store.delete_plan("north-field", self.root), 2)
        self.assertFalse(plan_store.plan_exists("north-field", self.root))
        with self.assertRaises(FileNotFoundError):
            plan_store.delete_plan("north-field", self.root)

    def test_unique_id_suffixes_a_collision(self):
        self.save()
        self.assertEqual(
            plan_store.unique_plan_id("north-field", self.root), "north-field-2"
        )
        self.save(plan_id="north-field-2")
        self.assertEqual(
            plan_store.unique_plan_id("north-field", self.root), "north-field-3"
        )

    def test_a_corrupt_plan_is_skipped_not_fatal(self):
        self.save()
        (self.root / "plans" / "broken").mkdir()
        (self.root / "plans" / "broken" / "meta.json").write_text("{not json")
        self.assertEqual(len(plan_store.list_plans(self.root)), 1)


class TestPathSafety(StoreTestCase):
    def test_traversal_ids_never_reach_the_filesystem(self):
        for bad in ("../../etc/passwd", "a/b", "..", "", "/abs", "x" * 200):
            with self.assertRaises(ValueError):
                plan_store.plan_dir(bad, self.root)
            with self.assertRaises(ValueError):
                plan_store.load_plan(bad, root=self.root)

    def test_plan_exists_is_false_rather_than_raising(self):
        self.assertFalse(plan_store.plan_exists("../escape", self.root))

    def test_view_ids_are_constrained(self):
        for bad in ("../x", "v", "vZZZ", "", "v0123456789abc", "0123456789ab"):
            with self.assertRaises(ValueError):
                plan_store.validate_view_id(bad)
        generated = plan_store.new_view_id()
        self.assertEqual(plan_store.validate_view_id(generated), generated)

    def test_tile_provider_names_are_sanitized(self):
        path = plan_store.tile_path("es/../ri", 3, 1, 2, self.root)
        self.assertNotIn("..", str(path))
        with self.assertRaises(ValueError):
            plan_store.tile_path("///", 3, 1, 2, self.root)


class TestCaps(StoreTestCase):
    def test_oversized_document_is_refused(self):
        plan = build_plan("big", "Big", POINTS)
        plan["generator"]["params"]["junk"] = "x" * (plan_store.MAX_PLAN_BYTES + 10)
        with self.assertRaises(ValueError) as ctx:
            plan_store.save_plan(plan, self.root)
        self.assertIn("byte limit", str(ctx.exception))

    def test_nan_never_reaches_disk(self):
        # json.dumps(allow_nan=False) refuses it; a NaN in a stored plan would
        # produce a file no other JSON reader can parse.
        plan = build_plan("p", "P", POINTS)
        plan["generator"]["derived"]["bad"] = float("nan")
        with self.assertRaises(ValueError):
            plan_store.save_plan(plan, self.root)

    def test_oversized_tile_is_refused(self):
        with self.assertRaises(ValueError):
            plan_store.write_tile(
                "esri", 1, 2, 3, b"x" * (plan_store.MAX_TILE_BYTES + 1), self.root
            )


class TestViews(StoreTestCase):
    def _view(self, view_id=None):
        return build_view(
            view_id=view_id or plan_store.new_view_id(),
            center_lat=47.3977, center_lon=8.5456, zoom=18,
            width_px=768, height_px=768, orientation="heading_up", heading_deg=72.4,
        )

    def test_saves_and_reloads_the_transform(self):
        view = self._view()
        plan_store.save_view(view, b"not-really-a-jpeg", self.root)
        loaded = plan_store.load_view(view.view_id, self.root)
        self.assertEqual(loaded.zoom, 18)
        self.assertAlmostEqual(loaded.rotation_deg, 72.4)
        self.assertEqual(loaded.pixel_to_lonlat(10, 20), view.pixel_to_lonlat(10, 20))

    def test_image_and_sidecar_sit_together(self):
        view = self._view()
        path = plan_store.save_view(view, b"bytes", self.root)
        self.assertEqual(path.suffix, ".jpg")
        self.assertTrue(plan_store.view_meta_path(view.view_id, self.root).is_file())

    def test_missing_view_explains_how_to_recover(self):
        with self.assertRaises(FileNotFoundError) as ctx:
            plan_store.load_view("v0123456789ab", self.root)
        self.assertIn("get_map_view", str(ctx.exception))

    def test_pruning_keeps_the_newest(self):
        for _ in range(5):
            plan_store.save_view(self._view(), b"x", self.root)
        removed = plan_store.prune_views(max_views=2, root=self.root)
        self.assertEqual(removed, 3)
        self.assertEqual(len(list((self.root / "views").glob("*.json"))), 2)
        self.assertEqual(len(list((self.root / "views").glob("*.jpg"))), 2)


class TestTiles(StoreTestCase):
    def test_round_trips_bytes(self):
        self.assertIsNone(plan_store.read_tile("esri", 18, 1, 2, self.root))
        plan_store.write_tile("esri", 18, 1, 2, b"tile", self.root)
        self.assertEqual(plan_store.read_tile("esri", 18, 1, 2, self.root), b"tile")

    def test_cache_is_keyed_by_provider_and_coordinate(self):
        plan_store.write_tile("esri", 18, 1, 2, b"a", self.root)
        self.assertIsNone(plan_store.read_tile("osm", 18, 1, 2, self.root))
        self.assertIsNone(plan_store.read_tile("esri", 18, 1, 3, self.root))

    def test_stats_report_what_is_cached(self):
        plan_store.write_tile("esri", 18, 1, 2, b"12345", self.root)
        stats = plan_store.tile_cache_stats(self.root)
        self.assertEqual(stats["tile_count"], 1)
        self.assertEqual(stats["bytes"], 5)

    def test_stats_on_an_empty_store(self):
        self.assertEqual(
            plan_store.tile_cache_stats(self.root), {"tile_count": 0, "bytes": 0}
        )


class TestFailClosed(unittest.TestCase):
    def test_status_err(self):
        self.assertEqual(
            plan_store.store_status_err("x"), {"status": "failed", "error": "x"}
        )


if __name__ == "__main__":
    unittest.main()
