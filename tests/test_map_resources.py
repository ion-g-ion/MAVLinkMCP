"""Offline unit tests for the map resource payloads (no drone, no network).

The resources are read-only descriptions of what is already on disk, so these
tests assert two things above all: that a misconfiguration is *reported* rather
than raised, and that an API key never reaches a payload every client can read.
"""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from mavlinkmcp import map_resources, map_source, map_view, plan_store, tile_helpers
from mavlinkmcp.map_transform import build_view


class ResourceTestCase(unittest.TestCase):
    """Every test runs against a temporary store, never the operator's own."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def view(self, view_id=None, **kw):
        params = {
            "center_lat": 47.3977,
            "center_lon": 8.5456,
            "zoom": 18,
            "width_px": 768,
            "height_px": 768,
            "provider": "esri",
            "attribution": "Imagery (c) Esri",
            "created_at": plan_store.utc_now_iso(),
        }
        params.update(kw)
        return build_view(view_id=view_id or plan_store.new_view_id(), **params)


class TestRedaction(unittest.TestCase):
    def test_strips_the_query_string(self):
        self.assertEqual(
            map_resources.redact_tile_url("https://h/{z}/{x}/{y}.png?key=s3cret"),
            "https://h/{z}/{x}/{y}.png?<redacted>",
        )

    def test_leaves_a_url_without_a_query_alone(self):
        url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
        self.assertEqual(map_resources.redact_tile_url(url), url)

    def test_empty_stays_empty(self):
        self.assertEqual(map_resources.redact_tile_url(""), "")
        self.assertEqual(map_resources.redact_tile_url(None), "")

    def test_redacts_a_placeholder_query_too(self):
        # Telling a live key from a {key} placeholder is guesswork, and guessing
        # wrong publishes a credential, so the query goes either way.
        catalog = map_resources.provider_catalog()["providers"]
        self.assertNotIn("access_token", catalog["mapbox"]["tile_url"])
        self.assertTrue(catalog["mapbox"]["tile_url"].endswith("?<redacted>"))


class TestProviderCatalog(unittest.TestCase):
    def test_lists_every_known_provider_plus_custom(self):
        catalog = map_resources.provider_catalog()["providers"]
        for name in map_source.PROVIDERS:
            self.assertIn(name, catalog)
        self.assertIn("custom", catalog)

    def test_never_emits_the_api_key(self):
        with mock.patch.dict(
            os.environ,
            {
                "MAVLINKMCP_MAP_PROVIDER": "mapbox",
                "MAVLINKMCP_MAP_API_KEY": "pk.supersecret",
            },
        ):
            payload = map_resources.provider_catalog()
        self.assertNotIn("pk.supersecret", json.dumps(payload))
        self.assertTrue(payload["active"]["api_key_configured"])
        self.assertNotIn("api_key", payload["active"])

    def test_reports_misconfiguration_instead_of_raising(self):
        # Reading this resource is how a client learns the map layer is broken;
        # raising would hide the very answer it came for.
        with mock.patch.dict(
            os.environ,
            {"MAVLINKMCP_MAP_PROVIDER": "mapbox", "MAVLINKMCP_MAP_API_KEY": ""},
        ):
            payload = map_resources.provider_catalog()
        self.assertIn("error", payload["active"])
        self.assertIn("MAVLINKMCP_MAP_API_KEY", payload["active"]["error"])

    def test_unknown_provider_is_reported(self):
        with mock.patch.dict(os.environ, {"MAVLINKMCP_MAP_PROVIDER": "nope"}):
            payload = map_resources.provider_catalog()
        self.assertIn("error", payload["active"])

    def test_blank_provider_says_there_is_no_imagery(self):
        with mock.patch.dict(os.environ, {"MAVLINKMCP_MAP_PROVIDER": "none"}):
            payload = map_resources.provider_catalog()
        self.assertFalse(payload["active"]["serves_tiles"])
        self.assertIn("blank canvas", payload["active"]["note"])

    def test_custom_template_query_is_redacted(self):
        with mock.patch.dict(
            os.environ,
            {
                "MAVLINKMCP_MAP_PROVIDER": "custom",
                "MAVLINKMCP_MAP_TILE_URL": "https://tiles.lan/{z}/{x}/{y}.png?key=abc123",
            },
        ):
            payload = map_resources.provider_catalog()
        self.assertNotIn("abc123", json.dumps(payload))


class TestLimits(unittest.TestCase):
    def test_quotes_the_limits_actually_enforced(self):
        limits = map_resources.map_limits()
        self.assertEqual(limits["view"]["size_px"]["max"], map_view.MAX_SIZE_PX)
        self.assertEqual(limits["view"]["radius_m"]["min"], map_view.MIN_RADIUS_M)
        self.assertEqual(limits["zoom"]["max"], tile_helpers.MAX_ZOOM)
        self.assertEqual(
            limits["tiles"]["max_per_view"], map_source.MAX_TILES_PER_REQUEST
        )
        self.assertEqual(
            limits["tiles"]["max_per_prefetch"], map_source.MAX_PREFETCH_TILES
        )
        self.assertEqual(limits["store"]["max_plans"], plan_store.MAX_PLANS)


class TestCacheReport(ResourceTestCase):
    def test_counts_tiles_and_views(self):
        plan_store.write_tile("esri", 18, 1, 2, b"tile-bytes", self.root)
        plan_store.save_view(self.view(), b"jpeg", self.root)
        with mock.patch.dict(
            os.environ, {"MAVLINKMCP_PLANS_DIR": str(self.root)}
        ):
            payload = map_resources.cache_report()
        self.assertEqual(payload["tiles"]["tile_count"], 1)
        self.assertEqual(payload["views"]["count"], 1)
        self.assertEqual(payload["paths"]["root"], str(self.root))

    def test_reports_a_bad_store_path_rather_than_raising(self):
        with mock.patch.dict(
            os.environ, {"MAVLINKMCP_PLANS_DIR": "relative/path"}
        ):
            payload = map_resources.cache_report()
        self.assertIn("error", payload["paths"])


class TestViewIndex(ResourceTestCase):
    def test_empty_store_lists_nothing(self):
        payload = map_resources.view_index(root=self.root)
        self.assertEqual(payload["views"], [])
        self.assertEqual(payload["count"], 0)

    def test_summarises_a_stored_view(self):
        view = self.view()
        plan_store.save_view(view, b"jpeg", self.root)
        payload = map_resources.view_index(root=self.root)
        self.assertEqual(payload["count"], 1)
        entry = payload["views"][0]
        self.assertEqual(entry["view_id"], view.view_id)
        self.assertEqual(entry["zoom"], 18)
        self.assertTrue(entry["has_image"])

    def test_reports_a_view_whose_image_is_gone(self):
        view = self.view()
        plan_store.save_view(view, b"jpeg", self.root)
        plan_store.view_image_path(view.view_id, self.root).unlink()
        entry = map_resources.view_index(root=self.root)["views"][0]
        self.assertFalse(entry["has_image"])

    def test_limit_caps_the_listing(self):
        for _ in range(4):
            plan_store.save_view(self.view(), b"jpeg", self.root)
        self.assertEqual(map_resources.view_index(2, self.root)["count"], 2)


class TestViewDetail(ResourceTestCase):
    def test_georeference_round_trips(self):
        view = self.view()
        plan_store.save_view(view, b"jpeg", self.root)
        payload = map_resources.view_detail(view.view_id, self.root)
        self.assertEqual(payload["bbox"], view.bbox())
        self.assertEqual(payload["up_is"], "north")
        corners = payload["corners_latlon"]
        self.assertEqual(corners["top_left"], list(view.corners_latlon()[0]))
        self.assertEqual(len(corners), 4)

    def test_heading_up_view_names_its_bearing(self):
        view = self.view(orientation="heading_up", heading_deg=72.4)
        plan_store.save_view(view, b"jpeg", self.root)
        payload = map_resources.view_detail(view.view_id, self.root)
        self.assertIn("72", payload["up_is"])

    def test_missing_view_raises_with_a_way_out(self):
        with self.assertRaises(FileNotFoundError) as ctx:
            map_resources.view_detail("v0123456789ab", self.root)
        self.assertIn("get_map_view", str(ctx.exception))

    def test_malformed_id_is_refused(self):
        with self.assertRaises(ValueError):
            map_resources.view_detail("../etc/passwd", self.root)


class TestViewImage(ResourceTestCase):
    def test_returns_the_stored_bytes(self):
        view = self.view()
        plan_store.save_view(view, b"jpeg-bytes", self.root)
        self.assertEqual(
            map_resources.view_image(view.view_id, self.root), b"jpeg-bytes"
        )

    def test_pruned_image_raises_with_a_way_out(self):
        with self.assertRaises(FileNotFoundError) as ctx:
            map_resources.view_image("v0123456789ab", self.root)
        self.assertIn("get_map_view", str(ctx.exception))

    def test_malformed_id_is_refused(self):
        with self.assertRaises(ValueError):
            map_resources.view_image("not-an-id", self.root)


class TestListViews(ResourceTestCase):
    """``plan_store.list_views`` underpins the index resource."""

    def test_newest_first(self):
        first = self.view()
        plan_store.save_view(first, b"a", self.root)
        second = self.view()
        plan_store.save_view(second, b"b", self.root)
        ids = [v.view_id for v in plan_store.list_views(root=self.root)]
        self.assertEqual(ids[0], second.view_id)
        self.assertIn(first.view_id, ids)

    def test_skips_an_unreadable_sidecar(self):
        good = self.view()
        plan_store.save_view(good, b"a", self.root)
        (self.root / "views" / "vdeadbeefdead.json").write_text("{not json")
        views = plan_store.list_views(root=self.root)
        self.assertEqual([v.view_id for v in views], [good.view_id])

    def test_missing_directory_is_empty_not_an_error(self):
        self.assertEqual(plan_store.list_views(root=self.root / "nope"), [])


if __name__ == "__main__":
    unittest.main()
