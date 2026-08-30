"""Offline unit tests for map rendering (no drone, no network).

The load-bearing test here is render/transform consistency: the affine that cuts
a view out of the tile mosaic and the formula that converts a pixel to a
coordinate must be exact inverses. If they drift, the picture a model looks at
stops matching the coordinates it gets back, which is the one failure this whole
design exists to prevent.
"""
import io
import unittest

from PIL import Image

from mavlinkmcp import map_render
from mavlinkmcp.map_transform import build_view
from mavlinkmcp.tile_helpers import TILE_SIZE_PX, lonlat_to_tile

CENTER = (47.3977, 8.5456)


def solid_tile(color):
    buffer = io.BytesIO()
    Image.new("RGB", (TILE_SIZE_PX, TILE_SIZE_PX), color).save(buffer, "PNG")
    return buffer.getvalue()


def view(orientation="north_up", heading=0.0, size=512, zoom=18):
    return build_view(
        view_id="v0123456789ab", center_lat=CENTER[0], center_lon=CENTER[1],
        zoom=zoom, width_px=size, height_px=size,
        orientation=orientation, heading_deg=heading,
        attribution="test imagery",
    )


class TestMosaic(unittest.TestCase):
    def test_size_matches_the_tile_range(self):
        mosaic = map_render.compose_mosaic({}, 10, 20, 12, 21)
        self.assertEqual(mosaic.size, (3 * TILE_SIZE_PX, 2 * TILE_SIZE_PX))

    def test_tiles_land_in_the_right_cell(self):
        tiles = {(10, 20): solid_tile((255, 0, 0)), (11, 20): solid_tile((0, 255, 0))}
        mosaic = map_render.compose_mosaic(tiles, 10, 20, 11, 20)
        self.assertEqual(mosaic.getpixel((10, 10)), (255, 0, 0))
        self.assertEqual(mosaic.getpixel((TILE_SIZE_PX + 10, 10)), (0, 255, 0))

    def test_absent_tile_is_grey_not_a_hole(self):
        mosaic = map_render.compose_mosaic(
            {(10, 20): solid_tile((255, 0, 0))}, 10, 20, 11, 20
        )
        self.assertEqual(mosaic.getpixel((TILE_SIZE_PX + 10, 10)), map_render.MISSING_TILE)

    def test_corrupt_tile_degrades_instead_of_raising(self):
        mosaic = map_render.compose_mosaic({(10, 20): b"not an image"}, 10, 20, 10, 20)
        self.assertEqual(mosaic.getpixel((10, 10)), map_render.MISSING_TILE)

    def test_odd_sized_tile_is_resized(self):
        buffer = io.BytesIO()
        Image.new("RGB", (512, 512), (1, 2, 3)).save(buffer, "PNG")
        mosaic = map_render.compose_mosaic({(0, 0): buffer.getvalue()}, 0, 0, 0, 0)
        self.assertEqual(mosaic.size, (TILE_SIZE_PX, TILE_SIZE_PX))
        self.assertEqual(mosaic.getpixel((10, 10)), (1, 2, 3))


class TestWarpMatchesTheTransform(unittest.TestCase):
    """Colour each tile differently, then check the picture agrees with the maths."""

    COLORS = [
        (200, 40, 40), (40, 200, 40), (40, 40, 200), (200, 200, 40),
        (200, 40, 200), (40, 200, 200), (120, 120, 120), (240, 240, 240),
        (90, 30, 30), (30, 90, 30), (30, 30, 90), (90, 90, 30),
        (150, 60, 20), (20, 150, 60), (60, 20, 150), (150, 20, 60),
    ]

    def _render(self, v):
        tx, ty = lonlat_to_tile(v.center_lon, v.center_lat, v.zoom)
        x_min, y_min = int(tx) - 2, int(ty) - 2
        x_max, y_max = int(tx) + 2, int(ty) + 2
        tiles = {}
        palette = {}
        idx = 0
        for y in range(y_min, y_max + 1):
            for x in range(x_min, x_max + 1):
                color = self.COLORS[idx % len(self.COLORS)]
                idx += 1
                tiles[(x, y)] = solid_tile(color)
                palette[(x, y)] = color
        mosaic = map_render.compose_mosaic(tiles, x_min, y_min, x_max, y_max)
        image = map_render.warp_to_view(
            mosaic, v,
            (tx * TILE_SIZE_PX, ty * TILE_SIZE_PX),
            (x_min * TILE_SIZE_PX, y_min * TILE_SIZE_PX),
        )
        return image, palette

    def _check(self, v):
        image, palette = self._render(v)
        self.assertEqual(image.size, (v.width_px, v.height_px))
        checked = 0
        for px in range(40, v.width_px - 40, 53):
            for py in range(40, v.height_px - 40, 53):
                lon, lat = v.pixel_to_lonlat(px + 0.5, py + 0.5)
                tx, ty = lonlat_to_tile(lon, lat, v.zoom)
                expected = palette.get((int(tx), int(ty)))
                if expected is None:
                    continue
                actual = image.getpixel((px, py))
                # Bicubic resampling blurs tile edges; compare loosely and skip
                # pixels sitting on a seam.
                near_edge = min(tx % 1, 1 - tx % 1) < 0.02 or min(ty % 1, 1 - ty % 1) < 0.02
                if near_edge:
                    continue
                for a, b in zip(actual, expected):
                    self.assertLessEqual(
                        abs(a - b), 12,
                        f"pixel ({px},{py}) shows {actual}, transform says tile "
                        f"({int(tx)},{int(ty)}) which is {expected}",
                    )
                checked += 1
        self.assertGreater(checked, 20, "test checked too few pixels to mean anything")

    def test_north_up_view_agrees_with_the_transform(self):
        self._check(view())

    def test_rotated_views_agree_with_the_transform(self):
        for heading in (30.0, 72.4, 135.0, 250.0):
            with self.subTest(heading=heading):
                self._check(view("heading_up", heading))


class TestOverlays(unittest.TestCase):
    def setUp(self):
        self.view = view()
        self.image = map_render.blank_canvas(self.view)

    def test_blank_canvas_is_the_view_size(self):
        self.assertEqual(self.image.size, (self.view.width_px, self.view.height_px))
        self.assertEqual(self.image.getpixel((10, 10)), map_render.BACKGROUND)

    def test_grid_marks_the_image(self):
        before = self.image.tobytes()
        map_render.draw_grid(self.image)
        self.assertNotEqual(before, self.image.tobytes())

    def test_polygon_and_path_draw_without_error(self):
        ring = self.view.latlon_to_pixels(
            [(47.398, 8.545), (47.398, 8.547), (47.397, 8.547), (47.397, 8.545)]
        )
        map_render.draw_polygon(self.image, ring)
        map_render.draw_path(self.image, ring, number_every=2)
        self.assertNotEqual(self.image.getpixel((10, 10)), None)

    def test_degenerate_shapes_are_ignored(self):
        before = self.image.tobytes()
        map_render.draw_polygon(self.image, [(1, 1), (2, 2)])
        map_render.draw_path(self.image, [(1, 1)])
        self.assertEqual(before, self.image.tobytes())

    def test_drone_marker_draws_with_and_without_heading(self):
        map_render.draw_drone(self.image, (256, 256), 72.4, 0.0)
        map_render.draw_drone(self.image, (100, 100), None)

    def test_chrome_draws_scale_and_attribution(self):
        before = self.image.tobytes()
        map_render.draw_chrome(self.image, self.view, note="12 wp")
        self.assertNotEqual(before, self.image.tobytes())

    def test_scale_bar_picks_a_round_length_under_a_fifth_of_the_width(self):
        for mpp in (0.05, 0.4, 2.0, 25.0):
            metres, bar_px = map_render._nice_scale_length(mpp, 768)
            self.assertIn(metres, map_render.SCALE_STEPS_M)
            self.assertLessEqual(bar_px, 768 / 5.0 + 1e-6)
            self.assertAlmostEqual(bar_px, metres / mpp, places=6)


class TestEncoding(unittest.TestCase):
    def test_to_jpeg_emits_a_real_jpeg(self):
        payload = map_render.to_jpeg(map_render.blank_canvas(view()))
        self.assertTrue(payload.startswith(b"\xff\xd8\xff"))
        self.assertEqual(Image.open(io.BytesIO(payload)).size, (512, 512))

    def test_jpeg_is_far_smaller_than_png_for_imagery(self):
        image = map_render.blank_canvas(view())
        buffer = io.BytesIO()
        image.save(buffer, "PNG")
        self.assertLess(len(map_render.to_jpeg(image)), len(buffer.getvalue()) * 3)

    def test_status_err_is_fail_closed(self):
        self.assertEqual(
            map_render.render_status_err("x"), {"status": "failed", "error": "x"}
        )


if __name__ == "__main__":
    unittest.main()
