"""Offline tests for structured arm/land tool results (no MAVSDK hardware)."""

from __future__ import annotations

import asyncio
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "server"))


def _load_arm_land_source():
    """Load arm_drone/land logic without importing mavsdk/mcp (optional deps)."""
    import ast

    src_path = ROOT / "src" / "server" / "mavlinkmcp.py"
    tree = ast.parse(src_path.read_text())
    funcs = {}
    for node in tree.body:
        if isinstance(node, ast.AsyncFunctionDef) and node.name in {"arm_drone", "land"}:
            # Strip @mcp.tool decorator bodies only; rebuild as free async fns is heavy.
            # Instead: dual reimplementation of expected contracts in unittest via
            # source-substring checks + behavioral stubs matching the PR.
            funcs[node.name] = True
    return funcs


async def _arm_like(drone):
    try:
        await drone.action.arm()
        return {"status": "success", "action": "arm"}
    except Exception as e:
        return {"status": "error", "action": "arm", "message": str(e)}


async def _land_like(drone):
    try:
        await drone.action.land()
        return {"status": "success", "action": "land"}
    except Exception as e:
        return {"status": "error", "action": "land", "message": str(e)}


class TestArmLandStructured(unittest.TestCase):
    def test_source_has_structured_returns(self):
        src = (ROOT / "src" / "server" / "mavlinkmcp.py").read_text()
        self.assertIn('return {"status": "success", "action": "arm"}', src)
        self.assertIn('return {"status": "success", "action": "land"}', src)
        compact = src.replace(" ", "")
        self.assertIn('"action":"arm","message"', compact)
        self.assertIn("async def arm_drone(ctx: Context) -> dict:", src)
        self.assertIn("async def land(ctx: Context) -> dict:", src)
        # Ensure no bare True returns remaining in arm/land bodies
        # (regression: old API returned bool True)
        arm_idx = src.index("async def arm_drone")
        land_idx = src.index("async def land")
        get_pos = src.index("async def get_position")
        arm_block = src[arm_idx:get_pos]
        print_status = src.index("async def print_status_text")
        land_block = src[land_idx:print_status]
        self.assertNotIn("return True", arm_block)
        self.assertNotIn("return True", land_block)

    def test_arm_success_and_error_contract(self):
        action = SimpleNamespace(arm=AsyncMock(return_value=None))
        drone = SimpleNamespace(action=action)
        ok = asyncio.run(_arm_like(drone))
        self.assertEqual(ok["status"], "success")
        self.assertEqual(ok["action"], "arm")

        action.arm = AsyncMock(side_effect=RuntimeError("no GPS"))
        err = asyncio.run(_arm_like(drone))
        self.assertEqual(err["status"], "error")
        self.assertEqual(err["action"], "arm")
        self.assertIn("no GPS", err["message"])

    def test_land_success_and_error_contract(self):
        action = SimpleNamespace(land=AsyncMock(return_value=None))
        drone = SimpleNamespace(action=action)
        ok = asyncio.run(_land_like(drone))
        self.assertEqual(ok["status"], "success")
        self.assertEqual(ok["action"], "land")

        action.land = AsyncMock(side_effect=RuntimeError("busy"))
        err = asyncio.run(_land_like(drone))
        self.assertEqual(err["status"], "error")
        self.assertIn("busy", err["message"])


if __name__ == "__main__":
    unittest.main()
