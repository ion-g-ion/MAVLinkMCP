"""Test package for offline MAVLink MCP unit tests.

The server is launched as a script (`python src/server/mavlinkmcp.py`), so its
modules import each other flatly (`from endpoint import ...`). Put `src/server`
on sys.path so the tests import them exactly the way the server does.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "server"))
