"""Pure MCP tool result dictionaries (no MAVSDK import)."""


def tool_ok(payload=None):
    """Structured success dictionary for MCP tools."""
    out = {"status": "success"}
    if payload is not None:
        out.update(payload)
    return out


def tool_err(message):
    """Structured failure dictionary for MCP tools (fail-closed)."""
    return {"status": "failed", "error": str(message)}
