"""
Agent which demonstrates Human Input tool
"""

import asyncio
from pathlib import Path

from fast_agent import FastAgent

# fast-agent discovers config in the home dir and the *current working directory*
# only — it does not walk parent (or child) directories. Anchoring the path to this
# file keeps `python examples/example_agent.py` working from the repository root.
# fast-agent.secrets.yaml is picked up from the same directory automatically.
CONFIG_PATH = Path(__file__).parent / "fast-agent.yaml"

# Create the application
fast = FastAgent("Human Input", config_path=str(CONFIG_PATH))


# Define the agent
# Just the role. The operating rules are NOT restated here -- they ship with the
# server, in its MCP `instructions` and in the tool descriptions themselves, so
# they reach whichever client connects rather than only this one.
@fast.agent(
    instruction=(
        "You are an agent that flies a drone over MAVLink, on behalf of a human "
        "operator who is watching."
    ),
    human_input=True,
    servers=["mavlink_mcp"],
)
async def main() -> None:
    async with fast.run() as agent:
        # No hardcoded opener: the first message is whatever the user types at the
        # interactive prompt, so the agent stays idle until asked for something.
        # Pressing Enter on an empty line sends "STOP", which ends the session.
        await agent.prompt(default_prompt="STOP")


if __name__ == "__main__":
    asyncio.run(main())
