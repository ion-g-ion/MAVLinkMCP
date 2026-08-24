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
@fast.agent(
    instruction="An AI agent that controls a drone.",
    human_input=True,
    servers=["mavlink_mcp"],
)
async def main() -> None:
    async with fast.run() as agent:
        # this usually causes the LLM to request the Human Input Tool
        await agent("Start a conversation.")
        await agent.prompt(default_prompt="STOP")


if __name__ == "__main__":
    asyncio.run(main())
