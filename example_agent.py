"""
Agent which demonstrates Human Input tool
"""

import asyncio

from mcp_agent.core.fastagent import FastAgent

# Create the application
fast = FastAgent("Human Input")


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