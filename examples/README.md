# MAVLinkMCP Example Agent

This example demonstrates how to use the Human Input tool with a FastAgent to control a drone.

## About MCP and MAVSDK

**Model Context Protocol (MCP)** is an open protocol for connecting AI agents to external tools and environments. It enables agents to interact with servers via a standardized interface, making it easy to extend agent capabilities with new tools and APIs.  
Learn more at [modelcontextprotocol.io/introduction](https://modelcontextprotocol.io/introduction).

**MAVSDK** is a modern, easy-to-use library for communicating with MAVLink-compatible drones (such as those running PX4). It provides a high-level API for drone control, telemetry, missions, and more.  
See [mavsdk.mavlink.io](https://mavsdk.mavlink.io/main/en/) for details.

**This library is an MCP server that wraps MAVSDK, exposing drone control and telemetry as MCP tools.** This allows AI agents (like FastAgent) to control drones and access their data using natural language or programmatic requests.

## Prerequisites

Before running the example, you must create a `fastagent.secrets.yaml` file in the `examples/` directory with your API keys.  
**Note:** The YAML below is just an example—include only the keys you actually use.

```yaml
openai:
    api_key: <your-api-key>
anthropic:
    api_key: <your-api-key-here>
```

Replace `<your-api-key>` and `<your-api-key-here>` with your actual API keys.

**Never commit your secrets file to version control.**

## Model Configuration

The model is set in `fastagent.config.yaml`.  
By default, this setup uses the OpenAI model (`gpt-4o-mini`).  
If you want to use a different model or provider, change the `default_model` field in the config file accordingly.

Example (`fastagent.config.yaml`):
```yaml
default_model: gpt-4o-mini  # Change this if you want to use another model
```

## Running the Example

You can run the example agent script using either of the following commands from the **repository root** (the script lives at `examples/example_agent.py`, not the repo root):

```sh
python examples/example_agent.py
```
or
```sh
uv run examples/example_agent.py
```

Start the MCP server separately (or via your FastAgent server config) with `python src/server/mavlinkmcp.py`. For PX4 SITL, the server defaults to UDP port **14540** (`MAVLINK_PORT`).

Make sure all dependencies are installed and your environment is properly configured.

After running `example_agent.py`, you can start chatting with an agent that can control your drone.

## Example Prompts

Here are some example prompts you can use with the agent:

- **"Arm the drone."**
- **"Take off to 5 meters altitude."**
- **"Move the drone 2 meters forward and 1 meter to the right."**
- **"What is the current position of the drone?"**
- **"Land the drone."**
- **"Show me the latest IMU data."**
- **"Start a mission with these waypoints: [list of coordinates]."**
- **"What is the current flight mode?"**
- **"Print the drone's status text."**

The agent will translate your requests into appropriate MAVSDK commands via the MCP server.

---

For more information about MCP, visit [modelcontextprotocol.io](https://modelcontextprotocol.io/introduction).  
For more about MAVSDK, see [mavsdk.mavlink.io](https://mavsdk.mavlink.io/main/en/).
