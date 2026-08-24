# MAVLinkMCP Example Agent

This example demonstrates how to use the Human Input tool with a FastAgent to control a drone.

## About MCP and MAVSDK

**Model Context Protocol (MCP)** is an open protocol for connecting AI agents to external tools and environments. It enables agents to interact with servers via a standardized interface, making it easy to extend agent capabilities with new tools and APIs.  
Learn more at [modelcontextprotocol.io/introduction](https://modelcontextprotocol.io/introduction).

**MAVSDK** is a modern, easy-to-use library for communicating with MAVLink-compatible drones (such as those running PX4). It provides a high-level API for drone control, telemetry, missions, and more.  
See [mavsdk.mavlink.io](https://mavsdk.mavlink.io/main/en/) for details.

**This library is an MCP server that wraps MAVSDK, exposing drone control and telemetry as MCP tools.** This allows AI agents (like FastAgent) to control drones and access their data using natural language or programmatic requests.

## Prerequisites

Before running the example, you must create a `fast-agent.secrets.yaml` file in the `examples/` directory with your API keys.  
**Note:** The YAML below is just an example—include only the keys you actually use.

```yaml
openai:
    api_key: <your-api-key>
anthropic:
    api_key: <your-api-key-here>
google:
    api_key: <your-api-key-here>   # required for Gemini models
```

Replace `<your-api-key>` and `<your-api-key-here>` with your actual API keys.

**Never commit your secrets file to version control.**

## Model Configuration

The model is set in `fast-agent.yaml`.  
By default, this setup uses `gemini-2.5-flash`.  
If you want to use a different model or provider, change the `default_model` field in the config file accordingly.

Example (`fast-agent.yaml`):
```yaml
default_model: gemini-2.5-flash  # Change this if you want to use another model
```

You can use either a short alias or a full model name. The aliases accepted by the
installed version of `fast-agent-mcp` are:

| Alias | Resolves to |
|---|---|
| `gemini` | `gemini-3.1-pro-preview` |
| `gemini2` | `gemini-2.0-flash` |
| `gemini25` | `gemini-2.5-flash` |
| `gemini25pro` | `gemini-2.5-pro` |
| `gemini3` | `gemini-3-pro-preview` |
| `gemini3flash` | `gemini-3-flash-preview` |
| `gemini3.1` / `gemini31pro` | `gemini-3.1-pro-preview` |
| `gemini3.1flashlite` | `gemini-3.1-flash-lite-preview` |
| `gemini35` / `gemini35flash` | `gemini-3.5-flash` |

Full names work too (`gemini-2.5-flash`, `gemini-3-pro-preview`, …), and you can force
the provider with a prefix: `google.gemini-2.5-flash` uses the native Google API, while
`googleoai.gemini-2.5-flash` goes through Google's OpenAI-compatible endpoint.

Unknown names are rejected at startup with `Unknown model or provider`, so a typo such as
`gemini-3.7-flash` fails immediately rather than at the first request. To list what your
installed version accepts:

```sh
uv run python -c "from fast_agent.llm.model_database import ModelDatabase; print([m for m in ModelDatabase.list_models() if 'gemini' in m])"
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

Start the MCP server separately (or via your FastAgent server config) with `mavlinkmcp`. For PX4 SITL, the server defaults to UDP port **14540** (`MAVLINK_PORT`).

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
