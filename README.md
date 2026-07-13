# MAVLink MCP Server

Python [Model Context Protocol (MCP)](https://modelcontextprotocol.io/) server for LLM agents talking to MAVLink-enabled vehicles (typically PX4 via [MAVSDK](https://mavsdk.mavlink.io/)).

## Prerequisites

- Python 3.10 or higher
- A MAVLink endpoint (PX4 SITL is the recommended first path; do not point an untrusted agent at a live airframe without a human in the loop)

## Installation

1. Clone the repository:

```bash
git clone https://github.com/ion-g-ion/MAVLinkMCP.git
cd MAVLinkMCP
```

2. Install the project (this repo ships `pyproject.toml` and a `uv.lock`; there is no root `requirements.txt`):

```bash
# with pip (editable)
pip install -e .

# or with uv
uv sync
```

## Configuration (SITL / connection)

The server reads:

| Env var | Default | Meaning |
|---------|---------|---------|
| `MAVLINK_ADDRESS` | empty string | Host part of MAVSDK `udp://` URL |
| `MAVLINK_PORT` | `14540` | UDP port (PX4 SITL commonly uses **14540**) |

Example for local PX4 SITL:

```bash
export MAVLINK_PORT=14540
# MAVLINK_ADDRESS can stay empty for default local UDP
```

## Usage

Run the MCP server (stdio transport):

```bash
python src/server/mavlinkmcp.py
```

or:

```bash
uv run src/server/mavlinkmcp.py
```

## Example agent usage

See `examples/README.md` and run:

```bash
python examples/example_agent.py
# or
uv run examples/example_agent.py
```

Export your LLM provider key as documented under `examples/` (never commit secrets). The example uses `mcp-agent` / FastAgent and the `mavlink_mcp` server entry.

### Safety note

MCP tools can arm, take off, and move a vehicle. Prefer SITL. Keep a human ready to kill switch / land. Tool failures should be treated as fail-closed by the client.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
