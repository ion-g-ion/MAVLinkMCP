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
| `MAVLINK_ADDRESS` | empty string | Peer host to send to. Empty means *listen* instead. |
| `MAVLINK_PORT` | `14540` | UDP port (PX4 SITL commonly uses **14540**) |
| `MAVLINK_CONNECT_TIMEOUT` | `60` | Seconds to keep dialling the vehicle before declaring it unreachable |

`MAVLINK_ADDRESS` decides the direction of the link:

- **Empty (default)** → `udpin://0.0.0.0:$MAVLINK_PORT`. The server binds the
  port and waits. This is what PX4 SITL needs: SITL *sends* to 14540 and
  expects a listener on the other end.
- **Set to a host** → `udpout://$MAVLINK_ADDRESS:$MAVLINK_PORT`. The server
  sends to a peer already listening there, e.g. a companion link on real
  hardware.

Example for local PX4:

```bash
export MAVLINK_PORT=14540
# MAVLINK_ADDRESS can stay empty: SITL talks to us, so we listen
```

### Link bring-up is non-blocking

The MCP handshake never waits on the vehicle. The session starts immediately
and the link comes up in the background, so tools report the link state instead
of hanging:

```json
{"status": "failed", "error": "MAVLink link is still coming up; retry shortly", "connected": false, "link_state": "connecting"}
```

Once the autopilot answers, tools switch to real results. If nothing answers
within `MAVLINK_CONNECT_TIMEOUT`, the failure becomes definitive and names the
address that was tried:

```json
{"status": "failed", "error": "no MAVLink vehicle reachable on udpin://0.0.0.0:14540 after 60s", "connected": false, "link_state": "failed"}
```

Waiting for a GPS/home position estimate is deliberately *not* part of this
gate — it is logged, but a converging position fix never blocks commands.

> **One vehicle link per session.** On the HTTP transports each MCP session
> opens its own MAVSDK connection, so a second concurrent client cannot bind
> the same UDP port and will see the fail-closed payload above.

## Usage

Installing the project puts a `mavlinkmcp` console script in your environment.
It serves MCP over stdio:

```bash
mavlinkmcp
# equivalently
python -m mavlinkmcp
```

### Launching from another working directory

Point your MCP client at the console script by **absolute path**. Its shebang
names the project interpreter, so it carries its own environment and does not
care where it is launched from:

```bash
/abs/path/to/MAVLinkMCP/.venv/bin/mavlinkmcp
```

This is the form to put in a chat app's stdio MCP server config.

Note that `uv run` resolves the project from your *current* directory, not from
the path you hand it — so `uv run /abs/path/to/src/mavlinkmcp/server.py` from
elsewhere builds an environment without this project's dependencies and fails
on import. Name the project explicitly if you want to go through uv:

```bash
uv run --project /abs/path/to/MAVLinkMCP mavlinkmcp
uv --directory /abs/path/to/MAVLinkMCP run mavlinkmcp
```

### Running over HTTP

The same tools can be served over a network socket instead of stdio:

```bash
# current MCP HTTP transport, on http://127.0.0.1:8000/mcp
mavlinkmcp --transport streamable-http

# pick the socket and endpoint path
mavlinkmcp --transport streamable-http --host 127.0.0.1 --port 9000 --path /drone
```

| Option | Applies to | Default | Meaning |
|--------|-----------|---------|---------|
| `--transport` | all | `stdio` | `stdio`, `streamable-http`, or `sse` |
| `--host` | HTTP | `127.0.0.1` | Bind address |
| `--port` | HTTP | `8000` | Bind port |
| `--path` | `streamable-http` | `/mcp` | Endpoint path |
| `--allowed-host` | HTTP | — | `Host` header to accept; repeatable, wildcards allowed |
| `--allow-any-host` | HTTP | off | Turn off `Host`/`Origin` checking entirely |

`sse` is the older transport, deprecated in the MCP spec. Prefer
`streamable-http` unless you have a client that needs `sse`. The HTTP options
are rejected under `--transport stdio` rather than silently ignored.

#### Binding beyond localhost

On a localhost bind, DNS-rebinding protection (`Host`/`Origin` validation) is on
automatically. Off localhost it cannot be inferred, so you must say what to
accept — the server refuses to start otherwise:

```bash
# declare the Host headers clients will send
mavlinkmcp --transport streamable-http --host 0.0.0.0 --port 8000 \
    --allowed-host drone.lan:8000

# or turn the check off deliberately
mavlinkmcp --transport streamable-http --host 0.0.0.0 --allow-any-host
```

A request whose `Host` is not on the list is answered `421 Misdirected Request`.

#### Before you expose it

There is **no authentication on the HTTP transports.** An open port here is a
port that can arm, take off, and move a vehicle — a materially larger exposure
than a stdio pipe that only the local client can talk to. Keep it on localhost,
or put it behind a network you control plus your own auth layer.

Note also that the MAVSDK connection is established **per MCP session**, not per
process. Over stdio that is one connection per client process. Over HTTP, every
session that connects opens its own link to the same vehicle, with nothing
arbitrating between them — two clients means two independent command sources to
one airframe. The server binds immediately either way; it does not wait for a
vehicle at startup.

## Example agent usage

See `examples/README.md` and run:

```bash
python examples/example_agent.py
# or
uv run examples/example_agent.py
```

Export your LLM provider key as documented under `examples/` (never commit secrets). The example uses `fast-agent-mcp` / FastAgent and the `mavlink_mcp` server entry.

### Safety note

MCP tools can arm, take off, and move a vehicle. Prefer SITL. Keep a human ready to kill switch / land. Tool failures should be treated as fail-closed by the client.

## Tests

The unit tests are offline — they exercise the pure validation/normalization
helpers and do not import `mavsdk` or talk to a vehicle. Run them from the
repository root:

```bash
python -m unittest discover
```

Tests import the helpers by their real package paths
(`from mavlinkmcp.endpoint import ...`), so the project must be installed in the
environment you run them with (`uv sync` or `pip install -e .`).

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
