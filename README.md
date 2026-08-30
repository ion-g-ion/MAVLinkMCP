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

## Flight plans

The server can generate, store, check and fly coverage missions. The point of
the split is that a route becomes reviewable *before* it reaches the vehicle:

```
get_map_view -> create_survey_plan -> render_plan_view
  -> validate_plan -> preflight_check -> upload_plan
  -> verify_uploaded_plan -> start_mission
```

### Seeing the ground

`get_map_view` returns a georeferenced satellite view as an **image** together
with its exact transform. Identify a feature in the picture, report its corners
as **pixels**, and let the server convert them — a vision model is good at
pointing at a field and bad at inventing latitudes, so it never has to:

```
get_map_view()                       # centred on the vehicle
  -> [image, {view_id, center, meters_per_pixel, drone: {pixel, heading_deg}, ...}]

create_survey_plan(name="Front Field", view_id=..., polygon_pixels=[[430,180], ...],
                   altitude_m=40, camera={...})
```

`map_transform` converts in either direction against a stored view. Pass
`orientation="heading_up"` to rotate the view so the vehicle's forward direction
is up, which makes "the field in front of the drone" a question about the
picture rather than about compass arithmetic.

Supplying `latitude_deg`/`longitude_deg` instead looks anywhere and needs no
vehicle at all — plans can be authored and costed at a desk and flown later.

### Map imagery

Fetching a basemap is the **only** outbound network use in this server. Tiles
are fetched on demand, cached to disk, and pinned to the configured provider's
host; a tile that fails renders as flat grey and the response says so, because
the georeferencing is still exact.

| Variable | Meaning |
|---|---|
| `MAVLINKMCP_MAP_PROVIDER` | `esri` (default, global aerial imagery, no key) · `osm` (street map, not imagery) · `mapbox` / `maptiler` (need a key) · `custom` · `none` |
| `MAVLINKMCP_MAP_API_KEY` | key for the providers that need one |
| `MAVLINKMCP_MAP_TILE_URL` | XYZ template for `custom`, e.g. a self-hosted tile server |
| `MAVLINKMCP_MAP_IMAGE_MODE` | `image` (default) or `path`, for clients that cannot display images |

`none` disables all network access and renders a correctly georeferenced blank
canvas — geometry and overlays still work, there is just nothing to identify
ground features from. `prefetch_map_area` warms the tile cache before going
somewhere without connectivity.

**Attribution is not optional.** Each provider's terms require it; the notice is
drawn onto every view and returned in the payload.

Views are rendered as JPEG, 768 px by default and capped at 1024. That ceiling is
about tokens rather than bytes: image cost scales with pixel area, so a 768 px
view is roughly 790 tokens and a 1024 px one about 1400.

### Where plans live

```
$MAVLINKMCP_PLANS_DIR/          # default $XDG_DATA_HOME/mavlinkmcp
├── plans/north-field/
│   ├── meta.json               # name, head revision, timestamps
│   ├── rev-001.json
│   └── rev-002.json
├── views/                      # rendered map views + their geotransforms
└── tiles/                      # tile cache
```

Plans are plain JSON and outlive the MCP session, so they can be read, diffed
and edited by a human without this server running.

A revision's **generated content is immutable**: `revise_plan` re-runs the
generator with new parameters and writes a new revision rather than editing
waypoints, so the stored parameters and the stored path can never disagree.
Only the lifecycle annotations (`status`, `checks`, `estimate`, `uploaded`)
change in place.

### The upload gate

A plan moves `draft -> validated -> uploaded`, and **`upload_plan` refuses
anything that has not passed `validate_plan`**. Any revision starts as a draft,
so a change always invalidates the previous check.

`validate_plan` reports findings; only an `error` blocks. The most valuable one
is `FAR_FROM_HOME` — a polygon drawn on the wrong map produces a perfectly
well-formed plan on the other side of the world, and distance from home is what
catches it. `preflight_check` then adds a live go/no-go from health, GPS fix,
battery and landed state, and `verify_uploaded_plan` downloads the mission back
off the vehicle and diffs it against what was reviewed.

Note that the plan's lifecycle state is reported as `plan_status`. The `status`
key is the call envelope (`success` / `failed`) that every tool in this server
returns, and it stays that.

### Survey geometry

`create_survey_plan` generates a boustrophedon (lawnmower) sweep. Line spacing
comes from either `line_spacing_m` or a `camera`; giving both is refused rather
than silently preferring one. A camera also sets the photo trigger distance and
reports ground sample distance:

```python
camera = {"sensor_width_mm": 13.2, "focal_length_mm": 8.8,
          "image_width_px": 5472, "image_height_px": 3648,
          "front_overlap": 0.75, "side_overlap": 0.65}
# at 40 m: 1.10 cm/px, 60 m footprint, 21 m line spacing, 10 m trigger distance
```

Omitting `sweep_angle_deg` sweeps along the area's long axis, which minimises
turns — where survey time and battery actually go.

### Safety note

MCP tools can arm, take off, and move a vehicle. Prefer SITL. Keep a human ready to kill switch / land. Tool failures should be treated as fail-closed by the client.

Flight plans add a review step rather than removing the need for one: `validate_plan` and `preflight_check` catch the mistakes that are mechanical (a route in the wrong place, no GPS fix, not enough battery), not the ones that are a matter of judgement. Look at `render_plan_view` before you fly.

## Tests

The unit tests are offline — they exercise the pure validation, geometry and
normalization helpers, never import `mavsdk`, never talk to a vehicle, and never
make a network request. Tile fetching is exercised by injecting a fake fetcher,
and the rendering path runs under `MAVLINKMCP_MAP_PROVIDER=none`. Run them from
the repository root:

```bash
python -m unittest discover
```

Tests import the helpers by their real package paths
(`from mavlinkmcp.endpoint import ...`), so the project must be installed in the
environment you run them with (`uv sync` or `pip install -e .`).

### SITL integration tests

`tests_sitl/` covers what the offline suite structurally cannot: the MAVSDK
handshake, the connection guard, and the mission-protocol round-trip. These need
a real autopilot, so `sitl/` builds a PX4 image running the SIH dynamics model —
no Gazebo, no GPU, ~120 MB, ~7 MB of RAM:

```bash
docker build -t px4-sih:v1.14.3 sitl/
docker run -d --rm --name px4 --network host \
  -e PX4_HOME_LAT=473977420 -e PX4_HOME_LON=85455940 \
  px4-sih:v1.14.3
python -m unittest discover
```

`network_mode: host` is required: PX4 sends to 14540 on loopback and will not
talk off-localhost without `MAV_2_BROADCAST=1`, so published ports do not help.
That works on Linux and on GitHub runners, but not on Docker Desktop for macOS.

Home coordinates are **scaled int32 in 1e-7 degrees, not decimal degrees** —
`px4-rc.simulator` passes `PX4_HOME_LAT` straight into `SIH_LOC_LAT0`. Passing
`47.397742` truncates to `47` and silently flies the vehicle 150 km away.

With no vehicle listening these tests skip rather than fail, after a ~1.5s
probe, so `python -m unittest discover` still works on a machine without Docker.
Raise `MAVLINKMCP_SITL_PROBE_TIMEOUT` if a slow container is being missed.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
