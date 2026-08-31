"""Text for the MCP prompts: how this server is put together and how to drive it.

A tool description says what one call does. None of them says why the pipeline is
split the way it is, why a route has to be validated before it can be uploaded,
or why a model should answer in pixels and never in latitudes. That context is
what turns fifty independent tools into something an agent can fly safely, so it
lives here and is served as prompts rather than being left for a client to guess.

Prose only -- every function returns a string and nothing here imports mavsdk,
touches the network or reads the store. Server-wide constants are interpolated
from the modules that own them so a limit quoted in a briefing cannot drift away
from the limit actually enforced.
"""

from __future__ import annotations

from . import map_source, map_view, plan_store
from .plan_helpers import STATUSES

PIPELINE = (
    "get_map_view -> create_survey_plan -> render_plan_view\n"
    "  -> validate_plan -> preflight_check -> upload_plan\n"
    "  -> verify_uploaded_plan -> start_mission"
)

# Repeated verbatim in several briefings: it is the single most common way for an
# agent to produce a well-formed plan in the wrong place.
PIXEL_RULE = (
    "Never estimate latitude and longitude by eye from a map image. Report what "
    "you see as [x, y] pixels and let map_transform or create_survey_plan convert "
    "them -- the stored transform is exact and your reading of the picture is not."
)

# The three rules below are served twice: verbatim in the initialize response
# (server_instructions) and again inside the overview prompt. They are constants
# rather than two copies of the prose so the always-on contract and the long-form
# briefing cannot drift into disagreeing about what an acked command means.
COMMAND_SENT_RULE = """**"command_sent" is not "done".** Every tool returns a `status` of
"success", "command_sent" or "failed". Commands that move the aircraft --
arm_drone, takeoff, land, return_to_launch, disarm_drone, move_to_relative,
start_mission, pause_mission -- return when the autopilot *acknowledges* the
command, not when the manoeuvre finishes. They carry `completed: false` and a
`verify_with` list naming the telemetry tools that say what actually happened.
Call those before you describe the vehicle's state to anyone. A takeoff is
routinely acknowledged and then does not happen: the autopilot may reject the
climb, or disarm itself a few seconds after arming if nothing has lifted off. So
"the drone is at 5 m" is a claim about get_altitude and get_landed_state, and
never about the reply to takeoff. Treat `"status": "failed"` as fail-closed --
the thing did not happen, and nothing downstream should assume it did."""

ALTITUDE_RULE = """**Altitudes are relative, and not all to the same thing.** takeoff's altitude
argument is metres above the point the vehicle lifts off from. Asking for 5
means climb 5 m -- never "5 m above sea level", and never "current altitude plus
5". get_altitude's `altitude_relative_m` uses that same takeoff reference and is
what confirms a climb. get_position's `relative_altitude_m` is measured from the
recorded *home* position instead, so the two disagree when home is set wrong. A
vehicle that get_landed_state calls ON_GROUND while get_position reports a large
`relative_altitude_m` has a bad home fix: report that rather than flying it."""

ASK_RULE = """**When you are not sure, ask.** You are commanding an aircraft on someone
else's behalf, and a wrong guess is expensive in a way a question is not. Ask
the operator -- do not pick a default and proceed -- when the request is
ambiguous or leaves an altitude, distance or direction unspecified; when
telemetry contradicts itself or contradicts the request; when a command is
acknowledged but the vehicle does not do the thing; and when what was asked
would move the aircraft in a way that looks unsafe or unintended. Say what you
observed, what you think it means and what you propose, then wait. "Takeoff was
accepted but the drone is still ON_GROUND and has disarmed itself -- shall I
investigate before retrying?" is the right shape. Silence between tool calls is
not consent."""


def server_instructions() -> str:
    """The always-on operating contract, served in the MCP initialize response.

    Unlike the prompts below, a client does not have to ask for this: it rides
    the handshake and most clients fold it into the model's system prompt
    (fast-agent does, via its `{{serverInstructions}}` template). That makes it
    the only place server-side guidance is guaranteed to be read, so it carries
    the two rules an agent cannot fly safely without -- what an acknowledged
    command does and does not mean, and which altitude is measured from where --
    and stops there. Everything longer stays in the prompts, which cost nothing
    until they are asked for.
    """
    return f"""These tools fly a real or simulated aircraft. Three rules govern every
call you make against this server.

1. {COMMAND_SENT_RULE}

2. {ALTITUDE_RULE}

3. {ASK_RULE}

Longer guidance is available as prompts: mavlink_overview, telemetry_guide,
map_view_guide, plan_lifecycle_guide."""


def overview() -> str:
    """How this server is organised and what its call convention is."""
    return f"""You are operating a MAVLink MCP server: MAVSDK-backed tools for a real or
simulated aircraft, plus a flight-plan pipeline for coverage missions.

There are two halves.

**Live vehicle tools** talk to the autopilot right now. Telemetry readers
(get_position, get_battery, get_gps_info, get_health, get_attitude_euler,
get_flight_mode, get_landed_state and the rest) and commands (arm_drone,
takeoff, land, return_to_launch, disarm_drone, move_to_relative). These need the
MAVLink link to be up.

**The flight-plan pipeline** authors, checks and flies routes:

{PIPELINE}

Every authoring and inspection step works with the link *down*, so a plan can be
built and costed at a desk and flown later.

**The call convention.** Every tool returns a dict carrying a `status`, and a
failure carries `error` with it. That status describes the call, not the
vehicle's state -- see the rule below. A plan's lifecycle value is reported
separately as `plan_status` ({" / ".join(STATUSES)}) precisely so the two can
never be confused.

{COMMAND_SENT_RULE}

**The link comes up in the background.** The server answers the MCP handshake
immediately and dials the vehicle behind it, so early calls can return
`connected: false` with a `link_state` of "connecting". That is normal -- retry.
Once `link_state` is "failed" the address is wrong or nothing is listening, and
retrying will not help; check MAVLINK_ADDRESS, MAVLINK_PORT and
MAVLINK_CONNECT_TIMEOUT.

{ALTITUDE_RULE}

{ASK_RULE}

**Safety.** These tools can arm, take off and move an aircraft. Prefer SITL.
Before any command that moves the vehicle, confirm the human operator is ready
and has a kill switch. Read get_health and get_landed_state before arming rather
than after.

Ask what the operator wants to do, and say which half of the server it lands in."""


def telemetry_guide() -> str:
    """Reading vehicle state, and what an absent reading means."""
    return """You are reading state from a vehicle over MAVLink through an MCP server.

**What is available.** Position (get_position, get_altitude, get_home_position),
motion (get_velocity_ned, get_heading, get_attitude_euler, get_odometry),
condition (get_battery, get_gps_info, get_health, get_rc_status, get_imu),
mode and phase (get_flight_mode, get_landed_state, get_is_armed, get_in_air,
get_vtol_state), environment (get_wind, get_distance_sensor) and timing
(get_unix_epoch_time). Mission progress comes from print_mission_progress and
is_mission_finished.

**Every one of them is fail-closed.** A reading that could not be taken comes
back as `{"status": "failed", "error": ...}` -- never as a zero, a null or a
stale value. So:

- Do not treat a failed read as "the value is zero". An altitude that failed to
  read is an unknown altitude, and unknown is not the ground.
- Do not average or carry forward a previous sample to fill a gap.
- Report the gap to the operator instead. "GPS fix is unreadable" is a useful
  sentence; a confidently wrong number is not.

**A missing link is different from a missing reading.** `connected: false` with
`link_state: "connecting"` means the server has not reached the vehicle yet --
retry. `link_state: "failed"` means it gave up; the configuration is wrong.

**get_imu is expensive.** It raises the IMU stream rate and collects n samples
(clamped to [1, 100]). Ask for the fewest you actually need.

Take the readings the operator asked about, and say plainly which ones you could
not get."""


def map_view_guide() -> str:
    """The perception layer: georeferenced imagery, and how to answer about it."""
    providers = ", ".join(sorted(map_source.PROVIDERS))
    return f"""You can see the ground. get_map_view returns a georeferenced satellite view
as an **image** together with the exact transform that places every pixel of it
on the Earth.

**How to use it.** Call get_map_view with no coordinates to centre on the
vehicle, or with latitude_deg/longitude_deg to look anywhere (which needs no
vehicle at all). You get back `[image, {{view_id, center, meters_per_pixel,
bbox, size_px, ...}}]`, and when the link is up a `drone` block giving the
vehicle's own pixel position and heading.

{PIXEL_RULE}

map_transform converts either direction against a stored view: pass `pixels` to
get coordinates, or `coordinates` to get pixels. Pixels outside the image are
rejected rather than clamped -- a point off the edge means the image was misread,
and quietly pulling it back to the border would put a plausible-looking polygon
in the wrong place.

**Orientation.** `orientation="heading_up"` rotates the view so the vehicle's
forward direction is up, which turns "the field in front of the drone" into a
question about the picture instead of compass arithmetic. The default is
north_up. The payload always states which.

**Cost and sizing.** Views render at {map_view.DEFAULT_SIZE_PX} px by default and
cap at {map_view.MAX_SIZE_PX} px. That ceiling is about tokens, not bytes: cost
scales with pixel area, so {map_view.DEFAULT_SIZE_PX} px is roughly 790 tokens
and {map_view.MAX_SIZE_PX} px about 1400. A survey conversation renders many
views -- do not ask for the maximum by reflex. radius_m sets the ground area
covered ({map_view.MIN_RADIUS_M:g}..{map_view.MAX_RADIUS_M:g} m) and, with
size_px, decides the zoom and the tile count. A view needing more than
{map_source.MAX_TILES_PER_REQUEST} tiles is refused: reduce radius_m or size_px.

**Imagery is the only network access this server makes.** Tiles come from the
configured provider ({providers}, or custom), are pinned to that provider's
host, and are cached on disk. A tile that fails renders as flat grey and the
payload says so -- the georeferencing is still exact, so the geometry is still
trustworthy even when the picture is patchy.

`MAVLINKMCP_MAP_PROVIDER=none` disables the network entirely and renders a
correctly georeferenced blank canvas. Overlays and coordinates still work; there
is simply nothing to identify ground features from, and the payload carries an
`imagery_warning` saying so. Do not describe terrain you cannot see.

**Attribution is not optional.** Provider terms require it. It is drawn onto
every view and returned in the payload; keep it with the image.

The `mavlinkmcp://map/providers`, `mavlinkmcp://map/limits` and
`mavlinkmcp://map/cache` resources report the live configuration if you need to
check it, and `mavlinkmcp://map/views` lists views already rendered."""


def plan_lifecycle_guide() -> str:
    """Why a plan has states, and what changes them."""
    return f"""Flight plans in this server have a lifecycle, and it is load-bearing.

**States.** A plan moves {" -> ".join(STATUSES)}.

- **draft** -- authored but unchecked. Every new plan and every new revision
  starts here.
- **validated** -- validate_plan ran and found no errors.
- **uploaded** -- the vehicle has it.

**upload_plan refuses anything that is not validated.** That refusal is the
point of the whole design: uploading is where a route stops being a document and
becomes something an aircraft will fly. Do not look for a way around it. If
upload_plan refuses, run validate_plan and fix what it reports.

**Revisions are immutable.** revise_plan re-runs the generator with new
parameters and writes a *new revision*; it never edits waypoints in place, so
the stored parameters and the stored path can never disagree. A new revision is
a draft, which means any change invalidates the previous check. Only the
lifecycle annotations (status, checks, estimate, uploaded) change on an existing
revision.

**What validate_plan is actually for.** It reports findings; only severity
"error" blocks. The most valuable finding is FAR_FROM_HOME, because the failure
it catches is the one that looks fine on paper: a polygon drawn on the wrong map
produces a perfectly well-formed plan on the other side of the world, and
distance from home is what exposes it. Pass home_lat/home_lon, or have the link
up so the live position can be used -- validating with no home position skips
that check, and the response says so.

**Then check the vehicle, not just the plan.** preflight_check reads health,
GPS fix, battery and landed state and returns a GO / NO-GO with blockers listed
separately from warnings. verify_uploaded_plan downloads the mission back off
the aircraft and diffs it against what was reviewed, so what will be flown is
confirmed rather than assumed.

**Where plans live.** Plain JSON under the data root (MAVLINKMCP_PLANS_DIR, else
the XDG data location), one directory per plan with numbered revisions. They
outlive the MCP session and can be read and diffed by a human without this
server running. The store holds at most {plan_store.MAX_PLANS} plans and
{plan_store.MAX_REVISIONS_PER_PLAN} revisions each.

**Look before you fly.** render_plan_view draws the plan over satellite imagery.
Reading coordinates will not tell you whether the path covers the area that was
meant; the picture will. preview_plan_geojson is the machine-readable
counterpart, and the same thing is served as the
`mavlinkmcp://plans/{{plan_id}}/geojson` resource."""


def survey_walkthrough(area: str = "", altitude_m: str = "") -> str:
    """End-to-end: from looking at the ground to a mission running."""
    target = f"\n\nThe operator wants to survey: {area.strip()}" if area.strip() else ""
    altitude = (
        f"\nRequested altitude: {altitude_m.strip()} m."
        if altitude_m.strip()
        else ""
    )
    return f"""Plan and fly a coverage survey with the MAVLink MCP server. The pipeline is
split so the route is reviewable before it reaches the aircraft:

{PIPELINE}{target}{altitude}

**1. See the ground.** get_map_view, centred on the vehicle or on coordinates
the operator gives. Look at the returned image and find the area to cover.

**2. Draw the area in pixels.**
{PIXEL_RULE}
Pass the corners to create_survey_plan as `view_id` plus `polygon_pixels`
([[x, y], ...]). If the operator gave real coordinates instead, pass `polygon`
([[lat, lon], ...]) -- but give exactly one of the two forms; supplying both is
refused rather than silently resolved.

**3. Set the geometry.** create_survey_plan generates a lawnmower sweep. Line
spacing comes from either `line_spacing_m` or a `camera` dict -- giving both is
refused, because one number cannot have two sources. A camera looks like:

    {{"sensor_width_mm": 13.2, "focal_length_mm": 8.8,
      "image_width_px": 5472, "image_height_px": 3648,
      "front_overlap": 0.75, "side_overlap": 0.65}}

and also sets the photo trigger distance and reports ground sample distance.
Omit `sweep_angle_deg` to sweep along the area's long axis, which minimises
turns -- where survey time and battery actually go.

**4. Look at what you generated.** render_plan_view. This is the step that
catches an area covered in the wrong place or at the wrong angle, and no amount
of reading coordinates substitutes for it. Show the operator.

**5. Cost it.** estimate_plan gives distance, duration and photo count. Battery
use is reported only when both `battery_capacity_mah` and `cruise_current_a` are
given; otherwise it stays null rather than being invented.

**6. Check it.** validate_plan. Only errors block. Give it a home position, or
have the link up, so FAR_FROM_HOME can be checked -- it is the finding that
catches a polygon drawn on the wrong map.

**7. Check the aircraft.** preflight_check returns GO or NO-GO from live health,
GPS fix, battery and landed state, with blockers listed separately from
warnings. Do not talk an operator past a NO-GO.

**8. Fly it.** upload_plan (which refuses an unvalidated plan), then
verify_uploaded_plan to confirm the aircraft holds what was reviewed, then
arm_drone, then start_mission.

Stop and report at any step that fails. Never skip 4 or 7 to save a call: they
are the two steps that catch the mistakes the others cannot."""


def preflight_briefing(plan_id: str) -> str:
    """The go/no-go sequence for one specific stored plan."""
    name = plan_id.strip() or "<plan_id>"
    return f"""Run the pre-flight sequence for stored plan `{name}` and give the operator a
clear go or no-go. Do not start the mission as part of this -- report, and let
them decide.

In order:

1. **get_plan("{name}")** -- confirm it exists, and note its `plan_status`,
   revision and waypoint count. If it is not the plan the operator meant, stop.

2. **render_plan_view("{name}")** -- look at the route over the imagery. Does it
   cover the intended area, at a sensible angle, without crossing anything it
   should not? Say what you see. This is the only check that catches a route in
   a plausible but wrong place.

3. **estimate_plan("{name}")** -- distance, duration and photo count. If the
   operator can give battery capacity and cruise current, pass them; otherwise
   battery stays null and you should say the endurance is unverified rather than
   guessing it.

4. **validate_plan("{name}")** -- pass home_lat/home_lon if you have them, or
   rely on the live position. Report every finding. Errors block; warnings are
   for the operator's judgement, not yours to dismiss.

5. **preflight_check("{name}")** -- the live go/no-go: health flags, GPS fix,
   battery, landed state, and the plan's fit against the vehicle's actual
   position. Report `verdict`, and list `blockers` and `warnings` separately.

Then summarise: the verdict, every blocker, every warning, and what would have
to change to clear each blocker. If anything in the chain returned
`"status": "failed"`, treat it as unresolved rather than passed -- an unverified
check is not a passed check.

Remember the upload gate: upload_plan will refuse this plan unless step 4
leaves it validated. That is intended. Do not look for a way around it."""
