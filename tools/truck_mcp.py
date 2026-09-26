"""
Speaker Truck MCP server — lets Claude Code see, drive and talk through the truck.

**Runs on your PC**, not the Pi, like detect_server.py. It speaks MCP over
stdio to Claude Code (or Claude Desktop) and plain HTTP to web_nav.py on the
Pi, so nothing new is installed or exposed on the truck.

    pip install "mcp>=1.2"
    claude mcp add -s user truck -e TRUCK_URL=http://shiv.local:5004 -- python C:\\Users\\vibhe\\OneDrive\\Desktop\\Speaker_truck\\tools\\truck_mcp.py

Then, in Claude Code: "look around and drive to the door", "say hello".

This is for driving the truck FROM a Claude Code session. For talking to the
truck hands-free, the on-board assistant (test/brain.py, inside web_nav.py)
already answers the phone microphone on its own; `listen` here only works
while that assistant is switched off on the cockpit's Audio tab.

The actions themselves live in test/truck_api.py, shared with the on-board
assistant, so both drive the truck the same way:

  status        obstacles in 8 directions, guard, pose, speech, mic
  look          one camera frame, as an image Claude can see
  drive         one short move: forward/backward/turn, <= 3 s, then stop
  stop / emergency_stop
  say / beep    through the truck's speaker
  listen        wait for what someone says into the phone's /talk page
  places / go_to_place / cancel_navigation
  play_song / stop_sound

Safety — the parts that do not depend on the AI behaving
--------------------------------------------------------
  * A person arms the motors. There is no enable tool; every move is refused
    until someone presses ENABLE in the cockpit.
  * Every move is bounded (3 s) and ends in an explicit stop.
  * Moves use the arrow keys' path: guard, floor check, speed limit apply.
  * web_nav's 0.6 s watchdog stops the motors if this process or wifi dies.
  * E-STOP in the cockpit (or the space bar) beats all of it.

*** Until the TB6612s are replaced, wheels off the ground. WIRING.md §8. ***
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test"))

import truck_api  # noqa: E402

try:                                                  # mcp 2.x
    from mcp.server.mcpserver import Image, MCPServer as _Server
    from mcp.server.mcpserver.exceptions import ToolError
except ImportError:                                   # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server, Image
    from mcp.server.fastmcp.exceptions import ToolError

TRUCK_URL = os.environ.get("TRUCK_URL", "http://shiv.local:5004").rstrip("/")
api = truck_api.TruckApi(TRUCK_URL)

INSTRUCTIONS = """\
You are driving a real 4-wheel robot (the Speaker Truck) through these tools.
Treat it like a real vehicle in a real house.

- Call `status` before moving and `look` whenever you need to see. Obstacle
  distances are from the truck's CENTRE: subtract half the body length (ahead
  / behind) or half the width (sides) for the actual gap.
- Move in short steps (0.5-1.5 s), then check `status` or `look` again.
  Never chain several moves blind.
- If a move reports the guard blocked it, do not retry the same direction;
  turn or back off first.
- If motors are disabled, ask the human to press ENABLE in the cockpit. You
  cannot and must not try to work around that.
- For a spoken conversation: `listen` for what the person says on the phone,
  answer with `say` (short, spoken-style, no markdown), then `listen` again.
  If `listen` reports the on-board assistant is answering, tell the human to
  switch it off on the cockpit's Audio tab first.
- If anything looks wrong, call `stop` (or `emergency_stop`) first, then explain.
"""

try:
    mcp = _Server("speaker-truck", instructions=INSTRUCTIONS)
except TypeError:
    mcp = _Server("speaker-truck")


async def _call(fn, *args):
    """Run a blocking truck_api call off the event loop. TruckError becomes a
    ToolError, whose message reaches Claude — any other exception is shown
    only as "Error executing tool X", hiding e.g. that web_nav is not running."""
    try:
        return await asyncio.to_thread(fn, *args)
    except truck_api.TruckError as e:
        raise ToolError(str(e)) from None


@mcp.tool()
async def status() -> str:
    """Current state of the truck: whether the motors are armed, the nearest
    obstacle in 8 directions, what the collision guard is blocking, pose,
    camera, speech and phone-microphone state. Call before every move."""
    return await _call(api.status_text)


@mcp.tool()
async def look() -> Image:
    """One frame from the truck's forward camera (about 54 degrees wide),
    as an image you can see. Use it to identify what an obstacle actually is."""
    return Image(data=await _call(api.look_jpeg), format="jpeg")


@mcp.tool()
async def drive(direction: str, seconds: float = 1.0, speed: float = 0.6) -> str:
    """Make ONE short move, then stop automatically.

    direction: "forward", "backward", "turn_left" or "turn_right" (turns are
        on the spot), or "forward_left" / "forward_right" to curve.
    seconds: 0.2 to 3.0. Prefer 0.5-1.5 and check status/look between moves.
    speed: 0.2 to 1.0, a fraction of the cockpit's speed limit.

    The collision guard still applies and may refuse or cut the move short;
    the result says so. Refused entirely if the motors are not enabled."""
    return await _call(api.drive, direction, seconds, speed)


@mcp.tool()
async def stop() -> str:
    """Stop the motors now (they stay armed). Also cancels navigation."""
    return await _call(api.stop)


@mcp.tool()
async def emergency_stop() -> str:
    """Cut power to both motor drivers immediately. A person must press
    ENABLE in the cockpit before the truck can move again."""
    return await _call(api.emergency_stop)


@mcp.tool()
async def say(text: str, wait: bool = True) -> str:
    """Speak text out loud through the truck's speaker (Piper neural voice).
    Keep it short and conversational — this is heard, not read. With wait
    (the default) this returns once the truck has finished speaking, so a
    following `listen` does not pick up the truck's own voice. A new `say`
    cuts off anything still being spoken."""
    return await _call(api.say, text, wait)


@mcp.tool()
async def listen(timeout_seconds: float = 30.0) -> str:
    """Wait for the person to say something into the phone microphone
    (the /talk page) and return their words. Returns everything said since
    the last listen straight away if something is already waiting.
    timeout_seconds: up to 120. On silence, says so — call listen again to
    keep the conversation open, or stop if they have gone."""
    return await _call(api.listen, timeout_seconds)


@mcp.tool()
async def beep(kind: str = "horn") -> str:
    """Sound the horn or a beep: "horn" (Indian truck horn, full volume),
    "beep", "double", "chirp", "reverse" (8 s backing-up alarm) or "alert"."""
    return await _call(api.beep, kind)


@mcp.tool()
async def places() -> str:
    """Named places saved on the truck's map, which go_to_place can drive to."""
    return await _call(api.places_text)


@mcp.tool()
async def go_to_place(name: str) -> str:
    """Start autonomous navigation to a saved place. The truck plans its own
    path with SLAM; check progress with status, cancel with cancel_navigation
    or stop. Refused if the motors are not enabled."""
    return await _call(api.go_to_place, name)


@mcp.tool()
async def cancel_navigation() -> str:
    """Stop autonomous navigation started by go_to_place."""
    return await _call(api.cancel_navigation)


@mcp.tool()
async def play_song(name: str = "") -> str:
    """Play a song from the truck's library. Empty name lists the songs."""
    return await _call(api.play_song, name)


@mcp.tool()
async def stop_sound() -> str:
    """Stop music, beeps and speech."""
    return await _call(api.stop_sound)


@mcp.tool()
async def music(action: str) -> str:
    """Control the song that is playing: "pause", "resume", "next" or "previous"."""
    return await _call(api.music, action)


@mcp.tool()
async def volume(change: str) -> str:
    """Speaker volume: "up", "down" (a quarter step each) or a percentage like "50"."""
    return await _call(api.volume, change)


@mcp.tool()
async def save_place(name: str) -> str:
    """Save the truck's current position on the map under a name, so
    go_to_place can drive back to it later."""
    return await _call(api.save_place, name)


@mcp.tool()
async def follow(action: str) -> str:
    """Follow the nearest person in view: "start" or "stop". It keeps to that
    person (clothing colours + where they are walking), steers round
    obstacles, and the collision guard still applies. Refused unless a person
    has already pressed ENABLE. Progress is in status."""
    return await _call(api.follow, action)


@mcp.tool()
async def mapping(action: str) -> str:
    """Autonomous exploration that builds the map: "start" or "stop". Start is
    refused unless a person has already pressed ENABLE and the LiDAR is
    connected. Check progress with status; save the result with save_map."""
    return await _call(api.explore, action)


@mcp.tool()
async def drive_route(name: str, loop: bool = False) -> str:
    """Drive a route drawn on the cockpit's map, waypoint by waypoint, with the
    normal planner and guard. loop=True repeats until stopped. Refused unless
    a person has already pressed ENABLE. list with `places`."""
    return await _call(api.drive_route, name, loop)


@mcp.tool()
async def new_map() -> str:
    """Restore known-good settings, clear the map (rooms and objects too), and
    map the house from where the truck stands. Destructive: only when the
    person asks for a new map. Mapping starts only if ENABLE was pressed."""
    return await _call(api.new_map)


@mcp.tool()
async def restore_settings() -> str:
    """Tuning back to the known-good set (code defaults + measured values)."""
    return await _call(api.restore_settings)


@mcp.tool()
async def label_questions() -> str:
    """Detections waiting for a person to confirm before they go on the map:
    key, what the detector thinks it is, the nearest named room, and how many
    times it was seen. Nothing is labelled on the map until answered."""
    return await _call(api.label_questions_text)


@mcp.tool()
async def answer_label(key: str, answer: str, name: str = "") -> str:
    """Answer a label question for the person: answer "yes", "no" or "skip".
    "yes" with a name labels it by that name instead ("no, it's a bed" is
    yes + name="bed"). "no" means it is never asked about at that spot again.
    Only answer what the person actually said — never guess for them."""
    return await _call(api.answer_label, key, answer, name or None)


@mcp.tool()
async def save_map() -> str:
    """Save the SLAM map built so far on the Pi."""
    return await _call(api.save_map)


@mcp.tool()
async def take_photo() -> str:
    """Save the current camera view to test/captures/ on the Pi, tagged with
    the pose. To see the view yourself, use look instead."""
    return await _call(api.take_photo)


if __name__ == "__main__":
    mcp.run()
