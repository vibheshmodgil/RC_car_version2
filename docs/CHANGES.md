# Changes - the September 2026 testing rounds

What was found by running the truck in a real house (and in a simulated one),
what was changed, and how each change was checked. Newest first. The *why*
for each also lives in a comment next to the code.

How things were tested:

- **Live**: on the truck, watched through the cockpit's `/state` at 4 Hz.
- **Sim**: a simulated house (hall, rooms, 550-700 mm doors, furniture) that
  runs the real `explore.py` and the real guard code from `web_nav.py`, with
  an 11 Hz scan, 5 Hz SLAM pose, the motors' wheel mix, and mapping gated like
  SLAM's (only after 40 mm or 4° of movement).

---

## Follow mode

Lock on to one person and follow them - `test/follow.py`, the **Follow** button
on the Vision tab, or "follow me" to the voice assistant / Claude Code.

| Design point | Why |
|---|---|
| The target is a **track**: world position + velocity + clothing-colour signature | The old "same bearing = same person" handed over to anyone walking past |
| Colours decide who it is: below 0.40 similarity a detection is *not* them however near; 0.60 to take them back after losing sight | Different shirts score ~0.25, the same shirt in different light ~1.0 (camera-frame test) |
| Legs in the LiDAR between camera frames and outside the 54° view - grouped into people, skipped when two are close | Averaging every return near the target let a passer-by drag the track away |
| Two look-alikes both fit and too close to tell: coast on the prediction, don't guess | A same-coloured person crossing through the target's spot took over |
| Velocity = slope over ~1.2 s, jumps limited to walking speed | Per-step velocity swung ±500 mm/s, so coasting went anywhere |
| Lost: close by → turn to look; far or behind a wall → go where they were heading | Spinning first cost 15 s at every doorway |
| Guard with the autonomous margin on every command; routes round obstacles via the explorer | "Without colliding" is the guard's job and cannot be bypassed |

Simulation (real `follow.py`, guard and planner; two-legged walkers, camera
only inside 54° with line of sight):

| Scenario | 0.6-2 m from them | On the wrong person | Closest to anyone |
|---|---|---|---|
| Hall, slow walk with pauses | 100 % | 0 s | 964 mm |
| Through a door and round the sofa | 96 % | 0 s | 887 mm |
| Someone crosses in between | 100 % | 0 s | 763 mm |
| Someone walks alongside, then leaves | 100 % | 0 s | 864 mm |
| Same clothes, crossing through their spot | 100 % | 0 s | 780 mm |
| They walk at 0.6 m/s | 57 % (truck max ~0.3 m/s) | 0 s | 964 mm |

Live on the truck (Sept 2026): person seen in 61 % of samples at ~4 Hz, lost
for 0.8-1.6 s at a time, colour match 0.63-1.0, followed across ~8 m. It
falls back to ~3.7 m when the guard refuses a turning arc, because the motors
are capped at 0.4 duty.

### Follow mode, live round 2 (2026-09-26)

**The Pi was too busy to see.** The person tracker ran at 1.6 detections a
second because fetching a camera frame took up to 2 s. The PC answered in
0.12 s. Python runs one thread at a time, and two things were hogging it:

- **The guard** re-simulated the drive path 50 times a second against a scan
  that changes 11 times a second. It now keeps its answer while the command
  and the scan are unchanged, for at most 0.1 s.
- **`/state`** was rebuilt in full for every reader: the page at 7 Hz, plus
  any logger or second tab. Readers now share one build every 0.12 s.

Result live: 5.5 detections a second (was 1.6), 108 ms each (was 1,770 ms),
seen 66 % of the time (was 48 %), and 4 losses in 94 s (was 13 in 228 s).

**Fixed from the same runs:**

- **Standing still while the camera misses them.** LiDAR-only tracking is now
  trusted for up to 30 s when the person stays within 0.4 m of where the
  camera last saw them. Walking away still needs the camera within 10 s.
  Simulated: 11.9 s of searching down to 0 s.
- **Cornered.** Live, the guard refused every forward arc and the truck sat for
  13 s while the person walked away. After 1 s of refusals it now works down a
  ladder: turn to face them, then straight ahead, then back up for room, and
  round again. Any movement ends the ladder.
- **Centring.** Steering is proportional at b/22 (was b/40), and at following
  distance it re-centres beyond 6 degrees (was 12).
- **Search spun in place.** "Carry on the way they went" projected from the
  truck, not from the person. Facing a wall, the goal was the truck's own
  position, so it spun five times and gave up. It now projects from where
  they were last seen. If that direction is behind a wall on the map, it
  skips the look-round and goes round the corner.

**Tried and taken out:** planning round as soon as the person is behind
something on the map. Over six simulated runs each it saved 1.5 s at a
door-and-sofa walk, but cost 5 s behind a wall. Routes still start only when
the guard blocks the way.

### Full test round, 2026-09-26: what passed and what was fixed

All of these were run through the chatbot (typed, as if spoken):

| Test | Result |
|---|---|
| Restart on a saved map | Found itself on the map automatically (it had been 1.7 m / 20 deg off) |
| "what rooms do you know" | Listed all three, in 6 s |
| "what do you see" | Described the scene correctly; 20 s, later 47 s (see below) |
| "start mapping" on a finished map | Looked round, added 5.7k cells, correctly said nothing reachable was left |
| "go to the hall" | Arrived 0.1 m from the spot, 3 m in 22 s |
| "take me to the kitchen" | Matched "kitchen entrance", arrived 0.08 m off, 4.8 m in 32 s |
| "go to bedroom two" | Matched "bedroom number 2", arrived 0.08 m off |
| "go to marker one" | Understood; **no route** - fixed, below |
| "come back to your initial position" | **Not understood** - fixed, below |

Fixed:

- **"No route" out of a room it had just driven into.** One noisy scan drew a
  wall across the bedroom doorway, and every trip out of the bedroom failed.
  When a plan fails, the planner now retries with the cells the truck has
  actually driven through marked passable. Replayed on the live map:
  bedroom 2 was cut off from everything, and with the retry every pair of
  places connects. The navigation simulation still passes all four trips.
- **Home.** "go home", "come back", "return to where you started" and "your
  initial position" drive to where the map was started, unless a place is
  saved as "home".
- **Word boundaries lost in edits.** Shell quoting had turned two regex
  word boundaries into backspace characters, which silently broke the
  home phrases and the "no, wrong" room correction. Fixed, and every code
  file was scanned for control characters.

Open: the photo answers ("what do you see") slowed from 20 s to 47 s. The GTX
1650's 4 GB also drives the screen, and Ollama had only 1.8 GB of it, so
47 % of `qwen3-vl:4b-instruct` was on the GPU. Choices: close GPU-heavy
programs on the PC, or use `qwen3-vl:2b-instruct` (fits on the GPU; weaker
at choosing tools, though the plain commands don't use the model).

### Finding itself on the saved map after a restart

A restart resumes the saved map at the pose the truck was switched off at.
Live, after a restart it believed it was 1.7 m and 20° from where it really
stood. Only 11 % of the live scan landed on walls there. The local matcher
searches a few hundred mm, so it could never have found the way back, and
everything mapped afterwards would have gone in the wrong place.

`Slam.relocalize` matches ONE scan against every confidently-free spot in the
map (every 150 mm) at every heading (every 5°), about 136,000 candidates, then
refines the best six.

- **Live test** (the truck's own map and scan, run on the PC): 74 % of the scan
  on walls at the found pose. The best different place scored 56 %, and the
  place it believed it was at scored 11 %. It took 0.8 s on the PC.
- **When a found pose is taken:** at least 45 % of the scan on walls, 15 points
  better than where it thought it was, and clearly better than any other place.
- **After a restart:** it runs by itself once the LiDAR has a scan, while SLAM
  holds off.
- **By hand:** *Find me on map* on the Map tab, or `/slam {relocalize: true}`.
  `force: true` takes the best guess.

### Markers, routes, and "go to the hall" by voice

Live, "go to the hall" through the assistant failed: `/goto` wanted the
saved name exactly, and the model passed something slightly different.
`find_place` now matches loosely:

- "the hall", "Hall" and "hal" all find `hall`;
- "kitchen" finds `kitchen entrance`;
- "bed room number two" finds `bedroom number 2`;
- "marker three" finds `marker 3`.

A place it still can't find gets a list of the known ones back.

- **Markers:** click the map, then *Drop marker here*. A marker is a place
  named "marker N", drawn as a red pin, so "go to marker 2" works by voice.
- **Routes:** *Draw route* on the map, click the waypoints, name it, save.
  `RouteRunner` drives one `explorer.goto` per waypoint, so each leg uses the
  planner and guard. It can run once or on a loop. It stops when a waypoint
  can't be reached, or when anything else takes over the truck (Stop, Cancel
  trip, Go here, mapping, follow mode). Voice: "drive the patrol route",
  "do the kitchen route on loop". Routes are cleared with the map, because
  they're in its coordinates.
- The spoken replies for going somewhere are now the result itself. The model
  used to rephrase it, which took 13 s.
- Room names that were spelled out ("hall h a l l") are cleaned up. Saying
  "start mapping" while already mapping no longer restarts the calibration.

Checked offline: the route runner (plain run, loop, unreachable point, sent
elsewhere mid-route), the place matching, the voice phrases, and a parse of
the whole page's JS.

### Known-good settings and Fresh map

Settings changed by mistake made the walls thick and the explorer stall. There
are 58 sliders, and "Revert to code defaults" also throws away what was measured
on this truck. Two fixes:

- **Restore known-good** (Tune tab, `/tuning {good: "restore"}`): code defaults
  plus `tuning_good.json`, or `tuning_good.default.json` from the repo, which
  holds the values the whole house mapped cleanly with. **Save as known-good**
  makes the current values the new set.
- **Fresh map** (Map tab, `/fresh_map`): known-good settings, clear the map,
  rooms and objects, then auto-map from here. The assistant asks the room
  names as it goes. It needs ENABLE to have been pressed already.
- **Voice / Claude Code:** the `new_map` and `restore_settings` tools. Short
  phrases ("reset the map", "restore the default settings", "start mapping")
  run without the model. Longer ones go to it.

### Truck sounds

The truck now makes sounds when things happen, instead of being silent
(`test/cues.py`, sounds in `audio.BEEPS`):

| When | Sound |
|---|---|
| Actually reversing, after the guard (a tap doesn't count) | Backing-up pip, once a second |
| ENABLE / motors off | Two notes up / two notes down |
| Follow mode locks on to someone | Three rising notes |
| Follow loses them / finds them again | Falling "uh-oh" / quick happy note |
| Follow gives up after 30 s | Long falling line (not when you press Stop) |
| Arrived where it was sent / auto-map finished / gave up | Chord / arpeggio and chord / two low notes |
| The guard refused a move you drove | A short low bonk (manual driving only) |

Rules:

- Never over its own speech: a cue waits up to 2 s and is dropped after that.
- Over a song, only the one-off events play, and the song resumes. The
  reverse alarm and the bonk stay quiet over a song.
- A reverse pip never cuts off the horn.
- The follower's own trips (to where it last saw you) never play "arrived".
- Drive tab → Speaker card: **Truck sounds** and **Reverse alarm** toggles,
  saved in `cues.json` on the Pi.

Checked with scripted events against the real `cues.py`: every event gave the
right sound, the follower's own trips and Stop were silent, and nothing
played over a song or over speech.

### Lost while standing still: no more driving up to them

Live, the camera lost someone standing ~2.5 m away. The search drove the
planner's path to where they were last seen, heading 34° off the spot (outside
the camera's ±27°), and stopped 0.75 m from the person. Two causes, both fixed:

- The search went to the trail's last point, which leaves out LiDAR-only
  readings, so it could be metres behind where they actually were. It now goes
  to the track's last measured position.
- On the way, if the LiDAR sees legs (not a mapped obstacle) at that spot
  within following distance, with nothing in between, the truck stops, turns
  the camera onto the spot, and looks for 1.5 s before searching further.

Sim, person standing still while the camera misses them for 22 s: closest
approach 326 mm -> 1020 mm, searching 17.6 s -> 9.6 s. The other scenarios
are unchanged (0 s on the wrong person, 0 s backing off from nobody).

### System tab

A task manager in the cockpit. On the Pi it shows CPU (total and per core),
RAM, temperature, under-voltage (`vcgencmd get_throttled`), and this
program's CPU split by thread, named by what each thread runs
(`SlamRunner._run`, `Lidar._loop`...). On the PC it shows the detection
container (people and objects: frames/s, ms per frame, container CPU and RAM,
the Docker VM's load), the speech container, and the models Ollama has loaded
with how much of each is on the GPU. Polling runs only while the tab is open.

### Person trail on the map

Where they walked is drawn on the map (the **Person trail** chip turns it on
and off; Reset map clears it). When the truck loses them, it searches along
the trail: it goes to where they were last seen, then turns to face the way
they were heading, looks round, and follows the trail onward.

The first live trail had spikes several metres long through walls. They came
from ranges that looked past thin legs and from LiDAR-only readings on
something else. Now a point is only recorded from a trusted reading: not a
rejected jump, and not LiDAR alone more than 2 s after the camera last saw
them. Each point is also averaged over the last 4 positions. In the
live-like sim, the longest single step fell from 3.0 m to 0.8 m, and the
worst point off their real path fell from 2.9 m to 1.4 m.

## Round 3 - mapping the whole house, then voice

| Problem (seen) | Change | Checked |
|---|---|---|
| Stuck 5 minutes in the front-door nook: wall 9 cm alongside, only forward free (live) | **Make room to turn**: when a turn is refused, try in order: forward pivot towards the goal, reverse pivot, both the other way, then spin the other way. A pivot swings the tail *away* from a wall alongside; a spin swings it in | Sim corner cases: 3 of 4 escape in 7-13 s (all 4 were stuck). Wedged diagonally 22 mm from a wall stays stuck - every move would push a corner closer |
| SLAM at 300-430 ms an update (budget 200) (live) | Loop closure in a background thread, at most every 2 s; result applied as an offset; dropped after a reset | SLAM thread's loop-closure cost ≤ 15 ms (was ~180 ms inline, on a PC) |
| Every `/state` and `/ai/status` returned HTTP 500 after one unroutable trip - cockpit and voice assistant both dead (live) | The explorer never stores `None` as its path; status tolerates it | Recovered live by a zero-length trip; code fixed |
| "Ceiling fan" on the floor map (live) | Ceiling-only labels ignored for the map | - |
| `shiv.local` refused (Claude Code's truck tools, some phones) (live) | Cockpit and `/talk` serve on IPv6 and IPv4 | Truck tools connected by name after the change |

## Round 2 - the first full auto-map

| Problem (seen) | Change | Checked |
|---|---|---|
| Sat still at start: its only frontier was the floor under its own chassis (live) | Frontiers within 400 mm are ignored; with nowhere to go it **looks around** (slow turn) before deciding; a look ends after 25 s or when both turn directions are blocked | Sim auto-map: done in 129-135 s, 61 of 65 m² |
| Stuck loops next to furniture: planner goals ~160 mm from obstacles, guard stops 300-350 mm short (sim + live) | A frontier only needs to be **seen**: within 0.7 m counts, or 1 m if the guard stops it; click-to-go reports "arrived as close as it can get" | Sim: stuck time 146 ticks → 0 |
| A long turn counted as "stuck" (live) | Turning 20°+ counts as progress | - |
| Planned a 2 m loop through unmapped space (live) | Unmapped cells cost 4× (was 1.8×) | - |
| One TV unit saved as armchair + television + sofa (live) | One object per spot (600 mm), named by its most-seen label; a person's rename is never overwritten | Unit test: 3 labels → 1 object; a chair 1.5 m away stays separate |
| 13 refused arcs in one run (live) | A refused arc → line up on the spot, then straight | - |

## Round 1 - navigation that works

| Problem (seen) | Change | Checked |
|---|---|---|
| Zig-zag: heading 107 → 153 → 118° on a straight run; 3 of 4 trips failed (live, sim) | The follower aimed at the first path point 450 mm away **from the path's start** - behind the truck once it had driven 450 mm. It now searches forward from the nearest path point | Sim: 0 spin reversals, trips 2-4× faster |
| Routes hugged walls; the guard kept cutting turns (live) | Clearance costmap: walls grown by half-width + the guard's side margin; cells within turning reach cost up to 9×. Waypoints at cell centres (were corners, 50 mm off) | Sim: every trip ≥ 60 mm from walls, 0 guard stops; a 550 mm door both ways |
| Cut door frames (sim: 13 mm) | Lookahead 450 mm in open space, 150 mm where it is tight | Sim closest pass 66 mm |
| Bang-bang steering | Spin above 35° off until under 10° (hysteresis); speed and steering scale with the error | - |
| Two driving threads after a new goal mid-trip | One thread per run, generation-counted | - |

## The guard

| Problem (seen) | Change | Checked |
|---|---|---|
| Stalled at random (live) | It was the camera floor check reading walls as drop-offs with the camera tilted up. **Removed from driving** | Live: stalls stopped |
| Froze with "no fresh LiDAR scan" while the scanner worked (live) | Revolutions end on the scanner's own start flag (the angle-wrap test failed with half the view blocked, and doubled the rate when both were used); port reopened after 2.5 s without a scan; badge shows *LIDAR STALLED* | Sim packets: one scan per revolution with the flag at 0° or 180°, no flag, 60° of view. Live: 11.6 Hz, 0 stalls in a corner |
| A wall beside the truck blocked forward; a turn was refused both ways if anything was within a corner's reach (live) | One rule - no scan point inside the truck's outline along the path the move really drives: straight lane (15 mm sides), spin 30° ahead (25 mm), arcs along the real wheel-mix path (10 mm when you drive, 20 mm when the explorer does). Only the unsafe part of a move is dropped; moving *away* from something close is never refused | Table of cases in TUNING.md; live |
| Hard to see what it thought | Drive plot: green = free floor, red = where the truck's centre cannot go, four arrows = its verdicts | Pixel-checked in Chrome |
| Truck size / LiDAR mounting lost on restart (live) | Every tuning change saved to the SD card 1.5 s after the last one, and on exit | Live: 0 unsaved after a change |

## Mapping, objects, voice

| Problem (seen) | Change |
|---|---|
| Rotated double walls (live) | Heading = IMU + an offset the matcher corrects (the IMU used to overwrite every correction). Sim: 45° worst error → 6° under heavy drift |
| IMU spike 15 → 255 → 15° with the motors off (live) | Spikes > 60° per update dropped; a lasting jump accepted without a heading step |
| Map ran off the edge half-way down the hall | 30 m grid; only the explored part is sent to the page |
| No zoom; no way to send it anywhere | Zoom, pan, follow, fit; click to go or name a room; route and goal drawn |
| Map gone after a restart | Autosave every 20 s and on exit; resumed at startup; Reset map clears map, rooms and objects |
| Objects: a question per chair, repeated forever; "not a television" saved as a name | Saved automatically after 3 sightings from different viewpoints (25 cm / 15° apart); the truck only asks room names, once per area, and "no, the hall" corrects a mishearing |
| Detection never ran while driving | It skipped whenever SLAM was busy; now only the Pi's own model is skipped, and the pose is taken when the photo is |
| Typing a room name drove the truck (W A S D, space = e-stop) | Keys in text boxes never reach the drive |
