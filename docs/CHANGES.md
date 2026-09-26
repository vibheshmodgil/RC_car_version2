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
