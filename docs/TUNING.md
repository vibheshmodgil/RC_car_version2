# Tuning

Every number that decides whether SLAM works, what it does, and how to find a
better value.

## Tune where you can see the effect

**Every control appears next to the thing it changes.** A slider is useless
somewhere you cannot see what it does — you cannot judge the scanner's
rotation without the plot in front of you, or a guard margin without the
clearances it produces.

| Group | Lives on | The evidence beside it |
|---|---|---|
| Vehicle | **Drive** | the plot — does the box match the returns off the chassis? |
| Scanner mounting | **Drive** | the plot — is the wall square across the top? |
| Collision guard | **Drive** | Ahead, and clear fwd/rev/turn |
| Scan matching, grid, mapping gate | **Map** | SLAM cpu, Match correction |
| Odometry | **Map** | Distance driven, Encoder counts |
| Explorer | **Map** | the map, auto-map state |
| Floor check, markers | **Vision** | the grid drawn on the live picture |
| Camera mounting, objects | **Vision** | the live picture, and what got labelled |

The **Tune** tab is the *index* — everything at once, plus Save and Revert.
It is not where you tune. Both views are the same controls, so a change in one
shows up immediately in the other.

## Saving

Changes apply live but are **not** written to the card until you save.

The **SAVE** button sits in the sticky top bar, visible from every tab, so you
can save from wherever you were tuning:

- **`SAVED`**, grey — everything matches `tuning.json` on the Pi.
- **`SAVE 3`**, amber — three values would be lost by a restart.

"Would be lost by a restart" is the right test, not "differs from the default":
a value you saved last week is not unsaved work, and flagging it as such
trains you to ignore the indicator.

The write is `fsync`'d and then renamed into place, so it is on the card
rather than sitting in the page cache — a power cut after saving cannot leave
a half-written file, and cannot leave you with nothing.

*Revert to code defaults* on the Tune tab undoes a whole session of fiddling
and saves that, which is the escape hatch when things have got worse and you
cannot remember what you changed.

The registry behind all of it is `test/tuning.py`. Each entry carries its own
one-line explanation, which is what the `?` button shows. **This file is the
narrative** — why the value matters, and how to tune it deliberately rather
than by wiggling.

---

## Measure it, don't wiggle it

Two of these can be **measured** rather than guessed. On the **Drive** tab,
*Measure by pushing*:

1. Press **Measure by pushing**.
2. Push the robot ~0.5 m in a straight line, by hand.
3. Press **Finish**.

It fits the range change across *every* bearing at once. For pure translation
of D, the range at bearing `a` changes by `-D*cos(a - nose)`, so the phase of
that cosine is the true nose bearing and the amplitude is how far the robot
actually went. That distance is measured against the room, independent of the
wheels — so comparing it with what the encoders claimed gives the odometry
scale error directly, with no tape measure and no assumption that the wheels
did not slip.

**One push calibrates both** `lidar_yaw` and `counts_per_rev`, and each result
comes with a residual and a quality score, so "is it right yet" has an answer.
Quality below 0.25 means the push curved, something moved in the room, or it
was too short — the panel says so rather than handing you a confident number.

Push rather than drive: wheels off the ground produce encoder counts with no
translation, which is exactly the thing being measured.

Two subtleties the maths has to handle, both of which produce plausible wrong
answers if skipped — a wall at an oblique angle amplifies the range change by
`1/cos(incidence)`, and that change measures travel along the wall's *normal*,
not along the ray. Ignoring either left a 9 degree bias pulled toward the
room's own axes. Detail in `test/calibrate.py`.

---

## How to tune anything, in general

1. **Change one thing.** Two at once and you learn nothing from the result.
2. **Watch the right number.** The Tune tab shows SLAM cpu, match correction,
   scan rate, points per turn, scans mapped and distance driven. Every knob
   below says which of those it should move.
3. **Drive a repeatable path.** A 4 m straight line, or one lap of a room.
   Comparing two runs down different corridors tells you nothing.
4. **Save only what earns it.** Most things you try are worse.

### The budget you are spending

The Pi 4 runs SLAM at 5 Hz — 200 ms per cycle. A scan match at the defaults
costs about 96 ms of that. Push **SLAM cpu** past ~150 ms and the loop has no
headroom: updates start landing late, and pose quality gets worse even though
you were trying to improve it.

---

## Vehicle

The footprint, and the first thing to get right — everything else is measured
relative to this box.

| Knob | Default |
|---|---|
| Body length | 300 mm |
| Body width | 400 mm |

It is not cosmetic. It decides three things at once:

1. **What gets thrown away as the robot seeing itself.** A corner-mounted
   scanner sees its own chassis: measured on this robot, 40 of 251 returns
   landed inside the footprint. Left in, they paint a permanent blob around
   the robot and the collision guard believes it is boxed in wherever it
   stands. Set the box too *large* instead and real obstacles silently vanish.
2. **Where the guard measures from.** Clearances are quoted from the leading
   edge, not the centre.
3. **The turning check.** The guard rotates this box on the spot (and moves
   it along the arc of a forward-and-turn) and refuses the move if a scan
   point would end up inside it. A box that is too small makes it agree to
   rotations it cannot physically complete.

Size and LiDAR mounting are saved to `tuning.json` automatically, 1.5 s after
the last change - there is no need to press SAVE.

### Measuring it

Park the robot against a wall and open the **Drive** tab. The scanner returns
off the robot's own body are visible in the plot as points clustered around
the origin — size the box until it covers them and no more. That is a direct
measurement of the footprint, which is otherwise something you would be
guessing with a tape measure and a lot of arithmetic about where the scanner
is relative to the body.

Changes apply live: `HALF_L`, `HALF_W`, the circumscribing radius, the
mapper's own chassis filter and the explorer's copy of the geometry are all
recomputed together, so nothing is left describing the old box.

---

## Scan matching

Odometry predicts where the robot went; scan matching corrects it against the
map already built. This is what makes it SLAM rather than dead reckoning with
a picture.

The matcher is brute force over a small search window. Cost is
`len(lin)² × len(ang) × points/decimate` grid lookups, so **the window is
cubic in cost and the decimate divides it**.

| Knob | Default | What moves |
|---|---|---|
| Matching on | on | SLAM cpu, match correction |
| Min confidence | 0.25 | scans refused |
| Search window | 80 mm | SLAM cpu ↑↑, accuracy |
| Search step | 40 mm | SLAM cpu, precision floor |
| Angle window | **8°** | SLAM cpu, heading |
| Angle step | 2° | SLAM cpu |
| Coarse pass | 3 | recovery from large errors |
| Decimate | 3 | SLAM cpu ↓↓ |

### The angle window was the bottleneck

It shipped at ±4° and **saturated on 58% of matches** — the matcher wanted to
rotate further than it was allowed to, every other frame. Measured over a 12 m
circuit with 3.5% slip and 8°/min of drift:

| Angle window | Final error | Heading | Cost |
|---|---|---|---|
| ±4° (as shipped) | 112 mm | 2.4° | 8.8 ms |
| **±8°** | **24 mm** | **0.4°** | **13.1 ms** ← default |
| ±12° | 24 mm | 0.4° | 17.5 ms |

±12° buys nothing over ±8°, so 8 is the knee. A *finer* linear step is
actively worse — 20 mm steps gave 180 mm, because searching below the 50 mm
grid resolution just measures noise.

### Min confidence

Confidence is how **pinned** the pose is, per axis: score at the winner
against the score one grid cell away along each axis, worst axis deciding.
Sliding freely along a corridor scores **0.11**; a normal room scores
**0.62**. Below the threshold the pose stays on odometry and the scan is *not*
mapped — a gap gets filled on the next pass, a smear never leaves.

An earlier version measured peak height and sharpness instead, and scored the
corridor 0.88 against the room 0.87 — indistinguishable, because the many
candidates that score badly keep "peak vs mean" high even while the pose
slides. Set it to 0 to accept everything, which is what this used to do.

### Coarse pass

Searches a window 3× wider at 3× the step first, then refines. Without it an
error larger than the fine window can never be recovered: a sudden 250 mm
error left **209 mm** of residual with the coarse pass off and **53 mm** with
it on.

**Narrower than the default and accuracy collapses** — the true offset falls
outside the search window, so the match locks onto a wrong local peak. That
cliff is why "just make it smaller to go faster" is the wrong instinct here.

**If SLAM cpu is too high, raise Decimate first.** The scan is only ~200
points; decimate 3 leaves ~66, which is plenty to match on, and it divides the
cost linearly instead of cubically.

**Search step below 40 mm is measuring noise** — the grid is 50 mm, so a finer
search cannot resolve anything real, and you pay for it quadratically.

One caveat: with a *perfect* heading, matching slightly hurts (59 mm vs 43 mm)
— it adds search noise to an estimate that was already right. It pays off as
soon as heading drifts at all, which on this robot it always does.

---

## Occupancy grid

Each LiDAR return is evidence twice: the endpoint is probably occupied, and
everything along the ray to it is probably empty. Accumulated as log-odds so
repeated looks reinforce and one bad reading cannot ruin a cell.

| Knob | Default | Notes |
|---|---|---|
| Occupied evidence | 0.85 | Higher: walls appear faster, moving objects smear |
| Free evidence | −0.40 | Smaller in magnitude, but applies to *every* cell on *every* ray |
| Certainty clamp | 5.0 | Ceiling on confidence |
| Max mapping range | 4000 mm | |
| Min mapping range | 120 mm | Inside the chassis |
| Free-ray fraction | 2 | Cast free rays for 1 point in N |

**Why free evidence is smaller and still wins.** Occupied evidence lands on
one cell per return. Free evidence lands on every cell along every ray — tens
per return. So free accumulates far faster in practice, which is exactly what
you want: a person who walks through the room should not leave a permanent
wall behind them.

**The clamp is what lets the map change its mind.** Without it a cell seen a
thousand times becomes so certain that no amount of new evidence can move it,
and a door that opens stays a wall forever.

**Max mapping range is not about seeing further.** Far returns are noisier in
*angle*, and one bad long ray erases a corridor of real cells on its way out.
Raising this in a large open space helps; raising it in a cluttered house
usually makes the map worse.

**Free-ray fraction is the cheapest CPU you can buy.** Ray casting dominates
the cost of integrating a scan. Setting it to 2 halves that, and the map
barely notices, because neighbouring rays sweep almost the same cells.

---

## Mapping gate

| Knob | Default |
|---|---|
| Move before mapping | 40 mm |
| Turn before mapping | 4° |

Scans are only integrated once the robot has actually moved. Integrating
hundreds of identical scans while parked makes the map over-confident about
one viewpoint and drowns out everything seen later. Set both to 0 and watch
**Scans mapped** climb while the robot sits still — that is the failure this
prevents.

---

## Odometry

**This is where map scale comes from.** If the map comes out uniformly too
large or too small, it is one of these two and nothing else.

| Knob | Default |
|---|---|
| Counts per rev | 330 |
| Wheel diameter | 68 mm |
| Track width | 340 mm |
| Left / right encoder sign | +1 / −1 |

### Calibrating counts-per-rev

The number was originally assumed to be 1320 — 11 PPR × 30:1 gearbox × 4 for
quadrature edges. It is actually **330**: gpiozero's `RotaryEncoder` counts
full quadrature *cycles*, not edges, so the ×4 does not apply. That was found
by driving a 1.0 s burst, reading 188 counts, and seeing only ~110 mm of real
travel on the LiDAR — a factor of ~4 out.

Two ways to tune it:

**Measured** — *Measure by pushing* on the Drive tab. Push half a metre, press
Finish, press *Use N counts/rev*. No tape measure needed.

**By hand** —
1. Mark the floor. Drive a straight line of a metre or so.
2. Compare **Distance driven** on the Map tab against a tape measure.
3. `new = old × (reported ÷ actual)`.
4. Repeat once to confirm.

**Do not tune wheel diameter as well.** It and counts-per-rev scale distance
identically, so from the map alone they are indistinguishable — tune both and
you can no longer say which was wrong. Measure the wheel with calipers, fix it,
and tune only counts-per-rev.

### Encoder signs

Push the robot forward by hand and watch **Encoder counts** (Map tab). **Both
must increase.** If one decreases, flip its sign here.

Symptom of getting it wrong: the map builds mirrored, or the robot appears to
reverse through its own map. Note this is a *different* thing from the drive
inversion toggles on the Sensors tab — those fix which way the motor turns and
say nothing about which way the encoder counts.

---

## LiDAR mounting

| Knob | Default | |
|---|---|---|
| X (forward +) | −150 mm | It is on the **back left corner**, not the centre |
| Y (left +) | +200 mm | |
| Rotation | 22° | |

**Rotation is the single most consequential number in this file.** It decides
whether "ahead" means ahead. Get it wrong and the collision guard checks the
wrong direction while the plot still looks entirely sensible — which is how
the robot jams for no visible reason.

It was measured twice, in opposite directions:

- forward 574 mm → nose at 22.7°
- reverse 612 mm → tail at 201.7°, so nose at 21.7°
- the two agree to 1.0°

For pure translation the range-change rate across bearings follows
`−cos(a − nose)`, so the nose is found by fitting that cosine over *all*
bearings (the first Fourier harmonic), not by picking whichever single bearing
closed fastest. The naive argmin approach produced an earlier wrong answer of
250° — with coarse bins and a short move it just tracks noise.

**Position matters as much as rotation**, and is easier to forget. The scanner
sits 250 mm from the body centre, so every return has to be translated as well
as rotated. Skip it and the scan origin orbits the true centre of rotation:
the map swings ~250 mm every time the robot turns on the spot.

**Measured:** *Measure by pushing* on the Drive tab, then *Use N rotation*.
This is the one to use — it is how the current 22 degrees was arrived at, and
it gives a residual so you know when to stop.

**By eye:** face a flat wall, open the Drive tab, and nudge Rotation with the
+-1 degree buttons until the wall sits square across the top of the plot.
Reset the map afterwards.

---

## Collision guard

| Knob | Default | |
|---|---|---|
| Stop distance | 350 mm | |
| Safety margin | 50 mm | Clearance around the footprint |
| Creep margin | 25 mm | |
| Creep throttle | 0.45 | |
| Guard sector | 50° | Forward wedge, for display and the *Ahead* number |

**Safety margin was 80 and is now 50 for a reason.** At 80 the robot needed a
330 mm radius of clear floor just to rotate, which a real house rarely offers
next to furniture. It wedged itself repeatedly, with every direction refused,
and had to be lifted out by hand.

**Creep is the escape hatch.** When *every* direction is blocked, the robot may
still move slowly in whichever has the most room, provided that is at least
the creep margin. A guard that cannot be escaped is a guard that gets switched
off — which is worse than a guard that occasionally creeps.

### One rule, per move

The guard allows a move only if no LiDAR point would end up inside the
truck's outline along the path that move actually drives - using the same
left/right wheel mix as the motors, so an arc is checked as the pivot it is:

| Move | What is checked |
|---|---|
| straight | a lane the truck's width + **15 mm** each side, out to the stop distance + margin |
| spin on the spot | the outline rotated up to **30°** the way you asked, **25 mm** clear |
| forward/back + turn | the outline moved along the real arc, **10 mm** clear when you drive, **20 mm** when the explorer does |

30°, not a few degrees: a scan arrives every ~90 ms and the truck coasts, so
it turns 15-30° between "the scan shows it" and "stopped". If only one part
of a combined move is unsafe, that part alone is dropped (straight on, or
turn only). A move that takes the truck *away* from something already close
is never refused, so it cannot trap itself. The Drive plot draws it: green is
free floor, red is where the truck's centre cannot go, and the four arrows
are the guard's verdicts.

The camera floor check no longer vetoes anything - with the camera tilt
unmeasured it read walls as drop-offs and stalled the truck at random.
**Nothing detects stair edges.**

---

## Explorer

Frontier-based: a frontier is a cell that is known-free and touches unknown
space, which is precisely what an open doorway looks like from inside a room.
So "find the doors and go through them" is not a special case — it is the only
thing the algorithm does.

| Knob | Default | |
|---|---|---|
| Cruise throttle | 1.0 | Fraction of the speed limit; eased off as the heading error grows |
| Spin above | 35° | Bearing error before turning on the spot; it spins until under 10° (hysteresis - no zig-zag) |
| Lookahead | 450 mm | In open space |
| Lookahead near walls | 150 mm | In doorways and beside furniture, so it does not cut door frames |
| Keep-away weight | 8 | How much dearer a route is right beside an obstacle - routes run down the middle |
| Goal reached | 250 mm | |
| Stuck timeout | 6 s | |
| Replan interval | 3.0 s | |
| Free threshold | −1.0 | Log-odds to drive through a cell |
| Obstacle threshold | 0.6 | Log-odds to refuse one |
| Min frontier size | 4 cells | Smaller clusters are noise, not doors |

**The two thresholds are deliberately asymmetric.** A cell must be *clearly*
free before the planner will drive through it, but only mildly suspect to be
treated as an obstacle. Symmetric thresholds make the robot confidently plan
through things.

**The planner and the guard agree.** Walls are grown by half the truck's
width plus the guard's 15 mm side margin, so the planner never routes where
the guard would refuse to drive straight. The steering aims at the path point
one lookahead AHEAD OF THE NEAREST point on the path - not the path's start,
which after 450 mm of driving is behind the truck (that bug spun it round
every few seconds). When the guard blocks for 0.8 s the explorer replans.

Measured in a simulated house (hall, two rooms, 550-700 mm doors, furniture):
every trip arrives, closest pass 60 mm or more, no guard stops; auto-mapping
covers ~61 of 65 m² in about 100 s. The version before this failed three of
four trips and mapped 25 m² in three minutes before getting stuck.

**Replanning is deliberately lazy.** SLAM already costs ~100 ms per update, so
a stale-but-cheap plan plus a reactive guard beats a perfect plan that arrives
too late.

---

## Vision

| Knob | Default | |
|---|---|---|
| Floor luma tolerance | 34 | Brightness change that stops being floor |
| Floor chroma tolerance | 13 | Colour change — the discriminating one |
| Drop-off darkness | 45 | Darker than this = hole, not object |
| Marker fix gain | 0.35 | How hard a tag pulls the pose |
| Marker max range | 2500 mm | |
| Marker sanity limit | 1500 mm | Reject fixes further than this |

**Chroma is tighter than luma on purpose.** U and V barely move across a single
floor surface, so a real change stands out; brightness alone varies with
lighting and shadow across every floor in the world.

**The floor thresholds lean toward false positives.** A false positive stops
the robot, which is annoying. A false negative is a robot at the bottom of the
stairs. If patterned rugs are stopping it constantly, raise chroma tolerance
before luma.

**Marker fix gain is not 1.0** because a hard snap teleports the robot
mid-map and smears the next scan across the jump — scan matching then spends
its whole search window undoing the correction. 0.35 converges over three or
four sightings, which is a second or two of driving past a tag.

**Marker max range exists because range error grows with the square of
distance.** A far tag is not a weak fix, it is a confidently wrong one.

---

## Loop closure

The answer to "the map bends more the further I drive". Scan matching corrects
against a map that has drifted *with* the robot, so on its own the error only
grows. Measured on a simulated 12 m circuit: **9.3 mm per metre on lap one and
19.9 by lap three**.

| Knob | Default |
|---|---|
| Loop closure on | on |
| Revisit radius | 700 mm |
| Closure gain | 0.30 |
| Views needed | 3 |

**Views needed is the one that matters.** A local map built from a *single*
old scan is too sparse to match against — with one keyframe, closure measured
*worse* than none (73 mm vs 50 at two laps, 128 vs 89 at four) because it
pulled the pose toward a bad match. With three it is never worse and clearly
better on long runs.

What it does **not** do is un-bend a map that is already bent. It stops the
drift growing and re-anchors the robot. Rewriting history needs a pose-graph
optimisation over all the keyframes, which is a separate piece of work; the
keyframes are stored so it stays possible.

The **markers** are still the strongest correction available — a printed tag
is the only input outside the closed loop. Loop closure is what you get where
there are no tags.

Measured end to end, with confidence rejection and the wider angle window:

| laps | path | before | after |
|---|---|---|---|
| 1 | 12 m | 112 mm | **24 mm** |
| 2 | 24 m | 319 mm | **41 mm** |
| 3 | 36 m | 718 mm | **70 mm** |
| 4 | 48 m | 844 mm | **82 mm** |

---

## Object detection

YOLOv8n at 320x320, ONNX, ~1 Hz. Roughly 250-400 ms per frame on a Pi 4 with
no accelerator, which at 1 Hz on two threads is about a fifth of one core.

| Knob | Default | |
|---|---|---|
| Detection threshold | 0.40 | how sure before a box is placed at all |
| Sightings to commit | 3 | agreeing views before a label sticks |
| Merge distance | 500 mm | sightings this close are the same object |
| Max placement range | 4000 mm | beyond this the bearing/range pairing is fragile |

**Max placement range** is the subtle one. The camera gives a bearing and the
LiDAR gives the range at that bearing — but a small bearing error at four
metres picks a return off something else entirely, and the label lands on the
wrong object. Near objects are placed accurately; far ones are guesses.

Set the model up once:

```bash
pip install onnxruntime        # on the Pi, inside the venv
# on a laptop, because ultralytics drags in torch:
pip install ultralytics
yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12
# then copy yolov8n.onnx into test/ on the Pi
```

Without either piece, detection says so on the page and everything else
carries on.

---

## Not tunable here

**Grid size and resolution** (`12000 mm`, `50 mm`) stay in `pins.py` and need a
restart. Changing them means reallocating the grid and discarding the map, so
there is no honest way to make them live.

**Camera height and pitch** (`CAM_HEIGHT_MM`, `CAM_PITCH_DEG`) are physical
measurements, not preferences. The cliff detector converts image rows to floor
distances with them, so guessing produces confident, wrong numbers. Measure
them and put them in `pins.py`.

**`MARKER_SIZE_MM`** is whatever you actually printed, measured with a ruler.
Every marker distance scales linearly with it and nothing else in the system
can catch the error.

---

## Where the values are stored

Tuned values go to `test/tuning.json`, alongside `lidar_cal.json`,
`drive_invert.json` and `places.json`. It is loaded at startup, so a good
value survives the restart that follows finding it.

**Only values that differ from the code default are written.** That way, a
later improvement to a default in the source is picked up rather than being
permanently masked by a file full of duplicates.

`tuning.json` is Pi-side and gitignored. It is not synced back to Windows — if
you find values worth keeping permanently, put them in `pins.py` (or wherever
the registry points) and sync that.
