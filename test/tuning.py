"""
Live tuning — every number that decides whether SLAM works, in one registry.

Shared library like pins.py and slam.py. web_nav.py imports it and serves the
registry to the Tune tab; nothing here is run directly.

    docs/TUNING.md      what each of these does, and how to tune it

Why a registry and not a pile of routes
---------------------------------------
These values used to live in five files — pins.py, slam.py, explore.py,
cliff.py and web_nav.py — and changing one meant editing on Windows, syncing
to the Pi, and restarting. For a value you want to try four settings of, that
is a four-minute loop for ten seconds of thinking.

Worse, there was no list. Nothing anywhere said "these thirty-five numbers are
what mapping depends on", so the only way to find them was to read all five
files.

This file is that list. Each entry knows where the value actually lives, what
range is sane, and what it does. The UI renders itself from the registry, so
adding a tunable is one line here and no UI work at all, and the same entry
feeds the page, the JSON API and the docs.

Where values live
-----------------
Three kinds of target, because the code was not written with this in mind and
bending it into one shape would be worse than describing three:

  ("module", "slam", "L_OCC")        a module-level global
  ("attr", "matcher", "decimate")    an attribute on a registered object
  ("call", "matcher_window")         a setter function, when a change needs
                                     more than an assignment

Objects are registered by name at startup with `bind()`, so this module never
imports web_nav and there is no cycle.

Persistence
-----------
Saved to tuning.json beside the scripts, like lidar_cal.json and places.json.
Loaded at startup, so a value you found by tuning survives the restart that
follows. `revert()` puts everything back to the code defaults, which is the
escape hatch when a session of fiddling has made things worse and you cannot
remember what you changed.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning.json")


class Tunable:
    """One number, where it lives, and what happens if you move it."""

    def __init__(self, key, label, group, target, lo, hi, step,
                 doc="", unit="", kind="float", restart=False):
        self.key = key
        self.label = label
        self.group = group
        self.target = target
        self.lo, self.hi, self.step = lo, hi, step
        self.doc = doc
        self.unit = unit
        self.kind = kind              # float | int | bool
        self.restart = restart        # needs a restart to fully take effect
        self.default = None           # captured at bind time, not written here

    def coerce(self, v):
        if self.kind == "bool":
            return bool(v)
        v = float(v)
        v = max(self.lo, min(self.hi, v))
        return int(round(v)) if self.kind == "int" else v

    def as_dict(self, value):
        return {"key": self.key, "label": self.label, "group": self.group,
                "lo": self.lo, "hi": self.hi, "step": self.step,
                "kind": self.kind, "unit": self.unit, "doc": self.doc,
                "restart": self.restart, "default": self.default,
                "value": value}


def T(key, label, group, target, lo, hi, step, doc="", unit="",
      kind="float", restart=False):
    return Tunable(key, label, group, target, lo, hi, step, doc, unit,
                   kind, restart)


# ---------------------------------------------------------------------------
# The registry. Order here is the order on the page.
# ---------------------------------------------------------------------------

VEH = "Vehicle"
SCAN = "Scan matching"
GRID = "Occupancy grid"
LOOP = "Loop closure"
GATE = "Mapping gate"
ODOM = "Odometry"
MOUNT = "LiDAR mounting"
GUARD = "Collision guard"
EXPL = "Explorer"
CAMM = "Camera mounting"
VIS = "Vision"
DET = "Object detection"

REGISTRY = [
    # --- the vehicle itself ------------------------------------------------
    #
    # First, because everything else is measured relative to this box. Park
    # the robot against a wall and size it against what the scanner actually
    # returns off its own chassis - that is the only direct measurement of
    # the footprint available, and it is why these live next to the plot.
    T("truck_len", "Body length", VEH, ("call", "truck"),
      100, 900, 5, unit="mm",
      doc="Front to back. Returns inside this box are discarded as the robot "
          "seeing itself. Too large and real obstacles vanish; too small and "
          "the robot paints a permanent blob around itself and believes it is "
          "boxed in wherever it stands. Also sets how far ahead the guard "
          "starts measuring from."),
    T("truck_wid", "Body width", VEH, ("call", "truck"),
      100, 900, 5, unit="mm",
      doc="Side to side. With length this gives the circumscribing radius the "
          "turning check uses - a skid-steer turns about its centre and its "
          "widest point is a corner, not the nose, so both matter for whether "
          "it will agree to rotate in a tight space."),

    # --- scan matching -----------------------------------------------------
    T("match_enabled", "Matching on", SCAN, ("attr", "slam", "match_enabled"),
      0, 1, 1, kind="bool",
      doc="Off leaves pure dead reckoning. Measured over a 4 m lap: 147 mm of "
          "error without matching, 28 mm with. Turn it off only to see what "
          "odometry alone is doing."),
    T("match_lin_mm", "Search window", SCAN, ("call", "match_window"),
      20, 200, 10, unit="mm",
      doc="How far the matcher will look for a better pose. Too small and the "
          "true offset falls outside the window, so it locks onto a wrong "
          "local peak and accuracy collapses. Too large and cost grows as the "
          "square of this."),
    T("match_lin_step", "Search step", SCAN, ("call", "match_window"),
      10, 80, 5, unit="mm",
      doc="Resolution of the position search. Finer is not better: below the "
          "grid resolution (50 mm) it is measuring noise, at real cost."),
    T("match_ang_deg", "Angle window", SCAN, ("call", "match_window"),
      1, 20, 1, unit="deg",
      doc="How far either side of the current heading to search. The IMU is "
          "good, so this only has to cover its drift between updates."),
    T("match_ang_step", "Angle step", SCAN, ("call", "match_window"),
      0.5, 5, 0.5, unit="deg",
      doc="Resolution of the heading search."),
    T("match_min_conf", "Min confidence", SCAN,
      ("attr", "slam", "min_conf"), 0.0, 0.8, 0.01,
      doc="Below this the match is not believed: the pose stays on odometry "
          "and the scan is NOT added to the map. Confidence measures how "
          "PINNED the pose is per axis - sliding freely along a corridor "
          "scores 0.11, a normal room 0.62, so 0.25 sits in the gap. Set it "
          "to 0 to accept everything, which is what this used to do and why "
          "corridors smeared the map."),
    T("match_coarse", "Coarse pass", SCAN, ("call", "match_coarse"),
      1, 6, 1, kind="int",
      doc="Search a window this many times wider first, at this many times "
          "the step, then refine. 1 disables it. Without a coarse pass an "
          "error larger than the fine window can never be recovered - "
          "measured, a sudden 250 mm error left 209 mm of residual with it "
          "off and 53 mm with it on."),
    T("match_decimate", "Decimate", SCAN, ("attr", "matcher", "decimate"),
      1, 8, 1, kind="int",
      doc="Use 1 point in N for matching. The scan is only ~200 points, so 3 "
          "leaves ~66 — enough to match on, a third of the cost. Raise this "
          "first when SLAM ms is too high."),

    # --- loop closure --------------------------------------------------------
    T("loop_enabled", "Loop closure on", LOOP,
      ("attr", "slam", "loop_enabled"), 0, 1, 1, kind="bool",
      doc="Recognise a place already visited and correct against how it "
          "looked then. Scan matching alone corrects against a map that has "
          "drifted WITH the robot, so error only ever grows - measured 9.3 mm "
          "per metre on the first lap and 19.9 by the third. This is what "
          "stops that."),
    T("loop_radius", "Revisit radius", LOOP,
      ("attr", "slam", "LOOP_RADIUS_MM"), 200, 2000, 50, unit="mm",
      doc="How close to an old keyframe counts as being back in the same "
          "place. Too large and it matches against somewhere else."),
    T("loop_gain", "Closure gain", LOOP,
      ("attr", "slam", "LOOP_GAIN"), 0.05, 1.0, 0.05,
      doc="How hard a closure pulls the pose. Blended rather than snapped, "
          "for the same reason marker fixes are: a jump smears the next scan "
          "across the discontinuity."),
    T("loop_min_keys", "Views needed", LOOP,
      ("attr", "slam", "LOOP_MIN_KEYS"), 1, 10, 1, kind="int",
      doc="How many old views of a place are needed before trusting a "
          "closure. One scan is too sparse to match against: with a single "
          "keyframe, closure was measurably WORSE than none at 2-4 laps "
          "because it pulled the pose toward a bad match."),

    # --- occupancy grid ----------------------------------------------------
    T("l_occ", "Occupied evidence", GRID, ("module", "slam", "L_OCC"),
      0.1, 3.0, 0.05,
      doc="Log-odds added to a cell each time a return lands in it. Higher "
          "makes walls appear faster and moving objects leave smears."),
    T("l_free", "Free evidence", GRID, ("module", "slam", "L_FREE"),
      -3.0, -0.05, 0.05,
      doc="Log-odds added to every cell a ray passes through. Free space is "
          "seen far more often than occupied, so this being smaller in "
          "magnitude than the occupied value still lets free win — which is "
          "what clears a moving obstacle back out of the map."),
    T("l_clamp", "Certainty clamp", GRID, ("module", "slam", "L_CLAMP"),
      1.0, 20.0, 0.5,
      doc="Ceiling on how certain a cell may become. Without it a cell seen a "
          "thousand times can never be corrected by new evidence, and a door "
          "that opens stays a wall forever."),
    T("map_max_mm", "Max mapping range", GRID,
      ("module", "slam", "MAX_MAP_RANGE_MM"), 1000, 8000, 250, unit="mm",
      doc="Returns beyond this are ignored for mapping. Far returns are "
          "noisier in ANGLE, and one bad long ray erases a corridor of real "
          "cells on its way out."),
    T("map_min_mm", "Min mapping range", GRID,
      ("module", "slam", "MIN_MAP_RANGE_MM"), 50, 500, 10, unit="mm",
      doc="Returns closer than this are inside the chassis and always "
          "spurious."),
    T("free_every", "Free-ray fraction", GRID,
      ("attr", "grid", "FREE_EVERY"), 1, 6, 1, kind="int",
      doc="Cast free-space rays for 1 point in N. Ray casting dominates the "
          "cost of integrating a scan; 2 halves it and the map barely "
          "notices, because neighbouring rays sweep nearly the same cells."),

    # --- mapping gate ------------------------------------------------------
    T("gate_move_mm", "Move before mapping", GATE,
      ("attr", "slam", "MOVE_MM"), 0, 300, 10, unit="mm",
      doc="Do not integrate a scan until the robot has moved this far. "
          "Integrating hundreds of identical scans while parked makes the map "
          "over-confident about one viewpoint and drowns out later evidence."),
    T("gate_turn_deg", "Turn before mapping", GATE,
      ("call", "gate_turn"), 0, 30, 1, unit="deg",
      doc="Same idea for rotation."),

    # --- odometry ----------------------------------------------------------
    T("counts_per_rev", "Counts per rev", ODOM,
      ("attr", "odom", "cpr"), 50, 3000, 1, kind="int",
      doc="Encoder counts per wheel revolution. THE calibration number: if "
          "the map comes out uniformly too large or too small, this is why. "
          "Measured 330 on this robot. Tune live by driving a known distance "
          "and comparing Distance driven against a tape measure."),
    T("wheel_diam_mm", "Wheel diameter", ODOM,
      ("attr", "odom", "wheel"), 20, 200, 0.5, unit="mm",
      doc="Also scales distance linearly. Measure it rather than tuning it — "
          "it and counts-per-rev are indistinguishable from the map alone, so "
          "tuning both leaves you unable to say which was wrong."),
    T("track_mm", "Track width", ODOM,
      ("attr", "odom", "track_mm"), 100, 800, 5, unit="mm",
      doc="Centre-to-centre between the left and right wheels. Only affects "
          "heading from wheels, which this robot mostly ignores in favour of "
          "the IMU — so it matters least of the three."),
    T("enc_sign_left", "Left encoder sign", ODOM,
      ("attr", "slamr", "ENC_SIGN_LEFT"), -1, 1, 2, kind="int",
      doc="Flip if pushing the robot forward makes the left count DECREASE. "
          "Symptom of getting it wrong: the map builds mirrored, or the robot "
          "reverses through its own map."),
    T("enc_sign_right", "Right encoder sign", ODOM,
      ("attr", "slamr", "ENC_SIGN_RIGHT"), -1, 1, 2, kind="int",
      doc="Same, for the right side. Measured -1 on this robot."),

    # --- lidar mounting ----------------------------------------------------
    T("lidar_x", "LiDAR X (fwd +)", MOUNT, ("attr", "lidar_cal", "x"),
      -400, 400, 5, unit="mm",
      doc="Where the scanner sits relative to the body centre. It is on a "
          "CORNER, not the middle, so without this the scan origin orbits the "
          "true centre of rotation and the map swings ~250 mm every time the "
          "robot turns on the spot."),
    T("lidar_y", "LiDAR Y (left +)", MOUNT, ("attr", "lidar_cal", "y"),
      -400, 400, 5, unit="mm",
      doc="As above, sideways."),
    T("lidar_yaw", "LiDAR rotation", MOUNT, ("attr", "lidar_cal", "yaw"),
      -180, 180, 1, unit="deg",
      doc="The one number that decides whether 'ahead' means ahead. Get it "
          "wrong and the collision guard checks the wrong direction while the "
          "plot still looks sensible — which is how the robot jams for no "
          "visible reason. Measured 22 deg."),

    # --- guard -------------------------------------------------------------
    T("guard_stop_mm", "Stop distance", GUARD, ("attr", "guard", "stop_mm"),
      100, 2000, 50, unit="mm",
      doc="How close the guard lets the robot get before refusing to drive "
          "further in that direction."),
    T("safety_margin_mm", "Safety margin", GUARD,
      ("module", "web_nav", "SAFETY_MARGIN_MM"), 0, 200, 5, unit="mm",
      doc="Clearance added around the footprint. At 80 the robot needed a "
          "330 mm radius of clear floor just to rotate, which a real house "
          "rarely offers next to furniture — it wedged itself repeatedly. 50 "
          "is ample at the duty this thing runs at."),
    T("creep_margin_mm", "Creep margin", GUARD,
      ("module", "web_nav", "CREEP_MARGIN_MM"), 0, 200, 5, unit="mm",
      doc="When EVERY direction is blocked, the robot may still creep in the "
          "least-blocked one provided it has at least this much room. A guard "
          "that cannot be escaped is a guard that gets switched off."),
    T("creep_throttle", "Creep throttle", GUARD,
      ("module", "web_nav", "CREEP_THROTTLE"), 0.1, 1.0, 0.05,
      doc="How slowly it creeps out."),
    T("guard_sector_deg", "Guard sector", GUARD,
      ("module", "web_nav", "GUARD_SECTOR_DEG"), 10, 120, 5, unit="deg",
      doc="Half-angle of the forward wedge shown on the plot and used for the "
          "Ahead reading."),

    # --- explorer ----------------------------------------------------------
    T("expl_cruise", "Cruise throttle", EXPL,
      ("attr", "explorer", "CRUISE"), 0.2, 1.0, 0.05,
      doc="Throttle the explorer asks for. The speed limit caps the actual "
          "duty, so this is a fraction of that."),
    T("expl_spin_enter", "Spin above", EXPL,
      ("attr", "explorer", "SPIN_ENTER"), 15, 90, 5, unit="deg",
      doc="Bearing error beyond which it turns on the spot. It keeps turning "
          "until the error is under 10 degrees, then drives - the gap is "
          "what stops it zig-zagging."),
    T("expl_lookahead", "Lookahead", EXPL,
      ("attr", "explorer", "LOOKAHEAD_MM"), 100, 1500, 50, unit="mm",
      doc="How far down the planned path it aims in open space. Short is "
          "twitchy, long cuts corners."),
    T("expl_lookahead_min", "Lookahead near walls", EXPL,
      ("attr", "explorer", "LOOKAHEAD_MIN_MM"), 100, 600, 25, unit="mm",
      doc="The lookahead in doorways and beside furniture. Shorter follows "
          "the centred path more tightly around door frames."),
    T("expl_clear_weight", "Keep-away weight", EXPL,
      ("module", "explore", "CLEAR_WEIGHT"), 0, 20, 0.5,
      doc="How much more a route costs right beside an obstacle. Higher "
          "keeps routes in the middle of open space; 0 hugs walls."),
    T("expl_goal_mm", "Goal reached", EXPL,
      ("attr", "explorer", "GOAL_REACHED_MM"), 100, 1000, 50, unit="mm",
      doc="How close counts as arrived. Too tight and it dances around a "
          "frontier it can never quite reach."),
    T("expl_stuck_s", "Stuck timeout", EXPL,
      ("attr", "explorer", "STUCK_S"), 2, 30, 1, unit="s",
      doc="No progress for this long and the target is abandoned as "
          "unreachable."),
    T("expl_replan_s", "Replan interval", EXPL,
      ("module", "explore", "REPLAN_EVERY_S"), 0.5, 15, 0.5, unit="s",
      doc="A stale-but-cheap plan plus a reactive guard beats a perfect plan "
          "that arrives too late — SLAM already costs ~100 ms per update."),
    T("expl_free_below", "Free threshold", EXPL,
      ("module", "explore", "FREE_BELOW"), -5, 0, 0.1,
      doc="Log-odds below which the planner will drive through a cell. "
          "Deliberately asymmetric with the obstacle threshold: a cell must "
          "be CLEARLY free to be driven through."),
    T("expl_occ_above", "Obstacle threshold", EXPL,
      ("module", "explore", "OCC_ABOVE"), 0, 5, 0.1,
      doc="Log-odds above which a cell blocks the plan. Only mildly suspect "
          "is enough."),
    T("expl_min_frontier", "Min frontier size", EXPL,
      ("module", "explore", "MIN_FRONTIER_CELLS"), 1, 30, 1, kind="int",
      doc="Smaller clusters are sensor noise, not doors."),

    # --- camera mounting -----------------------------------------------------
    T("cam_yaw", "Camera bearing", CAMM,
      ("module", "web_nav", "CAM_YAW_OFFSET"), -180, 180, 5, unit="deg",
      doc="Which way the lens points on the robot: 0 is straight ahead, "
          "positive is to the left. This is where the cyan wedge sits on the "
          "LiDAR plot and which way a detected marker is reported to lie - so "
          "it has to be right even when the picture looks fine. Aim at "
          "something you can also see on the plot and line the two up."),
    T("cam_rotation", "Picture rotation", CAMM,
      ("call", "cam_rot"), 0, 270, 90, unit="deg", kind="int",
      doc="Which way up the sensor sits in its bracket - a different question "
          "from bearing. 0 and 180 are done in the ISP, so everything "
          "including the floor check sees an upright frame. 90 and 270 rotate "
          "the DISPLAY ONLY: the floor check reads the raw frame and assumes "
          "the bottom row is the nearest floor, which is false on a "
          "sideways camera. Turn the floor check off if you use them."),

    # --- vision ------------------------------------------------------------
    T("cliff_luma", "Floor luma tolerance", VIS,
      ("module", "cliff", "LUMA_TOL"), 5, 120, 1,
      doc="How much darker or brighter than the reference floor a cell may be "
          "before it stops counting as floor. Lower is twitchier on patterned "
          "rugs; higher risks missing a step."),
    T("cliff_chroma", "Floor chroma tolerance", VIS,
      ("module", "cliff", "CHROMA_TOL"), 2, 60, 1,
      doc="Same for colour. This is the discriminating one — U and V barely "
          "move across a single floor surface, so a real change stands out."),
    T("cliff_darker", "Drop-off darkness", VIS,
      ("module", "cliff", "CLIFF_DARKER"), 10, 150, 5,
      doc="Darker than the floor by this much and it is called a hole rather "
          "than an object. A stair well is dramatically darker; a shadow is "
          "darker at the same hue, which the chroma test catches."),
    T("marker_gain", "Marker fix gain", VIS,
      ("module", "markers", "MARKER_FIX_GAIN"), 0.05, 1.0, 0.05,
      doc="How hard a tag sighting pulls the pose. Not 1.0: a hard snap "
          "teleports the robot mid-map and smears the next scan across the "
          "jump. 0.35 converges in three or four sightings."),
    T("marker_max_mm", "Marker max range", VIS,
      ("module", "markers", "MARKER_MAX_MM"), 500, 6000, 250, unit="mm",
      doc="Ignore tags further than this. Range error grows with the SQUARE "
          "of distance, so a far tag is not a weak fix — it is a confidently "
          "wrong one."),
    T("detect_conf", "Detection threshold", DET,
      ("module", "detect", "CONF_MIN"), 0.1, 0.9, 0.05,
      doc="How sure the model has to be before a box is placed at all. Low "
          "fills the map with hallucinations; high misses furniture seen "
          "edge-on from across a room, which is most of it."),
    T("detect_votes", "Sightings to commit", DET,
      ("module", "detect", "COMMIT_VOTES"), 1, 10, 1, kind="int",
      doc="How many agreeing sightings before a label sticks to the map. One "
          "frame is noise, and a single misfire would plant 'bed' in the "
          "hallway permanently with no way to remove it."),
    T("detect_cell_mm", "Merge distance", DET,
      ("module", "detect", "CELL_MM"), 100, 2000, 50, unit="mm",
      doc="Sightings within this of each other are the same object. Furniture "
          "is metre-scale; finer than this splits one sofa into three."),
    T("detect_max_mm", "Max placement range", DET,
      ("module", "detect", "MAX_RANGE_MM"), 500, 6000, 250, unit="mm",
      doc="Beyond this the bearing-to-range pairing gets fragile: a small "
          "bearing error picks a LiDAR return off something else entirely, "
          "and the label lands on the wrong object."),

    T("marker_sanity_mm", "Marker sanity limit", VIS,
      ("module", "markers", "MARKER_SANITY_MM"), 200, 5000, 100, unit="mm",
      doc="A fix further than this from the current pose is not believed. It "
          "means a duplicated tag id, a map entry learned from a bad pose, or "
          "a tag someone moved."),
]

BY_KEY = {t.key: t for t in REGISTRY}
GROUPS = []
for _t in REGISTRY:
    if _t.group not in GROUPS:
        GROUPS.append(_t.group)


# ---------------------------------------------------------------------------
# Binding — where the values actually live
# ---------------------------------------------------------------------------

_objects = {}       # name -> live object
_modules = {}       # name -> module
_defaults = {}      # key -> value at bind time, i.e. the code default
_ondisk = {}        # key -> value currently written in tuning.json


def bind(objects=None, modules=None):
    """Register what the registry points at, then capture the code defaults
    so revert() has something to go back to.

    Called once from web_nav.main() after everything is constructed. Binding
    by name keeps this module from importing web_nav, which would be a cycle.

    Objects and modules are separate dicts on purpose, not one namespace with
    a type check: "slam" means the Slam OBJECT in one target and the slam
    MODULE in another, and both are legitimate. Merging them would silently
    make one shadow the other.

    Modules are passed in rather than looked up in sys.modules, because
    web_nav registers itself and a script run as `python test/web_nav.py` is
    called "__main__" there, not "web_nav".
    """
    _objects.update(objects or {})
    _modules.update(modules or {})
    missing = []
    for t in REGISTRY:
        try:
            t.default = _defaults[t.key] = get(t.key)
        except (KeyError, AttributeError):
            t.default = None            # target not present in this run
            missing.append(t.key)
    return _defaults, missing


def _resolve(target):
    """(container, attribute) for a target, or (None, None) if unavailable."""
    kind = target[0]
    if kind == "module":
        return _modules.get(target[1]), target[2]
    if kind == "attr":
        return _objects.get(target[1]), target[2]
    return None, None


def get(key):
    t = BY_KEY[key]
    if t.target[0] == "call":
        return _CALL_GET[t.target[1]](key)
    obj, attr = _resolve(t.target)
    if obj is None:
        raise KeyError(key)
    return getattr(obj, attr)


def set_one(key, value):
    """Apply one value. Returns the coerced value actually stored."""
    t = BY_KEY[key]
    v = t.coerce(value)
    if t.target[0] == "call":
        _CALL_SET[t.target[1]](key, v)
        return v
    obj, attr = _resolve(t.target)
    if obj is None:
        raise KeyError(key)
    setattr(obj, attr, v)
    return v


# --- targets that need more than an assignment -----------------------------
#
# The search window is four numbers that together generate two precomputed
# lists, and the mapping turn gate is stored in radians while being tuned in
# degrees. Both are wrapped rather than contorting the code they belong to.

def _match_window_get(key):
    m = _objects["matcher"]
    return {"match_lin_mm": m.lin_mm, "match_lin_step": m.lin_step,
            "match_ang_deg": m.ang_deg, "match_ang_step": m.ang_step}[key]


def _match_window_set(key, v):
    m = _objects["matcher"]
    m.configure(**{{"match_lin_mm": "lin_mm", "match_lin_step": "lin_step",
                    "match_ang_deg": "ang_deg",
                    "match_ang_step": "ang_step"}[key]: v})


def _truck_get(key):
    wn = _modules["web_nav"]
    return wn.TRUCK_LENGTH_MM if key == "truck_len" else wn.TRUCK_WIDTH_MM


def _truck_set(key, v):
    # Through set_truck(), never by assignment: HALF_L, HALF_W, CORNER_R,
    # slam.SELF_L/SELF_W and the explorer's geometry all derive from these and
    # would otherwise be left describing the old box.
    wn = _modules["web_nav"]
    if key == "truck_len":
        wn.set_truck(length=v)
    else:
        wn.set_truck(width=v)


def _match_coarse_get(_key):
    return _objects["matcher"].coarse


def _match_coarse_set(_key, v):
    # configure() has to rerun: the coarse offsets are precomputed lists, so
    # changing the factor without rebuilding them changes nothing.
    m = _objects["matcher"]
    m.coarse = int(v)
    m.configure()


def _cam_rot_get(_key):
    return _modules["web_nav"].CAM_ROTATION


def _cam_rot_set(_key, v):
    # Snap to a quarter turn. Anything else is not a rotation the ISP or the
    # overlay alignment can express, and a slider will happily hand you 47.
    _modules["web_nav"].set_cam_rotation(int(round(v / 90.0)) * 90 % 360)


def _gate_turn_get(_key):
    return round(math.degrees(_objects["slam"].TURN_RAD), 2)


def _gate_turn_set(_key, v):
    _objects["slam"].TURN_RAD = math.radians(v)


_CALL_GET = {"match_window": _match_window_get, "gate_turn": _gate_turn_get,
             "truck": _truck_get, "cam_rot": _cam_rot_get,
             "match_coarse": _match_coarse_get}
_CALL_SET = {"match_window": _match_window_set, "gate_turn": _gate_turn_set,
             "truck": _truck_set, "cam_rot": _cam_rot_set,
             "match_coarse": _match_coarse_set}


# ---------------------------------------------------------------------------
# The API web_nav serves
# ---------------------------------------------------------------------------

def saved_value(key):
    """What a restart would load for this key: the file if it has it, the code
    default otherwise. That is the thing a change is 'unsaved' against - not
    the default, which is what the Revert button targets."""
    return _ondisk.get(key, _defaults.get(key))


def dirty():
    """Keys whose live value would be LOST by a restart.

    The distinction from 'differs from default' matters: a value you saved
    last week is not unsaved work, and flagging it as such trains you to
    ignore the indicator.
    """
    out = []
    for t in REGISTRY:
        try:
            v = get(t.key)
        except (KeyError, AttributeError):
            continue
        sv = saved_value(t.key)
        if sv is not None and v != sv:
            out.append(t.key)
    return out


def snapshot():
    """The whole registry with current values, ready to render."""
    out = []
    for t in REGISTRY:
        try:
            out.append(t.as_dict(get(t.key)))
        except (KeyError, AttributeError):
            continue                     # not available in this run
    return {"groups": GROUPS, "items": out, "file": os.path.basename(FILE),
            "dirty": dirty()}


def apply(values):
    """Set many at once. Returns {key: stored_value} for what took."""
    done = {}
    for k, v in (values or {}).items():
        if k not in BY_KEY:
            continue
        try:
            done[k] = set_one(k, v)
        except (KeyError, AttributeError, TypeError, ValueError):
            continue
    return done


def revert():
    """Back to the values the code shipped with."""
    return apply(dict(_defaults))


def save():
    """Persist to tuning.json. Only values that DIFFER from the code default
    are written — so a later change to a default in the source is picked up
    rather than being permanently masked by a file full of duplicates."""
    diff = {}
    for t in REGISTRY:
        try:
            v = get(t.key)
        except (KeyError, AttributeError):
            continue
        if t.default is not None and v != t.default:
            diff[t.key] = v
    tmp = FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"version": 1, "values": diff}, f, indent=1)
        f.flush()
        os.fsync(f.fileno())             # on the card, not just in page cache
    os.replace(tmp, FILE)                # atomic; a half-written file is worse
    global _ondisk
    _ondisk = dict(diff)
    return diff


# --- the known-good profile ----------------------------------------------------
#
# "Revert" goes to the CODE defaults, which also throws away what was measured
# on this truck (its size, the LiDAR's rotation). "Restore known-good" goes to
# code defaults PLUS a small set of values known to map well:
#
#   tuning_good.json          saved from the cockpit on the Pi (Save as known-good)
#   tuning_good.default.json  in the repo - the values the whole house was
#                             mapped with on 2026-09-26; used when the Pi has
#                             not saved its own
#
# Both hold only the values that differ from the code defaults, like tuning.json.

GOOD_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning_good.json")
GOOD_DEFAULT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning_good.default.json")


def _read_values(path):
    try:
        with open(path) as f:
            return json.load(f).get("values", {})
    except (OSError, ValueError, AttributeError):
        return None


def good_profile():
    """(values, source file name) of the known-good profile."""
    for path in (GOOD_FILE, GOOD_DEFAULT_FILE):
        vals = _read_values(path)
        if vals is not None:
            return vals, os.path.basename(path)
    return {}, None


def restore_good():
    """Code defaults, then the known-good values, then saved to tuning.json."""
    vals, src = good_profile()
    revert()
    applied = apply(vals)
    save()
    return {"source": src, "applied": applied}


def save_good():
    """Make what is set now the known-good profile (differences from default only)."""
    diff = {}
    for t in REGISTRY:
        try:
            v = get(t.key)
        except (KeyError, AttributeError):
            continue
        if t.default is not None and v != t.default:
            diff[t.key] = v
    tmp = GOOD_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"version": 1, "values": diff}, f, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, GOOD_FILE)
    return diff


def load():
    """Apply tuning.json, if there is one. Call after bind()."""
    try:
        with open(FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}
    vals = d.get("values", {})
    applied = apply(vals)
    global _ondisk
    _ondisk = {k: v for k, v in vals.items() if k in BY_KEY}
    return applied
