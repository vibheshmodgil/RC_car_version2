"""
Measuring the mounting constants instead of guessing them.

Shared library like slam.py. web_nav.py drives it from the Drive tab; nothing
here is run directly.

The problem this solves
-----------------------
Two numbers decide whether the map means anything, and both used to be found
by dragging a slider until the picture looked right:

  LIDAR_YAW_OFFSET   which way the scanner's zero bearing points. Get this
                     wrong and the collision guard checks the wrong direction
                     while the plot still looks entirely sensible - which is
                     how the robot jams for no visible reason.

  COUNTS_PER_REV     how far one encoder count is. Get it wrong and the whole
                     map comes out uniformly too large or too small, which is
                     almost impossible to see by eye because everything scales
                     together.

Eyeballing a slider gives you a number with no error bar and no way to know
you are done. This gives a measurement, a residual, and a verdict.

How it works
------------
Push the robot forward in a straight line. For pure translation of distance D,
the range at scanner bearing `a` changes by

    dr(a) = -D * cos(a - nose)

- ranges shrink fastest dead ahead, grow fastest dead astern, and are
unchanged abeam. One cosine cycle across the scan, whose phase is the nose
bearing and whose amplitude is how far the robot actually travelled. Both
unknowns fall out of one least-squares solve for the translation vector.

Fitting ALL bearings, rather than picking whichever single bearing closed
fastest, is the whole point. The argmin approach is what produced an earlier
wrong answer of 250 degrees on this robot: with coarse bins and a short move
it just tracks noise.

Two corrections that a naive version gets wrong
-----------------------------------------------
The formula above is exact only where the ray meets the surface square on. A
room is flat walls at every angle, so:

  1. Moving D toward an OBLIQUE wall changes the range by D/cos(incidence),
     amplified differently at every bearing.
  2. That change measures translation along the wall's NORMAL, not along the
     ray - so each sample constrains the answer at a different angle than the
     bearing it was taken at.

Skipping either leaves a bias pulled toward the room's own axes. Measured on
synthetic 4x3 m rooms: 9 degrees of nose error and 23% of distance error,
while still looking like a clean confident result. Both corrections applied,
the same rooms give under 2 degrees and under 6%.

One push, two calibrations
--------------------------
The amplitude is a distance measured by the LiDAR against the room - which is
independent of the wheels. Comparing it with what the encoders claimed over
the same push gives the odometry scale error directly, with no tape measure
and no assumption that the wheels did not slip.

Push, do not drive
------------------
The robot is pushed by hand rather than driven. The motor drivers currently
fitted cannot survive a stall (WIRING.md section 8), so the robot lives with
its wheels off the ground - and wheels in the air produce encoder counts with
no translation, which is exactly the thing being measured. Pushing also
removes wheel slip from the encoder side of the comparison.
"""

import math

# Bearings are binned before differencing. A scan is ~200 points spread over
# 360 degrees, so the bins have to be wide enough that the same bin is
# populated in both scans - 5 degrees gives ~72 bins and reliably fills.
BIN_DEG = 5.0

# A bearing whose range changed by more than this is looking at something that
# moved, or has wrapped onto a different surface entirely (a doorway edge
# sliding past). Those are not translation and they drag the fit.
MAX_CHANGE_MM = 900.0

# Below this many usable bearings the fit is not worth reporting.
MIN_BINS = 18


def _bin_scan(points, min_mm=150.0, max_mm=6000.0):
    """[(bearing, dist)] -> {bin_index: median distance}.

    Median, not mean: a bin spans 5 degrees and may straddle an edge where
    returns jump between a near and a far surface. A mean lands in the gap
    between them, which is a distance nothing is at.
    """
    buckets = {}
    for a, d in points:
        if d < min_mm or d > max_mm:
            continue
        buckets.setdefault(int((a % 360.0) // BIN_DEG), []).append(d)
    out = {}
    for k, vals in buckets.items():
        vals.sort()
        out[k] = vals[len(vals) // 2]
    return out


def fit_translation(scan_a, scan_b):
    """Nose bearing and distance travelled, from two scans of a straight push.

    Returns a dict with:
        nose_deg     scanner bearing that points along the direction of travel
        distance_mm  how far the LiDAR says the robot moved
        bins         how many bearings contributed
        residual_mm  RMS of what the cosine could not explain
        quality      0-1, and the thing to look at before believing any of it

    Returns None when there is not enough overlap to say anything.
    """
    a = _bin_scan(scan_a)
    b = _bin_scan(scan_b)
    common = [k for k in a if k in b and abs(b[k] - a[k]) <= MAX_CHANGE_MM]
    if len(common) < MIN_BINS:
        return None

    # Correct for surface incidence BEFORE fitting.
    #
    # dr = -D*cos(a-nose) is only exact where the ray meets the surface
    # square on. A room is flat walls at every angle, and moving D toward an
    # oblique wall changes the range by D/cos(incidence) - amplified, and
    # amplified differently at every bearing. Fitting the raw differences
    # therefore biases both the phase and the amplitude. Measured on
    # synthetic 4x3 m rooms: up to 8 degrees of nose error and 18% of
    # distance error, worst at bearings away from the room's axes.
    #
    # For a flat wall, r(a) = d/cos(a-perp), so dr/da = r*tan(incidence) and
    #   cos(incidence) = 1 / sqrt(1 + (r'/r)^2)
    # with r' the range gradient in mm per radian, taken from the first scan.
    # Multiplying the measured change by that puts every bearing back onto
    # the same footing.
    step = math.radians(BIN_DEG)
    sc = ss = 0.0
    used = []
    for k in common:
        kp, km = (k + 1) % int(360 // BIN_DEG), (k - 1) % int(360 // BIN_DEG)
        if kp not in a or km not in a:
            continue                      # no gradient available at an edge
        ang = math.radians((k + 0.5) * BIN_DEG)
        grad = (a[kp] - a[km]) / (2.0 * step)

        # Incidence, and where the surface's NORMAL points.
        #
        # For a flat wall r(a) = d/cos(a-p) with p the perpendicular bearing,
        # so dr/da = r*tan(a-p) and the incidence angle is atan(r'/r).
        inc = math.atan2(grad, max(a[k], 1.0))
        # A ray skimming along a wall says almost nothing about translation
        # and is enormously sensitive to noise. Drop it.
        if abs(math.cos(inc)) < 0.35:
            continue

        # Two corrections, and the second is the one that is easy to miss.
        #
        # 1. Moving D toward an OBLIQUE wall changes the range by
        #    D/cos(incidence), so multiply the measured change by cos(inc) to
        #    recover the true projection.
        #
        # 2. That projection is along the wall's NORMAL, not along the ray.
        #    So this sample constrains the translation at angle (a - inc),
        #    not at a. Fitting it at `a` is what left a 9 degree bias pulled
        #    toward the room's own axes - a wrong answer that looks entirely
        #    reasonable, which is the failure mode this module exists to stop.
        perp = ang - inc
        dr = (b[k] - a[k]) * math.cos(inc)
        sc += dr * math.cos(perp)
        ss += dr * math.sin(perp)
        used.append((k, dr, perp))

    if len(used) < MIN_BINS:
        return None
    n = len(used)

    # Least squares for the translation vector, NOT a Fourier projection.
    #
    # Writing dr = -(Dx*cos a + Dy*sin a) makes the model linear in the two
    # unknowns, and solving it as least squares is correct for whatever
    # bearings happen to be usable. A Fourier projection - which is what this
    # did first - is only equivalent to least squares when the bearings are
    # UNIFORMLY spaced. They are not: oblique bins get dropped above, and a
    # room's walls are not sampled evenly to begin with. That non-uniformity
    # leaks straight into the phase. Measured on synthetic rooms: 9 degrees
    # of nose error, biased toward the room's own axes, which is exactly the
    # kind of plausible-looking wrong answer this whole module exists to
    # avoid.
    scc = sss = scs = 0.0
    for _k, _dr, perp in used:
        c, s_ = math.cos(perp), math.sin(perp)
        scc += c * c
        sss += s_ * s_
        scs += c * s_
    det = scc * sss - scs * scs
    if abs(det) < 1e-9:
        return None                       # bearings too clustered to solve
    Dx = (-sc * sss + ss * scs) / det
    Dy = (-ss * scc + sc * scs) / det

    nose = math.degrees(math.atan2(Dy, Dx)) % 360.0
    dist = math.hypot(Dx, Dy)

    # What the model failed to explain. A clean straight push on a static room
    # leaves tens of mm; a turn mixed in, or someone walking past, leaves
    # hundreds.
    err = 0.0
    for _k, dr, perp in used:
        pred = -dist * math.cos(perp - math.radians(nose))
        err += (dr - pred) ** 2
    residual = math.sqrt(err / n)

    # Quality is the fit's signal against its noise, squashed to 0-1. A push
    # that barely moved has a tiny amplitude and no usable phase, so distance
    # gates it as well.
    q = 0.0
    if dist > 1.0:
        q = max(0.0, min(1.0, (dist / max(residual, 1.0)) / 8.0))
    if dist < 80.0:
        q *= dist / 80.0                  # too short to trust regardless

    return {"nose_deg": round(nose, 1), "distance_mm": round(dist, 1),
            "bins": n, "residual_mm": round(residual, 1),
            "quality": round(q, 2)}


def verdict(fit, encoder_mm, current_yaw, current_cpr):
    """Turn a fit into something a person can act on.

    Deliberately opinionated: it says whether to believe the result and what
    the new values would be, rather than printing numbers and leaving the
    reader to work out whether a residual of 140 mm is fine.
    """
    if fit is None:
        return {"ok": False,
                "message": "Not enough of the room was visible in both scans. "
                           "Push in a straight line, somewhere with walls "
                           "around, and try again."}
    out = dict(fit)
    out["ok"] = True
    out["current_yaw"] = round(current_yaw, 1)
    out["yaw_delta"] = round(((fit["nose_deg"] - current_yaw + 180) % 360) - 180, 1)

    if fit["quality"] < 0.25:
        out["ok"] = False
        out["message"] = ("Fit too noisy to use (quality %.2f, residual %.0f mm). "
                          "Usually a push that curved, something moving in the "
                          "room, or too short a distance."
                          % (fit["quality"], fit["residual_mm"]))
        return out
    if fit["distance_mm"] < 150.0:
        out["ok"] = False
        out["message"] = ("Only %.0f mm of travel. Push at least 300 mm - the "
                          "phase gets sharper the further it goes."
                          % fit["distance_mm"])
        return out

    # Odometry scale. The LiDAR measured the room; the encoders measured the
    # wheels. If they disagree, the wheels are wrong, because the room is not
    # moving.
    if encoder_mm and encoder_mm > 20.0:
        ratio = encoder_mm / fit["distance_mm"]
        out["encoder_mm"] = round(encoder_mm, 1)
        out["scale_error_pct"] = round((ratio - 1.0) * 100.0, 1)
        out["implied_cpr"] = int(round(current_cpr * ratio))
    else:
        out["encoder_mm"] = round(encoder_mm or 0.0, 1)
        out["scale_error_pct"] = None
        out["implied_cpr"] = None

    bits = ["Travelled %.0f mm, nose at %.1f°" % (fit["distance_mm"], fit["nose_deg"])]
    if abs(out["yaw_delta"]) < 2.0:
        bits.append("which matches the current %.1f° — nothing to change." % current_yaw)
    else:
        bits.append("current setting is %.1f°, so it is out by %+.1f°."
                    % (current_yaw, out["yaw_delta"]))
    if out["implied_cpr"] and abs(out["scale_error_pct"]) > 3.0:
        bits.append("Encoders claimed %.0f mm, so counts-per-rev should be "
                    "about %d rather than %d (%+.1f%%)."
                    % (encoder_mm, out["implied_cpr"], current_cpr,
                       out["scale_error_pct"]))
    elif out["implied_cpr"]:
        bits.append("Encoders agree to %+.1f%%, so counts-per-rev looks right."
                    % out["scale_error_pct"])
    out["message"] = " ".join(bits)
    return out


class PushMeasure:
    """Holds the 'before' half of a measurement between two HTTP calls."""

    def __init__(self):
        self.scan = None
        self.counts = None
        self.t = 0.0
        self.last = None            # the most recent verdict, for the page

    def start(self, scan, counts, now):
        self.scan = list(scan)
        self.counts = counts
        self.t = now
        return len(self.scan)

    @property
    def armed(self):
        return self.scan is not None

    def finish(self, scan, counts, mm_per_count, current_yaw, current_cpr):
        if not self.armed:
            return {"ok": False, "message": "Press Start first, then push."}
        fit = fit_translation(self.scan, scan)
        # Mean of the two sides: a straight push should turn them equally, and
        # averaging halves the effect of one wheel being nudged.
        enc = 0.0
        if self.counts and counts:
            enc = abs(((counts[0] - self.counts[0]) +
                       (counts[1] - self.counts[1])) / 2.0) * mm_per_count
        self.last = verdict(fit, enc, current_yaw, current_cpr)
        self.scan = self.counts = None
        return self.last
