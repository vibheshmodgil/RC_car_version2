"""
Cliff and low-obstacle detection — the half of the world the LiDAR cannot see.

Shared library like slam.py. web_nav.py imports it.

There is no cliff_test.py on purpose: this is one of the few things easier to
judge from the cockpit than from a terminal. The Floor check panel draws the
grid straight onto the live video, so you can see which cell fired and on
what.

The gap this fills
------------------
The scanner sweeps one horizontal plane at one height. Everything outside
that plane is invisible to it, and the list of things that live outside it is
alarming:

  a stair edge        the single failure that ends the robot
  a floor drop        thresholds, a step down to a sunken room
  a low obstacle      a shoe, a cable, a book — under the beam
  a table overhang    the legs are seen, the tabletop is not

The guard in web_nav.py is exact and trustworthy about the plane it can see,
and completely blind above and below it. A robot that trusts it alone will
drive off the top step, and it will do so at full confidence.

How it works
------------
No neural network, no training, no floor model. The floor immediately in
front of the wheels is floor BY DEFINITION — the robot is standing on it. So:

  1. Sample a reference patch at the bottom of the frame. That is floor.
  2. Sample a grid of patches further up the frame, which by perspective is
     further away.
  3. Any patch that does not look like the reference is not floor.

Colour and brightness only, on a 320x240-ish grid of cell means. It costs
about 2 ms. There is nothing clever here and that is the point: a stair edge
is a large, obvious, high-contrast change, and the simple test catches it.

Cliff versus obstacle
---------------------
Both read as "not floor". They are told apart by brightness: a drop-off is
looking into an unlit void and comes back much DARKER than the floor, while
an object in the way is usually a similar brightness and a different hue.
Chroma is what stops a mere shadow being called a cliff — a shadow is darker
at the same hue, a stair well is darker at a different one.

The distinction is advisory. Both block forward motion.

What it gets wrong
------------------
Honestly: patterned rugs, tile grout lines, sharp sunlight edges, and the
boundary between two floor surfaces will all produce false positives. A false
positive stops the robot, which is annoying. A false negative is a robot at
the bottom of the stairs. The thresholds are set accordingly, and RELEARN
after moving between very different floors.

It needs the camera TILTED DOWN. At zero tilt most of the frame is at or
above the horizon and never meets the floor, so five of the six rows have
nothing to check and the near field goes unwatched entirely. 15-25 degrees
down. See CAM_PITCH_DEG.
"""

import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import (  # noqa: E402
    CAM_HEIGHT_MM, CAM_PITCH_DEG, cam_vfov, TRUCK_LENGTH_MM,
)

# Grid over the lower part of the frame. 8 wide covers the body plus a margin
# either side; 6 deep is enough to say near/mid/far without pretending to a
# precision the projection does not have.
#
# The cell is the resolution limit, and it is a MEAN, so a small object gets
# diluted by the floor around it. Verified on synthetic frames: an object
# filling one whole cell trips; one filling half a cell averages to 40 below
# the reference and slips under CLIFF_DARKER.
#
# In practice that is fine. At the nearest row — 179 mm out with the camera
# 120 mm up and 15 degrees down — the frame spans 181 mm of floor, so a cell
# is about 23 mm wide and half a cell is 12 mm. Anything worth stopping for
# is bigger than that, and a stair edge spans the entire width.
COLS = 8
ROWS = 6

# Fraction of frame height the grid covers, measured from the bottom.
# The top half of a forward-tilted frame is wall and ceiling — nothing there
# is floor and testing it only invites false positives.
BAND = 0.55

# How different a cell has to be before it stops being floor.
#
# Luma is 0-255. 34 is roughly "clearly a different surface" while surviving
# the brightness falloff toward the top of the frame that every floor has.
LUMA_TOL = 34.0
# Chroma is the discriminating one and much tighter — U and V barely move
# across a single floor surface, so a real change stands out.
CHROMA_TOL = 13.0

# Darker than the reference by this much, and it is a hole rather than a
# thing. A stair well or a step down is dramatically darker; an object is not.
CLIFF_DARKER = 45.0


def ground_distance(row_frac, height_mm=None, pitch_deg=None, vfov_deg=None):
    """Where a point at this height up the frame meets the floor, in mm ahead.

    row_frac is 0 at the bottom of the frame and 1 at the top.

    Flat-floor pinhole projection: a pixel's angle below the horizon is the
    camera's downward tilt plus its angle below the optical axis, and a ray at
    angle b below horizontal from height h meets the ground at h/tan(b).

    Returns None for anything at or above the horizon, where the ray never
    meets the floor at all — which is exactly what a camera at zero tilt sees
    for most of its frame, and why CAM_PITCH_DEG has to be non-zero.
    """
    h = CAM_HEIGHT_MM if height_mm is None else height_mm
    pitch = CAM_PITCH_DEG if pitch_deg is None else pitch_deg
    vfov = cam_vfov() if vfov_deg is None else vfov_deg
    # +vfov/2 at the bottom of the frame, -vfov/2 at the top.
    below = math.radians(pitch) + math.radians(vfov) * (0.5 - row_frac)
    if below <= math.radians(0.5):
        return None
    d = h / math.tan(below)
    return d if d < 6000 else None       # beyond this the estimate is fiction


class FloorRef:
    """What the floor looks like: one mean per channel, per grid row.

    Per row, not one number for the whole frame, because a floor is not
    uniformly lit — it falls off with distance and under the robot's own
    shadow. A single global reference makes the far rows look like obstacles
    on every floor in the world.
    """

    def __init__(self):
        self.y = None                    # [ROWS] luma means
        self.u = None
        self.v = None
        self.t = 0.0

    @property
    def ready(self):
        return self.y is not None


class CliffDetector:
    """Watches the floor ahead. Blocks forward motion when it stops being one.

    Runs in its own thread. The guard reads `blocked` and `reason`; it does
    not wait on this, so a slow or dead detector degrades to "no veto" rather
    than to a stalled control loop.
    """

    def __init__(self, camera, hz=5.0, enabled=True):
        self.camera = camera
        self.enabled = enabled
        self.ref = FloorRef()
        self.cells = []                  # [ROWS][COLS] of 0 ok / 1 obj / 2 cliff
        self.blocked = False
        self.is_cliff = False
        self.reason = ""
        self.clear_mm = None             # nearest anomaly, mm ahead
        self.error = ""
        self.ms = 0.0
        self._relearn = True             # learn on the first frame
        self._hz = hz
        # Distance to the centre of each grid row, computed once.
        #
        # Row 0 is the TOP of the band and therefore the FURTHEST away; row
        # ROWS-1 is at the bottom of the frame, just in front of the wheels.
        # The band itself only covers the bottom BAND of the frame, so a row's
        # height up the frame is BAND * (1 - (r+0.5)/ROWS) — not 1 minus that,
        # which would place the whole grid up in the ceiling and make every
        # reported distance far too large.
        self.row_mm = [ground_distance(BAND * (1.0 - (r + 0.5) / ROWS))
                       for r in range(ROWS)]
        threading.Thread(target=self._run, daemon=True).start()

    def relearn(self):
        """Take the floor under the robot as the new reference. Do this after
        moving onto a different surface."""
        self._relearn = True

    # --- the loop ---------------------------------------------------------

    def _run(self):
        period = 1.0 / self._hz
        while True:
            time.sleep(period)
            if self.camera is None or self.camera.cam is None:
                continue
            if not self.enabled:
                self.blocked, self.reason = False, ""
                continue
            try:
                t0 = time.perf_counter()
                self._tick()
                self.ms = (time.perf_counter() - t0) * 1000.0
                self.error = ""
            except Exception as e:                            # noqa: BLE001
                # A detector that has fallen over must not silently keep
                # reporting "clear" — that is the one failure that matters.
                self.error = str(e)
                self.blocked, self.reason = False, "cliff check failed"

    @staticmethod
    def _block(plane):
        """One mean per grid cell, over the lower BAND of a plane.

        Reshape-and-mean rather than a Python loop. The 48 cells cost a
        fraction of a millisecond in numpy and tens of milliseconds looped —
        which on this Pi is the difference between free and not worth doing.

        Works for the full-size Y plane and the half-size U and V planes
        alike, because everything is expressed as a fraction of whatever
        plane it is handed.
        """
        ph, pw = plane.shape
        t = int(ph * (1.0 - BAND))
        band = plane[t:t + ((ph - t) // ROWS) * ROWS, :(pw // COLS) * COLS]
        bh, bw = band.shape
        return band.reshape(ROWS, bh // ROWS, COLS, bw // COLS).mean(axis=(1, 3))

    def _grid(self):
        """Mean Y, U and V for each cell of the grid over the lower band."""
        y, u, v = self.camera.cam.yuv_planes()
        return self._block(y), self._block(u), self._block(v)

    def _tick(self):
        gy, gu, gv = self._grid()

        if self._relearn or not self.ref.ready:
            # One reference per ROW, taken as that row's own mean across the
            # frame. A floor is not evenly lit — it dims with distance and
            # under the robot's own shadow — so a single number for the whole
            # frame makes the far rows look like obstacles on every floor in
            # the world. Learning the falloff is what stops that.
            #
            # This assumes the view is mostly floor at the moment you press
            # relearn. Do it pointing at open floor, not at a wall.
            self.ref.y = [float(gy[r].mean()) for r in range(ROWS)]
            self.ref.u = [float(gu[r].mean()) for r in range(ROWS)]
            self.ref.v = [float(gv[r].mean()) for r in range(ROWS)]
            self.ref.t = time.time()
            self._relearn = False

        cells, worst, cliff = [], None, False
        for r in range(ROWS):
            row = []
            for c in range(COLS):
                dy = float(gy[r][c]) - self.ref.y[r]
                du = abs(float(gu[r][c]) - self.ref.u[r])
                dv = abs(float(gv[r][c]) - self.ref.v[r])
                bad = abs(dy) > LUMA_TOL or du > CHROMA_TOL or dv > CHROMA_TOL
                if not bad:
                    row.append(0)
                    continue
                kind = 2 if dy < -CLIFF_DARKER else 1
                row.append(kind)
                # Only the middle of the frame is in the robot's path. A cell
                # off to the side is a wall or furniture the LiDAR already
                # has, and blocking on it would make the robot useless in a
                # corridor.
                if 1 <= c <= COLS - 2:
                    d = self.row_mm[r]
                    if d is not None and (worst is None or d < worst):
                        worst = d
                        cliff = kind == 2
            cells.append(row)

        self.cells = cells
        self.clear_mm = worst
        if worst is None:
            self.blocked, self.is_cliff, self.reason = False, False, ""
            return

        # Only veto once it is close enough to matter. Something 2 m away in
        # a camera with no depth is not worth stopping for; the LiDAR guard
        # owns that range anyway.
        limit = TRUCK_LENGTH_MM / 2.0 + 250.0
        self.is_cliff = cliff
        self.blocked = worst <= limit
        self.reason = ("%s %.0f mm ahead"
                       % ("drop-off" if cliff else "low obstacle", worst))

    # --- for the page -----------------------------------------------------

    @property
    def state(self):
        return {
            "enabled": self.enabled, "ready": self.ref.ready,
            "blocked": self.blocked, "cliff": self.is_cliff,
            "reason": self.reason, "clear_mm": self.clear_mm,
            "cells": self.cells, "rows": ROWS, "cols": COLS,
            "row_mm": [None if d is None else round(d) for d in self.row_mm],
            "ms": round(self.ms, 1), "error": self.error,
            "pitch": CAM_PITCH_DEG, "height": CAM_HEIGHT_MM,
        }
