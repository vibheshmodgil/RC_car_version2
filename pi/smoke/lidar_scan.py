"""Smoke test 5: spin the LiDAR, print 5 scans' point count + nearest object."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from lidar import Lidar

lidar = Lidar()
try:
    for i, pts in enumerate(lidar.scans()):   # pts = [(angle_deg, dist_mm), ...]
        dists = [d for _a, d in pts]
        print(f"scan {i}: {len(pts)} pts, nearest {min(dists) / 1000:.2f} m"
              if dists else f"scan {i}: no returns")
        if i >= 4:
            break
finally:
    lidar.stop()
print("OK")
