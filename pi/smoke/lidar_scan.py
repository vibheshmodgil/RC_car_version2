"""Smoke test 5: spin the LiDAR, print 5 scans' point count + nearest object."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from lidar import Lidar

lidar = Lidar()
try:
    info, health = lidar.info()
    print(f"model={info} health={health}")
    for i, scan in enumerate(lidar.scans()):
        dists = [d for _q, _a, d in scan if d > 0]
        print(f"scan {i}: {len(scan)} pts, nearest {min(dists)/1000:.2f} m"
              if dists else f"scan {i}: no returns")
        if i >= 4:
            break
finally:
    lidar.stop()
print("OK")
