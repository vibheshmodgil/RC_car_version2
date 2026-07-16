"""Device hub: owns every car subsystem the dashboard talks to.

One instance per process (created by webapp.server). Each sensor is
optional — if it fails to open, its slot in the merged telemetry reports
offline and the rest of the dashboard keeps working.

Background threads cache the latest state of every device so the
WebSocket push loop never blocks on I/O.
"""
import threading
import time

import requests

from config import CAM_STATUS_URL, CAM_POLL_S, LIDAR_WS_POINTS
from esp32_link import Esp32Link


def _try_open(name, factory):
    try:
        dev = factory()
        print(f"[hub] {name}: ok")
        return dev
    except Exception as e:
        print(f"[hub] {name}: offline ({e})")
        return None


class Hub:
    def __init__(self):
        # ESP32 link is mandatory-ish, but its constructor never touches the
        # network — telemetry just stays stale until the DevKit shows up.
        self.link = Esp32Link()
        self.link.start_telemetry()

        # Optional Pi-side devices (import inside the lambda so a missing
        # library only takes out that one device).
        self.imu = _try_open("IMU (BNO055)", lambda: __import__("imu").Imu())
        self.gimbal = _try_open("Gimbal (pigpio)", lambda: __import__("gimbal").Gimbal())
        self.gimbal_pos = {"pan": 90.0, "tilt": 90.0}
        if self.gimbal:
            self.gimbal.center()

        # Latest cached state per device, replaced (never mutated) by the
        # worker threads so snapshot() can read without locks. IMU owns its
        # own background thread (see imu.py); the rest are driven here.
        self.cam_state = {"ok": False}
        self.lidar_state = {"running": False, "pts": []}

        self._lidar = None
        self._lidar_stop = threading.Event()

        threading.Thread(target=self._cam_loop, daemon=True).start()

    # ------------------------------------------------------------- ESP32
    def snapshot(self) -> dict:
        """One merged telemetry frame for the browser WebSocket."""
        return {
            "esp32": self.link.telemetry if self.link.telemetry_fresh() else None,
            "imu": self.imu.reading if self.imu else {"ok": False},
            "cam": self.cam_state,
            "lidar": self.lidar_state,
            "gimbal": {"ok": self.gimbal is not None, **self.gimbal_pos},
        }

    # ------------------------------------------------------------ gimbal
    def gimbal_set(self, pan=None, tilt=None) -> bool:
        if not self.gimbal:
            return False
        if pan is not None:
            self.gimbal.pan(float(pan))
            self.gimbal_pos["pan"] = float(pan)
        if tilt is not None:
            self.gimbal.tilt(float(tilt))
            self.gimbal_pos["tilt"] = float(tilt)
        return True

    # ------------------------------------------------------------- LiDAR
    # Started on demand from the dashboard — spinning the motor 24/7 wears
    # it out for nothing while the panel is not being watched.
    def lidar_start(self) -> bool:
        if self.lidar_state.get("running"):
            return True
        try:
            self._lidar = __import__("lidar").Lidar()
        except Exception as e:
            self.lidar_state = {"running": False, "pts": [], "err": str(e)}
            return False
        self._lidar_stop.clear()
        self.lidar_state = {"running": True, "pts": []}
        threading.Thread(target=self._lidar_loop, daemon=True).start()
        return True

    def lidar_stop(self):
        self._lidar_stop.set()

    def _lidar_loop(self):
        try:
            for pts in self._lidar.scans():   # one rotation: [(angle_deg, dist_mm), ...]
                if self._lidar_stop.is_set():
                    break
                # Downsample so a WS frame stays small.
                step = max(1, len(pts) // LIDAR_WS_POINTS)
                self.lidar_state = {"running": True, "pts": pts[::step]}
        except Exception as e:
            self.lidar_state = {"running": False, "pts": [], "err": str(e)}
        finally:
            try:
                self._lidar.stop()
            except Exception:
                pass
            self._lidar = None
            if self.lidar_state.get("running"):
                self.lidar_state = {"running": False, "pts": []}

    # ------------------------------------------------------------ camera
    # Poll the CAM's cumulative counters and rate the deltas — same maths
    # as the ESP32 camera page.
    def _cam_loop(self):
        prev = None
        while True:
            try:
                d = requests.get(CAM_STATUS_URL, timeout=1.0).json()
                state = {"ok": True, "streaming": d.get("streaming", False)}
                if prev and d["ms"] > prev["ms"] and d["frames"] >= prev["frames"]:
                    dt = d["ms"] - prev["ms"]
                    df = d["frames"] - prev["frames"]
                    db = d["bytes"] - prev["bytes"]
                    state["fps"] = round(df * 1000 / dt, 1)
                    state["kbps"] = round(db * 8 / dt)
                    state["kb"] = round(db / df / 1024, 1) if df else None
                prev = d
                self.cam_state = state
            except Exception:
                prev = None
                self.cam_state = {"ok": False}
            time.sleep(CAM_POLL_S)
