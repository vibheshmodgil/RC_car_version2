"""YDLIDAR X2 — raw serial packet parser, no vendor SDK dependency (only
pyserial). Scans for the \\xaa\\x55 sync header, extracts angle/distance
pairs, interpolates per-point angles across each packet's first/last-angle
span. Ported from the proven bench-tested reader in the LIDAR prototype
project; register/packet layout is from the YDLIDAR X2 protocol.

Reconnects automatically on serial errors (2 s backoff) until stop().
"""
import time
from collections import deque

import serial

from config import LIDAR_PORT, LIDAR_BAUD


class Lidar:
    def __init__(self, port: str = LIDAR_PORT, baud: int = LIDAR_BAUD):
        self.port = port
        self.baud = baud
        self._ser = serial.Serial(port, baud, timeout=1)  # fail fast if not present
        self._stop = False

    def scans(self):
        """Yields one [(angle_deg, dist_mm), ...] list per completed
        rotation. Points with no return (dist 0) are already dropped."""
        rot_times = deque(maxlen=12)
        buf = b""
        angles, dists = [], []

        while not self._stop:
            try:
                chunk = self._ser.read(2048)
            except Exception:
                try:
                    self._ser.close()
                except Exception:
                    pass
                time.sleep(2)
                try:
                    self._ser = serial.Serial(self.port, self.baud, timeout=1)
                except Exception:
                    continue
                continue

            buf += chunk
            while True:
                idx = buf.find(b"\xaa\x55")
                if idx < 0 or len(buf) < idx + 10:
                    break
                lsn = buf[idx + 3]
                if lsn == 0 or lsn > 120:
                    buf = buf[idx + 2:]
                    continue
                end = idx + 10 + lsn * 2
                if len(buf) < end:
                    break
                pkt = buf[idx:end]
                buf = buf[end:]

                if pkt[2] & 0x01 and angles:  # rotation complete
                    rot_times.append(time.time())
                    yield list(zip(angles, dists))
                    angles, dists = [], []

                fsa = ((pkt[4] | pkt[5] << 8) >> 1) / 64.0
                lsa = ((pkt[6] | pkt[7] << 8) >> 1) / 64.0
                diff = (lsa - fsa) % 360
                for i in range(lsn):
                    raw = pkt[10 + 2 * i] | pkt[11 + 2 * i] << 8
                    if raw:
                        ang = (fsa + diff * i / max(lsn - 1, 1)) % 360
                        angles.append(round(ang, 1))
                        dists.append(round(raw / 4))

    def stop(self):
        self._stop = True
        try:
            self._ser.close()
        except Exception:
            pass
