"""RPLidar A1/C1 over USB serial."""
from rplidar import RPLidar  # pip: rplidar-roboticia

from config import LIDAR_PORT, LIDAR_BAUD


class Lidar:
    def __init__(self, port: str = LIDAR_PORT, baud: int = LIDAR_BAUD):
        self._l = RPLidar(port, baudrate=baud)

    def scans(self):
        """Yields full 360-degree scans: lists of (quality, angle_deg, dist_mm).
        dist_mm == 0 means no return at that angle."""
        return self._l.iter_scans()

    def info(self):
        return self._l.get_info(), self._l.get_health()

    def stop(self):
        self._l.stop()
        self._l.stop_motor()
        self._l.disconnect()
