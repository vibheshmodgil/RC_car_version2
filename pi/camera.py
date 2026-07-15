"""MJPEG feed from the ESP32-CAM, as OpenCV frames."""
import cv2  # apt: python3-opencv

from config import CAM_STREAM_URL


class Camera:
    def __init__(self, url: str = CAM_STREAM_URL):
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open stream {url} — is the CAM powered "
                               "and on the AP?")

    def read(self):
        """Latest frame as a BGR numpy array, or None if the stream hiccuped."""
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        self.cap.release()
