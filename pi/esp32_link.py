"""Client for the ESP32 test bench: REST control + WebSocket telemetry.

Control is plain HTTP POSTs to the API documented in CLAUDE.md.
Telemetry arrives on a background thread; read .telemetry / .telemetry_fresh().
"""
import json
import threading
import time

import requests
import websocket  # pip: websocket-client

from config import ESP32_HOST, TELEMETRY_STALE_S


class Esp32Link:
    def __init__(self, host: str = ESP32_HOST, timeout: float = 1.0):
        self.host = host
        self.base = f"http://{host}"
        self.timeout = timeout
        self.telemetry: dict | None = None   # latest decoded /ws frame
        self.telemetry_at = 0.0              # time.time() it arrived
        self._ws = None

    # ------------------------------------------------------------- control
    def _post(self, path: str) -> None:
        requests.post(self.base + path, timeout=self.timeout)

    def estop(self):        self._post("/api/estop")
    def estop_clear(self):  self._post("/api/estop/clear")

    def arm_all(self, on: bool = True):
        self._post(f"/api/motor?ch=all&arm={1 if on else 0}")

    def motor_pwm(self, ch: str, pwm: int):
        """Single wheel: ch = lf|lr|rf|rr, pwm = -255..255."""
        self._post(f"/api/motor?ch={ch}&pwm={int(pwm)}")

    def drive(self, direction: str, pwm: int):
        """Whole car: direction = fwd|rev|left|right, pwm = 0..255.
        Re-send this at least every DEADMAN_RESEND_S while moving."""
        self._post(f"/api/drive?dir={direction}&pwm={int(pwm)}")

    def stop(self):
        self._post("/api/drive?dir=stop")

    # ----------------------------------------------------------- telemetry
    def start_telemetry(self) -> None:
        """Background WS reader; reconnects on its own if the link drops."""
        def on_message(_ws, msg):
            self.telemetry = json.loads(msg)
            self.telemetry_at = time.time()

        def run():
            while True:
                try:
                    self._ws = websocket.WebSocketApp(
                        f"ws://{self.host}/ws", on_message=on_message)
                    self._ws.run_forever()
                except Exception:
                    pass
                time.sleep(2.0)   # lost the AP? keep retrying quietly

        threading.Thread(target=run, daemon=True).start()

    def telemetry_fresh(self, max_age: float = TELEMETRY_STALE_S) -> bool:
        return (self.telemetry is not None
                and time.time() - self.telemetry_at < max_age)
