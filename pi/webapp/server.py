"""RC Car dashboard — the FastAPI app phones talk to (Pi-centric phase).

The Pi serves the main UI; the ESP32 DevKit keeps its own UI at
http://<ESP32_HOST> as a debug fallback. Control POSTs are forwarded to
the DevKit verbatim so its REST API stays the single contract; the
browser gets one merged telemetry WebSocket (/ws) with ESP32 + IMU +
LiDAR + CAM state at ~10 Hz.

Run from pi/:
    .venv/bin/python -m uvicorn webapp.server:app --host 0.0.0.0 --port 80
or install webapp/car-dashboard.service (see pi/README.md).

Safety: if the browser dies mid-drive, its 0.3 s drive re-sends stop
arriving and the DevKit's own 500 ms deadman coasts the wheels.
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # pi/ modules

import requests
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse

from config import CAM_HOST, ESP32_HOST, WS_PUSH_S
from webapp.hub import Hub

app = FastAPI(title="RC Car dashboard")
hub = Hub()

STATIC = Path(__file__).parent / "static"


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/config")
def api_config():
    """Addresses the browser needs (CAM stream is pulled directly)."""
    return {"esp32": ESP32_HOST, "cam": CAM_HOST}


# ---------------------------------------------------------------- ESP32
# Forward the query string untouched — the DevKit API is the contract.
def _forward(path: str, query: str) -> PlainTextResponse:
    url = f"http://{ESP32_HOST}{path}" + (f"?{query}" if query else "")
    try:
        r = requests.post(url, timeout=1.0)
        return PlainTextResponse(r.text, status_code=r.status_code)
    except requests.RequestException as e:
        return PlainTextResponse(f"esp32 unreachable: {e}", status_code=502)


@app.post("/api/drive")
def api_drive(request: Request):
    return _forward("/api/drive", request.url.query)


@app.post("/api/motor")
def api_motor(request: Request):
    return _forward("/api/motor", request.url.query)


@app.post("/api/estop")
def api_estop():
    return _forward("/api/estop", "")


@app.post("/api/estop/clear")
def api_estop_clear():
    return _forward("/api/estop/clear", "")


# --------------------------------------------------------------- gimbal
@app.post("/api/gimbal")
def api_gimbal(pan: float | None = None, tilt: float | None = None):
    if hub.gimbal_set(pan=pan, tilt=tilt):
        return PlainTextResponse("OK")
    return PlainTextResponse("gimbal offline", status_code=503)


# ---------------------------------------------------------------- LiDAR
@app.post("/api/lidar/start")
def api_lidar_start():
    if hub.lidar_start():
        return PlainTextResponse("OK")
    err = hub.lidar_state.get("err", "unknown")
    return PlainTextResponse(f"lidar offline: {err}", status_code=503)


@app.post("/api/lidar/stop")
def api_lidar_stop():
    hub.lidar_stop()
    return PlainTextResponse("OK")


# ------------------------------------------------------------ telemetry
@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            await sock.send_json(hub.snapshot())
            await asyncio.sleep(WS_PUSH_S)
    except (WebSocketDisconnect, RuntimeError):
        pass
