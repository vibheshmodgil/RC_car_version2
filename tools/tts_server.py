"""
Speech server — runs on your PC, not the robot.

    pip install "piper-tts>=1.3" numpy
    python tools/tts_server.py

or, with everything else the PC runs for the truck:

    cd tools && docker compose up -d

Then point the robot at it: the Speak card on the cockpit's Audio tab, field
"Speech server on the PC" -> http://<your-pc-ip>:5005, Apply. Remembered on
the Pi in test/tts.json. Or on the Pi:

    python test/web_nav.py --tts-url http://<your-pc-ip>:5005

Why offload at all
------------------
Piper on a Pi 4 is several times faster than real time — when it has the CPU
to itself. In the cockpit it shares four cores with SLAM, the camera, the
cliff check and detection, at nice 10 so that SLAM always wins; measured here
that put 5-17 s between the model's reply and the first spoken word. A desktop
CPU synthesises the same sentence in tens of milliseconds, and the PCM comes
back over the LAN at 44 kB/s — nothing next to the camera preview.

Same code on both ends
----------------------
The text splitting, speed, volume and the voice list are test/tts.py's own,
imported here, so a sentence sounds the same whichever machine made it. If
this server is off or unreachable the Pi speaks for itself and says so on the
Speak card; a PC that goes to sleep must not leave the truck mute.

Protocol
--------
POST /say   {"voice", "text", "speed", "volume", "warm"}
            -> 200, X-Sample-Rate header, raw 16-bit mono PCM streamed as
               each piece is made; the connection closes at the end.
               warm=true loads the voice and returns no audio.
GET  /      health: which voices are loaded and downloaded.

A voice the PC does not have yet is downloaded from the Piper voice server on
first use (~60 MB) into TTS_VOICE_DIR, default tools/voices/.
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_HERE = os.path.dirname(os.path.abspath(__file__))
# The repo layout (tools/ next to test/), or the Docker image (both in /app).
for _p in (_HERE, os.path.join(_HERE, "..", "test")):
    sys.path.insert(0, os.path.abspath(_p))
import tts                                                     # noqa: E402

try:                                    # task-manager numbers for the cockpit's System tab
    import sysstats                                            # noqa: E402
    SAMPLER = sysstats.Sampler()
except ImportError:
    SAMPLER = None

VOICE_DIR = os.environ.get("TTS_VOICE_DIR", os.path.join(_HERE, "voices"))
# onnxruntime's default is a thread per core that busy-waits between runs.
# Inside Docker Desktop's VM that fought the rest of the PC: measured 0.6-1.1 s
# a sentence with spikes to 7 s, against a steady ~0.4 s with 4 threads that
# sleep instead of spinning.
THREADS = int(os.environ.get("TTS_THREADS", "4"))

_voices = {}
_load_lock = threading.Lock()


def _download(vid, dest):
    """Written to .part files and renamed at the end, like the Pi does, so an
    interrupted download never leaves a model that looks installed."""
    os.makedirs(VOICE_DIR, exist_ok=True)
    model_url, cfg_url = tts.voice_urls(vid)
    for url, path in ((cfg_url, dest + ".json"), (model_url, dest)):
        print(f"  downloading {os.path.basename(path)} ...", flush=True)
        req = urllib.request.Request(url, headers={"User-Agent": "speaker-truck"})
        with urllib.request.urlopen(req, timeout=30) as r, open(path + ".part", "wb") as f:
            while chunk := r.read(1 << 16):
                f.write(chunk)
    os.replace(dest + ".json.part", dest + ".json")
    os.replace(dest + ".part", dest)


def voice(vid):
    """Loaded once and kept: a few voices at ~60-120 MB each is nothing on a PC.
    The throwaway sentence is there because the first inference in a fresh
    onnxruntime session is several times slower than every one after it."""
    with _load_lock:
        if vid not in _voices:
            from piper import PiperVoice                       # noqa: PLC0415
            path = os.path.join(VOICE_DIR, vid + ".onnx")
            if not os.path.isfile(path) or not os.path.isfile(path + ".json"):
                _download(vid, path)
            import onnxruntime                                 # noqa: PLC0415
            t0 = time.monotonic()
            v = PiperVoice.load(path)
            so = onnxruntime.SessionOptions()
            so.intra_op_num_threads, so.inter_op_num_threads = THREADS, 1
            so.add_session_config_entry("session.intra_op.allow_spinning", "0")
            v.session = onnxruntime.InferenceSession(path, sess_options=so,
                                                     providers=["CPUExecutionProvider"])
            for _ in tts._chunks(v, "Ready.", 1.0, 1.0):
                pass
            print(f"  loaded {vid} in {time.monotonic() - t0:.1f} s", flush=True)
            _voices[vid] = v
        return _voices[vid]


class Handler(BaseHTTPRequestHandler):
    # HTTP/1.0: no Content-Length, the body is the PCM until the socket
    # closes — which is what lets audio stream while it is being made.
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def _json(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        have = sorted(f[:-5] for f in os.listdir(VOICE_DIR) if f.endswith(".onnx")) \
            if os.path.isdir(VOICE_DIR) else []
        self._json(200, {"ok": True, "loaded": sorted(_voices), "downloaded": have,
                         "stats": SAMPLER.sample() if SAMPLER else None})

    def do_POST(self):
        if self.path.rstrip("/") != "/say":
            return self._json(404, {"ok": False, "error": "POST /say"})
        try:
            d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            vid = d.get("voice") or tts.DEFAULT_VOICE
            v = voice(vid)
        except Exception as e:                                 # noqa: BLE001
            return self._json(400, {"ok": False, "error": f"{e.__class__.__name__}: {e}"})
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("X-Sample-Rate", str(v.config.sample_rate))
        self.end_headers()
        if d.get("warm"):
            return
        text = " ".join(str(d.get("text") or "").split())[:tts.MAX_CHARS]
        t0, first, n = time.monotonic(), None, 0
        try:
            for pcm, _rate in tts._chunks(v, text, d.get("speed", 1.0), d.get("volume", 1.0)):
                if first is None:
                    first = time.monotonic() - t0
                self.wfile.write(pcm)
                self.wfile.flush()
                n += len(pcm)
        except (BrokenPipeError, ConnectionResetError):
            print(f"  cut off  {text[:50]!r}", flush=True)     # a newer sentence won
            return
        print(f"  {vid}  first {1000 * (first or 0):4.0f} ms  total {1000 * (time.monotonic() - t0):4.0f} ms"
              f"  {n / 2 / v.config.sample_rate:4.1f} s audio  {text[:50]!r}", flush=True)


def _lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))
        return s.getsockname()[0]
    except OSError:
        return "<your-pc-ip>"
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description="Off-board speech for the truck")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--preload", default=tts.DEFAULT_VOICE,
                    help="comma-separated voices to load at start, so the first "
                         "sentence does not wait for a download")
    args = ap.parse_args()

    for vid in filter(None, (s.strip() for s in args.preload.split(","))):
        try:
            voice(vid)
        except Exception as e:                                 # noqa: BLE001
            print(f"  could not preload {vid}: {e}", flush=True)

    # Inside Docker this prints the container's address, not the PC's; use
    # the PC's own LAN IP (ipconfig) in that case.
    print(f"\n  ready on http://{_lan_ip()}:{args.port}")
    print("  On the truck: Audio tab -> Speak -> Speech server on the PC -> that URL -> Apply\n",
          flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())
