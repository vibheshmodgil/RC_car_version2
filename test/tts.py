"""
Text to speech — Piper neural voices, spoken through the truck's speaker.
Shared library.

Used by web_nav.py (the Speak card on the Audio tab) and speaker_test.py.

    source .venv/bin/activate
    pip install "piper-tts>=1.3"

then download a voice from the Audio tab (or `speaker_test.py --download`).

Why Piper
---------
It is a neural TTS built for exactly this: a Raspberry Pi, offline, no
account, no per-word bill. The "medium" voices sound like a person rather
than espeak's robot and synthesise several times faster than real time on a
Pi 4. "high" voices sound a little richer and run at roughly real time here
— fine for a sentence, slow for a paragraph. The model runs on onnxruntime,
which detect.py already uses.

1.3 or newer, specifically: 1.2 depends on piper-phonemize, which has no
wheels for the Python 3.13 that Trixie ships, so pip fails trying to build it.

How it runs
-----------
Synthesis happens in a separate worker process (this file, `--worker`) at
nice 10. The model is ~60 MB of floats and onnxruntime grabs every core it
can see; in the cockpit process that would stall scan matching for as long
as the sentence takes. At nice 10 SLAM always wins the CPU. The worker keeps
the voice loaded, and runs one throwaway sentence at start-up, because the
first inference in a fresh onnxruntime session is several times slower than
every one after it.

Streaming, and the latest click wins:

  * Text is cut into short pieces (sentences, then commas) and each piece's
    audio goes to the speaker the moment it exists — through a pipe into
    one aplay that was opened before the first piece was even ready. Sound
    starts after the first piece, not after the whole text.
  * A new Speak cuts off whatever is sounding at once, and the worker drops
    the old text before its next piece. Nothing queues behind anything.

Speech plays through audio.py as an interjection, like a beep: a song that
is playing is held and resumes where it was when the speaking ends.

Speaking from the PC
--------------------
With a speech server URL set (the Speak card, `--tts-url`, or TRUCK_TTS_URL)
the worker sends the text to tools/tts_server.py on the PC and streams the PCM
it returns into the same aplay pipe — tens of milliseconds a sentence instead
of seconds, and no Pi CPU. Everything above (pieces, latest-wins, held songs)
is unchanged: only where the PCM comes from differs. If the PC does not
answer, the Pi synthesises for itself and tries the PC again after
REMOTE_RETRY_S; local Piper is only ever loaded if that happens.

Voices live in test/voices/ on the Pi — never synced, gitignored.
"""

import glob
import json
import os
import queue
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
import wave
from array import array

_HERE = os.path.dirname(os.path.abspath(__file__))
VOICE_DIR = os.path.join(_HERE, "voices")
SETTINGS_FILE = os.path.join(_HERE, "tts.json")
HF_BASE = "https://huggingface.co/rhasspy/piper-voices/resolve/main"

MAX_CHARS = 1000
HISTORY = 12
# Longest wait for the NEXT piece of audio. Generous, because the very first
# request after start-up includes importing onnxruntime and loading the model.
SYNTH_TIMEOUT_S = 120
# Pieces longer than this are cut at a comma — see _pieces.
PIECE_CHARS = 90
# The PC speech server: how long to wait for it to accept a connection, and
# how long the Pi speaks for itself after it did not, before asking again.
REMOTE_CONNECT_S = 1.5
REMOTE_RETRY_S = 30

# Curated from the Piper voice list: the ones that sound good on a small
# speaker. Any other Piper voice dropped into test/voices/ is picked up too.
#   (id, description, approximate download MB)
VOICES = (
    ("en_US-lessac-medium", "US English · female · clear, the default", 63),
    ("en_US-amy-medium", "US English · female · warm", 63),
    ("en_US-ryan-medium", "US English · male", 63),
    ("en_US-hfc_male-medium", "US English · male · deeper", 63),
    ("en_US-hfc_female-medium", "US English · female · bright", 63),
    ("en_GB-alba-medium", "British English · female", 63),
    ("en_GB-northern_english_male-medium", "British English · male", 63),
    ("en_US-lessac-high", "US English · female · richest, ~real time on a Pi", 114),
    ("en_US-ryan-high", "US English · male · richest, ~real time on a Pi", 121),
    ("hi_IN-priyamvada-medium", "Hindi · female", 63),
    ("hi_IN-pratham-medium", "Hindi · male", 63),
)
DEFAULT_VOICE = "en_US-lessac-medium"


def voice_urls(vid):
    """Piper's layout: <family>/<lang>/<name>/<quality>/<id>.onnx(.json)."""
    try:
        lang, name, quality = vid.split("-")
    except ValueError:
        raise ValueError(f"not a Piper voice id: {vid}") from None
    base = f"{HF_BASE}/{lang.split('_')[0]}/{lang}/{name}/{quality}/{vid}"
    return base + ".onnx", base + ".onnx.json"


def _model_path(vid):
    return os.path.join(VOICE_DIR, vid + ".onnx")


def _sample_rate(model):
    """From the voice's .onnx.json, so aplay can be opened before any audio
    exists. 22050 Hz is what nearly every Piper voice uses."""
    try:
        with open(model + ".json", encoding="utf-8") as f:
            return int(json.load(f)["audio"]["sample_rate"])
    except (OSError, ValueError, KeyError, TypeError):
        return 22050


def _quiet_kill(proc):
    try:
        if proc.poll() is None:
            proc.kill()
    except OSError:
        pass


def _quiet_close(proc):
    """Let an aplay finish what it has buffered, then return."""
    try:
        proc.stdin.close()
        proc.wait(timeout=30)
    except (OSError, ValueError, subprocess.TimeoutExpired):
        _quiet_kill(proc)


def piper_available():
    import importlib.util                                      # noqa: PLC0415
    return importlib.util.find_spec("piper") is not None


# ---------------------------------------------------------------------------
# Worker process — the only place Piper is imported
# ---------------------------------------------------------------------------

def _pieces(text):
    """Cut text into units Piper turns round quickly: (text, pause_after).

    Sound starts when the FIRST unit is synthesised, and a new click can only
    take over between units. A 300-character run-on sentence as one unit
    meant seconds of silence before anything played, and seconds more before
    a correction could interrupt it. Sentences first; anything still long is
    cut at a comma, semicolon or colon, and gets back a short pause there."""
    out = []
    for sent in re.split(r"(?<=[.!?\u0964])\s+", text):
        sent = sent.strip()
        while len(sent) > PIECE_CHARS:
            cut = max(sent.rfind(c, 0, PIECE_CHARS) for c in ",;:")
            if cut < 20:
                break
            out.append((sent[:cut + 1], True))
            sent = sent[cut + 1:].strip()
        if sent:
            out.append((sent, False))
    return out


def _gain(pcm, volume):
    if abs(volume - 1.0) < 0.01:
        return pcm
    import numpy as np                                         # noqa: PLC0415
    a = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) * volume
    return np.clip(a, -32768, 32767).astype(np.int16).tobytes()


def _chunks(voice, text, speed, volume):
    """Yield (pcm, sample_rate) as audio is produced — Piper gives one chunk
    per sentence of each piece. Supports piper-tts 1.3+ and 1.2."""
    length_scale = 1.0 / max(0.5, min(2.0, float(speed)))
    for piece, pause in _pieces(text):
        rate = None
        if hasattr(voice, "synthesize_wav"):                   # 1.3+
            from piper import SynthesisConfig                  # noqa: PLC0415
            cfg = SynthesisConfig(length_scale=length_scale)
            for ch in voice.synthesize(piece, syn_config=cfg):
                rate = ch.sample_rate
                yield _gain(ch.audio_int16_bytes, volume), rate
        else:                                                  # 1.2
            rate = voice.config.sample_rate
            for pcm in voice.synthesize_stream_raw(piece, length_scale=length_scale):
                yield _gain(pcm, volume), rate
        if pause and rate:
            yield bytes(int(rate * 0.12) * 2), rate            # the comma we cut at


def _remote_chunks(url, req):
    """Open a stream from the PC's tts_server and return a generator of
    (pcm, rate). Raises here, before any audio, if the PC is not there —
    the caller then speaks locally instead.

    The reachability check is a bare connect with a short timeout; the
    request itself gets the long one, because the PC may be downloading or
    loading the voice before it answers."""
    parts = urllib.parse.urlsplit(url)
    socket.create_connection((parts.hostname, parts.port or 80), REMOTE_CONNECT_S).close()
    body = json.dumps({"voice": req.get("voice"), "text": req.get("text", ""),
                       "speed": req.get("speed", 1.0), "volume": req.get("volume", 1.0),
                       "warm": bool(req.get("warm"))}).encode()
    try:
        r = urllib.request.urlopen(urllib.request.Request(
            url.rstrip("/") + "/say", body, {"Content-Type": "application/json"}),
            timeout=SYNTH_TIMEOUT_S)
    except urllib.error.HTTPError as e:
        try:
            msg = json.loads(e.read()).get("error") or f"HTTP {e.code}"
        except ValueError:
            msg = f"HTTP {e.code}"
        raise OSError(msg) from None
    rate = int(r.headers.get("X-Sample-Rate") or 22050)

    def stream():
        try:
            while pcm := r.read1(1 << 15):   # whatever has arrived, not a full block
                yield pcm, rate
        finally:
            r.close()                        # a cut-off sentence stops the PC too
    return stream()


def worker():
    """JSON lines in, JSON lines out, audio handed over as raw PCM files.

    The newest request always wins. A reader thread records the newest id
    the moment a line arrives, and synthesis checks it between chunks — so
    new text, or a stop, takes over within one piece instead of after the
    old text has been synthesised to the end."""
    try:
        os.nice(10)
    except (AttributeError, OSError):
        pass
    # Piper is imported on first local use: with the PC doing the speaking,
    # the Pi never pays the import, the ~60 MB model, or its warm-up.
    piper = {}

    def piper_voice_cls():
        if "cls" not in piper:
            t_import = time.monotonic()
            from piper import PiperVoice                       # noqa: PLC0415
            piper["cls"] = PiperVoice
            piper["import_ms"] = round((time.monotonic() - t_import) * 1000)
        return piper["cls"]

    remote_down = {"until": 0.0, "why": ""}

    jobs = queue.Queue()
    newest = [0]
    out_lock = threading.Lock()

    def reader():
        for line in sys.stdin:
            try:
                req = json.loads(line)
            except ValueError:
                continue
            newest[0] = max(newest[0], int(req.get("id") or 0))
            jobs.put(req)
        jobs.put(None)

    def send(obj):
        with out_lock:
            sys.stdout.write(json.dumps(obj) + "\n")
            sys.stdout.flush()

    threading.Thread(target=reader, daemon=True).start()
    loaded = {}
    while True:
        req = jobs.get()
        if req is None:
            return
        rid = req.get("id")
        if req.get("stop") or rid != newest[0]:
            continue                     # superseded before it even started
        source = None
        try:
            warm = bool(req.get("warm"))
            text = "Ready." if warm else req["text"]
            t0, n = time.monotonic(), 0
            url, fresh = req.get("url"), False
            if url and time.monotonic() >= remote_down["until"]:
                try:
                    source = _remote_chunks(url, req)
                    remote_down["why"] = ""
                except (OSError, ValueError) as e:
                    remote_down["until"] = time.monotonic() + REMOTE_RETRY_S
                    remote_down["why"] = f"PC speech server not answering ({e}) — the Pi is speaking"
            if url:
                send({"id": rid, "engine": "pi" if source is None else "pc",
                      "warn": remote_down["why"]})
            if source is None:
                model = req["model"]
                fresh = model not in loaded
                if fresh:
                    cls = piper_voice_cls()
                    loaded.clear()       # one voice in memory: ~60-120 MB each
                    t_load = time.monotonic()
                    loaded[model] = cls.load(model)
                    load_ms = round((time.monotonic() - t_load) * 1000)
                source = _chunks(loaded[model], text,
                                 req.get("speed", 1.0), req.get("volume", 1.0))
            first_ms = None
            for pcm, rate in source:
                if first_ms is None:
                    first_ms = round((time.monotonic() - t0) * 1000)
                    if fresh:
                        # Where start-up time goes, once per model load —
                        # the first inference is the surprising one.
                        send({"id": rid, "info": {"import_ms": piper["import_ms"], "load_ms": load_ms,
                                                  "first_inference_ms": first_ms}})
                if newest[0] != rid:
                    break
                if warm:
                    continue
                path = os.path.join(req["dir"], f"truck_tts_{rid}_{n}.raw")
                with open(path, "wb") as f:
                    f.write(pcm)
                send({"id": rid, "chunk": path, "rate": rate,
                      "ms": round((time.monotonic() - t0) * 1000)})
                n += 1
            send({"id": rid, "done": True, "chunks": n})
        except Exception as e:                                 # noqa: BLE001
            send({"id": rid, "error": f"{e.__class__.__name__}: {e}"})
        finally:
            if source is not None:
                source.close()


# ---------------------------------------------------------------------------
# Speaker-side front end
# ---------------------------------------------------------------------------

class Tts:
    """Latest text -> worker -> streamed into audio.Audio.interject_stream."""

    def __init__(self, player, warm=True):
        os.makedirs(VOICE_DIR, exist_ok=True)
        self.player = player
        self.have_piper = piper_available()
        self.voice = DEFAULT_VOICE
        self.speed = 1.0
        self.volume = 1.0
        self.history = []
        self.status = "idle"        # idle | loading | synth | speaking
        self.text = ""
        self.error = ""
        self.first_sound_ms = None  # click -> first audio, last time it spoke
        self.startup = None         # worker timings from the last model load
        self.downloads = {}         # vid -> {"pct", "error", "stage", ...}
        self.url = os.environ.get("TRUCK_TTS_URL", "").rstrip("/")   # PC speech server
        self.engine = "pi"          # who made the last speech: "pc" or "pi"
        self.remote_error = ""      # why the PC was not used, when it was set
        self._proc = None
        self._replies = queue.Queue()
        self._err = None
        self._rid = 0
        # One slot, not a queue: the job waiting to run. A new say() simply
        # replaces it. _gen is bumped by every say() and stop(), and a job
        # that sees a newer generation abandons itself wherever it is.
        self._job = None
        self._gen = 0
        self._busy = False
        # Sentences said with append=True while something is already speaking.
        # They play after it, gaplessly, instead of cutting it off — that is
        # how the assistant streams a reply sentence by sentence.
        self._queue = deque()
        self._cv = threading.Condition()
        self._send_lock = threading.Lock()
        self._chunk_dir = tempfile.gettempdir()
        for old in glob.glob(os.path.join(self._chunk_dir, "truck_tts_*.raw")):
            try:
                os.remove(old)
            except OSError:
                pass
        self._load()
        if not os.path.isfile(_model_path(self.voice)):
            have = self.installed()
            if have:
                self.voice = have[0]
        threading.Thread(target=self._run, daemon=True).start()
        if warm:
            self._warm()

    # --- settings -----------------------------------------------------------

    def _load(self):
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        self.voice = d.get("voice") or self.voice
        self.speed = float(d.get("speed", self.speed))
        self.volume = float(d.get("volume", self.volume))
        self.history = [h for h in d.get("history", []) if isinstance(h, str)][:HISTORY]
        self.url = d.get("url", self.url)

    def _save(self):
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump({"voice": self.voice, "speed": self.speed,
                           "volume": self.volume, "history": self.history,
                           "url": self.url},
                          f, indent=2)
        except OSError:
            pass

    def set(self, voice=None, speed=None, volume=None, url=None):
        if url is not None:
            url = url.strip().rstrip("/")
            if url and not url.startswith(("http://", "https://")):
                raise ValueError("speech server URL must start with http:// — e.g. http://192.168.1.7:5005")
            self.url, self.remote_error = url, ""
            if not url:
                self.engine = "pi"
            self._warm()
        if voice is not None:
            # The PC fetches any Piper voice on first use; only the Pi needs
            # it downloaded beforehand.
            if voice not in self.installed() and not self.url:
                raise ValueError(f"voice not downloaded: {voice}")
            self.voice = voice
            self._warm()
        if speed is not None:
            self.speed = max(0.5, min(2.0, float(speed)))
        if volume is not None:
            self.volume = max(0.1, min(2.0, float(volume)))
        self._save()

    def forget(self, text=None):
        self.history = [] if text is None else [h for h in self.history if h != text]
        self._save()

    # --- voices -------------------------------------------------------------

    def installed(self):
        out = []
        for fn in sorted(os.listdir(VOICE_DIR)):
            if fn.endswith(".onnx") and os.path.isfile(os.path.join(VOICE_DIR, fn + ".json")):
                out.append(fn[:-5])
        return out

    def voices(self):
        have = set(self.installed())
        known = {v[0] for v in VOICES}
        rows = [{"id": vid, "label": label, "mb": mb, "installed": vid in have}
                for vid, label, mb in VOICES]
        rows += [{"id": vid, "label": "added by hand", "mb": None, "installed": True}
                 for vid in sorted(have - known)]
        for r in rows:
            r["download"] = self.downloads.get(r["id"])
        return rows

    def download(self, vid):
        """Fetch a voice from Hugging Face in the background. Written to .part
        files and renamed at the end, so a dropped wifi connection never
        leaves a truncated model that looks installed."""
        voice_urls(vid)                                   # validates the id
        if (self.downloads.get(vid) or {}).get("pct", 100) < 100:
            return
        self.downloads[vid] = {"pct": 0, "error": "", "stage": "starting",
                               "got": 0, "total": 0}
        threading.Thread(target=self._download, args=(vid,), daemon=True).start()

    def _download(self, vid):
        model_url, cfg_url = voice_urls(vid)
        dest = _model_path(vid)
        state = self.downloads[vid]
        # Expected size when the server does not say. Hugging Face serves the
        # model from a CDN redirect that can omit Content-Length, and a
        # percentage computed only from that header sat at 0% for the whole
        # 63 MB — indistinguishable from a hang.
        known_mb = next((mb for v, _, mb in VOICES if v == vid), None)
        try:
            for url, path, is_model in ((cfg_url, dest + ".json", False),
                                        (model_url, dest, True)):
                state["stage"] = "connecting" if is_model else "config"
                req = urllib.request.Request(url, headers={"User-Agent": "speaker-truck"})
                # timeout covers the connect AND every read, so a stalled
                # transfer raises after 30 s instead of waiting forever.
                with urllib.request.urlopen(req, timeout=30) as r, \
                        open(path + ".part", "wb") as f:
                    total = int(r.headers.get("Content-Length") or 0)
                    if is_model:
                        state["stage"] = "model"
                        state["total"] = total or (known_mb or 0) * 1024 * 1024
                    got = 0
                    while True:
                        chunk = r.read(1 << 16)
                        if not chunk:
                            break
                        f.write(chunk)
                        got += len(chunk)
                        if is_model:
                            state["got"] = got
                            if state["total"]:
                                state["pct"] = min(99, int(got * 100 / state["total"]))
            os.replace(dest + ".json.part", dest + ".json")
            os.replace(dest + ".part", dest)
            state["pct"] = 100
            if self.voice not in self.installed() or self.voice == vid:
                self.voice = vid
                self._save()
            self.downloads.pop(vid, None)
        except Exception as e:                                 # noqa: BLE001
            code = getattr(e, "code", None)
            state["error"] = ("not on the Piper voice server (404)" if code == 404
                              else f"{e.__class__.__name__}: {e}")
            state["pct"] = 100
            for part in (dest + ".part", dest + ".json.part"):
                try:
                    os.remove(part)
                except OSError:
                    pass

    def delete(self, vid):
        if vid not in self.installed():
            raise FileNotFoundError(vid)
        for path in (_model_path(vid), _model_path(vid) + ".json"):
            os.remove(path)
        if self.voice == vid:
            have = self.installed()
            self.voice = have[0] if have else DEFAULT_VOICE
            self._save()

    # --- speaking -----------------------------------------------------------

    def say(self, text, voice=None, speed=None, volume=None, append=False, remember=True):
        """Speak text now, cutting off anything already speaking.

        append=True queues it after whatever is speaking instead (and speaks
        at once if nothing is). remember=False keeps it out of the Speak
        card's recent-phrases list — for the assistant's own replies."""
        text = " ".join((text or "").split())
        if not text:
            raise ValueError("nothing to say")
        if len(text) > MAX_CHARS:
            raise ValueError(f"too long — {MAX_CHARS} characters at most")
        if not self.have_piper and not self.url:
            raise RuntimeError('Piper not installed — in the venv: pip install "piper-tts>=1.3"')
        voice = voice or self.voice
        if voice not in self.installed() and not self.url:
            raise RuntimeError("no voice downloaded yet — pick one under Voices and press Download")
        job = {"text": text, "voice": voice,
               "speed": self.speed if speed is None else max(0.5, min(2.0, float(speed))),
               "volume": self.volume if volume is None else max(0.1, min(2.0, float(volume)))}
        if append:
            with self._cv:
                if self._busy:
                    job["gen"], job["t"] = self._gen, time.monotonic()
                    self._queue.append(job)
                    self._cv.notify()
                    return
        # Cut what is sounding NOW, not when the new text is ready. Letting
        # the old sentence play out behind a new click was the buffer that
        # kept on playing. The song, if any, stays held for the new speech.
        #
        # Order matters, twice over. The generation is bumped BEFORE the cut,
        # so the old speaking thread sees its broken pipe as "superseded",
        # not as an error (which would also hand the song back for a blip).
        # And the new job is published AFTER the cut, so the cut can never
        # land on the new job's own freshly opened aplay.
        with self._cv:
            self._gen += 1
        self._cut(resume=False)
        self._submit(job)
        if remember:
            self.history = [text] + [h for h in self.history if h != text][:HISTORY - 1]
            self._save()

    def stop(self):
        """Silence speech at once and drop anything still being synthesised."""
        with self._cv:
            self._gen += 1
            self._job = None
            self._queue.clear()
            self._busy = False
            self.status, self.text = "idle", ""
        if self._proc and self._proc.poll() is None:
            try:
                self._send({"stop": True})      # a newer id: the worker drops the old text
            except OSError:
                pass
        self._cut(resume=True)

    @property
    def busy(self):
        return self._busy

    def wait(self, timeout=None):
        """Block until speaking has finished. For the CLI."""
        end = None if timeout is None else time.monotonic() + timeout
        while self._busy and (end is None or time.monotonic() < end):
            time.sleep(0.05)

    def _warm(self):
        local = self.have_piper and self.voice in self.installed()
        if (local or self.url) and not self._busy:
            self._submit({"warm": True, "voice": self.voice})

    def _submit(self, job):
        with self._cv:
            self._gen += 1
            job["gen"], job["t"] = self._gen, time.monotonic()
            self._job = job
            self._queue.clear()
            self._busy = True
            self._cv.notify()

    def _current(self, job):
        return job["gen"] == self._gen

    def _cut(self, resume):
        if self.player._kind == "speech":
            self.player.cancel_interjection(resume=resume)

    # --- worker plumbing ----------------------------------------------------

    def _ensure_worker(self):
        if self._proc and self._proc.poll() is None:
            return
        self._err = tempfile.TemporaryFile()
        self._proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--worker"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._err,
            text=True, bufsize=1)
        self._replies = queue.Queue()
        out, replies = self._proc.stdout, self._replies

        def pump():
            for line in out:
                try:
                    replies.put(json.loads(line))
                except ValueError:
                    pass
            replies.put(None)                 # worker exited

        threading.Thread(target=pump, daemon=True).start()

    def _send(self, req):
        with self._send_lock:
            self._ensure_worker()
            self._rid += 1
            req["id"] = self._rid
            self._proc.stdin.write(json.dumps(req) + "\n")
            self._proc.stdin.flush()
            return self._rid

    def _worker_error(self):
        try:
            self._err.seek(0)
            lines = self._err.read().decode(errors="replace").strip().splitlines()
            return lines[-1] if lines else "no message"
        except (OSError, ValueError, AttributeError):
            return "no message"

    @staticmethod
    def _discard(rep):
        if rep and rep.get("chunk"):
            try:
                os.remove(rep["chunk"])
            except OSError:
                pass

    def _replies_for(self, rid, job):
        """Worker replies to request rid, until done / error / superseded.
        Polls in 50 ms steps so a newer click is noticed almost at once."""
        idle = 0.0
        while self._current(job):
            try:
                rep = self._replies.get(timeout=0.05)
            except queue.Empty:
                idle += 0.05
                if idle > SYNTH_TIMEOUT_S:
                    self._proc.kill()
                    raise RuntimeError(f"no audio for {SYNTH_TIMEOUT_S} s — speech worker restarted")
                continue
            if rep is None:
                raise RuntimeError("speech worker crashed: " + self._worker_error())
            if rep.get("id") != rid:
                self._discard(rep)            # left over from superseded text
                continue
            idle = 0.0
            if "engine" in rep:
                self.engine, self.remote_error = rep["engine"], rep.get("warn", "")
            yield rep
            if rep.get("done") or rep.get("error"):
                return

    # --- the speaking thread ------------------------------------------------

    def _run(self):
        carry = None          # (aplay, rate, gen) left open for a queued sentence
        while True:
            with self._cv:
                while self._job is None and not self._queue:
                    self._cv.wait()
                if self._job is not None:
                    job, self._job = self._job, None
                else:
                    job = self._queue.popleft()
            if carry and carry[2] != job["gen"]:
                _quiet_kill(carry[0])          # superseded; normally already cut
                carry = None
            if not self._current(job):
                continue
            try:
                if job.get("warm"):
                    self._do_warm(job)
                else:
                    carry = self._speak(job, carry)
            except Exception as e:                             # noqa: BLE001
                carry = None
                if self._current(job):
                    self.error = f"{e.__class__.__name__}: {e}"
            finally:
                with self._cv:
                    if (self._current(job) and self._job is None
                            and not self._queue and carry is None):
                        self._busy = False
                        self.status, self.text = "idle", ""

    def _do_warm(self, job):
        self.status = "loading"
        rid = self._send({"model": _model_path(job["voice"]), "voice": job["voice"],
                          "url": self.url, "warm": True})
        for rep in self._replies_for(rid, job):
            if rep.get("error"):
                self.error = rep["error"]
            if rep.get("info"):
                self.startup = rep["info"]

    def _speak(self, job, carry=None):
        """Speak one job. Returns (aplay, rate, gen) if the pipe was left open
        for a queued sentence of the same utterance, else None."""
        self.status, self.text, self.error = "synth", job["text"], ""
        label = job["text"] if len(job["text"]) <= 60 else job["text"][:57] + "…"
        model = _model_path(job["voice"])
        rate = _sample_rate(model)
        proc, keep = None, False
        try:
            rid = self._send({"model": model, "voice": job["voice"], "url": self.url,
                              "text": job["text"], "speed": job["speed"],
                              "volume": job["volume"], "dir": self._chunk_dir})
            if carry and carry[1] == rate and carry[0].poll() is None:
                # Same stream as the sentence before: no gap, no device
                # re-open, and a held song stays held across the whole reply.
                proc = carry[0]
                self.player.relabel(label)
            else:
                if carry:
                    _quiet_close(carry[0])     # different sample rate: let it finish
                # Open the speaker now, while the first piece is still being
                # synthesised, so device start-up is not added on top of it.
                # aplay waits on the empty pipe without playing: no underrun.
                proc = self.player.interject_stream("speech", label, rate)
            for rep in self._replies_for(rid, job):
                if rep.get("error"):
                    raise RuntimeError(rep["error"])
                if rep.get("info"):
                    self.startup = rep["info"]
                if "chunk" not in rep:
                    continue
                with open(rep["chunk"], "rb") as f:
                    pcm = f.read()
                self._discard(rep)
                if not self._current(job):
                    break
                if self.first_sound_ms is None or self.status != "speaking":
                    self.first_sound_ms = round((time.monotonic() - job["t"]) * 1000)
                    self.status = "speaking"
                proc.stdin.write(pcm)          # blocks while aplay is behind: fine
                proc.stdin.flush()
            if self._current(job):
                with self._cv:
                    nxt = self._queue[0] if self._queue else None
                    keep = bool(nxt) and nxt["gen"] == job["gen"] and not nxt.get("warm")
                if keep:
                    return proc, rate, job["gen"]
                proc.stdin.close()             # aplay plays out what it has, then exits
                while proc.poll() is None and self._current(job):
                    time.sleep(0.05)
        except (BrokenPipeError, ValueError, OSError) as e:
            # A newer click killed aplay mid-write: expected, not an error.
            if self._current(job):
                self.error = self.player.last_error or f"speaker: {e}"
        except RuntimeError as e:
            if self._current(job):
                self.error = str(e)
        finally:
            if self._current(job) and not keep:
                if proc is not None and proc.poll() is None:
                    self._cut(resume=False)
                # A song held for speech that never played out gets its turn back.
                self.player.release_hold()
        return None

    # --- state --------------------------------------------------------------

    @property
    def brief(self):
        return {"status": self.status, "text": self.text, "error": self.error,
                "first_sound_ms": self.first_sound_ms, "startup": self.startup,
                "engine": self.engine}

    @property
    def state(self):
        s = self.brief
        s.update({
            "have_piper": self.have_piper,
            "voice": self.voice,
            "speed": self.speed,
            "volume": self.volume,
            "voices": self.voices(),
            "history": self.history,
            "max_chars": MAX_CHARS,
            "url": self.url,
            "remote_error": self.remote_error,
        })
        return s

    def close(self):
        self.stop()
        if self._proc and self._proc.poll() is None:
            self._proc.kill()


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

def blueprint(tts):
    from flask import Blueprint, jsonify, request              # noqa: PLC0415

    bp = Blueprint("tts", __name__)
    errors = (RuntimeError, ValueError, OSError, FileNotFoundError, TypeError)

    def run(fn):
        try:
            fn(request.get_json(force=True, silent=True) or {})
        except errors as e:
            # The request's own failure wins over the worker's last error.
            return jsonify({**tts.brief, "error": str(e)}), 400
        return jsonify({**tts.brief, "ok": True})

    @bp.route("/tts/state")
    def tts_state():
        return jsonify(tts.state)

    @bp.route("/tts/say", methods=["POST"])
    def tts_say():
        return run(lambda d: tts.say(d.get("text"), d.get("voice"),
                                     d.get("speed"), d.get("volume")))

    @bp.route("/tts/stop", methods=["POST"])
    def tts_stop():
        return run(lambda d: tts.stop())

    @bp.route("/tts/settings", methods=["POST"])
    def tts_settings():
        return run(lambda d: tts.set(d.get("voice"), d.get("speed"), d.get("volume"),
                                     d.get("url")))

    @bp.route("/tts/download", methods=["POST"])
    def tts_download():
        return run(lambda d: tts.download(d.get("voice")))

    @bp.route("/tts/delete", methods=["POST"])
    def tts_delete():
        return run(lambda d: tts.delete(d.get("voice")))

    @bp.route("/tts/forget", methods=["POST"])
    def tts_forget():
        return run(lambda d: tts.forget(d.get("text")))

    return bp


if __name__ == "__main__":
    if "--worker" in sys.argv:
        worker()
    else:
        print("Library + worker. Try:  python test/speaker_test.py --say 'hello'")
