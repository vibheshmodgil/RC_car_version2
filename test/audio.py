"""
Audio — MAX98357A over I2S. Shared library.

Used by web_nav.py (Audio tab + horn), web_dashboard.py and speaker_test.py,
so a fix to playback lands everywhere at once.

    sudo apt install -y alsa-utils ffmpeg

What plays how
--------------
  Tone / sweep / beep   A WAV generated in-process and played with `aplay`.
                        Needs only alsa-utils, so it works before ffmpeg is
                        installed and is the first thing to try when the
                        speaker is silent.
  Music                 Uploaded files decoded by `ffmpeg` straight to the
                        ALSA device (`-f alsa`), with bass, treble and volume
                        applied in the same filter chain.

Why ffmpeg and not ffplay: ffplay reaches ALSA through SDL, which takes its
device from environment variables whose names changed between SDL2 and SDL3
(Trixie ships sdl2-compat on SDL3). `ffmpeg -f alsa plughw:1,0` names the
device on the command line and has no such layer to go wrong.

The MAX98357A has no volume register — it plays whatever samples arrive — so
`amixer` shows no control and every level here is applied in software.

One player at a time
--------------------
The raw I2S device cannot mix two streams, so there is only ever one player
process. Pause, a volume change and a beep over music all work the same way:
note the song position, stop the process, and restart it later with
`-ss <position>`. A beep — or speech from tts.py — over music therefore
interrupts the song for its length and then picks it up where it left off.

Wiring, the SD/GAIN pins and the config.txt overlay: WIRING.md section 7.
"""

import json
import math
import os
import re
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import wave

_HERE = os.path.dirname(os.path.abspath(__file__))
AUDIO_DIR = os.path.join(_HERE, "uploads")
SETTINGS_FILE = os.path.join(_HERE, "audio.json")
ALLOWED_AUDIO = {".mp3", ".wav", ".ogg", ".flac", ".m4a", ".aac"}
MAX_UPLOAD_MB = 64
RATE = 44100

# Ceiling on generated tone amplitude, 0.0-1.0. A full-scale sine into a
# class-D amp is loud enough to damage a small speaker and your hearing, and
# the tone is a diagnostic, not a demo.
MAX_TONE_LEVEL = 0.5

# Beeps are short, so they are allowed louder than a sustained test tone —
# a horn nobody can hear over the motors is not a horn.
MAX_BEEP_LEVEL = 0.8

# The horn ignores the beep slider and always plays flat out. It is shaped
# to be loud within that ceiling too: a saturated waveform carries several
# times the energy of a sine at the same peak, and its harmonics land at
# 1-3 kHz, where a small speaker is most efficient and ears most sensitive.
# The MAX98357A delivers ~3 W into 4 ohm from 5 V, so a 12 W speaker is not
# at risk. Beyond this, loudness is the amp's GAIN pin — WIRING.md section 7.
HORN_LEVEL = 0.97

# Bump when a beep's sound changes. Rendered WAVs are cached in /tmp by
# name, and without this the old sound keeps playing until a reboot.
BEEP_CACHE_VERSION = 3

# Music gain ceiling. Above 1.0 the volume filter clips loud passages.
MAX_VOLUME = 1.5

# Card names that mean "this is the speaker", not HDMI. ALSA puts HDMI at
# card 0, so "first device" is the wrong default on exactly this build.
SPEAKER_HINTS = ("max98357", "hifiberry", "i2s", "dac")

# Each beep is a list of (frequencies, seconds). Several frequencies in one
# segment are mixed (a chord); none is a silence.
BEEPS = {
    "beep":    [((1000,), 0.15)],
    "double":  [((1000,), 0.10), ((), 0.07), ((1000,), 0.10)],
    "horn":    "horn",                                        # Indian truck, below
    "chirp":   "chirp",                                       # rising sweep
    # Backing-up alarm: a real one sounds about once a second for as long as
    # the truck reverses, so 8 s of it rather than three quick pips. Stop on
    # the Audio tab cuts it short.
    "reverse": [((1100,), 0.5), ((), 0.5)] * 8,
    "alert":   [((880,), 0.16), ((660,), 0.16)] * 3,          # two-tone

    # Cues - the truck telling you what it is doing (test/cues.py). Short,
    # and each with a shape you can learn without looking: rising = good /
    # on, falling = lost / off, a chord = done.
    "reverse_pip": [((1100,), 0.40)],                         # one pulse of the backing-up alarm
    "armed":    [((523,), 0.08), ((), 0.03), ((784,), 0.13)],
    "disarmed": [((784,), 0.08), ((), 0.03), ((523,), 0.15)],
    "locked":   [((659,), 0.07), ((880,), 0.07), ((1319,), 0.15)],          # "got you"
    "found":    [((784,), 0.07), ((1047,), 0.13)],                          # "there you are"
    "lost":     [((698,), 0.15), ((587,), 0.15), ((466,), 0.30)],           # "uh-oh"
    "gave_up":  [((587,), 0.2), ((494,), 0.2), ((392,), 0.2), ((294,), 0.45)],
    "arrived":  [((523, 659, 784), 0.12), ((), 0.05), ((523, 659, 784, 1047), 0.32)],
    "mapped":   [((523,), 0.1), ((659,), 0.1), ((784,), 0.1), ((1047,), 0.1),
                 ((), 0.05), ((523, 784, 1047), 0.45)],
    "failed":   [((330,), 0.2), ((262,), 0.4)],
    "bonk":     [((300, 450), 0.09)],                          # the guard said no
}

MODES = ("single", "all", "repeat")     # after a song: stop / next / again

# Player kinds that hold a song and hand back to it when they finish.
INTERJECTIONS = ("beep", "speech")


# ---------------------------------------------------------------------------
# Sample generation
# ---------------------------------------------------------------------------

def _write_wav(path, samples, rate=RATE):
    """16-bit mono WAV from an iterable of floats in -1.0 .. 1.0."""
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(b"".join(
            struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32767))
            for s in samples
        ))


def _envelope(i, n, fade):
    """10 ms raised edges. Without them a tone starts and ends with a click,
    which on a class-D amp is a broadband transient into the speaker."""
    if i < fade:
        return i / fade
    if i > n - fade:
        return (n - i) / fade
    return 1.0


def tone_samples(freq, seconds, level, rate=RATE):
    return chord_samples((freq,), seconds, level, rate)


def chord_samples(freqs, seconds, level, rate=RATE):
    n = int(rate * seconds)
    fade = max(1, int(rate * 0.01))
    if not freqs:
        return [0.0] * n
    k = level / len(freqs)          # keep the sum of the chord inside level
    w = [2 * math.pi * f / rate for f in freqs]
    return [k * _envelope(i, n, fade) * sum(math.sin(x * i) for x in w)
            for i in range(n)]


def sweep_samples(f0, f1, seconds, level, rate=RATE):
    """Logarithmic sweep — equal time per octave, which is how a speaker's
    response is actually read. A linear sweep spends most of its time above
    5 kHz and tells you nothing about the bottom end."""
    n = int(rate * seconds)
    fade = max(1, int(rate * 0.01))
    ratio = f1 / f0
    phase = 0.0
    out = []
    for i in range(n):
        phase += 2 * math.pi * f0 * (ratio ** (i / n)) / rate
        out.append(level * _envelope(i, n, fade) * math.sin(phase))
    return out


# --- the horn ----------------------------------------------------------------
#
# An Indian truck pressure horn, not a car's polite beep. What makes it that
# sound, and what each part below is for:
#
#   Two trumpets     a low and a high horn blown together, about a major
#                    third apart and not quite in tune with each other. The
#                    mistuning is the point: the pair beats against itself
#                    and comes out raucous instead of musical.
#   Brassy tone      a reed horn is rich in harmonics, not a sine. Built from
#                    a wavetable of the first twelve with a slow roll-off.
#   Air pressure     each blast starts flat and rises to pitch over ~80 ms as
#                    the pressure builds, and sags slightly as it dies.
#   Flutter          a small fast wobble in pitch, from the compressor.
#   Saturation       driven into a soft clip, which is both how an overblown
#                    horn sounds and what makes it loud at a fixed peak.
#   Pattern          "paa-paa-paaaam" — the HORN OK PLEASE rhythm.

HORN_TONES = (349.0, 442.0)       # low and high trumpet, beating a little
HORN_PATTERN = ((0.17, 0.08), (0.17, 0.08), (0.95, 0.0))   # (blast s, gap s)
_HORN_HARMONICS = (1.0, 0.9, 0.75, 0.62, 0.5, 0.42, 0.34, 0.27, 0.2, 0.15, 0.11, 0.08)


def _horn_table(n=4096):
    t = [sum(a * math.sin(2 * math.pi * (k + 1) * i / n)
             for k, a in enumerate(_HORN_HARMONICS)) for i in range(n)]
    peak = max(abs(v) for v in t)
    return [v / peak for v in t]


def horn_samples(level=HORN_LEVEL, rate=RATE):
    table = _horn_table()
    size = len(table)
    drive = 4.5                     # soft clip to a ~1.6 crest factor: loud, still brassy
    norm = math.tanh(drive)
    out = []
    phases = [0.0, 0.37]            # the two horns never start in step
    for blast, gap in HORN_PATTERN:
        n = int(rate * blast)
        attack, release = int(rate * 0.018), int(rate * 0.045)
        for i in range(n):
            t = i / rate
            rise = min(1.0, t / 0.08)                   # pressure building
            pitch = 0.9 + 0.1 * (1 - (1 - rise) ** 2)   # flat -> in tune
            if i > n - release:
                pitch -= 0.03 * (i - (n - release)) / release   # pressure dying
            pitch *= 1.0 + 0.004 * math.sin(2 * math.pi * 23 * t)   # flutter
            amp = min(1.0, i / attack, (n - i) / release)
            x = 0.0
            for h, f in enumerate(HORN_TONES):
                phases[h] = (phases[h] + f * pitch / rate) % 1.0
                x += table[int(phases[h] * size)]
            out.append(level * amp * math.tanh(drive * x / len(HORN_TONES)) / norm)
        out += [0.0] * int(rate * gap)
    return out


def beep_samples(kind, level):
    pattern = BEEPS[kind]
    if pattern == "horn":
        return horn_samples()
    if pattern == "chirp":
        return sweep_samples(600, 2400, 0.22, level)
    out = []
    for freqs, secs in pattern:
        out += chord_samples(freqs, secs, level)
    return out


def _safe_name(name):
    """Reduce a client-supplied file name to [A-Za-z0-9._-], no path.

    Same job as werkzeug's secure_filename, without making the CLI test
    depend on Flask. Anything that survives also interpolates safely into
    the page's HTML and onclick handlers."""
    base = os.path.basename((name or "").replace("\\", "/"))
    return re.sub(r"[^A-Za-z0-9._-]+", "_", base).strip("._")


# ---------------------------------------------------------------------------
# Player
# ---------------------------------------------------------------------------

class Audio:
    """One player process at a time, driven by ALSA and ffmpeg tools."""

    def __init__(self):
        os.makedirs(AUDIO_DIR, exist_ok=True)
        self._lock = threading.RLock()
        self._proc = None
        self._kind = None          # "tone" | "beep" | "speech" | "music"
        self._label = None
        self._err = None           # temp file holding the player's stderr
        self.have_aplay = shutil.which("aplay") is not None
        self.have_ffmpeg = shutil.which("ffmpeg") is not None
        self.have_ffprobe = shutil.which("ffprobe") is not None
        self._wav = os.path.join(tempfile.gettempdir(), "truck_tone.wav")
        self.last_error = ""

        # Music transport. _pos is where the running process started in the
        # song; while playing, the position is _pos + time since _t0.
        self.track = None
        self._pos = 0.0
        self._t0 = None
        self.paused = False
        self._resume_at = None     # song position to return to after a beep

        self._dur = {}             # name -> (mtime, seconds)
        self._devs = (0.0, [])     # (when, list) — `aplay -l` is a fork

        # Settings, remembered across restarts in audio.json.
        self.device = None
        self.volume = 0.8
        self.bass = 0.0
        self.treble = 0.0
        self.mode = "all"
        self.beep_level = 0.5
        self._load()
        if self.device is None:
            self.device = self.pick_speaker()

        threading.Thread(target=self._watch, daemon=True).start()
        # The horn takes ~1 s of pure Python to render on a Pi 4. Do it now,
        # so the first press on the road is not the one that waits.
        threading.Thread(target=self._prerender_horn, daemon=True).start()

    # --- settings -----------------------------------------------------------

    def _load(self):
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        self.device = d.get("device") or None
        self.set_levels(d.get("volume"), d.get("bass"), d.get("treble"),
                        restart=False)
        if d.get("mode") in MODES:
            self.mode = d["mode"]
        if d.get("beep_level") is not None:
            self.beep_level = max(0.0, min(MAX_BEEP_LEVEL, float(d["beep_level"])))

    def _save(self):
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump({"device": self.device, "volume": self.volume,
                           "bass": self.bass, "treble": self.treble,
                           "mode": self.mode, "beep_level": self.beep_level},
                          f, indent=2)
        except OSError:
            pass                    # a read-only card must not stop the music

    # --- devices ------------------------------------------------------------

    def devices(self, fresh=False):
        """Playback devices from `aplay -l`, as selectable plughw:C,D strings.
        Cached for a few seconds: the page polls, and each call is a fork."""
        when, found = self._devs
        if not fresh and time.monotonic() - when < 5.0:
            return found
        found = []
        if self.have_aplay:
            try:
                out = subprocess.run(["aplay", "-l"], capture_output=True,
                                     text=True, timeout=5).stdout or ""
            except (OSError, subprocess.SubprocessError):
                out = ""
            for line in out.splitlines():
                # card 1: MAX98357A [MAX98357A], device 0: bcm2835-i2s-... []
                if not line.startswith("card ") or "device " not in line:
                    continue
                try:
                    card = line.split("card ", 1)[1].split(":", 1)[0].strip()
                    dev = line.split("device ", 1)[1].split(":", 1)[0].strip()
                    name = line.split("[", 1)[1].split("]", 1)[0]
                except IndexError:
                    continue
                # plughw, not hw. `hw:` is the raw device and accepts only the
                # exact format the hardware wants — the bcm2835 I2S interface
                # wants stereo, so a mono tone is rejected with "Channels count
                # non available" and nothing plays. `plughw:` inserts ALSA's
                # conversion plugin, which fixes up channels, rate and format.
                found.append({"dev": f"plughw:{card},{dev}", "name": name})
        self._devs = (time.monotonic(), found)
        return found

    def pick_speaker(self):
        """The I2S amp if ALSA lists one, else None (ALSA's default)."""
        for d in self.devices(fresh=True):
            if any(h in d["name"].lower() for h in SPEAKER_HINTS):
                return d["dev"]
        return None

    def card(self):
        """Name of the selected card, for display only."""
        devs = self.devices()
        for d in devs:
            if d["dev"] == self.device:
                return d["name"]
        return devs[0]["name"] if devs else None

    def set_device(self, dev):
        """dev is one of the plughw:C,D strings from devices(), or None/''
        for ALSA's default."""
        if dev and dev not in [d["dev"] for d in self.devices(fresh=True)]:
            raise ValueError(f"unknown device: {dev}")
        self.device = dev or None
        self._save()
        if self.music_playing:
            self._music(self.track, self.position)      # follow to the new card

    def _alsa_dev(self):
        return self.device or "default"

    # --- process plumbing ---------------------------------------------------

    def _kill(self):
        """Stop whatever is running without touching the transport state."""
        with self._lock:
            p, self._proc, self._kind, self._label = self._proc, None, None, None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                p.kill()

    def _spawn(self, cmd, kind, label):
        """Start a player and confirm it is still alive a moment later.

        Popen succeeds even when the player dies instantly — wrong ALSA
        device, no sound card, unsupported format — so without this check the
        request returns 200 while nothing plays and nothing is reported.
        stderr goes to a temp file rather than a pipe: a pipe nobody reads
        fills at 64 kB and freezes the player mid-song, which a damaged MP3
        spewing decode warnings will do.
        """
        self._kill()
        err = tempfile.TemporaryFile()
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL, stderr=err)
        with self._lock:
            self._proc, self._kind, self._label, self._err = proc, kind, label, err
        time.sleep(0.3)
        if proc.poll() is not None and proc.returncode != 0:
            with self._lock:
                if self._proc is proc:
                    self._proc = self._kind = self._label = None
            msg = self._stderr(err, proc.returncode)
            self.last_error = msg
            raise RuntimeError(msg)
        self.last_error = ""

    @staticmethod
    def _stderr(err, code):
        try:
            err.seek(0)
            raw = err.read().decode(errors="replace").strip()
        except (OSError, ValueError):
            raw = ""
        return raw.splitlines()[-1] if raw else f"player exited {code}"

    def _aplay(self, path, kind, label, samples=None):
        """Play a WAV. With `samples`, (re)write it first — after the old
        player is gone, never under an aplay that is still reading it."""
        if not self.have_aplay:
            raise RuntimeError("aplay not found — sudo apt install alsa-utils")
        self._kill()
        if samples is not None:
            _write_wav(path, samples)
        self._spawn(["aplay", "-q", "-D", self._alsa_dev(), path], kind, label)

    def _watch(self):
        """What happens when a player finishes on its own: a beep hands back
        to the song it interrupted, a song moves on according to the mode."""
        while True:
            time.sleep(0.2)
            with self._lock:
                p, kind = self._proc, self._kind
                if p is None or p.poll() is None:
                    continue
                code, err = p.returncode, self._err
                self._proc = self._kind = self._label = None
            try:
                if kind in INTERJECTIONS:
                    if code not in (0, None):
                        self.last_error = self._stderr(err, code)
                    pos = self._take_resume()
                    if pos is not None and self.track:
                        self._music(self.track, pos)
                elif kind == "music" and code != 0:
                    self.last_error = self._stderr(err, code)
                    self.track, self.paused = None, False
                elif kind == "music":
                    self._advance()
            except (RuntimeError, OSError, FileNotFoundError) as e:
                self.last_error = str(e)
                self.track, self.paused = None, False

    # --- music --------------------------------------------------------------

    @property
    def music_playing(self):
        p = self._proc
        return self._kind == "music" and p is not None and p.poll() is None

    @property
    def position(self):
        if self.music_playing and self._t0 is not None:
            return self._pos + (time.monotonic() - self._t0)
        return self._pos

    def _music(self, name, pos=0.0):
        if not self.have_ffmpeg:
            raise RuntimeError("ffmpeg not found — sudo apt install ffmpeg")
        path = self.resolve(name)
        dur = self.duration(name)
        pos = max(0.0, float(pos or 0.0))
        if dur and pos >= dur:
            pos = 0.0
        # One filter chain does decode + shelves + gain in a single pass.
        af = (f"bass=g={self.bass:.1f},treble=g={self.treble:.1f},"
              f"volume={self.volume:.3f}")
        cmd = ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error"]
        if pos > 0:
            cmd += ["-ss", f"{pos:.2f}"]         # before -i: fast input seek
        cmd += ["-i", path, "-vn", "-af", af, "-f", "alsa", self._alsa_dev()]
        self.track, self._pos, self._t0, self.paused = name, pos, time.monotonic(), False
        self._resume_at = None
        try:
            self._spawn(cmd, "music", os.path.basename(path))
        except RuntimeError:
            self.track, self._pos, self._t0 = None, 0.0, None   # nothing is loaded
            raise

    def play(self, name, pos=0.0, volume=None, bass=None, treble=None):
        """Play an uploaded file from `pos` seconds. Levels, if given, become
        the new settings."""
        self.set_levels(volume, bass, treble, restart=False)
        self._music(_safe_name(name), pos)

    def pause(self):
        if self.music_playing:
            pos = self.position
            self._kill()
            self._pos, self._t0, self.paused = pos, None, True
        elif self._resume_at is not None:
            # Paused while a beep has the song on hold: stay paused after it.
            self._resume_at, self.paused = None, True

    def resume(self):
        if self.track and not self.music_playing:
            self._music(self.track, self._pos)

    def toggle_pause(self):
        if self.music_playing or self._resume_at is not None:
            self.pause()
        elif self.track:
            self.resume()
        else:
            self.skip(1)            # nothing loaded: play means the first song

    def seek(self, pos):
        if not self.track:
            return
        if self.music_playing:
            self._music(self.track, pos)
        else:
            self._pos = max(0.0, float(pos))

    def skip(self, step=1):
        names = [f["name"] for f in self.files()]
        if not names:
            return
        i = names.index(self.track) if self.track in names else -1
        self._music(names[(i + step) % len(names)], 0.0)

    def _advance(self):
        names = [f["name"] for f in self.files()]
        if self.mode == "repeat" and self.track in names:
            self._music(self.track, 0.0)
        elif self.mode == "all" and names:
            i = names.index(self.track) if self.track in names else -1
            self._music(names[(i + 1) % len(names)], 0.0)
        else:
            self.track, self._pos, self.paused = None, 0.0, False

    def set_levels(self, volume=None, bass=None, treble=None, restart=True):
        """Filters are fixed when ffmpeg starts, so a change while a song is
        playing restarts it at the same position — a gap of a few hundred ms,
        which beats a slider that silently does nothing until the next song."""
        old = (self.volume, self.bass, self.treble)
        if volume is not None:
            self.volume = max(0.0, min(MAX_VOLUME, float(volume)))
        if bass is not None:
            self.bass = max(-12.0, min(12.0, float(bass)))
        if treble is not None:
            self.treble = max(-12.0, min(12.0, float(treble)))
        if (self.volume, self.bass, self.treble) == old:
            return
        self._save()
        if restart and self.music_playing:
            self._music(self.track, self.position)

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {', '.join(MODES)}")
        self.mode = mode
        self._save()

    # --- tones and beeps ----------------------------------------------------

    def tone(self, freq, seconds, level):
        """Test tone. Ends any song — this is a diagnostic, not a jingle."""
        freq = max(20.0, min(20000.0, float(freq)))
        seconds = max(0.2, min(10.0, float(seconds)))
        level = max(0.0, min(MAX_TONE_LEVEL, float(level)))
        self.stop()
        self._aplay(self._wav, "tone", f"{freq:.0f} Hz tone",
                    tone_samples(freq, seconds, level))

    def sweep(self, f0, f1, seconds, level):
        f0 = max(20.0, min(20000.0, float(f0)))
        f1 = max(20.0, min(20000.0, float(f1)))
        seconds = max(1.0, min(20.0, float(seconds)))
        level = max(0.0, min(MAX_TONE_LEVEL, float(level)))
        self.stop()
        self._aplay(self._wav, "tone", f"sweep {f0:.0f}-{f1:.0f} Hz",
                    sweep_samples(f0, f1, seconds, level))

    def beep(self, kind="beep", level=None):
        """Short sound over whatever is playing. A song is paused for the
        length of the beep and then carries on from the same spot."""
        if kind not in BEEPS:
            raise ValueError(f"unknown beep: {kind}")
        if level is not None:
            self.beep_level = max(0.0, min(MAX_BEEP_LEVEL, float(level)))
            self._save()
        path, fresh = self._beep_wav(kind)
        self.interject(path, "beep", kind, fresh)

    def _beep_wav(self, kind):
        """Cached WAV path for a beep, plus its samples if not yet rendered.
        Once per kind and level, so the horn sounds the instant it is pressed
        instead of after tens of thousands of struct.pack calls."""
        level = HORN_LEVEL if kind == "horn" else self.beep_level
        path = os.path.join(tempfile.gettempdir(),
                            f"truck_beep_v{BEEP_CACHE_VERSION}_{kind}_{round(level * 100)}.wav")
        return path, (None if os.path.isfile(path) else beep_samples(kind, level))

    def _prerender_horn(self):
        path, samples = self._beep_wav("horn")
        if samples is not None:
            try:
                _write_wav(path, samples)
            except OSError:
                pass

    def _hold_song(self):
        if self.music_playing:
            # Only an interjection over a RUNNING song records the position.
            # A second one while the first is sounding finds no song running
            # and leaves it alone — otherwise mashing the horn rewinds it.
            self._pos = self._resume_at = self.position
            self._t0 = None

    def _take_resume(self):
        """Claim the held song position, once. Both the watcher (player
        exited) and release_hold (speech gave up) try to resume, and without
        this they occasionally both did — a restart glitch in the song."""
        with self._lock:
            pos, self._resume_at = self._resume_at, None
        return pos

    def interject(self, path, kind, label, samples=None):
        """Play a WAV over whatever is on — a beep, or speech from tts.py.
        A running song is held and resumes from the same spot afterwards."""
        self._hold_song()
        self._aplay(path, kind, label, samples)

    def interject_stream(self, kind, label, rate):
        """Like interject(), but aplay reads raw 16-bit mono PCM from a pipe,
        so sound starts as soon as the first piece exists instead of after
        the whole file. Write to .stdin; close it to let the sound finish.

        No liveness wait, unlike _spawn: that would add 0.3 s to every spoken
        sentence. A dead aplay shows up as a broken pipe on the first write."""
        if not self.have_aplay:
            raise RuntimeError("aplay not found — sudo apt install alsa-utils")
        self._hold_song()
        self._kill()
        err = tempfile.TemporaryFile()
        proc = subprocess.Popen(
            ["aplay", "-q", "-D", self._alsa_dev(), "-t", "raw", "-f", "S16_LE",
             "-c", "1", "-r", str(int(rate))],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=err)
        with self._lock:
            self._proc, self._kind, self._label, self._err = proc, kind, label, err
        self.last_error = ""
        return proc

    def relabel(self, label):
        """New caption for the interjection already playing — streamed
        speech keeps one aplay open across several sentences."""
        with self._lock:
            if self._kind in INTERJECTIONS:
                self._label = label

    def stop_music(self):
        """Forget the song without touching a beep or speech that is playing
        over it. stop() would kill whatever process is current — which, while
        the assistant is talking, is its own voice."""
        if self.music_playing:
            self.stop()
            return
        with self._lock:
            self._resume_at = None
        self.track, self._pos, self._t0, self.paused = None, 0.0, None, False

    def release_hold(self):
        """Resume a song held by an interjection that ended without playing
        out — speech that failed, or was cut short by Stop."""
        if self.interjecting:
            return
        pos = self._take_resume()
        if pos is not None and self.track:
            self._music(self.track, pos)

    @property
    def interjecting(self):
        p = self._proc
        return self._kind in INTERJECTIONS and p is not None and p.poll() is None

    def cancel_interjection(self, resume=True):
        """Cut a beep or speech short. resume=False keeps the song held — for
        speech being replaced by newer speech, where handing back to the song
        for a moment would be a blip of music between two sentences."""
        if self._kind not in INTERJECTIONS:
            return
        self._kill()
        if resume:
            self.release_hold()

    def stop(self):
        """Stop everything, forget the song."""
        self._resume_at = None
        self.track, self._pos, self._t0, self.paused = None, 0.0, None, False
        self._kill()

    def wait(self, timeout=None):
        """Block until the current player exits. For the command-line test."""
        p = self._proc
        if p is not None:
            try:
                p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass

    # --- library ------------------------------------------------------------

    def resolve(self, name):
        """Map a client-supplied name onto a real file inside AUDIO_DIR.
        _safe_name strips any path, and the isfile check keeps the result
        inside the upload directory."""
        safe = _safe_name(name)
        path = os.path.join(AUDIO_DIR, safe)
        if not safe or not os.path.isfile(path):
            raise FileNotFoundError(f"no such file: {name}")
        return path

    def duration(self, name):
        """Song length in seconds from ffprobe, cached per file. None when
        it cannot be read — the page then shows elapsed time only."""
        if not self.have_ffprobe:
            return None
        try:
            path = self.resolve(name)
            mtime = os.path.getmtime(path)
        except (OSError, FileNotFoundError):
            return None
        hit = self._dur.get(name)
        if hit and hit[0] == mtime:
            return hit[1]
        try:
            out = subprocess.run(
                ["ffprobe", "-v", "error", "-show_entries", "format=duration",
                 "-of", "csv=p=0", path],
                capture_output=True, text=True, timeout=10).stdout.strip()
            dur = float(out)
        except (OSError, subprocess.SubprocessError, ValueError):
            dur = None
        self._dur[name] = (mtime, dur)
        return dur

    def files(self):
        out = []
        for fn in sorted(os.listdir(AUDIO_DIR), key=str.lower):
            p = os.path.join(AUDIO_DIR, fn)
            safe = _safe_name(fn)
            if safe and safe != fn and os.path.isfile(p) \
                    and not os.path.exists(os.path.join(AUDIO_DIR, safe)):
                # Copied in by scp as "My Song.mp3". Every name the page sees
                # has to survive _safe_name, or it lists a file it cannot play.
                os.rename(p, os.path.join(AUDIO_DIR, safe))
                fn, p = safe, os.path.join(AUDIO_DIR, safe)
            if os.path.isfile(p) and os.path.splitext(fn)[1].lower() in ALLOWED_AUDIO:
                out.append({"name": fn, "kb": round(os.path.getsize(p) / 1024),
                            "dur": self.duration(fn)})
        return out

    def save(self, storage):
        """storage is a werkzeug FileStorage from a Flask upload."""
        name = _safe_name(storage.filename)
        ext = os.path.splitext(name)[1].lower()
        if not name or ext not in ALLOWED_AUDIO:
            raise ValueError(f"unsupported file type: {ext or '(none)'} — "
                             f"use {', '.join(sorted(ALLOWED_AUDIO))}")
        if name == self.track:
            self.stop()             # overwriting the file under a live decoder
        storage.save(os.path.join(AUDIO_DIR, name))
        return name

    def delete(self, name):
        path = self.resolve(name)
        if self.track == os.path.basename(path):
            self.stop()
        os.remove(path)

    # --- state --------------------------------------------------------------

    @property
    def brief(self):
        """Cheap summary for a page that polls fast. No forks."""
        with self._lock:
            kind, label = self._kind, self._label
        return {
            "playing": self.music_playing or kind in ("tone",) + INTERJECTIONS,
            "kind": kind,
            "now": label,
            "track": self.track,
            "paused": self.paused,
            "pos": round(self.position, 1),
            "dur": self.duration(self.track) if self.track else None,
            "volume": self.volume,
        }

    @property
    def state(self):
        devs = self.devices()
        s = self.brief
        s.update({
            "files": self.files(),
            "devices": devs,
            "device": self.device,
            "card": self.card(),
            "bass": self.bass,
            "treble": self.treble,
            "mode": self.mode,
            "beep_level": self.beep_level,
            "beeps": list(BEEPS),
            "have_aplay": self.have_aplay,
            "have_ffmpeg": self.have_ffmpeg,
            "max_tone_level": MAX_TONE_LEVEL,
            "max_beep_level": MAX_BEEP_LEVEL,
            "max_volume": MAX_VOLUME,
            "max_upload_mb": MAX_UPLOAD_MB,
            "last_error": self.last_error,
        })
        return s


# ---------------------------------------------------------------------------
# Flask routes — one implementation for every page that has a speaker panel
# ---------------------------------------------------------------------------

def blueprint(player):
    """All /audio/* routes for `player`. Register with
    app.register_blueprint(audio.blueprint(player)).

    Every route returns JSON with an `error` rather than a bare 204, so the
    page can say why something failed (missing ffmpeg, bad device, file type)
    instead of silently doing nothing.
    """
    from flask import Blueprint, jsonify, request      # noqa: PLC0415

    bp = Blueprint("audio", __name__)
    errors = (RuntimeError, ValueError, OSError, FileNotFoundError, TypeError)

    def body():
        return request.get_json(force=True, silent=True) or {}

    def run(fn):
        try:
            fn(body())
        except errors as e:
            return jsonify(error=str(e), **player.brief), 400
        return jsonify(ok=True, **player.brief)

    @bp.route("/audio/state")
    def audio_state():
        return jsonify(player.state)

    @bp.route("/audio/tone", methods=["POST"])
    def audio_tone():
        return run(lambda d: player.tone(d.get("freq", 440), d.get("seconds", 2),
                                         d.get("level", 0.25)))

    @bp.route("/audio/sweep", methods=["POST"])
    def audio_sweep():
        return run(lambda d: player.sweep(d.get("f0", 40), d.get("f1", 15000),
                                          d.get("seconds", 8), d.get("level", 0.25)))

    @bp.route("/audio/beep", methods=["POST"])
    def audio_beep():
        return run(lambda d: player.beep(d.get("kind", "beep"), d.get("level")))

    @bp.route("/audio/play", methods=["POST"])
    def audio_play():
        return run(lambda d: player.play(d.get("file"), d.get("pos", 0),
                                         d.get("volume"), d.get("bass"),
                                         d.get("treble")))

    @bp.route("/audio/pause", methods=["POST"])
    def audio_pause():
        return run(lambda d: player.toggle_pause())

    @bp.route("/audio/seek", methods=["POST"])
    def audio_seek():
        return run(lambda d: player.seek(d.get("pos", 0)))

    @bp.route("/audio/skip", methods=["POST"])
    def audio_skip():
        return run(lambda d: player.skip(-1 if d.get("back") else 1))

    @bp.route("/audio/stop", methods=["POST"])
    def audio_stop():
        return run(lambda d: player.stop())

    @bp.route("/audio/stop_music", methods=["POST"])
    def audio_stop_music():
        return run(lambda d: player.stop_music())

    @bp.route("/audio/levels", methods=["POST"])
    def audio_levels():
        return run(lambda d: player.set_levels(d.get("volume"), d.get("bass"),
                                               d.get("treble")))

    @bp.route("/audio/mode", methods=["POST"])
    def audio_mode():
        return run(lambda d: player.set_mode(d.get("mode")))

    @bp.route("/audio/device", methods=["POST"])
    def audio_device():
        return run(lambda d: player.set_device(d.get("device")))

    @bp.route("/audio/upload", methods=["POST"])
    def audio_upload():
        f = request.files.get("file")
        if f is None:
            return jsonify(error="no file in request"), 400
        try:
            return jsonify(ok=True, name=player.save(f))
        except errors as e:
            return jsonify(error=str(e)), 400

    @bp.route("/audio/delete", methods=["POST"])
    def audio_delete():
        return run(lambda d: player.delete(d.get("file")))

    @bp.app_errorhandler(413)
    def too_large(_e):
        return jsonify(error=f"file is larger than {MAX_UPLOAD_MB} MB"), 413

    return bp
