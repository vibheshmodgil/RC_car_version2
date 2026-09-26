"""Truck sounds: the truck telling you what it is doing.

Watches the rest of the program ten times a second and plays a short sound
(audio.BEEPS) when something happens:

  reverse_pip   every second while it is actually reversing - after the
                guard, so a refused reverse makes no sound
  armed         ENABLE pressed, motors live           disarmed  and off again
  locked        follow mode has picked its person
  lost          ...and lost sight of them              found     ...and found them again
  gave_up       ...and stopped looking
  arrived       reached a place it was sent to (not the follower's own trips)
  mapped        auto-map finished                      failed    auto-map / go-to gave up
  bonk          the guard refused a move you drove (manual only - the
                autonomous modes are refused all the time, by design)

It never talks over speech: a cue that comes up while the truck is speaking
waits up to PENDING_S and is dropped after that. Over a song, only the
one-off event cues play (the player holds the song and resumes it); the
reverse alarm and bonk stay quiet rather than chop the music every second.

Settings are in cues.json next to this file (Pi-side), set from the Drive
tab's Speaker card: all sounds on/off, and the reverse alarm on its own.
"""

import json
import os
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(_HERE, "cues.json")

PERIOD_S = 0.1
REVERSE_EVERY_S = 1.0       # one pip a second, like a real backing-up alarm
REVERSE_AFTER_S = 0.25      # reversing this long before the first pip (a tap is not reversing)
REVERSE_MIN = 0.05          # throttle below -this counts as reversing
BONK_GAP_S = 2.0            # at most one bonk this often
PENDING_S = 2.0             # a cue held back by speech is dropped after this

# Only the latest of these is kept when several wait on speech.
EVENT_CUES = ("armed", "disarmed", "locked", "lost", "found", "gave_up",
              "arrived", "mapped", "failed")


class Cues:
    """speaker: audio.Audio. The rest are callables, so this file needs
    nothing from web_nav.py:

      armed()     -> bool
      drive()     -> (throttle_in, steer_in, throttle_out, steer_out, source)
      follow()    -> (state, message): "off" / "locking" / "following" / "searching"
      explore()   -> (state, message, is_follower_trip)
    """

    def __init__(self, speaker, armed, drive, follow, explore):
        self.speaker = speaker
        self._armed, self._drive, self._follow, self._explore = armed, drive, follow, explore
        self.enabled = True
        self.reverse = True
        self._load()
        self.last = ""                      # last cue played, for the page
        self.last_t = 0.0
        self._prev = None
        self._pending = None                # (kind, since)
        self._rev_since = None
        self._rev_last = 0.0
        self._bonk_last = 0.0
        self._refused = False
        threading.Thread(target=self._prerender, daemon=True, name="cues-render").start()
        threading.Thread(target=self._loop, daemon=True, name="cues").start()

    # --- settings -------------------------------------------------------------

    def _load(self):
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
            self.enabled = bool(d.get("enabled", True))
            self.reverse = bool(d.get("reverse", True))
        except (OSError, ValueError):
            pass

    def set(self, enabled=None, reverse=None):
        if enabled is not None:
            self.enabled = bool(enabled)
        if reverse is not None:
            self.reverse = bool(reverse)
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump({"enabled": self.enabled, "reverse": self.reverse}, f)
        except OSError:
            pass
        return self.state

    @property
    def state(self):
        return {"enabled": self.enabled, "reverse": self.reverse,
                "last": self.last, "last_s_ago": round(time.monotonic() - self.last_t, 1) if self.last_t else None}

    # --- playing ----------------------------------------------------------------

    def _prerender(self):
        """Render every cue's WAV once at start-up, so the first "lost" plays
        the instant it happens instead of after the samples are computed."""
        import audio                                          # noqa: PLC0415
        for kind in EVENT_CUES + ("reverse_pip", "bonk"):
            try:
                path, samples = self.speaker._beep_wav(kind)
                if samples is not None:
                    audio._write_wav(path, samples)
            except Exception:                                 # noqa: BLE001
                pass

    def _speaking(self):
        sp = self.speaker
        return getattr(sp, "_kind", None) == "speech" and sp.interjecting

    def _music(self):
        return bool(getattr(self.speaker, "music_playing", False))

    def _play(self, kind):
        try:
            self.speaker.beep(kind)
            self.last, self.last_t = kind, time.monotonic()
        except Exception:                                     # noqa: BLE001
            pass                                              # no speaker: sounds are optional

    def cue(self, kind):
        """An event cue: now, or after the truck stops talking."""
        if not self.enabled:
            return
        if self._speaking():
            self._pending = (kind, time.monotonic())
            return
        self._play(kind)

    # --- watching ---------------------------------------------------------------

    def _loop(self):
        while True:
            time.sleep(PERIOD_S)
            try:
                self._tick(time.monotonic())
            except Exception:                                 # noqa: BLE001
                pass                                          # a sound must never take anything down

    def _tick(self, t):
        armed = bool(self._armed())
        fstate, fmsg = self._follow()
        fstate = fstate or "off"
        estate, emsg, ftrip = self._explore()
        now = (armed, fstate, estate)
        prev, self._prev = self._prev, now
        if not self.enabled:
            self._rev_since = None
            return

        if self._pending is not None and not self._speaking():
            kind, since = self._pending
            self._pending = None
            if t - since < PENDING_S:
                self._play(kind)

        if prev is not None:                                  # nothing at start-up
            p_armed, p_follow, p_explore = prev
            if armed != p_armed:
                self.cue("armed" if armed else "disarmed")
            if fstate != p_follow:
                if p_follow == "locking" and fstate == "following":
                    self.cue("locked")
                elif p_follow == "following" and fstate == "searching":
                    self.cue("lost")
                elif p_follow == "searching" and fstate == "following":
                    self.cue("found")
                elif p_follow == "searching" and fstate == "off" and (fmsg or "").startswith("lost them"):
                    self.cue("gave_up")                       # not when you pressed Stop
            if estate != p_explore and not ftrip and fstate == "off":
                msg = (emsg or "").lower()
                if estate == "done" and msg.startswith("arrived"):
                    self.cue("arrived")
                elif estate == "done" and "mapped" in msg:
                    self.cue("mapped")
                elif estate == "failed":
                    self.cue("failed")

        th_in, st_in, th_out, st_out, source = self._drive()
        # Quiet over speech, a song, or any other sound still playing - a
        # pip must not cut off the horn you are sounding while reversing.
        quiet = self._speaking() or self._music() or self.speaker.interjecting

        # Reverse alarm: what the motors are REALLY doing, after the guard.
        if armed and th_out < -REVERSE_MIN:
            if self._rev_since is None:
                self._rev_since = t
            if (self.reverse and not quiet and t - self._rev_since >= REVERSE_AFTER_S
                    and t - self._rev_last >= REVERSE_EVERY_S):
                self._rev_last = t
                self._play("reverse_pip")
        else:
            self._rev_since = None

        # Bonk: you drove, the guard refused. Once per refusal.
        refused = (armed and source == "manual" and (abs(th_in) > 0.05 or abs(st_in) > 0.05)
                   and abs(th_out) < 1e-3 and abs(st_out) < 1e-3)
        if refused and not self._refused and not quiet and t - self._bonk_last > BONK_GAP_S:
            self._bonk_last = t
            self._play("bonk")
        self._refused = refused
