"""
The truck's AI-facing actions, over web_nav.py's HTTP API. Shared library.

One implementation, two users:

  brain.py              the on-board voice assistant, running inside
                        web_nav.py on the Pi — base URL http://127.0.0.1:5004
  tools/truck_mcp.py    the MCP server on the PC for Claude Code —
                        base URL http://shiv.local:5004

Going through HTTP even from inside web_nav is deliberate: every move then
takes exactly the path the arrow keys take — /drive intent, collision guard,
floor check, speed limit, watchdog — with no second route to the motors to
keep correct.

Safety that holds whoever is calling:
  * no method arms the motors; drive() and go_to_place() refuse until a
    person presses ENABLE in the cockpit
  * drive() is bounded (MAX_MOVE_S) and always ends with an explicit stop
  * no method turns the guard off
  * if the caller dies mid-move, web_nav's 0.6 s watchdog stops the motors

Standard library only, so the PC needs nothing beyond the mcp package.
"""

import json
import time
import urllib.error
import urllib.request

MAX_MOVE_S = 3.0
DRIVE_PERIOD_S = 0.15          # well inside web_nav's 0.6 s watchdog
# Clockwise from the nose. JSON from Flask arrives key-sorted, which reads as
# "ahead, ahead-left, ahead-right, behind..." — useless for picturing a room.
DIRECTIONS = ("ahead", "ahead-right", "right", "behind-right",
              "behind", "behind-left", "left", "ahead-left")
MOVES = {"forward": (1, 0), "backward": (-1, 0), "turn_left": (0, -1),
         "turn_right": (0, 1), "forward_left": (1, -0.6), "forward_right": (1, 0.6)}
BEEP_KINDS = ("horn", "beep", "double", "chirp", "reverse", "alert")


class TruckError(Exception):
    pass


def _mm(v):
    return "nothing seen" if v is None else (f"{v / 1000:.2f} m" if v >= 1000 else f"{v:.0f} mm")


class TruckApi:
    def __init__(self, base_url):
        self.base = base_url.rstrip("/")

    # --- transport ----------------------------------------------------------

    def request(self, method, path, body=None, timeout=8.0, raw=False):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"} if data else {})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                payload = r.read()
        except urllib.error.HTTPError as e:
            payload = e.read()
            try:
                msg = json.loads(payload).get("error") or payload.decode(errors="replace")
            except ValueError:
                msg = payload.decode(errors="replace")[:200]
            raise TruckError(f"{path}: HTTP {e.code} — {msg}") from None
        except (urllib.error.URLError, OSError) as e:
            raise TruckError(f"cannot reach the truck at {self.base} ({e}). "
                             "Is web_nav.py running?") from None
        if raw:
            return payload
        return json.loads(payload) if payload else {}

    def get(self, path, **kw):
        return self.request("GET", path, None, **kw)

    def post(self, path, body=None, **kw):
        return self.request("POST", path, body or {}, **kw)

    # --- reading ------------------------------------------------------------

    def ai_status(self):
        return self.get("/ai/status")

    def status_text(self, s=None):
        s = s or self.ai_status()
        m, g, li = s["motors"], s["guard"], s["lidar"]
        p, body = s["pose_mm_deg"], s["body_mm"]
        lines = [
            f"motors: {'ENABLED' if m['enabled'] else 'DISABLED — a person must press ENABLE in the cockpit'}"
            f"{' (watchdog tripped)' if m['watchdog_tripped'] else ''}, speed limit {m['speed_limit']:.2f}",
            f"pose: x {p['x'] / 1000:+.2f} m, y {p['y'] / 1000:+.2f} m, heading {p['deg']:.0f} deg"
            + (f"; compass {s['compass_deg']:.0f} deg" if s.get("compass_deg") is not None else ""),
            f"body: {body['length']:.0f} mm long, {body['width']:.0f} mm wide",
            f"lidar: {'%.1f Hz' % li['hz'] if li['connected'] else 'NOT CONNECTED — obstacle data missing, do not drive'}",
            "nearest obstacle by direction (from centre): "
            + ", ".join(f"{k} {_mm(li['nearest_mm_from_centre'].get(k))}" for k in DIRECTIONS),
            f"guard: {'on' if g['enabled'] else 'OFF'}, stop distance {g['stop_mm']:.0f} mm"
            + (f", BLOCKED: {g['reason']}" if g["blocked"] else "")
            + (f", clearance {json.dumps(g['clearance_mm'])}" if g.get("clearance_mm") else ""),
            f"floor check: {'on' if s['floor_check']['enabled'] else 'off'}"
            + (f" — {s['floor_check']['reason']}" if s["floor_check"].get("reason") else ""),
            f"camera: {'live' if s['camera_live'] else 'off'}",
        ]
        nav = s.get("navigation") or {}
        if nav.get("running"):
            lines.append(f"navigating: {nav.get('state')} {nav.get('goal') or ''} — {nav.get('message') or ''}")
        if s.get("places"):
            lines.append("saved places: " + ", ".join(s["places"]))
        sp = s.get("speech") or {}
        if sp.get("status") and sp["status"] != "idle":
            lines.append(f"speech: {sp['status']} \"{sp.get('text', '')}\"")
        if sp.get("error"):
            lines.append(f"speech error: {sp['error']}")
        mic = s.get("microphone") or {}
        lines.append(f"phone microphone: {'connected' if mic.get('phone_connected') else 'not connected'}"
                     + (f", {mic['pending']} unheard message(s) waiting" if mic.get("pending") else ""))
        au = s.get("audio") or {}
        if au.get("track"):
            lines.append(f"music: {au['track']}{' (paused)' if au.get('paused') else ''}")
        return "\n".join(lines)

    def look_jpeg(self):
        return self.get("/camera/still.jpg", raw=True)

    # --- moving -------------------------------------------------------------

    def drive(self, direction, seconds=1.0, speed=0.6):
        if direction not in MOVES:
            return f"unknown direction {direction!r}; use one of: {', '.join(MOVES)}"
        seconds = max(0.2, min(MAX_MOVE_S, float(seconds)))
        speed = max(0.2, min(1.0, float(speed)))

        before = self.ai_status()
        if not before["motors"]["enabled"]:
            return ("Not moved: the motors are DISABLED. Ask the person to press ENABLE "
                    "in the cockpit — this deliberately cannot arm them.")
        if not before["lidar"]["connected"]:
            return "Not moved: the LiDAR is not connected, so the collision guard is blind."

        th, st = MOVES[direction]
        body = {"throttle": th * speed, "steer": st * speed}
        blocked = set()
        t_end = time.monotonic() + seconds
        n = 0
        try:
            while time.monotonic() < t_end:
                self.post("/drive", body, timeout=2.0)
                time.sleep(DRIVE_PERIOD_S)
                n += 1
                if n % 4 == 0:
                    g = self.ai_status()["guard"]
                    if g["blocked"]:
                        blocked.add(g["reason"])
        finally:
            # Always stop explicitly, even if a request above failed. If this
            # also fails, web_nav's watchdog stops the motors within 0.6 s.
            try:
                self.post("/drive", {"throttle": 0, "steer": 0}, timeout=2.0)
                self.post("/stop", timeout=2.0)
            except TruckError:
                pass

        after = self.ai_status()
        p0, p1 = before["pose_mm_deg"], after["pose_mm_deg"]
        dist = ((p1["x"] - p0["x"]) ** 2 + (p1["y"] - p0["y"]) ** 2) ** 0.5
        turn = (p1["deg"] - p0["deg"] + 180) % 360 - 180
        out = [f"Moved {direction} for {seconds:.1f} s at speed {speed:.1f}: "
               f"travelled {dist:.0f} mm, turned {turn:+.0f} deg (odometry; + = left)."]
        if after["guard"]["blocked"]:
            blocked.add(after["guard"]["reason"])
        if blocked:
            out.append("GUARD BLOCKED part of this move: " + "; ".join(sorted(blocked)))
            out.append("Do not repeat this direction — turn or back away first.")
        if dist < 20 and abs(turn) < 3 and not blocked:
            out.append("Barely moved: the speed may be too low to overcome friction, or the wheels are off the ground.")
        out.append(self.status_text(after))
        return "\n".join(out)

    def stop(self):
        self.post("/drive", {"throttle": 0, "steer": 0})
        self.post("/stop")
        try:
            self.post("/goto", {"stop": True})
        except TruckError:
            pass
        return "Stopped."

    def emergency_stop(self):
        self.post("/estop")
        return "EMERGENCY STOP: drivers disabled. A person must press ENABLE to continue."

    def places_text(self):
        names = self.ai_status().get("places") or []
        routes = sorted((self.get("/routes").get("routes") or {}))
        out = ("Saved places: " + ", ".join(names) + ".") if names else \
            "No places saved. A person can save one from the cockpit's Map tab."
        if routes:
            out += " Routes: " + ", ".join(routes) + "."
        return out

    def go_to_place(self, name):
        """Drive to a saved place or map marker ("hall", "marker 2"). The
        cockpit matches loosely - "the hall", "kitchen" for "kitchen entrance"."""
        if not self.ai_status()["motors"]["enabled"]:
            return "Not started: motors are DISABLED. Ask the person to press ENABLE in the cockpit."
        try:
            r = self.post("/goto", {"name": name})
        except TruckError as e:
            if "unknown place" not in str(e):
                raise
            names = self.ai_status().get("places") or []
            return (f"I don't know a place called {name}. "
                    + (("I know: " + ", ".join(names) + ".") if names else "No places are saved yet."))
        # The first plan happens a moment later, in the explorer's thread.
        # Live, "On my way to kitchen entrance" was said for a trip that had
        # already failed with "no route" - so wait for the plan first.
        goal = r.get("goal") or name
        for _ in range(8):
            time.sleep(0.25)
            nav = self.ai_status().get("navigation") or {}
            if nav.get("state") == "failed":
                return f"I can't find a way to {goal} from here. {nav.get('message') or ''}".strip()
            if nav.get("state") == "done":
                return f"I'm already at {goal}."
            if nav.get("running") and self.get("/state").get("explore", {}).get("path_len"):
                break
        return f"On my way to {goal}."

    def drive_route(self, name, loop=False):
        """Drive a route drawn on the map, waypoint by waypoint."""
        if not self.ai_status()["motors"]["enabled"]:
            return "Not started: motors are DISABLED. Ask the person to press ENABLE in the cockpit."
        try:
            r = self.post("/route/go", {"name": name, "loop": bool(loop)})
        except TruckError as e:
            if "unknown route" not in str(e):
                raise
            routes = sorted((self.get("/routes").get("routes") or {}))
            return (f"I don't know a route called {name}. "
                    + (("Routes: " + ", ".join(routes) + ".") if routes else
                       "No routes yet - draw one on the cockpit's Map tab."))
        n = r.get("points") or 0
        return (f"Driving the {r.get('name')} route, {n} point{'s' if n != 1 else ''}"
                + (", round and round until you say stop." if loop else "."))

    def cancel_navigation(self):
        self.post("/goto", {"stop": True})
        self.post("/stop")
        return "Navigation cancelled and motors stopped."

    # --- sound --------------------------------------------------------------

    def beep(self, kind="horn"):
        if kind not in BEEP_KINDS:
            return f"unknown sound {kind!r}; use one of: {', '.join(BEEP_KINDS)}"
        self.post("/audio/beep", {"kind": kind})
        return f"Played {kind}."

    def songs(self):
        return [f["name"] for f in self.get("/audio/state").get("files", [])]

    def find_song(self, name):
        songs = self.songs()
        want = (name or "").lower().replace(" ", "_")
        return next((s for s in songs if want and want in s.lower()), None), songs

    def play_song(self, name=""):
        match, songs = self.find_song(name)
        if not name:
            return ("Songs: " + ", ".join(songs)) if songs else "No songs uploaded."
        if match is None:
            return f"No song matching {name!r}. Songs: {', '.join(songs) or 'none'}"
        self.post("/audio/play", {"file": match})
        return f"Playing {match}."

    def stop_music(self):
        self.post("/audio/stop_music")
        return "Music stopped."

    def music(self, action):
        """pause | resume | next | previous — the player's own buttons."""
        a = self.get("/audio/state")
        if action == "pause":
            if not a.get("playing"):
                return "No music is playing."
            self.post("/audio/pause")
            return "Music paused."
        if action == "resume":
            if a.get("playing"):
                return "Music is already playing."
            if not a.get("track"):
                return "Nothing is paused. Ask for a song by name."
            self.post("/audio/pause")                 # the toggle: paused -> playing
            return f"Resuming {a['track']}."
        if action in ("next", "previous"):
            r = self.post("/audio/skip", {"back": action == "previous"})
            return f"Playing {r.get('track') or 'the next song'}."
        return f"unknown music action {action!r}; use pause, resume, next or previous"

    def volume(self, change):
        """up | down | a percentage of full volume. Steps of a quarter, like
        a volume button, rather than a jump the listener did not expect."""
        a = self.get("/audio/state")
        top = float(a.get("max_volume") or 1.5)
        now = float(a.get("volume") or 1.0)
        if change == "up":
            new = now + 0.25 * top
        elif change == "down":
            new = now - 0.25 * top
        else:
            try:
                new = float(str(change).rstrip("%")) / 100.0 * top
            except ValueError:
                return f"unknown volume {change!r}; use up, down or a percentage"
        new = max(0.1 * top, min(top, new))
        self.post("/audio/levels", {"volume": round(new, 2)})
        return f"Volume {round(100 * new / top)} percent."

    # --- map ----------------------------------------------------------------

    def save_place(self, name):
        """Name the spot the truck is standing on, for go_to_place later."""
        name = (name or "").strip()
        if not name:
            return "A place needs a name."
        self.post("/places", {"name": name})
        return f"Saved this spot as {name}."

    def explore(self, action):
        """Start or stop autonomous mapping. Starting needs the motors already
        enabled by a person: /explore would arm them itself, and a voice
        command must never be what makes the truck start moving on its own."""
        if action == "stop":
            self.post("/explore", {"stop": True})
            self.post("/stop")
            return "Mapping stopped."
        if action != "start":
            return f"unknown mapping action {action!r}; use start or stop"
        s = self.ai_status()
        nav = s.get("navigation") or {}
        # Already mapping: say so, don't restart. Live, "okay sure, do the
        # mapping" said just after "make a new map" restarted the explorer
        # in the middle of its calibration spin.
        if nav.get("running") and str(nav.get("state", "")).startswith(("cal_", "explore")):
            return "I'm already mapping the house."
        if not s["motors"]["enabled"]:
            return "Not started: motors are DISABLED. Ask the person to press ENABLE in the cockpit."
        if not (s.get("lidar") or {}).get("connected", True):
            return "Not started: the LiDAR is not connected, so I cannot map."
        r = self.post("/explore", {"start": True, "calibrate": False})
        if not r.get("running"):
            return f"Mapping did not start: {r.get('message') or r.get('state')}"
        return "Mapping started. I'll ask you the room names as I go."

    def restore_settings(self):
        """Tuning back to the known-good set: code defaults plus the values
        measured on this truck (tuning_good.json)."""
        r = self.post("/tuning", {"good": "restore"})
        n = len(r.get("applied") or {})
        return f"Settings restored to the known-good set ({n} values)."

    def new_map(self):
        """Known-good settings, clear the map (rooms and objects too), then map
        the house from here. Mapping only starts if a person has already
        pressed ENABLE - otherwise it stops after clearing and says so."""
        r = self.post("/fresh_map")
        if r.get("started"):
            return ("Settings restored and old map cleared. Mapping the house now; "
                    "I'll ask you the room names as I go.")
        return "Settings restored and map cleared, but my motors are disabled. Please press ENABLE, then say start mapping."

    def follow(self, action):
        """Follow the nearest person in view ("start"), or stop following.
        Like mapping, starting needs the motors already enabled by a person."""
        if action == "stop":
            self.post("/follow", {"stop": True})
            return "Stopped following."
        if action != "start":
            return f"unknown follow action {action!r}; use start or stop"
        if not self.ai_status()["motors"]["enabled"]:
            return "Not started: motors are DISABLED. Ask the person to press ENABLE in the cockpit."
        r = self.post("/follow", {"start": True})
        if r.get("error"):
            return f"Not started: {r['error']}"
        return "Following you. Walk slowly; say stop when you want me to stop."

    # --- labels a person confirms ------------------------------------------

    def label_questions(self):
        """Detections waiting for a yes/no, oldest-asked first."""
        return self.get("/labels").get("pending", [])

    def label_questions_text(self):
        qs = self.label_questions()
        if not qs:
            return "Nothing waiting to be checked."
        return "Waiting for a yes or no: " + "; ".join(
            f"{q['key']}: a {q['label'].replace('_', ' ')}" + (f" near {q['room']}" if q.get("room") else "")
            + f" (seen {q['n']} times)" for q in qs[:10])

    def answer_label(self, key, answer, name=None):
        """answer: yes | no | skip. yes with a name relabels it."""
        self.post("/labels/answer", {"key": key, "answer": answer, "name": name})
        return {"yes": f"Labelled {name or 'it'} on the map." if name else "Confirmed; it is on the map now.",
                "no": "Removed; I will not ask about it there again.",
                "skip": "Skipped; I will ask later."}.get(answer, "Done.")

    def label_asked(self, key):
        self.post("/labels/asked", {"key": key})

    def save_map(self):
        # /map/save overwrites house_map.json, with no backup. Saving an empty
        # map (no LiDAR, or just restarted) would replace a real one with
        # nothing — it happened once while testing this tool.
        scans = (self.get("/state").get("slam") or {}).get("scans", 0)
        if not scans:
            return "Not saved: the map is empty, and saving would erase the map saved before."
        r = self.post("/map/save")
        return f"Map saved, {r.get('scans', scans)} scans."

    def take_photo(self):
        """Save the current camera view to test/captures/ on the Pi."""
        r = self.post("/camera/snap")
        return f"Photo saved as {r.get('file')}." if r.get("ok") else \
            f"Could not take a photo: {r.get('error') or 'no frame'}."

    def stop_sound(self):
        self.post("/tts/stop")
        self.post("/audio/stop")
        return "Sound stopped."

    def say(self, text, wait=True):
        self.post("/tts/say", {"text": text})
        if not wait:
            return "Speaking."
        t_end = time.monotonic() + 90
        time.sleep(0.3)
        while time.monotonic() < t_end:
            sp = self.ai_status()["speech"] or {}
            if sp.get("status") == "idle":
                return f"Said it. ({sp['error']})" if sp.get("error") else "Said it."
            time.sleep(0.25)
        return "Still speaking after 90 s; returning."

    def listen(self, timeout_seconds=30.0):
        timeout_seconds = max(1.0, min(120.0, float(timeout_seconds)))
        st = self.get("/voice/state")
        a = st.get("assistant") or {}
        if a.get("enabled") and a.get("status") not in ("off", "offline", "no_model", "error"):
            return ("The truck's on-board voice assistant is answering the phone microphone "
                    "right now, so there is nothing for this session to hear. To talk through "
                    "Claude Code instead, turn the assistant off on the cockpit's Audio tab.")
        r = self.get(f"/voice/listen?timeout={timeout_seconds:.0f}", timeout=timeout_seconds + 10)
        if r.get("heard"):
            return f'They said: "{r["text"]}"'
        if not self.get("/voice/state").get("phone_connected"):
            return ("Nothing heard — and no phone is connected. The person needs to open the "
                    "talk page (https://<pi-ip>:5443/talk) on their phone and tap the mic.")
        return f"Nothing heard in {timeout_seconds:.0f} s (phone is connected)."
