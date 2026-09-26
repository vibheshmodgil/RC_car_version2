"""
The truck's voice assistant — a free, local model via Ollama on your PC. Shared library.

Runs inside web_nav.py on the Pi; the language model runs on the PC's GPU:

    phone mic (/talk) -> Ears -> Brain --HTTP--> Ollama on the PC (e.g. qwen3-vl:4b-instruct)
                                  |
                                  +-> Tts -> speaker        (sentence by sentence)
                                  +-> truck_api: look, status, drive, horn, music, places

No API keys, no credits, nothing leaves your network except the phone's own
speech recognition. The Pi does the listening, speaking and driving; the PC
only thinks. If the PC is off, the truck says so and everything else works.

Why the model is on the PC, not the Pi: a Pi 4 CPU runs even a 1B model at a
few words a second while SLAM needs a core. A GTX 1650 runs a 4B model
several times faster, and can also look at camera frames.

PC setup, once (full commands in Start_pi.md §5.12):
    ollama pull qwen3-vl:4b-instruct        vision + tools, fits a 4 GB GPU
    OLLAMA_HOST=0.0.0.0 (user env var), restart Ollama, allow port 11434

The reply is STREAMED: each sentence goes to the speaker as soon as it is
complete, while the model is still writing the next one.

Settings live in brain.json and on the cockpit's Audio tab; environment
variables give the first-run defaults:
  TRUCK_OLLAMA_URL   default http://192.168.1.12:11434   (the PC)
  TRUCK_MODEL        default qwen3-vl:4b-instruct
  TRUCK_NAME         what the truck calls itself, default "Speaker Truck"
"""

import base64
import json
import os
import re
import threading
import time
import unicodedata
import urllib.error
import urllib.request

_HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(_HERE, "brain.json")

DEFAULT_URL = os.environ.get("TRUCK_OLLAMA_URL", "http://192.168.1.12:11434")
DEFAULT_MODEL = os.environ.get("TRUCK_MODEL", "qwen3-vl:4b-instruct")
NAME = os.environ.get("TRUCK_NAME", "Speaker Truck")
MAX_TOOL_ROUNDS = 6
HISTORY_MESSAGES = 16          # older turns are dropped, whole exchanges at a time
FORGET_AFTER_S = 600           # a new conversation after 10 min of quiet
KEEP_ALIVE = "30m"             # keep the model in GPU memory between questions
FIRST_BYTE_TIMEOUT_S = 180     # a cold model load on a 4 GB GPU can take a while
HEALTH_EVERY_S = 10
# Context window. Explicit because the KV cache for it lives in GPU memory: on
# a 4 GB card a 4B model plus a large default context spills onto the CPU,
# and measured on this build that turns ~1 s answers into ~12 s ones.
NUM_CTX = int(os.environ.get("TRUCK_CTX", "4096"))
SILENT = "[silent]"

# When the chosen model is not installed, the first of these that is gets
# used instead — the truck keeps talking while a better model downloads.
# Best first: vision + tools and small enough for a 4 GB GPU, then text-only.
#
# "-instruct", not the plain tags. Measured on this build: plain qwen3-vl:2b
# is the THINKING variant and thinks before every answer even with
# think=false — 300 tokens of reasoning about a joke and no joke, and 78 s of
# silence in the assistant. Those plain tags come last, as a last resort.
PREFERRED_MODELS = ("qwen3-vl:4b-instruct", "qwen3-vl:2b-instruct", "llama3.2:3b", "qwen2.5:3b",
                    "llama3.1:8b", "mistral:7b", "qwen3-vl:4b", "qwen3-vl:2b")

# Said at once when the model calls a tool before saying anything — so the
# truck reacts in the time it takes to decide, not after a second slow pass.
ACKS = {"drive": "Okay, moving.", "look": "Let me look.", "status": "Let me check.",
        "go_to_place": "Okay, on my way.", "places": "Let me check.", "play_song": "Sure."}


def _quick_reply(name, result):
    """For simple actions the tool's own outcome is the whole answer; a second
    model pass would only rephrase it, at several seconds a sentence on a
    small GPU. Returns the sentence to speak, or None when the model needs
    to read the result (status, places, a song list...)."""
    r = result.lower()
    if "motors are disabled" in r or "disabled" in r and "enable" in r:
        return "My motors are disabled. Please press ENABLE in the cockpit."
    if name == "horn":
        return ""                      # the horn is the answer
    if name in ("stop", "stop_music"):
        return "Stopped."
    if name in SPOKEN_RESULTS:
        return "Sorry, that did not work." if r.startswith("error") else result
    if name == "drive":
        if "guard blocked" in r:
            return "Something is in the way, so I stopped."
        if "lidar is not connected" in r:
            return "I can't see obstacles right now, so I won't move."
        if r.startswith("error"):
            return "I couldn't move."
        return "Done."
    return None

# Plain commands answered without the model. Measured with qwen3-vl:2b: 12 of
# 16 requests called the right tool, and the misses were exactly these — "I'll
# turn the volume down for you" said, nothing done. These need no judgement,
# so they are matched here, run at once, and cost no model time at all.
# Anything with a name in it (a song, a place), driving, and looking still
# goes to the model. The WHOLE utterance must be the command, after dropping
# politeness, so "I love the horn in that song" is still just chat.
# Longest first: "a bit" has to go before "a" can strand the "bit".
_FILLER = re.compile(r"\b(a little bit|a little|a bit|for me|can you|could you|will you|would you|"
                     r"make it|please|hey|truck|speaker|ok|okay|now|just|the|a|some)\b")
_FAST = [
    (r"stop( (moving|driving|there|it|everything|right there))?", "stop", {}),
    (r"(sound|blow|honk|play)? ?(your )?(horn|honk)", "horn", {"kind": "horn"}),
    (r"beep", "horn", {"kind": "beep"}),
    (r"stop (music|song|playing|singing)", "stop_music", {}),
    (r"pause( (music|song))?", "music", {"action": "pause"}),
    (r"(resume|continue|unpause)( (music|song|playing))?", "music", {"action": "resume"}),
    (r"(next|skip)( (song|track|one))?|play next( (song|one))?|skip (this|song|track)( song)?", "music",
     {"action": "next"}),
    (r"(previous|last)( (song|track))?|play (previous|last)( (song|one))?|go back( (a|one))? song", "music",
     {"action": "previous"}),
    (r"(louder|volume up|turn (it|volume) up|increase volume|raise volume|turn up volume)", "volume",
     {"change": "up"}),
    (r"(quieter|softer|volume down|turn (it|volume) down|decrease volume|lower volume|turn down volume)",
     "volume", {"change": "down"}),
    (r"(what|which) (places|rooms)( do you know| are (there|saved)| have you saved)?|list places",
     "places", {}),
    (r"save (map|your map)", "save_map", {}),
    (r"(take|click|snap) (photo|picture|pic|snapshot)", "take_photo", {}),
]
_FAST = [(re.compile(p), name, args) for p, name, args in _FAST]


# --- questions the truck asks while mapping ---------------------------------
#
# "Is this a sofa?" when the detector has seen one three times, and "What room
# am I in?" when exploring somewhere unnamed. Asked only in a quiet moment,
# never over a conversation, and the next thing said is read as the answer.
ASK_GAP_S = 25          # at least this long between two questions
QUIET_S = 8             # and this long after the last exchange
ANSWER_WINDOW_S = 45    # an answer after this long is just conversation
ROOM_ASK_GAP_S = 90     # the room question is rarer: mapping moves slowly
ROOM_SKIP_MM = 2500     # "don't know" here means don't ask again near here
ROOM_FIX_S = 20         # "no, the hall" this soon after naming a room corrects it

_ROOM_WORDS = ("room", "kitchen", "bathroom", "washroom", "toilet", "hall", "hallway", "corridor",
               "lounge", "office", "study", "garage", "balcony", "porch", "attic", "basement",
               "laundry", "nursery", "pantry", "store", "entrance", "lobby", "terrace", "veranda")
_YES = re.compile(r"^(yes|yeah|yep|yup|ya|correct|right|that's right|thats right|it is|true|sure|"
                  r"of course|haan|han|ha|ji)\b")
_NO = re.compile(r"^(no|nope|nah|wrong|not really|incorrect|nahi|nahin|na)\b")
_SKIP = re.compile(r"\b(skip|later|not now|don't know|dont know|not sure|no idea|pata nahi|"
                   r"never ?mind|ask me later)\b")
_IT_IS = re.compile(r"(?:it'?s|it is|that'?s|that is|this is|thats)\s+(?:actually\s+|really\s+)?"
                    r"(?:an?\s+|the\s+|my\s+|our\s+)?(?P<name>[\w' -]{2,40}?)\s*$")
_ROOM_HERE = re.compile(r"^(?:this|here) is (?:the |my |our |a )?(?P<name>[\w' -]{2,30}?)\s*$|"
                        r"^(?:we are|we're|you are|you're|i am|i'm) in (?:the |my |our |a )?"
                        r"(?P<name2>[\w' -]{2,30}?)\s*$|"
                        r"^(?:call|name) (?:this room|this place|this|here) (?:as )?(?:the )?(?P<name3>[\w' -]{2,30}?)\s*$")


_SHORT_OK = {"hi", "hey", "yes", "no", "go", "ok", "stop", "hello", "haan", "nahi"}


def _noise(text):
    """One short word that is not a word anyone says to a robot alone."""
    s = _clean_answer(text)
    return len(s.split()) <= 1 and len(s) < 4 and s not in _SHORT_OK


def _clean_answer(text):
    s = re.sub(r"[^\w\s']", " ", text.lower().replace("’", "'"))
    return " ".join(s.split())


def _parse_label_answer(text):
    """"yes" / "no" / "no, it's a bed" / "skip" -> (answer, new_name or None),
    or None if this is not an answer at all."""
    s = _clean_answer(text)
    if not s:
        return None
    if _SKIP.search(s):
        return "skip", None
    m = _IT_IS.search(s)
    if m and not _YES.fullmatch(s):
        name = re.sub(r"^(a|an|the)\s+", "", m.group("name")).strip()
        # "that's not a television, don't save it" is a no, not a new name.
        if re.match(r"(not|no|nothing)\b", name) or re.search(r"\b(don't|dont|do not)\b", name):
            return "no", None
        if len(name.split()) > 4:
            name = ""                               # a sentence, not a name
        if name and name not in ("right", "correct", "it", "wrong", "true"):
            return "yes", name                     # "no, it's a bed" -> label it bed
    if _NO.match(s):
        return "no", None
    if _YES.match(s):
        return "yes", None
    return None


def _parse_room_answer(text):
    """"the kitchen" / "it's the living room" / "don't know" -> name, "" for
    a skip, None if it does not sound like an answer."""
    s = _clean_answer(text)
    if not s or _SKIP.search(s) or _NO.fullmatch(s) or _YES.fullmatch(s):
        return "" if s else None
    s = re.sub(r"^(?:it'?s|it is|this is|that'?s|you'?re in|you are in|we'?re in|we are in)\s+", "", s)
    s = re.sub(r"^(?:the|my|our|a)\s+", "", s).strip()
    return s if 0 < len(s.split()) <= 4 else None


def _room_name(text):
    """"this is the kitchen" / "we're in the bedroom" -> "kitchen" — only when
    the name sounds like a room, so "this is great" stays conversation."""
    m = _ROOM_HERE.match(_clean_answer(text))
    if not m:
        return None
    name = (m.group("name") or m.group("name2") or m.group("name3") or "").strip()
    if name and any(w in name.split() or name.endswith(w) for w in _ROOM_WORDS):
        return name
    return None


def _fast_command(text):
    """(tool, args) when the utterance is one of the plain commands above."""
    room = _room_name(text)
    if room:
        return "save_place", {"name": room}
    s = re.sub(r"[^\w\s]", " ", text.lower())
    s = " ".join(_FILLER.sub(" ", s).split())
    if not s or len(s.split()) > 5:
        return None
    for rx, name, args in _FAST:
        if rx.fullmatch(s):
            return name, dict(args)
    return None


TOOLS = [
    {"name": "status", "description": "Obstacles around you, whether motors are armed, pose.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "look", "description": "Take a camera photo of what is in front of you.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "drive", "description": "Make one short move. Only when asked to move.",
     "parameters": {"type": "object", "properties": {
         "direction": {"type": "string", "enum": ["forward", "backward", "turn_left", "turn_right",
                                                  "forward_left", "forward_right"]},
         "seconds": {"type": "number", "description": "0.2 to 3"}},
         "required": ["direction"]}},
    {"name": "stop", "description": "Stop moving now.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "horn", "description": "Sound the horn or a beep.",
     "parameters": {"type": "object", "properties": {
         "kind": {"type": "string", "enum": ["horn", "beep", "double", "chirp", "reverse", "alert"]}},
         "required": ["kind"]}},
    {"name": "play_song", "description": "Play a song by name; empty name lists songs.",
     "parameters": {"type": "object", "properties": {"name": {"type": "string"}}}},
    {"name": "stop_music", "description": "Stop the music.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "music", "description": "Pause, resume, or skip to the next or previous song.",
     "parameters": {"type": "object", "properties": {
         "action": {"type": "string", "enum": ["pause", "resume", "next", "previous"]}},
         "required": ["action"]}},
    {"name": "volume", "description": "Make the speaker louder or quieter.",
     "parameters": {"type": "object", "properties": {
         "change": {"type": "string", "description": "up, down, or a percentage like 50"}},
         "required": ["change"]}},
    {"name": "places", "description": "List saved places.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "save_place", "description": "Remember the spot you are on now under a name.",
     "parameters": {"type": "object", "properties": {"name": {"type": "string"}},
                    "required": ["name"]}},
    {"name": "go_to_place", "description": "Drive by yourself to a saved place.",
     "parameters": {"type": "object", "properties": {"name": {"type": "string"}},
                    "required": ["name"]}},
    {"name": "mapping", "description": "Start or stop exploring the room by yourself to build the map.",
     "parameters": {"type": "object", "properties": {
         "action": {"type": "string", "enum": ["start", "stop"]}},
         "required": ["action"]}},
    {"name": "follow", "description": "Follow the person in front of you (start), or stop following. Use when asked to follow or come along.",
     "parameters": {"type": "object", "properties": {
         "action": {"type": "string", "enum": ["start", "stop"]}},
         "required": ["action"]}},
    {"name": "save_map", "description": "Save the map built so far.",
     "parameters": {"type": "object", "properties": {}}},
    {"name": "take_photo", "description": "Save a camera picture for the person to see later.",
     "parameters": {"type": "object", "properties": {}}},
]

# Tools whose result text is already the spoken answer — see _quick_reply.
SPOKEN_RESULTS = ("music", "volume", "save_place", "mapping", "save_map", "take_photo", "follow")


def _system_prompt(voice_id, can_see):
    """Kept short and byte-identical from turn to turn: Ollama reuses the
    processed prompt when the start of the conversation is unchanged, which
    measured 0.2 s instead of 9 s for the same 872 tokens."""
    if voice_id.startswith("hi_"):
        language = "Speak Hindi, written in Devanagari."
    else:
        language = ("Reply in the language the person used. Your voice only reads Latin letters, "
                    "so if they speak Hindi, reply in Hinglish written in Latin letters.")
    sight = "" if can_see else " You cannot see images."
    return f"""You are {NAME}, a small robot truck with a speaker, talking out loud with people.
Your words are spoken aloud: reply in one or two short, friendly sentences, with no symbols, lists or emoji. {language}
You hear through speech recognition, so guess misheard words. If it is only noise, reply exactly {SILENT}.
Call a tool only when asked to act or asked something a tool answers; for chat, just talk.{sight}
If a tool says the motors are disabled, ask the person to press ENABLE. Never claim what a tool did not report."""


class BrainError(Exception):
    """kind: offline | no_model | http"""

    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


class Ollama:
    """Just enough of Ollama's HTTP API: /api/chat streaming, /api/tags, /api/show."""

    def __init__(self, url):
        self.url = url.rstrip("/")

    def _req(self, path, body=None, timeout=5.0):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data,
                                     method="POST" if data else "GET",
                                     headers={"Content-Type": "application/json"})
        try:
            return urllib.request.urlopen(req, timeout=timeout)
        except urllib.error.HTTPError as e:
            try:
                msg = json.loads(e.read()).get("error", "")
            except ValueError:
                msg = ""
            if e.code == 404:
                raise BrainError("no_model", msg or "model not found") from None
            raise BrainError("http", f"Ollama HTTP {e.code}: {msg}") from None
        except (urllib.error.URLError, OSError) as e:
            raise BrainError("offline", f"cannot reach Ollama at {self.url} ({getattr(e, 'reason', e)})") from None

    def models(self):
        with self._req("/api/tags", timeout=4.0) as r:
            return sorted(m["name"] for m in json.load(r).get("models", []))

    def capabilities(self, model):
        with self._req("/api/show", {"model": model}, timeout=10.0) as r:
            return set(json.load(r).get("capabilities") or [])

    def pull(self, model, progress):
        """Download a model onto the PC. progress(dict) gets Ollama's stream:
        {"status", "total", "completed"}. Resumes a half-finished download."""
        # The timeout is the stall detector: no progress line for 30 s raises,
        # and the caller resumes. Measured here: pulls from the Ollama
        # registry stop dead partway, twice, and never recover on their own.
        with self._req("/api/pull", {"model": model, "stream": True}, timeout=30.0) as r:
            try:
                for line in r:
                    if line.strip():
                        ev = json.loads(line)
                        if ev.get("error"):
                            raise BrainError("http", ev["error"])
                        progress(ev)
            except (OSError, ValueError) as e:
                raise BrainError("offline", f"download stalled ({e.__class__.__name__})") from None

    def load(self, model):
        """An empty generate request loads the model into memory."""
        with self._req("/api/generate", {"model": model, "keep_alive": KEEP_ALIVE,
                                         "options": {"num_ctx": NUM_CTX}},
                       timeout=FIRST_BYTE_TIMEOUT_S) as r:
            r.read()

    def chat_stream(self, model, messages, tools, max_tokens=None):
        """Yields ("text", str), ("tool_calls", list), ("done", dict)."""
        # Only the fields Ollama defines: the history carries bookkeeping
        # (which user message is a camera photo) that is ours, not the API's.
        messages = [{k: m[k] for k in ("role", "content", "images", "tool_calls", "tool_name") if k in m}
                    for m in messages]
        body = {"model": model, "messages": messages, "stream": True,
                "keep_alive": KEEP_ALIVE,
                "options": {"num_ctx": NUM_CTX, **({"num_predict": max_tokens} if max_tokens else {})},
                # Thinking first would mean seconds of silence before the
                # first spoken word; this is conversation, not a puzzle.
                "think": False}
        if tools:
            body["tools"] = [{"type": "function", "function": t} for t in tools]
        with self._req("/api/chat", body, timeout=FIRST_BYTE_TIMEOUT_S) as r:
            for line in r:
                if not line.strip():
                    continue
                ev = json.loads(line)
                if ev.get("error"):
                    raise BrainError("http", ev["error"])
                msg = ev.get("message") or {}
                if msg.get("content"):
                    yield "text", msg["content"]
                if msg.get("thinking"):
                    yield "thinking", msg["thinking"]
                if msg.get("tool_calls"):
                    yield "tool_calls", msg["tool_calls"]
                if ev.get("done"):
                    yield "done", ev


def _clean_for_speech(text):
    """Markdown or thinking tags the model slipped in would be read out."""
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    text = re.sub(r"[*_`#>]+", "", text)
    text = re.sub(r"^\s*[-•]\s+", "", text, flags=re.M)
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)
    # Emoji and other symbols: small models add them despite the prompt, and
    # Piper turns a smiley into 2.5 s of noise. So/Sk = symbols and skin
    # tones, Cs/Co = surrogates and private use; U+FE0F / U+200D are the
    # emoji joiners left behind.
    text = "".join(c for c in text
                   if unicodedata.category(c) not in ("So", "Sk", "Cs", "Co")
                   and c not in (chr(0xFE0F), chr(0x200D)))
    return " ".join(text.split())


class _SentenceSpeaker:
    """Feeds streamed text to the speaker a sentence at a time."""

    END = re.compile(r"[.!?।…](?=\s)|\n")

    def __init__(self, talker, on_first):
        self.talker = talker
        self.on_first = on_first
        self.buf = ""
        self.spoken = []

    def feed(self, piece):
        self.buf += piece
        s = self.buf.lstrip()
        # Hold back what might still become "[silent]", a <think> block, or a
        # tool call a small model wrote as JSON text instead of calling it.
        if (SILENT.startswith(s[:len(SILENT)]) and len(s) <= len(SILENT)) \
                or s.startswith("<think>") and "</think>" not in s \
                or s.startswith("{"):
            return
        while True:
            m = self.END.search(self.buf)
            if not m:
                return
            sentence, self.buf = self.buf[:m.end()], self.buf[m.end():]
            self._speak(sentence)

    def flush(self):
        rest, self.buf = self.buf, ""
        if rest.strip() != SILENT and not rest.lstrip().startswith("{"):
            self._speak(rest)

    def _speak(self, sentence):
        s = _clean_for_speech(sentence.replace(SILENT, ""))
        if not s:
            return
        if not self.spoken:
            self.on_first()
        self.spoken.append(s)
        try:
            self.talker.say(s, append=True, remember=False)
        except (RuntimeError, ValueError) as e:
            raise _SpeechUnavailable(str(e)) from None


class _SpeechUnavailable(Exception):
    pass


class Brain:
    def __init__(self, ears, talker, player, base_url):
        import truck_api                                        # noqa: PLC0415
        self.ears, self.talker, self.player = ears, talker, player
        self.api = truck_api.TruckApi(base_url)
        self.enabled = True
        self.ollama_url = DEFAULT_URL
        self.model = DEFAULT_MODEL
        self._load()
        self.ollama = Ollama(self.ollama_url)
        self.status = "starting"     # listening|thinking|speaking|using X|off|offline|no_model|error
        self.error = ""
        self.heard = ""
        self.reply = ""
        self.latency_ms = None       # heard -> first sentence handed to the voice
        self.turns = 0
        self.history = []
        self.models = []
        self.active = self.model      # what is really answering: self.model, or a fallback
        self.pull_state = None        # {"model", "status", "completed", "total", "error", "done"}
        self.can_see = False
        self.can_use_tools = True
        self.always_thinks = False    # ignores think=false: answers only after a long pause
        self._last_turn = 0.0
        self._last_health = 0.0
        self._loaded_model = None
        # The question the truck asked out loud and is waiting on:
        # {"kind": "label"|"room", "key", "label", "t", "pose"}, or None.
        self.question = None
        self._last_ask = 0.0
        self._last_room_ask = 0.0
        self._last_ask_check = 0.0
        self._room_skips = []          # poses where "don't know" was the answer
        self._room_said = None         # the room just named, for "no, the hall"
        threading.Thread(target=self._run, daemon=True).start()

    # --- settings -----------------------------------------------------------

    def _load(self):
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        self.enabled = bool(d.get("enabled", True))
        self.ollama_url = d.get("ollama_url") or self.ollama_url
        self.model = d.get("model") or self.model

    def _save(self):
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump({"enabled": self.enabled, "ollama_url": self.ollama_url,
                           "model": self.model}, f, indent=2)
        except OSError:
            pass

    def configure(self, enabled=None, ollama_url=None, model=None):
        if enabled is not None:
            self.enabled = bool(enabled)
        if ollama_url:
            url = ollama_url.strip().rstrip("/")
            if not url.startswith("http"):
                url = "http://" + url
            if ":" not in url.split("//", 1)[1]:
                url += ":11434"
            self.ollama_url, self.ollama = url, Ollama(url)
        if model:
            self.model = model.strip()
        if ollama_url or model:
            self.history, self._loaded_model, self._last_health = [], None, 0.0
        self._save()

    def pull(self, model):
        """Start downloading a model onto the PC, in the background."""
        model = (model or "").strip()
        if not model:
            raise ValueError("no model name")
        if self.pull_state and not self.pull_state.get("done"):
            raise RuntimeError(f"already downloading {self.pull_state['model']}")
        state = {"model": model, "status": "starting", "completed": 0, "total": 0,
                 "error": "", "done": False, "t0": time.time()}
        self.pull_state = state

        def progress(ev):
            state["status"] = ev.get("status", state["status"])
            if ev.get("total"):
                state["total"], state["completed"] = ev["total"], ev.get("completed", 0)

        def run():
            try:
                for attempt in range(1, 31):
                    try:
                        self.ollama.pull(model, progress)
                        state["status"] = "done"
                        self._last_health = 0.0          # see the new model at once
                        return
                    except BrainError as e:
                        if e.kind != "offline" or attempt == 30:
                            state["error"] = str(e)
                            return
                        # Ollama keeps the partial file, so this resumes.
                        state["status"] = f"stalled — resuming (attempt {attempt + 1})"
                        time.sleep(2)
            finally:
                state["done"] = True

        threading.Thread(target=run, daemon=True).start()

    # --- health -------------------------------------------------------------

    def _health(self):
        """Is the PC reachable, is the model installed, what can it do — and
        load it into GPU memory so the first question is not the slow one."""
        self._last_health = time.monotonic()
        try:
            self.models = self.ollama.models()
        except BrainError as e:
            self.status, self.error = "offline", (
                f"{e} — is the PC on, Ollama running, and OLLAMA_HOST set to 0.0.0.0?")
            self._loaded_model = None
            return False
        note = ""
        if _installed(self.model, self.models):
            self.active = self.model
        else:
            fallback = next((m for m in PREFERRED_MODELS if _installed(m, self.models)), None) \
                or next((m for m in self.models if "embed" not in m), None)
            if fallback is None:
                self.status, self.error = "no_model", (
                    f"no chat model on the PC — download {self.model} on the Assistant card")
                return False
            self.active = fallback
            note = f"{self.model} is not installed, so {fallback} is answering — download it on the Assistant card"
        if self._loaded_model != self.active:
            try:
                caps = self.ollama.capabilities(self.active)
                self.can_see = "vision" in caps
                # An Ollama too old to report capabilities says nothing, which
                # is not the same as "no tools": try them, and fall back if
                # the server refuses ("does not support tools", below).
                self.can_use_tools = "tools" in caps or not caps
                self.status = "loading"
                self.ollama.load(self.active)
                self._prime()
                self._loaded_model = self.active
            except BrainError as e:
                self.status, self.error = "offline" if e.kind == "offline" else "error", str(e)
                return False
        if self.status in ("starting", "offline", "no_model", "loading", "off"):
            self.status, self.error = "listening", ""
        if self.always_thinks:
            note = (note + " · " if note else "") + (
                f"{self.active} always thinks before answering, so replies come slowly — "
                f"use an -instruct model such as qwen3-vl:2b-instruct")
        if note:
            self.error = note
        elif not self.can_use_tools:
            self.error = f"{self.active} cannot use tools — it can chat but not drive or look"
        return True

    def _messages_head(self):
        """System prompt + tool list: identical on every request, so Ollama's
        prompt cache covers them after the first."""
        system = {"role": "system", "content": _system_prompt(self.talker.voice, self.can_see)}
        tools = TOOLS if self.can_use_tools else None
        if tools and not self.can_see:
            tools = [t for t in tools if t["name"] != "look"]
        return system, tools

    def _prime(self):
        """Process the system prompt and tools once, right after loading, so
        the first real question does not pay for reading them (9 s measured
        with a model half on the CPU). Also finds out whether the model
        ignores think=false: its reply then starts with reasoning."""
        system, tools = self._messages_head()
        self.always_thinks = False
        try:
            for kind, value in self.ollama.chat_stream(self.active, [system, {"role": "user", "content": "hello"}],
                                                       tools, max_tokens=8):
                if kind == "thinking" or kind == "text" and value.lstrip().startswith("<think>"):
                    self.always_thinks = True
        except BrainError:
            pass

    # --- the loop -----------------------------------------------------------

    def _run(self):
        while True:
            if not self.enabled:
                self.status = "off"
                time.sleep(0.5)
                continue
            healthy = self.status in ("listening", "error") and self._loaded_model == self.active
            if not healthy or time.monotonic() - self._last_health > HEALTH_EVERY_S * 6:
                if not self._health():
                    time.sleep(HEALTH_EVERY_S / 2)
                    continue
            text = self.ears.listen(timeout=1.0)
            if text is None:
                try:
                    self._maybe_ask()
                except Exception:                              # noqa: BLE001
                    pass                  # a question is never worth an error state
                continue
            if not self.enabled:
                # Switched off while waiting: these words belong to whoever
                # listens next (Claude Code's `listen`), not to us.
                self.ears.putback(text)
                continue
            try:
                self._respond(text)
            except Exception as e:                             # noqa: BLE001
                self.status, self.error = "error", f"{e.__class__.__name__}: {e}"
                time.sleep(1.0)

    # --- the truck's own questions -----------------------------------------

    def _maybe_ask(self):
        """In a quiet moment, while exploring somewhere unnamed, ask which
        room this is — the one thing the truck cannot work out alone, and
        what "go to the kitchen" needs later. Objects are NOT asked about:
        they are saved automatically once seen from three viewpoints, since
        a question per chair proved tedious. Only with the phone connected:
        a question nobody can answer is just the truck talking to itself."""
        now = time.monotonic()
        if (self.question and now - self.question["t"] < ANSWER_WINDOW_S) or self.talker.busy \
                or now - self._last_ask < ASK_GAP_S or now - self._last_turn < QUIET_S \
                or now - self._last_ask_check < 3.0 or self.status != "listening" \
                or not self.ears.state.get("phone_connected"):
            return
        self._last_ask_check = now
        self.question = None
        s = self.api.ai_status()
        # Driven by hand or mapping on its own, the same rule: once per
        # unnamed area. The spot is remembered when ASKED, not only when
        # skipped, so an unanswered question is not repeated every 90 s.
        pose = s.get("pose_mm_deg") or {}
        if not s.get("room_here") and now - self._last_room_ask > ROOM_ASK_GAP_S:
            x, y = pose.get("x", 0.0), pose.get("y", 0.0)
            if all((x - sx) ** 2 + (y - sy) ** 2 > ROOM_SKIP_MM ** 2 for sx, sy in self._room_skips):
                self._last_room_ask = now
                self._room_skips.append((x, y))
                self._ask("What room am I in?", {"kind": "room", "pose": (x, y)})

    def _ask(self, sentence, question):
        self.talker.say(sentence, append=True, remember=False)
        question["t"] = self._last_ask = time.monotonic()
        self.question = question
        self.reply = sentence

    def _answer_question(self, text):
        """The words right after a question. Returns the sentence to say if
        they answered it, or None if they are just conversation — the
        question then waits in the queue and the model gets the words."""
        q, self.question = self.question, None
        if q["kind"] == "label":
            parsed = _parse_label_answer(text)
            if parsed is None:
                return None
            answer, name = parsed
            self.api.answer_label(q["key"], answer, name)
            if answer == "yes":
                return f"Got it, a {name}. It's on the map now." if name else \
                    f"Great, the {q['label']} is on the map now."
            return "Okay, I won't label it." if answer == "no" else "Okay, I'll ask later."
        name = _parse_room_answer(text)
        if name is None:
            return None
        if not name:
            self._room_skips.append(q["pose"])
            return "Okay."
        self._name_room(name, q["pose"])
        return f"Got it, this is the {name}."

    def _name_room(self, name, pose):
        """Save a room where the question was ASKED — the truck may have
        driven on while the person answered — and remember it briefly so a
        mishearing can be corrected ("this is whole" for "hall")."""
        x, y = pose
        self.api.post("/places", {"name": name, "x": x, "y": y})
        self._room_said = {"name": name, "pose": pose, "t": time.monotonic()}

    def _fix_room(self, text):
        """"No" / "no, the hall" just after a room was named: the speech
        recogniser got it wrong. Returns what to say ("" if it asked again
        itself), or None if this is not a correction."""
        rs = self._room_said
        if not rs or time.monotonic() - rs["t"] > ROOM_FIX_S:
            return None
        s = _clean_answer(text)
        m = _NO.match(s) or re.match(r"^wrong", s)
        if not m:
            return None
        self._room_said = None
        self.api.post("/places", {"name": rs["name"], "delete": True})
        rest = re.sub(r"^(?:it'?s|it is|this is|that'?s|i said)\s+", "", s[m.end():].strip(" ,"))
        name = _parse_room_answer(rest) if rest else None
        if name:
            self._name_room(name, rs["pose"])
            return f"Sorry. This is the {name}."
        self._ask("Sorry, which room is this?", {"kind": "room", "pose": rs["pose"]})
        return ""                       # handled; _ask has already spoken"

    def _respond(self, text):
        try:
            fixed = self._fix_room(text)
        except Exception:                                      # noqa: BLE001
            fixed = None
        if fixed is not None:                    # "" = it re-asked, already spoken
            if fixed:
                self.heard, self.reply, self.error = text, fixed, ""
                self.talker.say(fixed, append=True, remember=False)
            self._last_turn = time.monotonic()
            return
        if self.question and time.monotonic() - self.question["t"] < ANSWER_WINDOW_S:
            try:
                said = self._answer_question(text)
            except Exception:                                  # noqa: BLE001
                said = None
            if said:
                self.heard, self.reply, self.error = text, said, ""
                self.talker.say(said, append=True, remember=False)
                self.turns += 1
                self._last_turn = time.monotonic()
                self.status = "listening"
                return
        self.question = None
        # A fragment is the recogniser catching noise ("se", "koi"), not a
        # question; answering it has the truck talk to nobody.
        if _noise(text):
            self.heard = text
            return
        t0 = time.monotonic()
        self.heard, self.reply, self.error, self.latency_ms = text, "", "", None
        self.status = "thinking"
        if time.monotonic() - self._last_turn > FORGET_AFTER_S:
            self.history = []
        self._trim()
        before = len(self.history)       # rollback point if this turn fails
        self.history.append({"role": "user", "content": text})

        # A song is paused, not merely held, while the truck answers: held
        # music comes back between two sentences if the model stops to call a
        # tool. Resumed at the end unless the conversation changed the music.
        music = self.player.track if self.player.music_playing else None
        if music:
            self.player.pause()
        deferred_song = None

        def first_sentence():
            self.latency_ms = round((time.monotonic() - t0) * 1000)
            self.status = "speaking"

        system, tools = self._messages_head()
        spoken = []
        fast = _fast_command(text) if tools else None
        try:
            for round_no in range(MAX_TOOL_ROUNDS):
                speaker = _SentenceSpeaker(self.talker, first_sentence)
                content, calls = "", []
                if round_no == 0 and fast:
                    # A plain command: the call the model should have made,
                    # without asking it — see _fast_command.
                    calls = [{"function": {"name": fast[0], "arguments": fast[1]}}]
                else:
                    for kind, value in self.ollama.chat_stream(self.active, [system] + self.history, tools):
                        if kind == "text":
                            content += value
                            speaker.feed(value)
                        elif kind == "tool_calls":
                            calls += value
                speaker.flush()
                spoken += speaker.spoken

                calls += _json_tool_calls(content) if not calls else []
                msg = {"role": "assistant", "content": content}
                if calls:
                    msg["tool_calls"] = calls
                self.history.append(msg)
                if not calls:
                    break

                photos, quick = [], []
                for call in calls:
                    fn = call.get("function") or {}
                    name, args = fn.get("name", ""), fn.get("arguments") or {}
                    if isinstance(args, str):
                        try:
                            args = json.loads(args)
                        except ValueError:
                            args = {}
                    self.status = f"using {name}"
                    if not spoken and name in ACKS and len(calls) == 1:
                        spoken.append(ACKS[name])
                        self.talker.say(ACKS[name], append=True, remember=False)
                        if self.latency_ms is None:
                            first_sentence()
                    result, photo, song = self._run_tool(name, args)
                    quick.append(_quick_reply(name, result))
                    deferred_song = song or deferred_song
                    if photo:
                        photos.append(photo)
                    self.history.append({"role": "tool", "content": result, "tool_name": name})
                if not photos and all(q is not None for q in quick):
                    # Every call was a simple action: speak its outcome and end
                    # the turn without asking the model to rephrase it. The
                    # assistant message keeps the history a normal exchange.
                    sentence = " ".join(q for q in quick if q)
                    if sentence and (not spoken or spoken[-1] != sentence):
                        self.talker.say(sentence, append=True, remember=False)
                        spoken.append(sentence)
                        if self.latency_ms is None:
                            first_sentence()
                    self.history.append({"role": "assistant", "content": sentence or "Done."})
                    break
                if photos:
                    # Images go on a user message: every vision model in
                    # Ollama reads those, not all read images on tool results.
                    self.history.append({"role": "user", "content": "(This is the photo from your camera, taken just now.)",
                                         "images": photos, "photo": True})
                self.status = "thinking"
            else:
                del self.history[before:]
                self.talker.say("That is taking me too many steps. Let's try something simpler.",
                                append=True, remember=False)
        except _SpeechUnavailable as e:
            self.error = f"cannot speak: {e}"
            del self.history[before:]
        except BrainError as e:
            del self.history[before:]
            if e.kind == "offline":
                self.error = f"{e} — is the PC on and Ollama running?"
                self._say_problem("My brain computer is not reachable right now.")
                self._loaded_model = None
            elif e.kind == "no_model":
                self.error = f"model {self.active} is not installed any more — download it on the Assistant card"
                self._say_problem("My brain model is not installed.")
                self._loaded_model = None
            elif "does not support tools" in str(e):
                self.can_use_tools = False
                self.error = f"{self.active} cannot use tools — ask again and it will just chat"
                self._say_problem("My model can't use my tools. Please ask again.")
            else:
                self.error = str(e)
                self._say_problem("Something went wrong. Please try again.")
        finally:
            # The photo has done its job once the reply that describes it is
            # written. Dropping it now, at the end of that turn, costs the
            # prompt cache once; dropping it turns later — as the trim used
            # to — meant re-reading the whole conversation then (measured
            # 13.8 s for a one-line joke two turns after a look).
            for m in self.history:
                if m.get("photo") and m.get("images"):
                    m["images"], m["content"] = [], "(camera photo, already described above)"
            self.reply = " ".join(spoken)
            self.turns += 1
            self._last_turn = time.monotonic()
            self._finish(music, deferred_song)

    def _finish(self, music, deferred_song):
        """After the reply has been spoken: the song the conversation asked
        for, or the one it interrupted."""
        if deferred_song or music:
            self.talker.wait(timeout=60)
        try:
            if deferred_song:
                self.api.post("/audio/play", {"file": deferred_song})
            elif music and self.player.track == music and self.player.paused:
                self.player.resume()
        except Exception as e:                                 # noqa: BLE001
            self.error = self.error or f"music: {e}"
        if self.status not in ("error", "offline", "no_model"):
            self.status = "listening"

    def _say_problem(self, sentence):
        try:
            self.talker.say(sentence, append=True, remember=False)
        except (RuntimeError, ValueError):
            pass
        self.status = "error"

    def _trim(self):
        """Keep the last HISTORY_MESSAGES, cutting only at a spoken user turn
        — never between a tool call and its result."""
        if len(self.history) <= HISTORY_MESSAGES:
            return
        for i in range(len(self.history) - HISTORY_MESSAGES, len(self.history)):
            m = self.history[i]
            if m["role"] == "user" and not m.get("photo"):
                self.history = self.history[i:]
                return

    # --- tools --------------------------------------------------------------

    def _run_tool(self, name, args):
        """Returns (text_result, base64_photo_or_None, song_to_play_after_speaking)."""
        import truck_api                                        # noqa: PLC0415
        try:
            if name == "status":
                return self.api.status_text(), None, None
            if name == "look":
                if not self.can_see:
                    return "This brain model cannot see images.", None, None
                jpeg = self.api.look_jpeg()
                return "Photo taken; it is in the next message.", base64.b64encode(jpeg).decode(), None
            if name == "drive":
                return self.api.drive(args.get("direction"), args.get("seconds", 1.0),
                                      args.get("speed", 0.6)), None, None
            if name == "stop":
                return self.api.stop(), None, None
            if name == "horn":
                return self.api.beep(args.get("kind", "horn")), None, None
            if name == "play_song":
                match, songs = self.api.find_song(args.get("name", ""))
                if not args.get("name"):
                    return ("Songs: " + ", ".join(songs)) if songs else "No songs uploaded.", None, None
                if match is None:
                    return f"No song matches. Songs: {', '.join(songs) or 'none'}", None, None
                # Playing now would cut off the truck's own voice mid-sentence.
                return f"{match} will start as soon as you finish speaking.", None, match
            if name == "stop_music":
                return self.api.stop_music(), None, None
            if name == "places":
                return self.api.places_text(), None, None
            if name == "go_to_place":
                return self.api.go_to_place(args.get("name", "")), None, None
            if name == "music":
                return self.api.music(args.get("action", "")), None, None
            if name == "volume":
                return self.api.volume(args.get("change", "")), None, None
            if name == "save_place":
                return self.api.save_place(args.get("name", "")), None, None
            if name == "mapping":
                return self.api.explore(args.get("action", "")), None, None
            if name == "follow":
                return self.api.follow(args.get("action", "")), None, None
            if name == "save_map":
                return self.api.save_map(), None, None
            if name == "take_photo":
                return self.api.take_photo(), None, None
            return f"unknown tool {name}", None, None
        except truck_api.TruckError as e:
            return f"Error: {e}", None, None

    # --- state --------------------------------------------------------------

    @property
    def brief(self):
        return {"enabled": self.enabled, "status": self.status, "error": self.error,
                "provider": "ollama", "ollama_url": self.ollama_url, "model": self.model,
                "active_model": self.active, "pull": self.pull_state, "always_thinks": self.always_thinks,
                "models": self.models, "can_see": self.can_see, "can_use_tools": self.can_use_tools,
                "heard": self.heard, "reply": self.reply,
                "latency_ms": self.latency_ms, "turns": self.turns,
                # What it asked out loud and is waiting on, if anything.
                "asking": ({"kind": self.question["kind"], "label": self.question.get("label")}
                           if self.question else None)}


def _installed(model, models):
    return model in models or f"{model}:latest" in models


def _json_tool_calls(content):
    """Small models sometimes write a tool call as JSON text instead of
    making one. Recognise the common shape so the move still happens —
    and it was never spoken, because the speaker holds back text starting
    with '{'."""
    s = content.strip()
    if not s.startswith("{"):
        return []
    try:
        d = json.loads(s)
    except ValueError:
        return []
    name = d.get("name") or (d.get("function") or {}).get("name")
    args = d.get("arguments") or d.get("parameters") or {}
    if name in {t["name"] for t in TOOLS}:
        return [{"function": {"name": name, "arguments": args}}]
    return []


def blueprint(brain):
    from flask import Blueprint, jsonify, request               # noqa: PLC0415

    bp = Blueprint("assistant", __name__)

    @bp.route("/assistant", methods=["GET", "POST"])
    def assistant():
        if request.method == "POST":
            d = request.get_json(force=True, silent=True) or {}
            brain.configure(d.get("enabled"), d.get("ollama_url"), d.get("model"))
            if d.get("forget"):
                brain.history = []
            if d.get("pull"):
                try:
                    brain.pull(d["pull"])
                except (ValueError, RuntimeError) as e:
                    return jsonify({**brain.brief, "error": str(e)}), 400
        return jsonify(brain.brief)

    return bp
