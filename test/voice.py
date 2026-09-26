"""
Ears — a phone's microphone as the truck's microphone. Shared library.

Used by web_nav.py. Open  https://<pi-ip>:5443/talk  on a phone, tap the
mic, and speak: the phone turns speech into text and sends it here, where
tools/truck_mcp.py's `listen` tool picks it up. With `say` on the way back
out, that is a spoken conversation with Claude through the truck.

Why the phone does the speech recognition
-----------------------------------------
The browser's built-in SpeechRecognition (Chrome on Android, Safari on iOS)
is fast, handles Indian English and Hindi, and costs the Pi nothing. Running
Whisper on the Pi instead would take seconds per sentence on a CPU that is
already doing SLAM. The trade-off: on Android the audio goes to Google's
recognition service, so the phone needs internet.

Why HTTPS, and a second port
----------------------------
Browsers only allow the microphone on a "secure context": https, or
localhost. A phone on the LAN reaching http://192.168.1.11 is neither, so
the mic button would simply do nothing. web_nav.py therefore also serves the
same app over HTTPS on 5443, with a self-signed certificate made once with
openssl and kept in test/ (Pi-side, never synced). The phone warns about the
certificate the first time — "Advanced", "Proceed" — and then remembers.
The cockpit stays on plain http :5004, unchanged.

Not hearing itself
------------------
The phone sits near the speaker, so it would transcribe the truck's own
voice and Claude would answer itself. Two guards: the page pauses the mic
while the truck is speaking, and anything that arrives while speech is
playing, or within ECHO_TAIL_S after, is dropped here.
"""

import os
import subprocess
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
CERT_FILE = os.path.join(_HERE, "talk_cert.pem")
KEY_FILE = os.path.join(_HERE, "talk_key.pem")
HTTPS_PORT = 5443
ECHO_TAIL_S = 1.0          # speaker ring-down + recogniser latency
MAX_PENDING = 10
LOG_LEN = 30


_ip_cache = [0.0, None]


def lan_ip():
    """The Pi's address on the LAN, for links a PHONE will open. Not
    shiv.local: Chrome on Android generally cannot resolve .local names, so a
    link built from the hostname the PC used is a dead link on the phone.
    Cached — the page polls, and the answer changes only on a new DHCP lease."""
    import socket                                              # noqa: PLC0415
    if time.monotonic() - _ip_cache[0] < 30 and _ip_cache[1]:
        return _ip_cache[1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))                          # sends nothing
        ip = s.getsockname()[0]
    except OSError:
        ip = None
    finally:
        s.close()
    _ip_cache[:] = [time.monotonic(), ip]
    return ip


def talk_url(fallback_host=None):
    host = lan_ip() or fallback_host or "raspberrypi.local"
    return f"https://{host}:{HTTPS_PORT}/talk"


class Ears:
    """Utterances from the phone, waiting for someone to listen()."""

    def __init__(self, talker=None):
        self.talker = talker
        self._cv = threading.Condition()
        self._pending = []
        self.log = []                 # [{"who": "you"|"truck", "text", "t"}]
        self.dropped = 0
        self.phone_seen = 0.0
        self._speaking_until = 0.0
        self._last_said = ""
        self.assistant = None         # callable -> brain.Brain.brief, set by web_nav
        threading.Thread(target=self._watch_speech, daemon=True).start()

    # --- the truck's own voice ------------------------------------------------

    def _watch_speech(self):
        """Note when the truck is talking, and log what it said."""
        while True:
            t = self.talker
            if t is not None and t.status in ("synth", "speaking"):
                self._speaking_until = time.monotonic() + ECHO_TAIL_S
                if t.status == "speaking" and t.text and t.text != self._last_said:
                    self._last_said = t.text
                    self._add_log("truck", t.text)
            elif t is not None and t.status == "idle":
                self._last_said = ""
            time.sleep(0.1)

    @property
    def speaking(self):
        return time.monotonic() < self._speaking_until

    def _add_log(self, who, text):
        with self._cv:
            self.log.append({"who": who, "text": text, "t": round(time.time(), 1)})
            del self.log[:-LOG_LEN]

    # --- from the phone -----------------------------------------------------

    def heard(self, text, typed=False):
        """Returns False if it was dropped as the truck hearing itself.
        Typed text is never dropped — a keyboard cannot pick up the speaker."""
        text = " ".join((text or "").split())[:1000]
        self.phone_seen = time.monotonic()
        if not text:
            return False
        if self.speaking and not typed:
            self.dropped += 1
            return False
        with self._cv:
            self._pending.append(text)
            del self._pending[:-MAX_PENDING]
            self._cv.notify_all()
        self._add_log("you", text)
        return True

    # --- to Claude ----------------------------------------------------------

    def listen(self, timeout=30.0):
        """Everything said since the last listen, or wait up to timeout for
        something. Returns None on silence."""
        end = time.monotonic() + max(0.0, min(300.0, float(timeout)))
        with self._cv:
            while not self._pending:
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self._cv.wait(left)
            text = " ".join(self._pending)
            self._pending.clear()
        return text

    def putback(self, text):
        """Return words to the front of the queue — for a listener that took
        them just as it was switched off, so the next listener still hears."""
        with self._cv:
            self._pending.insert(0, text)
            self._cv.notify_all()

    @property
    def state(self):
        return {
            "speaking": self.speaking,
            "truck_text": self._last_said,
            "pending": len(self._pending),
            "phone_connected": time.monotonic() - self.phone_seen < 15,
            "dropped_echo": self.dropped,
            "talk_url": talk_url(),
            "assistant": self.assistant() if self.assistant else None,
            "log": self.log[-LOG_LEN:],
        }


# ---------------------------------------------------------------------------
# HTTPS
# ---------------------------------------------------------------------------

def ensure_cert():
    """Self-signed certificate, made once. openssl ships with Raspberry Pi
    OS; returns (cert, key) or None with the reason printed."""
    if os.path.isfile(CERT_FILE) and os.path.isfile(KEY_FILE):
        return CERT_FILE, KEY_FILE
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-keyout", KEY_FILE, "-out", CERT_FILE, "-days", "3650",
             "-subj", "/CN=speaker-truck"],
            check=True, capture_output=True, timeout=60)
        os.chmod(KEY_FILE, 0o600)
        return CERT_FILE, KEY_FILE
    except (OSError, subprocess.SubprocessError) as e:
        print(f"  https: could not make a certificate ({e}) — phone mic unavailable")
        return None


def serve_https(app, port=HTTPS_PORT):
    """The same Flask app over HTTPS, in a background thread. Returns the
    port, or None if it could not start."""
    pair = ensure_cert()
    if pair is None:
        return None
    try:
        from werkzeug.serving import make_server               # noqa: PLC0415
        try:                            # IPv6 + IPv4, as web_nav's http server
            srv = make_server("::", port, app, threaded=True, ssl_context=pair)
        except OSError:
            srv = make_server("0.0.0.0", port, app, threaded=True, ssl_context=pair)
    except (OSError, SystemExit) as e:
        print(f"  https: port {port} unavailable ({e})")
        return None
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return port


# ---------------------------------------------------------------------------
# Flask
# ---------------------------------------------------------------------------

def blueprint(ears):
    from flask import Blueprint, Response, jsonify, request    # noqa: PLC0415

    bp = Blueprint("voice", __name__)

    @bp.route("/talk")
    def talk_page():
        page = TALK_PAGE.replace("%%TALK_URL%%", talk_url(request.host.split(":")[0]))
        return Response(page, mimetype="text/html",
                        headers={"Cache-Control": "no-store"})

    @bp.route("/voice/heard", methods=["POST"])
    def voice_heard():
        d = request.get_json(force=True, silent=True) or {}
        kept = ears.heard(d.get("text"), typed=bool(d.get("typed")))
        return jsonify(ok=True, kept=kept, **ears.state)

    @bp.route("/voice/listen")
    def voice_listen():
        try:
            timeout = float(request.args.get("timeout", 30))
        except ValueError:
            timeout = 30.0
        text = ears.listen(timeout)
        return jsonify(text=text, heard=text is not None)

    @bp.route("/voice/state")
    def voice_state():
        if request.args.get("phone"):          # the talk page's own poll
            ears.phone_seen = time.monotonic()
        return jsonify(ears.state)

    return bp


# Served as-is (not through Jinja), so braces in the JS need no escaping.
TALK_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0d1117">
<title>Speaker Truck — Talk</title>
<style>
  :root{color-scheme:dark;--bg:#0d1117;--s1:#161b22;--s2:#1c2430;--bd:#2a323d;
        --t1:#fff;--t2:#a9b4c0;--t3:#6e7b8a;--you:#3987e5;--truck:#d670c0;
        --good:#3fb950;--warn:#d29922;--crit:#f85149}
  *{box-sizing:border-box;margin:0;padding:0}
  html,body{height:100%}
  body{background:var(--bg);color:var(--t1);font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
       display:flex;flex-direction:column;padding:max(14px,env(safe-area-inset-top)) 16px
       max(16px,env(safe-area-inset-bottom))}
  header{display:flex;align-items:center;gap:8px;margin-bottom:10px}
  .brand{font-size:.75rem;font-weight:700;letter-spacing:.14em;text-transform:uppercase}
  .brand span{color:var(--t3);font-weight:400}
  select{margin-left:auto;background:var(--s2);color:var(--t1);border:1px solid var(--bd);
         border-radius:7px;padding:6px 8px;font-size:.8rem}
  .banner{display:none;padding:10px 12px;border-radius:9px;font-size:.82rem;line-height:1.5;margin-bottom:10px;
          color:var(--warn);background:color-mix(in srgb,var(--warn) 12%,transparent);
          border:1px solid color-mix(in srgb,var(--warn) 40%,transparent)}
  .banner.show{display:block}
  .banner a{color:var(--t1)}
  #log{flex:1;overflow-y:auto;display:flex;flex-direction:column;gap:8px;padding:4px 0 12px}
  .msg{max-width:85%;padding:9px 12px;border-radius:14px;font-size:.95rem;line-height:1.4;word-wrap:break-word}
  .you{align-self:flex-end;background:color-mix(in srgb,var(--you) 28%,var(--s2));border-bottom-right-radius:4px}
  .truck{align-self:flex-start;background:color-mix(in srgb,var(--truck) 22%,var(--s2));border-bottom-left-radius:4px}
  .interim{align-self:flex-end;color:var(--t3);font-style:italic;font-size:.9rem}
  .empty{color:var(--t3);text-align:center;margin:auto;font-size:.9rem;line-height:1.6;max-width:28ch}
  #status{text-align:center;font-size:.8rem;color:var(--t2);min-height:1.3em;margin:6px 0 12px;line-height:1.45}
  #status.err{color:var(--warn);font-weight:600}
  #assist{text-align:center;font-size:.85rem;font-weight:650;min-height:1.3em;margin-top:8px;color:var(--truck)}
  #assist.warn{color:var(--warn);font-weight:600;font-size:.78rem;line-height:1.45}
  #assist.quiet{color:var(--t3);font-weight:500}
  #mic{align-self:center;width:112px;height:112px;border-radius:50%;border:none;cursor:pointer;
       background:var(--s2);color:var(--t2);box-shadow:0 0 0 2px var(--bd) inset;
       display:flex;align-items:center;justify-content:center;transition:background .15s,box-shadow .15s;
       -webkit-tap-highlight-color:transparent;touch-action:manipulation}
  #mic svg{width:44px;height:44px}
  #mic.on{background:color-mix(in srgb,var(--you) 30%,var(--s2));color:#fff;
          box-shadow:0 0 0 3px var(--you) inset,0 0 0 10px color-mix(in srgb,var(--you) 18%,transparent)}
  #mic.hearing{animation:pulse 1.1s ease-in-out infinite}
  #mic.paused{background:color-mix(in srgb,var(--truck) 25%,var(--s2));box-shadow:0 0 0 3px var(--truck) inset;color:#fff}
  @keyframes pulse{50%{box-shadow:0 0 0 3px var(--you) inset,0 0 0 18px color-mix(in srgb,var(--you) 10%,transparent)}}
  form{display:flex;gap:8px;margin-top:14px}
  input{flex:1;min-width:0;background:var(--s2);border:1px solid var(--bd);color:var(--t1);
        border-radius:9px;padding:10px 12px;font-size:1rem}
  button.send{background:var(--s2);color:var(--t1);border:1px solid var(--bd);border-radius:9px;
              padding:0 16px;font-weight:650}
  @media (prefers-reduced-motion: reduce){#mic.hearing{animation:none}}
</style>
</head>
<body>
<header>
  <span class="brand">Speaker Truck <span>/ talk</span></span>
  <select id="lang" aria-label="Language">
    <option value="en-IN">English (India)</option>
    <option value="en-US">English (US)</option>
    <option value="en-GB">English (UK)</option>
    <option value="hi-IN">हिन्दी</option>
  </select>
</header>
<div id="insecure" class="banner"></div>
<div id="unsupported" class="banner">This browser has no speech recognition. Use Chrome on Android or Safari on iPhone — or type below.</div>
<div id="log"><div class="empty" id="empty">Tap the mic and talk. What you say goes to Claude; the truck answers out loud.</div></div>
<div id="assist"></div>
<div id="status">Mic off</div>
<button id="mic" aria-label="Microphone">
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
    <rect x="9" y="2" width="6" height="12" rx="3"/><path d="M5 10a7 7 0 0 0 14 0"/><path d="M12 17v5"/></svg>
</button>
<form id="typed"><input id="text" placeholder="…or type to the truck" autocomplete="off"><button class="send">Send</button></form>

<script>
const $ = id => document.getElementById(id);
const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
// Built on the Pi from its LAN IP, not location.hostname: Android Chrome
// cannot resolve shiv.local, so a hostname-based link is dead on the phone.
const TALK_URL = '%%TALK_URL%%';

let rec = null, want = false, running = false, speaking = false, offline = false;
let interimEl = null, lastLog = '', interimText = '', gotFinal = false, restarts = 0;
let problem = '', notice = '', noticeTimer = null, lastSent = {text: '', t: 0};

// What each recognition error means, in words a person can act on. The old
// page only handled one of these, and even that message was overwritten a
// moment later by "Mic off" — so on a phone the button looked dead.
const ERRORS = {
  'not-allowed': 'Microphone blocked. Tap the icon left of the address → Permissions → Microphone → Allow, then tap the mic again.',
  'service-not-allowed': 'Speech recognition is blocked for this page. Chrome: ⋮ → Settings → Site settings → Microphone → allow this site.',
  'audio-capture': 'No microphone available — is another app (a call, a recorder) using it?',
  'network': 'Speech recognition needs internet on the phone. Retrying…',
  'language-not-supported': 'That language is not available here — pick another at the top.',
};

try { $('lang').value = localStorage.getItem('talk.lang') || 'en-IN'; } catch(e) {}
$('lang').onchange = () => {
  try { localStorage.setItem('talk.lang', $('lang').value); } catch(e) {}
  if(running) rec.abort();                      // restarts in the new language
};

if(!window.isSecureContext){
  const a = document.createElement('a');
  a.href = TALK_URL; a.textContent = TALK_URL;
  $('insecure').append('The microphone only works over https. Open ', a,
    ' — the phone warns about the certificate once: tap Advanced → Proceed.');
  $('insecure').classList.add('show');
}
if(!SR) $('unsupported').classList.add('show');

function setStatus(){
  const m = $('mic');
  m.classList.toggle('on', want && !speaking);
  m.classList.toggle('paused', want && speaking);
  m.classList.toggle('hearing', want && running && !speaking);
  const msg = offline ? 'Cannot reach the truck — is web_nav.py running?'
            : problem || notice
            || (!want ? 'Mic off — tap to talk' : speaking ? 'Truck is talking — mic paused'
                : running ? 'Listening…' : 'Starting mic…');
  $('status').textContent = msg;
  $('status').classList.toggle('err', !!(offline || problem));
}

function flash(msg){
  notice = msg; setStatus();
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => { notice = ''; setStatus(); }, 2500);
}

function send(text, typed){
  return fetch('/voice/heard', {method:'POST', headers:{'Content-Type':'application/json'},
                                body: JSON.stringify({text, typed: !!typed})})
    .then(r => r.json())
    .then(st => {
      // Dropped as the truck hearing itself: say so, rather than let the
      // words silently vanish.
      if(st && st.kept === false) flash('Ignored — the truck was talking. Say it again.');
      render(st);
    })
    .catch(() => flash('Could not reach the truck — not sent.'));
}

function deliver(t){
  t = (t || '').trim();
  if(!t) return;
  const key = t.toLowerCase(), now = Date.now();
  // Some Android builds report the same phrase twice in quick succession.
  if(key === lastSent.text && now - lastSent.t < 2500) return;
  lastSent = {text: key, t: now};
  send(t, false);
}

function start(){
  if(!SR || running || speaking || !want) return;
  rec = new SR();
  rec.lang = $('lang').value;
  // One phrase per session, restarted from onend. continuous=true is the
  // unreliable mode on Android Chrome: it repeats earlier words inside later
  // results and ends on its own after a pause anyway.
  rec.continuous = false;
  rec.interimResults = true;
  rec.maxAlternatives = 1;
  interimText = ''; gotFinal = false;
  rec.onstart = () => { running = true; restarts = 0;
                        if(problem !== ERRORS.network) problem = ''; setStatus(); };
  rec.onresult = e => {
    let interim = '';
    for(let i = e.resultIndex; i < e.results.length; i++){
      const r = e.results[i];
      if(r.isFinal){ gotFinal = true; deliver(r[0].transcript); }
      else interim += r[0].transcript;
    }
    interimText = interim;
    showInterim(interim);
  };
  rec.onerror = e => {
    if(!(e.error in ERRORS)) return;           // no-speech, aborted: normal
    problem = ERRORS[e.error];
    if(e.error !== 'network') want = false;     // needs the person to act first
    setStatus();
  };
  rec.onend = () => {
    running = false;
    // Some Android builds end a session without ever marking the phrase
    // final. Keep the words — unless we stopped because the truck began
    // talking, in which case they may be the truck's own voice.
    if(!gotFinal && interimText.trim() && want && !speaking) deliver(interimText);
    interimText = ''; showInterim('');
    if(problem === ERRORS.network && gotFinal) problem = '';
    setStatus();
    if(want && !speaking){
      restarts++;
      // Back off when sessions end instantly, instead of a tight restart loop.
      setTimeout(start, problem ? 2000 : Math.min(250 * restarts, 2000));
    }
  };
  try { rec.start(); }
  catch(err){ problem = 'Could not start the microphone: ' + err.message; want = false; setStatus(); }
}

$('mic').onclick = () => {
  if(!want){
    if(!window.isSecureContext){ problem = 'The mic needs the https address — tap the link in the yellow box.'; setStatus(); return; }
    if(!SR){ problem = 'This browser has no speech recognition — use Chrome on Android or Safari on iPhone.'; setStatus(); return; }
  }
  want = !want;
  problem = '';
  if(want) start();
  else if(rec){ try { rec.abort(); } catch(e) {} }
  setStatus();
};

$('typed').onsubmit = e => {
  e.preventDefault();
  const t = $('text').value.trim();
  if(t){ send(t, true); $('text').value = ''; }
};

function showInterim(t){
  if(!t){ if(interimEl){ interimEl.remove(); interimEl = null; } return; }
  if(!interimEl){ interimEl = document.createElement('div'); interimEl.className = 'msg interim'; $('log').append(interimEl); }
  interimEl.textContent = t;
  $('log').scrollTop = $('log').scrollHeight;
}

// What the truck's assistant is doing. Without this the page showed your
// words arriving and then nothing — indistinguishable from being ignored.
function renderAssistant(a){
  const el = $('assist');
  if(!a){ el.textContent = ''; return; }
  let text, cls = '';
  if(!a.enabled){ text = 'Assistant is off — turn it on in the cockpit (Audio tab).'; cls = 'warn'; }
  else if(a.status === 'offline'){ text = 'The brain PC is not reachable: ' + (a.error || ''); cls = 'warn'; }
  else if(a.status === 'no_model'){ text = 'Brain model missing: ' + (a.error || ''); cls = 'warn'; }
  else if(a.status === 'loading' || a.status === 'starting'){ text = 'Waking up the brain on the PC…'; cls = 'quiet'; }
  else if(a.status === 'error'){ text = 'Assistant problem: ' + (a.error || 'unknown'); cls = 'warn'; }
  else if(a.status === 'thinking'){ text = 'Thinking…'; }
  else if(a.status === 'speaking'){ text = 'Speaking…'; }
  else if(a.status && a.status.startsWith('using ')){
    const what = {look: 'Looking with the camera…', status: 'Checking surroundings…', drive: 'Driving…',
                  stop: 'Stopping…', horn: 'Horn!', play_song: 'Finding the song…', places: 'Checking places…',
                  go_to_place: 'Starting navigation…', stop_music: 'Stopping the music…'};
    text = what[a.status.slice(6)] || 'Working…';
  }
  else { text = 'Assistant ready — just talk'; cls = 'quiet'; }
  el.textContent = text;
  el.className = cls;
}

// Messages are built with textContent: what anyone says is data, not HTML.
function render(st){
  if(!st) return;
  if('assistant' in st) renderAssistant(st.assistant);
  if(st.speaking !== speaking){
    speaking = st.speaking;
    if(speaking && rec && running){ try { rec.abort(); } catch(e) {} }   // not the truck's voice
    if(!speaking && want) setTimeout(start, 150);
    setStatus();
  }
  if(!st.log) return;
  const sig = JSON.stringify(st.log);
  if(sig === lastLog) return;
  lastLog = sig;
  const box = $('log');
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  box.replaceChildren(...st.log.map(m => {
    const d = document.createElement('div');
    d.className = 'msg ' + m.who; d.textContent = m.text; return d;
  }));
  if(!st.log.length){ const e = document.createElement('div'); e.className = 'empty';
    e.textContent = 'Tap the mic and talk. What you say goes to Claude; the truck answers out loud.'; box.append(e); }
  if(interimEl) box.append(interimEl);
  if(atBottom) box.scrollTop = box.scrollHeight;
}

function poll(){
  fetch('/voice/state?phone=1')
    .then(r => r.json())
    .then(st => { if(offline){ offline = false; setStatus(); } render(st); })
    .catch(() => { if(!offline){ offline = true; setStatus(); } })
    .finally(() => setTimeout(poll, 400));
}
poll(); setStatus();
</script>
</body>
</html>"""
