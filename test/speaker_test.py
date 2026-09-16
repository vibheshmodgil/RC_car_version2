"""
Audio / amplifier test.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/speaker_test.py                 # list devices, then tone + sweep + beeps
    python test/speaker_test.py --list          # just show what ALSA sees
    python test/speaker_test.py --beep horn     # one beep: beep double horn chirp reverse alert
    python test/speaker_test.py --play song.mp3 # play a file (path, or a name in test/uploads/)
    python test/speaker_test.py -d plughw:1,0   # force a specific device
    python test/speaker_test.py --voices        # text-to-speech voices, installed or not
    python test/speaker_test.py --download en_US-lessac-medium
    python test/speaker_test.py --say "Hello, I am the speaker truck"

No Python audio libraries needed — the player (audio.py, shared with the web
pages) drives `aplay` for tones and `ffmpeg` for music:

    sudo apt install -y alsa-utils ffmpeg

The MAX98357A is picked automatically when ALSA lists it. HDMI is card 0 on
this Pi, so "first device" would be the wrong default.

Speech needs Piper in the venv (tts.py):  pip install "piper-tts>=1.3"

START WITH THE AMP GAIN LOW. A full-scale sine into a class-D amp at high
gain is loud enough to damage a small speaker, and your ears.
"""

import argparse
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import audio  # noqa: E402


def list_devices(player):
    devs = player.devices(fresh=True)
    print("=== ALSA playback devices ===")
    if not devs:
        print("  (none)\n")
        print("No sound card found. Check:")
        print("  - `dtoverlay=max98357a` and `dtparam=audio=off` in")
        print("    /boot/firmware/config.txt, then a reboot. WIRING.md section 7.")
        return
    for d in devs:
        mark = "  <- speaker" if d["dev"] == player.device else ""
        print(f"  {d['dev']:<14} {d['name']}{mark}")
    print(f"\n  ffmpeg: {'ok' if player.have_ffmpeg else 'MISSING — no MP3'}")
    print("  The MAX98357A has no mixer control — volume is software only.")


def speech(args):
    import time                                                # noqa: PLC0415
    import tts                                                 # noqa: PLC0415

    class Silent:                     # voice management needs no sound card
        _kind = None
    talker = tts.Tts(Silent(), warm=False)       # listing voices loads no model
    missing = 'MISSING — pip install "piper-tts>=1.3"'
    print("Piper:", "installed" if talker.have_piper else missing)
    if args.download:
        talker.download(args.download)
        state = talker.downloads.get(args.download) or {}
        print(f"  from {tts.voice_urls(args.download)[0]}")
        while args.download in talker.downloads and state.get("pct", 100) < 100:
            mb, tot = state.get("got", 0) / 1e6, state.get("total", 0) / 1e6
            size = f"{mb:5.1f} / {tot:.0f} MB" if tot else f"{mb:5.1f} MB"
            print(f"\r  {state.get('stage', '')}: {state['pct']:3d}%  {size}   ",
                  end="", flush=True)
            time.sleep(0.5)
        err = state.get("error")
        print("\n  " + ("FAILED: " + err if err else "done"))
    print("\n=== voices ===")
    for v in talker.voices():
        mark = "*" if v["id"] == talker.voice and v["installed"] else " "
        have = "installed " if v["installed"] else f"{v['mb'] or '?':>3} MB    "
        print(f" {mark} {v['id']:<36} {have} {v['label']}")
    talker.close()


def play_and_wait(fn, label):
    print(f"  {label}")
    fn()
    player.wait()


def main():
    global player
    ap = argparse.ArgumentParser(description="Speaker / amplifier test")
    ap.add_argument("-d", "--device", help="ALSA device, e.g. plughw:1,0")
    ap.add_argument("--list", action="store_true", help="list devices and exit")
    ap.add_argument("--freq", type=int, default=440, help="tone Hz")
    ap.add_argument("--level", type=float, default=0.25,
                    help=f"tone level 0-{audio.MAX_TONE_LEVEL}")
    ap.add_argument("--beep", choices=list(audio.BEEPS), help="play one beep and exit")
    ap.add_argument("--play", metavar="FILE", help="play an audio file and exit")
    ap.add_argument("--volume", type=float, default=0.8, help="music volume 0-1.5")
    ap.add_argument("--say", metavar="TEXT", action="append",
                    help="speak TEXT with a Piper voice; repeat to say several — "
                         "the second one shows the real, warm speed")
    ap.add_argument("--voice", help="voice id for --say (default: the one last chosen)")
    ap.add_argument("--voices", action="store_true", help="list speech voices")
    ap.add_argument("--download", metavar="VOICE", help="download a Piper voice")
    args = ap.parse_args()

    if args.voices or args.download:
        speech(args)
        return

    if shutil.which("aplay") is None:
        print("Missing: aplay.  Install with:  sudo apt install -y alsa-utils")
        sys.exit(1)

    player = audio.Audio()
    if args.device:
        player.device = args.device        # not saved: a one-off override
    list_devices(player)
    if args.list:
        return

    try:
        if args.say:
            import tts                                         # noqa: PLC0415
            import time                                        # noqa: PLC0415
            # warm=False: this process speaks straight away, so a separate
            # warm-up would only be cut off by the first sentence anyway.
            talker = tts.Tts(player, warm=False)
            print(f"  voice {args.voice or talker.voice}")
            print("  A fresh process loads the model first; web_nav.py does that once, at start-up.")
            for i, text in enumerate(args.say):
                print(f"\n  [{i + 1}] {text[:70]}")
                talker.say(text, voice=args.voice)
                # Report each stage as it happens: a silent 15 s of model
                # loading looks exactly like a hang.
                t0, last = time.monotonic(), None
                while talker.busy:
                    if talker.status != last:
                        last = talker.status
                        print(f"  {time.monotonic() - t0:5.1f} s  {last}")
                    time.sleep(0.05)
                print(f"  {time.monotonic() - t0:5.1f} s  done"
                      + (f" · first sound after {talker.first_sound_ms} ms" if talker.first_sound_ms else ""))
                if i == 0 and talker.startup:
                    st = talker.startup
                    print(f"         start-up: import piper {st['import_ms']} ms · "
                          f"load voice {st['load_ms']} ms · first inference {st['first_inference_ms']} ms")
                if talker.error:
                    print("\n  Speech failed: " + talker.error)
                    sys.exit(1)
            player.wait()
            return
        if args.beep:
            play_and_wait(lambda: player.beep(args.beep), f"beep: {args.beep}")
            return
        if args.play:
            path = os.path.abspath(args.play)
            if os.path.isfile(path):
                # The player only plays from test/uploads/, so a file given
                # by path is copied in — it then shows up in the web UI too.
                name = audio._safe_name(os.path.basename(path))
                dest = os.path.join(audio.AUDIO_DIR, name)
                if os.path.abspath(dest) != path:
                    shutil.copyfile(path, dest)
            else:
                name = args.play
            play_and_wait(lambda: player.play(name, volume=args.volume),
                          f"playing {name} — Ctrl-C to stop")
            return

        print("\n" + "=" * 58)
        print("  TURN THE AMP GAIN DOWN before continuing.")
        print("=" * 58)
        input("  Enter to start, Ctrl-C to abort: ")

        print(f"\n--- {args.freq} Hz tone, 3 s ---")
        play_and_wait(lambda: player.tone(args.freq, 3, args.level), "tone")

        # If low tones are inaudible but high ones are fine, that is the
        # speaker or its enclosure, not the Pi.
        print("\n--- frequency steps ---")
        for f in (110, 220, 440, 880, 1760, 3520):
            play_and_wait(lambda f=f: player.tone(f, 1.0, args.level), f"{f} Hz")

        print("\n--- log sweep 40 Hz - 15 kHz, 8 s ---")
        play_and_wait(lambda: player.sweep(40, 15000, 8, args.level), "sweep")

        print("\n--- beeps ---")
        for kind in audio.BEEPS:
            play_and_wait(lambda k=kind: player.beep(k), kind)

        print("\nAudio test complete.")
        print("Heard nothing? Check, in order: amp Vin at 5 V, SD pin > 1.4 V,")
        print("speaker wiring, then `-d` device choice, then config.txt overlay.")
    except RuntimeError as e:
        print(f"\n  Player failed: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        player.stop()


player = None

if __name__ == "__main__":
    main()
