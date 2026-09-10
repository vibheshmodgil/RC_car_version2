"""
Audio / amplifier test.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/speaker_test.py             # list devices, then tone + sweep
    python test/speaker_test.py --list      # just show what ALSA sees
    python test/speaker_test.py -d hw:1,0   # force a specific device

No Python audio libraries needed — this drives the ALSA command-line tools
that ship with Raspberry Pi OS (`aplay`, `speaker-test`, `amixer`).

START WITH THE AMP VOLUME LOW. A full-scale sine into a class-D amp at high
gain is loud enough to damage a small speaker, and your ears.
"""

import argparse
import shutil
import subprocess
import sys

TOOLS = ("aplay", "speaker-test")


def run(cmd, **kw):
    """Run a command, return CompletedProcess. Never raises on non-zero."""
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def check_tools():
    missing = [t for t in TOOLS if shutil.which(t) is None]
    if missing:
        print(f"Missing: {', '.join(missing)}")
        print("Install with:  sudo apt install -y alsa-utils")
        sys.exit(1)


def list_devices():
    print("=== ALSA playback devices (aplay -l) ===")
    r = run(["aplay", "-l"])
    out = (r.stdout or r.stderr).strip()
    print(out if out else "  (none)")

    if "no soundcards" in out.lower() or not out:
        print("\nNo sound card found. Check:")
        print("  - Analog amp? The 3.5 mm jack needs `dtparam=audio=on`")
        print("    in /boot/firmware/config.txt")
        print("  - I2S amp?  Needs e.g. `dtoverlay=max98357a` in the same")
        print("    file, then a reboot. See WIRING.md section 6.")
        return []

    # Lines look like: card 0: Headphones [bcm2835 Headphones], device 0: ...
    devices = []
    for line in out.splitlines():
        if line.startswith("card "):
            try:
                card = line.split("card ")[1].split(":")[0].strip()
                dev = line.split("device ")[1].split(":")[0].strip()
                name = line.split("[")[1].split("]")[0]
                devices.append((f"hw:{card},{dev}", name))
            except (IndexError, ValueError):
                continue

    print("\n=== Usable device strings ===")
    for d, name in devices:
        print(f"  {d:<10} {name}")
    return devices


def show_mixer():
    print("\n=== Mixer ===")
    r = run(["amixer", "scontrols"])
    controls = (r.stdout or "").strip()
    print(controls if controls else "  (no controls — common on I2S amps)")
    if "PCM" in controls:
        print("\n  Set volume to 80%:  amixer set PCM 80%")
    elif "Master" in controls:
        print("\n  Set volume to 80%:  amixer set Master 80%")


def tone(device, freq, seconds):
    print(f"\n--- {freq} Hz sine, {seconds}s, on {device or 'default'} ---")
    cmd = ["speaker-test", "-t", "sine", "-f", str(freq), "-l", "1",
           "-P", "2", "-s", "1"]
    if device:
        cmd += ["-D", device]
    # speaker-test with -l 1 still runs until killed on some versions,
    # so bound it with a timeout rather than trusting it to exit.
    try:
        subprocess.run(cmd, timeout=seconds, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        pass  # expected — the timeout IS the duration control
    print("  done")


def sweep(device):
    """Step through the band a small speaker can actually reproduce.

    If low tones are inaudible but high ones are fine, that is the speaker
    or the amp's coupling caps, not the Pi.
    """
    print("\n--- frequency sweep ---")
    for f in (110, 220, 440, 880, 1760, 3520):
        print(f"  {f} Hz")
        tone(device, f, 1.2)


def noise_check(device):
    print("\n--- stereo channel check (front left / front right) ---")
    cmd = ["speaker-test", "-t", "wav", "-c", "2", "-l", "1"]
    if device:
        cmd += ["-D", device]
    r = run(cmd, timeout=20)
    if r.returncode != 0:
        # -t wav needs the sample files; fall back to pink noise.
        print("  wav samples unavailable, using pink noise")
        cmd = ["speaker-test", "-t", "pink", "-c", "2", "-l", "1"]
        if device:
            cmd += ["-D", device]
        try:
            subprocess.run(cmd, timeout=10, capture_output=True)
        except subprocess.TimeoutExpired:
            pass
    print("  done")


def main():
    ap = argparse.ArgumentParser(description="Speaker / amplifier test")
    ap.add_argument("-d", "--device", help="ALSA device, e.g. hw:0,0")
    ap.add_argument("--list", action="store_true", help="list devices and exit")
    ap.add_argument("--freq", type=int, default=440, help="tone Hz")
    args = ap.parse_args()

    check_tools()
    devices = list_devices()
    show_mixer()

    if args.list:
        return

    device = args.device
    if not device and devices:
        device = devices[0][0]
        print(f"\nUsing first device: {device}")
        print("Override with -d if that is the wrong one "
              "(HDMI is often card 0).")

    print("\n" + "=" * 58)
    print("  TURN THE AMP VOLUME DOWN before continuing.")
    print("=" * 58)
    if input("  Enter to start, Ctrl-C to abort: ") is None:
        return

    try:
        tone(device, args.freq, 3)
        sweep(device)
        noise_check(device)
        print("\nAudio test complete.")
        print("Heard nothing? Check, in order: amp power, amp gain, speaker")
        print("wiring, then `-d` device choice, then config.txt overlay.")
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
