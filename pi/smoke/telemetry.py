"""Smoke test 2: subscribe to the WebSocket and print 10 telemetry frames."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from esp32_link import Esp32Link

link = Esp32Link()
link.start_telemetry()
print("Waiting for telemetry ...")
seen = 0
deadline = time.time() + 15
while seen < 10 and time.time() < deadline:
    if link.telemetry_fresh():
        d = link.telemetry
        print(f"up={d['up']}s estop={d['estop']} pwm={d['m']} "
              f"rpm={[round(e['r']) for e in d['enc']]}")
        seen += 1
        time.sleep(0.5)
    else:
        time.sleep(0.1)
print("OK" if seen else "FAILED — no telemetry. Is the Pi on the car AP?")
