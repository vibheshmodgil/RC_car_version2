"""Car brain skeleton: connects everything, prints a 1 Hz status line.

This is where autonomy code will grow. The safety pattern is already in
place: telemetry watch + guaranteed stop on the way out. Sensors are
optional at this stage — whatever is missing is reported and skipped.
"""
import time

from esp32_link import Esp32Link


def try_open(name, factory):
    try:
        dev = factory()
        print(f"[ok]   {name}")
        return dev
    except Exception as e:
        print(f"[skip] {name}: {e}")
        return None


def main():
    link = Esp32Link()
    link.start_telemetry()

    imu = try_open("IMU (BNO055)", lambda: __import__("imu").Imu())
    cam = try_open("Camera stream", lambda: __import__("camera").Camera())
    gimbal = try_open("Gimbal", lambda: __import__("gimbal").Gimbal())
    # LiDAR is opened on demand by mapping code — spinning it constantly
    # wears the motor for nothing while we're only monitoring.

    print("Running. Ctrl+C to exit (car will be stopped).")
    try:
        while True:
            if link.telemetry_fresh():
                d = link.telemetry
                line = (f"esp32 up={d['up']}s estop={d['estop']} "
                        f"pwm={d['m']} rpm={[round(e['r']) for e in d['enc']]}")
            else:
                line = "esp32 TELEMETRY STALE"
            if imu:
                h = imu.reading.get("h") if imu.reading.get("ok") else None
                line += f" | heading={h}"
            print(line)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    except Exception:
        link.estop()      # anything unexpected: hard stop, then re-raise
        raise
    finally:
        link.stop()       # normal exit: coast the wheels
        if gimbal:
            gimbal.release()
        if cam:
            cam.release()
        print("stopped cleanly")


if __name__ == "__main__":
    main()
