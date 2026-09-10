"""
Raspberry Pi CSI camera driver — MJPEG frames off the hardware encoder.

Shared library, like pins.py and imu.py. Imported by camera_test.py and
web_nav.py; not meant to be run directly.

    sudo apt install -y python3-picamera2
    rpicam-hello --list-cameras        # the module should be listed

*** python3-picamera2 comes from APT, never pip. ***
`pip install picamera2` drags in libcamera, which does not build from source
on a Pi without a large toolchain and a long wait, and the result is not the
version Raspberry Pi OS tests against. The venv sees the apt package through
--system-site-packages, which is why CLAUDE.md insists on that flag. Same
rule as gpiozero and lgpio.

Why the hardware encoder and not capture_array()
------------------------------------------------
The obvious loop — grab a numpy array, JPEG-encode it in Python — costs
15-25 ms of CPU per frame on a Pi 4. At 15 fps that is a quarter of a core,
permanently, and this robot has none to spare: SLAM scan-matching already
takes ~96 ms every 200 ms, and the control loop runs at 50 Hz.

Picamera2's MJPEGEncoder hands the frame to the Pi 4's hardware JPEG block
instead. The ISP writes it, the encoder compresses it, and Python only ever
sees a finished JPEG arriving at a file-like object. That is the difference
between a camera you can leave running while driving and one you cannot.

The stream is deliberately small (640x480, 15 fps). This is a driving aid,
not a recording rig — the LiDAR does the measuring.

Model detection
---------------
The sensor identifies itself, so no camera model has to be assumed:

  ov5647   Camera Module 1
  imx219   Camera Module 2  /  NoIR v2
  imx477   HQ Camera
  imx296   Global Shutter Camera
  imx708   Camera Module 3  /  Module 3 Wide / NoIR

Mounting
--------
CSI is a ribbon cable to its own connector. It uses NO GPIO, so nothing in
the reserved-pin list changes and the camera cannot conflict with the
motors, encoders, I2S audio or the I2C IMU. Orientation is a software
setting — CAM_HFLIP / CAM_VFLIP in pins.py.
"""

import io
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Friendly names for the sensors Raspberry Pi ships. Anything not listed
# still works — it just gets reported by its raw sensor name.
MODELS = {
    "ov5647": "Camera Module 1",
    "imx219": "Camera Module 2",
    "imx477": "HQ Camera",
    "imx296": "Global Shutter Camera",
    "imx708": "Camera Module 3",
    "imx500": "AI Camera",
}

DEFAULT_SIZE = (640, 480)
DEFAULT_FPS = 15


class CameraError(RuntimeError):
    """Raised when the camera cannot be opened. Carries a message meant to be
    read by a human, not a traceback."""


def _friendly(model):
    """'/base/soc/i2c0mux/i2c@1/imx708@1a' or 'imx708' -> 'Camera Module 3'."""
    m = (model or "").lower()
    for key, name in MODELS.items():
        if key in m:
            return "%s (%s)" % (name, key)
    return model or "unknown sensor"


def detect():
    """List the attached CSI cameras.

    Returns [] when nothing is plugged in, and raises only when picamera2
    itself is missing — the two failures need completely different advice,
    so they must not collapse into one empty list.
    """
    try:
        from picamera2 import Picamera2
    except ImportError as e:
        raise CameraError(
            "picamera2 not installed — sudo apt install -y python3-picamera2 "
            "(and the venv needs --system-site-packages to see it)") from e
    return Picamera2.global_camera_info()


class _FrameSink(io.BufferedIOBase):
    """Where the hardware encoder puts finished frames.

    MJPEGEncoder calls write() once per frame with one complete JPEG, so
    there is no reassembly to do — the newest buffer is always a whole,
    displayable image. Only the newest is kept: a slow browser has to skip
    frames, never make the robot queue them up in RAM.
    """

    def __init__(self):
        self.frame = None
        self.count = 0
        self.stamp = 0.0
        self.cond = threading.Condition()

    def write(self, buf):
        with self.cond:
            self.frame = bytes(buf)
            self.count += 1
            self.stamp = time.monotonic()
            self.cond.notify_all()
        return len(buf)

    def wait(self, last_count, timeout=2.0):
        """Block until a frame newer than last_count arrives.

        Returns (jpeg, count), or (None, last_count) on timeout. The timeout
        is what ends the HTTP response when the camera stalls, instead of
        leaving the browser hanging on a connection that will never produce
        another byte.
        """
        with self.cond:
            if self.count <= last_count:
                self.cond.wait(timeout)
            if self.frame is None or self.count <= last_count:
                return None, last_count
            return self.frame, self.count


class Camera:
    """One CSI camera, streaming MJPEG in the background.

    Raises CameraError when there is no camera or picamera2 is missing.
    Callers that must survive a missing camera catch it — see CameraReader
    in web_nav.py, which reports the failure on the page rather than
    refusing to start the cockpit.
    """

    def __init__(self, size=DEFAULT_SIZE, fps=DEFAULT_FPS,
                 hflip=False, vflip=False, bitrate=None, index=0,
                 lores=None):
        from picamera2 import Picamera2

        cams = Picamera2.global_camera_info()
        if not cams:
            raise CameraError(
                "no CSI camera detected — check the ribbon is seated and the "
                "right way round, then run `rpicam-hello --list-cameras`")
        if index >= len(cams):
            raise CameraError("camera %d requested, only %d attached"
                              % (index, len(cams)))

        self.info = cams[index]
        self.name = _friendly(self.info.get("Model"))
        self.size = tuple(size)
        self.fps = fps
        self.sink = _FrameSink()
        self.error = ""
        self._closed = False
        self._cap_lock = threading.Lock()
        self._cap = None
        self._cap_t = 0.0

        self.picam = Picamera2(index)

        transform = None
        try:
            from libcamera import Transform
            transform = Transform(hflip=1 if hflip else 0,
                                  vflip=1 if vflip else 0)
        except ImportError:
            pass                      # no flip available; framing still works

        # FrameDurationLimits, not the "FrameRate" control: the former is
        # accepted by every picamera2 on Raspberry Pi OS, the latter is not,
        # and this file is written on Windows and only ever run on the Pi.
        # Microseconds per frame, min and max pinned to the same value.
        period = int(round(1_000_000 / max(1, fps)))
        kwargs = {}
        if transform is not None:
            kwargs["transform"] = transform

        # A SECOND stream, for the code that has to look at pixels rather
        # than ship them: marker detection and the cliff check.
        #
        # The ISP produces it in parallel with the one being encoded, so
        # analysis never has to decode a JPEG or interrupt the video. And
        # YUV420 means the first `h` rows ARE the greyscale image, so the
        # conversion every vision library starts with has already happened —
        # gray() is a slice, not a colourspace conversion.
        #
        # Same size as main. Smaller would be cheaper, but marker range is set
        # by how many pixels a tag covers, and halving the width halves the
        # distance a tag can be read from.
        self.lores_size = tuple(lores or self.size)
        cfg = self.picam.create_video_configuration(
            main={"size": self.size},
            lores={"size": self.lores_size, "format": "YUV420"},
            controls={"FrameDurationLimits": (period, period)},
            **kwargs)
        self.picam.configure(cfg)

        # MJPEGEncoder is the Pi 4's hardware JPEG block (V4L2). JpegEncoder
        # is software (simplejpeg, threaded) and is the fallback.
        #
        # The fallback is tried on ANY failure, not just ImportError, and
        # around start_recording rather than around the constructor: the
        # hardware encoder can also refuse the pixel format at start time,
        # and this file is authored on Windows where none of that can be
        # tested. Software encoding costs real CPU — self.encoder says which
        # one is running, and camera_test.py prints it, so a Pi that quietly
        # fell back is visible rather than just mysteriously slow.
        from picamera2.outputs import FileOutput
        self.encoder = "MJPEG (hardware)"
        try:
            from picamera2.encoders import MJPEGEncoder
            enc = MJPEGEncoder(bitrate) if bitrate else MJPEGEncoder()
            self.picam.start_recording(enc, FileOutput(self.sink))
        except Exception as e:                                # noqa: BLE001
            from picamera2.encoders import JpegEncoder
            self.encoder = "JPEG (software)"
            self.error = "hardware encoder unavailable (%s)" % e
            self.picam.start_recording(JpegEncoder(), FileOutput(self.sink))

        self._t0 = time.monotonic()

    # --- reading ----------------------------------------------------------

    def frame(self):
        """The newest JPEG, or None if no frame has arrived yet."""
        return self.sink.frame

    def frames(self, timeout=2.0):
        """Yield each new JPEG as it is encoded. One generator per client.

        Ends when the camera goes quiet for `timeout`, which closes the HTTP
        response cleanly instead of holding a Flask worker forever.
        """
        n = 0
        while not self._closed:
            jpeg, n = self.sink.wait(n, timeout)
            if jpeg is None:
                return
            yield jpeg

    # How long a captured lores frame may be reused, seconds.
    #
    # Two consumers run off this stream — marker detection at 4 Hz and the
    # cliff check at 5 Hz — from two threads. Without sharing they force two
    # separate captures of what is usually the very same frame, and each
    # capture blocks until the ISP produces one. 60 ms is under a frame
    # period at 15 fps, so nothing ever gets a stale picture; it just stops
    # near-simultaneous callers paying twice.
    LORES_TTL = 0.06

    def _lores(self):
        """The newest lores frame, captured at most once per LORES_TTL.

        The lock matters as much as the cache. picamera2 serialises requests
        internally, but two threads racing into capture_array is not a thing
        this code should rely on being safe — and it is one of the failures
        that would show up as an occasional hang on the robot rather than an
        error here.
        """
        with self._cap_lock:
            now = time.monotonic()
            if self._cap is None or now - self._cap_t > self.LORES_TTL:
                self._cap = self.picam.capture_array("lores")
                self._cap_t = now
            return self._cap

    def gray(self):
        """Greyscale frame as a numpy array, shape (h, w).

        This is the Y plane of the lores YUV420 stream, taken as a slice —
        no JPEG decode, no colour conversion, no copy of the chroma. Marker
        detection wants greyscale and nothing else, so this is the whole
        pipeline for it.
        """
        w, h = self.lores_size
        return self._lores()[:h, :w]

    def yuv_planes(self):
        """(Y, U, V) as numpy arrays. Y is (h, w); U and V are (h/2, w/2),
        which is what the 420 in YUV420 means and is ample for judging
        whether two patches of floor are the same colour.

        The cliff check needs chroma as well as brightness. A shadow is dark
        and still floor; a stair drop is dark AND a different hue. On luma
        alone those two are the same reading, and telling them apart is the
        entire job.

        Layout of the (h*3/2, w) buffer picamera2 returns for YUV420:

            rows 0        .. h        Y, full resolution
            rows h        .. h+h/4    U, h/2 x w/2 folded into w-wide rows
            rows h+h/4    .. h+h/2    V, likewise
        """
        w, h = self.lores_size
        a = self._lores()
        q = h // 4
        y = a[:h, :w]
        u = a[h:h + q, :w].reshape(h // 2, w // 2)
        v = a[h + q:h + 2 * q, :w].reshape(h // 2, w // 2)
        return y, u, v

    def still(self, path):
        """Write the newest frame to disk. Returns the path, or None if no
        frame has arrived yet.

        Deliberately the STREAM frame rather than a reconfigure-and-capture:
        a full-resolution still stops the video stream for around half a
        second, which on a driving robot means the operator's view freezes
        every time they take a photo.
        """
        jpeg = self.sink.frame
        if jpeg is None:
            return None
        with open(path, "wb") as f:
            f.write(jpeg)
        return path

    def hz(self):
        """Measured frame rate since start. Compare it against self.fps — a
        big gap means the encoder is being starved, usually by something
        else on the Pi eating the CPU."""
        dt = time.monotonic() - self._t0
        return self.sink.count / dt if dt > 0 else 0.0

    @property
    def live(self):
        """True while frames are actually arriving. A camera that opened and
        then stopped delivering looks identical to a working one otherwise."""
        return (self.sink.frame is not None
                and time.monotonic() - self.sink.stamp < 2.0)

    @property
    def state(self):
        return {
            "present": True, "name": self.name, "live": self.live,
            "size": list(self.size), "fps": self.fps,
            "encoder": self.encoder,
            "hz": round(self.hz(), 1), "frames": self.sink.count,
            "error": self.error,
        }

    def close(self):
        """Release the camera. Safe to call twice — the cockpit's finally
        block and an explicit close on the way out both land here."""
        if self._closed:
            return
        self._closed = True
        with self.sink.cond:
            self.sink.cond.notify_all()       # wake every blocked generator
        try:
            self.picam.stop_recording()
        except Exception:                                     # noqa: BLE001
            pass
        try:
            self.picam.close()
        except Exception:                                     # noqa: BLE001
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# Multipart MJPEG framing. It lives here so the two servers that stream
# (camera_test.py and web_nav.py) cannot drift apart on the boundary.
BOUNDARY = "frame"
MJPEG_MIME = "multipart/x-mixed-replace; boundary=" + BOUNDARY


def mjpeg_stream(cam):
    """Wrap a Camera's frames in the multipart encoding browsers expect."""
    for jpeg in cam.frames():
        yield (b"--" + BOUNDARY.encode() + b"\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
               + jpeg + b"\r\n")
