"""
1.54" 240x240 IPS LCD, ST7789 controller, on SPI0. Shared library.

Used by web_nav.py (the status screen on the truck) and display_test.py.

    sudo apt install -y python3-spidev python3-pil python3-numpy

and in /boot/firmware/config.txt, then reboot:

    dtparam=spi=on
    dtoverlay=spi0-1cs

`spi0-1cs` is not optional. Plain `dtparam=spi=on` gives SPI0 TWO chip
selects and the kernel claims GPIO8 AND GPIO7 for them. GPIO7 is this
display's DC line, so without the overlay opening it fails with "GPIO busy".
The overlay keeps CE0 (GPIO8, the display's CS) and hands GPIO7 back.

Wiring — full table and reasoning in WIRING.md section 14:

    LCD   Pi header        LCD   Pi header
    GND   25 (GND)         RES   3V3 — tied to VCC at the display
    VCC   17 (3V3)         DC    26 (GPIO7)
    SCL   23 (GPIO11)      CS    24 (GPIO8, CE0)
    SDA   19 (GPIO10)      BLK   3V3 — tied to VCC at the display

RES and BLK are tied high because the header has no spare GPIO left that
does not already belong to something (servo, amp mute, ESP32 UART). The
controller is reset in software instead, which clears the same registers.
If a pin is freed later, set LCD_RST / LCD_BLK in pins.py and this driver
uses it — BLK then gets brightness control.

Every failure is reported in `.error` rather than raised from the status
thread: a dead display must never take the cockpit down with it.
"""

import os
import socket
import threading
import time

from pins import (
    LCD_SPI_BUS, LCD_SPI_DEV, LCD_DC, LCD_RST, LCD_BLK, LCD_SPI_HZ,
    LCD_SPI_MODE, LCD_SIZE, LCD_ROTATION, LCD_INVERT, LCD_BGR, LCD_FPS,
)

# ST7789 commands used here.
SWRESET, SLPIN, SLPOUT, NORON = 0x01, 0x10, 0x11, 0x13
INVOFF, INVON, DISPOFF, DISPON = 0x20, 0x21, 0x28, 0x29
CASET, RASET, RAMWR, MADCTL, COLMOD = 0x2A, 0x2B, 0x2C, 0x36, 0x3A

# Power, porch and gamma registers for the common 1.3"/1.54" IPS modules
# (the vendor reference sequence). The controller's reset defaults also
# light the panel, but with washed-out gamma and on some batches a flicker.
PANEL_INIT = (
    (0xB2, b"\x0C\x0C\x00\x33\x33"),        # porch
    (0xB7, b"\x35"),                         # gate voltages
    (0xBB, b"\x19"),                         # VCOM
    (0xC0, b"\x2C"),                         # LCM control
    (0xC2, b"\x01"),                         # VDV/VRH from command
    (0xC3, b"\x12"),                         # VRH
    (0xC4, b"\x20"),                         # VDV
    (0xC6, b"\x0F"),                         # 60 Hz frame rate
    (0xD0, b"\xA4\xA1"),                     # power control
    (0xE0, bytes([0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                  0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23])),
    (0xE1, bytes([0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                  0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23])),
)

# MADCTL bits per rotation, and where the 240x240 window sits inside the
# controller's 240x320 frame memory. At 180 and 270 the scan starts from the
# far end of the 320, so the window is 80 lines in — get this wrong and the
# picture is shifted with a band of noise along one edge.
MX, MY, MV, BGR = 0x40, 0x80, 0x20, 0x08
ROTATIONS = {0: (0x00, 0, 0), 90: (MX | MV, 0, 0),
             180: (MX | MY, 0, 80), 270: (MY | MV, 80, 0)}

FONT_DIRS = ("/usr/share/fonts/truetype/dejavu",)

# Colours, matching the cockpit page so the truck and the browser read alike.
BG = (13, 17, 23)
SURFACE = (28, 36, 48)
TEXT1 = (255, 255, 255)
TEXT2 = (169, 180, 192)
TEXT3 = (110, 123, 138)
GOOD = (63, 185, 80)
WARN = (210, 153, 34)
CRIT = (248, 81, 73)
AUDIO = (214, 112, 192)
POINT = (57, 135, 229)


def font(size, mono=False, bold=False):
    """DejaVu from fonts-dejavu-core (always on Raspberry Pi OS), else
    Pillow's built-in font, so a missing font file costs looks, not a crash."""
    from PIL import ImageFont                                  # noqa: PLC0415
    name = "DejaVuSans" + ("Mono" if mono else "") + ("-Bold" if bold else "") + ".ttf"
    for d in FONT_DIRS:
        try:
            return ImageFont.truetype(os.path.join(d, name), size)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:                     # Pillow < 10.1 has no size argument
        return ImageFont.load_default()


def lan_ip():
    """The address other devices reach the Pi on. Sends no packet."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class ST7789:
    """Raw panel access: commands, window, RGB565 frames from PIL images."""

    def __init__(self, rotation=LCD_ROTATION, hz=LCD_SPI_HZ, mode=LCD_SPI_MODE,
                 invert=LCD_INVERT, bgr=LCD_BGR, dc=LCD_DC, rst=LCD_RST, blk=LCD_BLK):
        import spidev                                          # noqa: PLC0415
        from gpiozero import DigitalOutputDevice, PWMOutputDevice  # noqa: PLC0415

        if rotation not in ROTATIONS:
            raise ValueError("rotation must be 0, 90, 180 or 270")
        self.width, self.height = LCD_SIZE
        self.rotation, self.invert, self.bgr = rotation, invert, bgr
        self._lock = threading.Lock()
        self.dc = self.rst = self.blk = self.spi = None
        try:
            # Safe state first, as every output in this project: backlight
            # dark and the controller held in reset until the SPI is up.
            if blk is not None:
                self.blk = PWMOutputDevice(blk, initial_value=0, frequency=1000)
            if rst is not None:
                self.rst = DigitalOutputDevice(rst, initial_value=False)
            self.dc = DigitalOutputDevice(dc, initial_value=False)

            self.spi = spidev.SpiDev()
            try:
                self.spi.open(LCD_SPI_BUS, LCD_SPI_DEV)
            except FileNotFoundError:
                raise RuntimeError(
                    f"/dev/spidev{LCD_SPI_BUS}.{LCD_SPI_DEV} missing — add "
                    "dtparam=spi=on and dtoverlay=spi0-1cs to "
                    "/boot/firmware/config.txt and reboot") from None
            self.spi.mode = mode
            # Pi 4 SPI divides 125 MHz by powers of two, so 32 MHz asks for
            # and gets 31.25. The next step up, 62.5, is past what most of
            # these modules' ribbon and wiring will carry cleanly.
            self.spi.max_speed_hz = hz
            self.init()
        except Exception:
            self.close()
            raise

    @property
    def actual_hz(self):
        return self.spi.max_speed_hz if self.spi else None

    # --- low level ----------------------------------------------------------

    def _cmd(self, c, data=b""):
        self.dc.off()
        self.spi.writebytes([c])
        if data:
            self.dc.on()
            self.spi.writebytes2(data)

    def reset(self):
        if self.rst is not None:
            self.rst.off()
            time.sleep(0.02)
            self.rst.on()
            time.sleep(0.15)
        self._cmd(SWRESET)
        time.sleep(0.15)             # datasheet: 120 ms before SLPOUT

    def init(self):
        with self._lock:
            self.reset()
            self._cmd(SLPOUT)
            time.sleep(0.12)         # datasheet: 120 ms after SLPOUT
            self._cmd(COLMOD, b"\x55")                    # 16-bit RGB565
            for c, d in PANEL_INIT:
                self._cmd(c, d)
            self._cmd(MADCTL, bytes([self._madctl()]))
            # IPS panels are built inverted. Without INVON every colour comes
            # out as its negative — white background, black text.
            self._cmd(INVON if self.invert else INVOFF)
            self._cmd(NORON)
            time.sleep(0.01)
            self._cmd(DISPON)
            time.sleep(0.05)
        self.backlight(1.0)

    def _madctl(self):
        return ROTATIONS[self.rotation][0] | (BGR if self.bgr else 0)

    def _window(self, x0, y0, x1, y1):
        _, ox, oy = ROTATIONS[self.rotation]
        x0, x1, y0, y1 = x0 + ox, x1 + ox, y0 + oy, y1 + oy
        self._cmd(CASET, bytes([x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF]))
        self._cmd(RASET, bytes([y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF]))
        self._cmd(RAMWR)

    # --- drawing ------------------------------------------------------------

    @staticmethod
    def rgb565(img):
        """PIL RGB image -> big-endian RGB565 bytes, vectorised. A Python
        loop over 57 600 pixels costs ~150 ms on a Pi 4; this costs ~3."""
        import numpy as np                                     # noqa: PLC0415
        a = np.asarray(img.convert("RGB"), dtype=np.uint16)
        px = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
        return px.astype(">u2").tobytes()

    def image(self, img):
        if img.size != (self.width, self.height):
            img = img.resize((self.width, self.height))
        self.frame(self.rgb565(img))

    def frame(self, buf):
        with self._lock:
            self._window(0, 0, self.width - 1, self.height - 1)
            self.dc.on()
            self.spi.writebytes2(buf)       # chunks past spidev's 4096 limit

    def fill(self, rgb):
        from PIL import Image                                  # noqa: PLC0415
        self.image(Image.new("RGB", (self.width, self.height), rgb))

    def set_rotation(self, rotation):
        if rotation not in ROTATIONS:
            raise ValueError("rotation must be 0, 90, 180 or 270")
        self.rotation = rotation
        with self._lock:
            self._cmd(MADCTL, bytes([self._madctl()]))

    def backlight(self, level):
        """0.0-1.0. Only does anything when BLK is on a GPIO (LCD_BLK)."""
        if self.blk is not None:
            self.blk.value = max(0.0, min(1.0, float(level)))

    def power(self, on):
        """Panel on or asleep. With BLK tied to 3V3 the backlight stays lit
        while asleep, so this is a black screen rather than a dark one."""
        with self._lock:
            if on:
                self._cmd(SLPOUT)
                time.sleep(0.12)
                self._cmd(DISPON)
            else:
                self._cmd(DISPOFF)
                self._cmd(SLPIN)
        self.backlight(1.0 if on else 0.0)

    def close(self, sleep=True):
        """Blank and sleep the panel, then release the pins and the bus."""
        if sleep and self.spi and self.dc:
            try:
                self.fill((0, 0, 0))
                self.power(False)
            except Exception:                                  # noqa: BLE001
                pass
        for dev in (self.blk, self.rst, self.dc):
            if dev is not None:
                try:
                    dev.close()
                except Exception:                              # noqa: BLE001
                    pass
        if self.spi:
            try:
                self.spi.close()
            except Exception:                                  # noqa: BLE001
                pass
        self.blk = self.rst = self.dc = self.spi = None


# ---------------------------------------------------------------------------
# Status screen
# ---------------------------------------------------------------------------

def _fit(g, text, f, width):
    """Shorten text with an ellipsis until it fits `width` pixels."""
    while g.textlength(text, font=f) > width and len(text) > 2:
        text = text.rstrip("…")[:-1] + "…"
    return text


def render_status(s, size=LCD_SIZE):
    """One status frame from a plain dict, so it can be rendered — and
    checked — without any hardware. Keys, all optional:

        ip, port, enabled, tripped, lidar_hz, lidar_ok, imu_ok, heading,
        cam_ok, blocked, reason, pose (x, y, deg), explore,
        song, paused, pos, dur, beep, volume
    """
    from PIL import Image, ImageDraw                           # noqa: PLC0415
    w, h = size
    img = Image.new("RGB", size, BG)
    g = ImageDraw.Draw(img)
    f_small, f_mid, f_tiny = font(13), font(14, bold=True), font(11, bold=True)
    f_ip, f_mono = font(21, mono=True, bold=True), font(14, mono=True)

    # --- header: name and drive state ---
    enabled, tripped = s.get("enabled"), s.get("tripped")
    state, colour = (("TRIPPED", WARN) if tripped else
                     ("ENABLED", GOOD) if enabled else ("DISABLED", CRIT))
    g.text((10, 8), "SPEAKER TRUCK", font=f_mid, fill=TEXT1)
    tw = g.textlength(state, font=f_tiny)
    g.rounded_rectangle((w - tw - 22, 8, w - 8, 25), radius=8, outline=colour)
    g.text((w - tw - 15, 10), state, font=f_tiny, fill=colour)

    # --- the address: the thing you actually need from this screen ---
    ip = s.get("ip") or "no network"
    g.rounded_rectangle((8, 34, w - 8, 84), radius=8, fill=SURFACE)
    g.text((16, 38), "open in a browser", font=f_small, fill=TEXT3)
    g.text((16, 56), ip, font=f_ip, fill=POINT if s.get("ip") else CRIT)
    if s.get("ip") and s.get("port"):
        g.text((16 + g.textlength(ip, font=f_ip), 60), f":{s['port']}",
               font=f_mono, fill=TEXT2)

    # --- sensors ---
    y = 94
    chips = (
        ("LIDAR", f"{s['lidar_hz']:.0f}Hz" if s.get("lidar_ok") else "off", s.get("lidar_ok")),
        ("IMU", f"{s['heading']:.0f}°" if s.get("heading") is not None else "off", s.get("imu_ok")),
        ("CAM", "on" if s.get("cam_ok") else "off", s.get("cam_ok")),
    )
    cw = (w - 16 - 8) // 3
    for i, (k, v, ok) in enumerate(chips):
        x = 8 + i * (cw + 4)
        g.rounded_rectangle((x, y, x + cw, y + 40), radius=6, fill=SURFACE)
        g.text((x + 7, y + 3), k, font=f_small, fill=TEXT3)
        g.text((x + 7, y + 19), v, font=f_mono, fill=TEXT1 if ok else CRIT)

    # --- guard or pose ---
    y = 142
    if s.get("blocked"):
        g.rounded_rectangle((8, y, w - 8, y + 26), radius=6, fill=(66, 44, 16))
        g.text((15, y + 5), _fit(g, "BLOCKED  " + (s.get("reason") or ""), f_small, w - 30),
               font=f_small, fill=WARN)
    else:
        p = s.get("pose")
        line = (f"x {p[0] / 1000:+.2f}  y {p[1] / 1000:+.2f} m  {p[2]:.0f}°"
                if p else "no pose")
        g.text((10, y + 5), line, font=f_mono, fill=TEXT2)
    if s.get("explore"):
        g.text((10, y + 26), _fit(g, s["explore"], f_small, w - 20), font=f_small, fill=GOOD)

    # --- audio ---
    y = 190
    song, beep = s.get("song"), s.get("beep")
    g.line((8, y - 4, w - 8, y - 4), fill=SURFACE)
    if beep:
        label, col = f"♪ {beep}", AUDIO
    elif song:
        label, col = ("❚❚ " if s.get("paused") else "♪ ") + song, AUDIO if not s.get("paused") else TEXT2
    else:
        label, col = "♪ nothing playing", TEXT3
    g.text((10, y), _fit(g, label, f_small, w - 20), font=f_small, fill=col)
    dur, pos = s.get("dur"), s.get("pos") or 0
    g.rounded_rectangle((10, y + 22, w - 10, y + 28), radius=3, fill=SURFACE)
    if song and dur:
        g.rounded_rectangle((10, y + 22, 10 + int((w - 20) * min(1.0, pos / dur)), y + 28),
                            radius=3, fill=AUDIO)
        t = f"{int(pos // 60)}:{int(pos % 60):02d} / {int(dur // 60)}:{int(dur % 60):02d}"
        g.text((10, y + 32), t, font=f_small, fill=TEXT3)
    if s.get("volume") is not None:
        v = f"vol {round(s['volume'] * 100)}%"
        g.text((w - 10 - g.textlength(v, font=f_small), y + 32), v, font=f_small, fill=TEXT3)
    return img


def render_test(size=LCD_SIZE, ip=None):
    """Colour bars plus orientation marks. Read it like this:
    TOP at the top = rotation right; bars in R G B order = colour order right
    (swapped red/blue = LCD_BGR); white background = LCD_INVERT wrong;
    a noise band along one edge = the rotation offset is wrong."""
    from PIL import Image, ImageDraw                           # noqa: PLC0415
    w, h = size
    img = Image.new("RGB", size, (0, 0, 0))
    g = ImageDraw.Draw(img)
    bars = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255))
    for i, c in enumerate(bars):
        g.rectangle((i * w // 4, 40, (i + 1) * w // 4 - 1, 110), fill=c)
    for i, name in enumerate("RGBW"):
        g.text((i * w // 4 + w // 8 - 5, 115), name, font=font(16, bold=True), fill=(255, 255, 255))
    g.rectangle((0, 0, w - 1, h - 1), outline=(255, 255, 0))
    g.polygon(((w // 2, 6), (w // 2 - 12, 30), (w // 2 + 12, 30)), fill=(255, 255, 0))
    g.text((w // 2 + 18, 8), "TOP", font=font(16, bold=True), fill=(255, 255, 0))
    for i in range(12):
        g.line((0, 145 + i * 3, w * (i + 1) // 12, 145 + i * 3), fill=(90 + i * 13,) * 3)
    g.text((10, 190), "ST7789 240x240", font=font(15, bold=True), fill=(255, 255, 255))
    g.text((10, 212), ip or lan_ip() or "no network", font=font(15, mono=True), fill=(57, 135, 229))
    return img


class StatusScreen:
    """Keeps the panel showing `source()`, a dict for render_status.

    Runs at LCD_FPS and only sends a frame when it differs from the last
    one: a 115 kB SPI transfer 2x a second is nothing, but rendering and
    sending an identical picture is CPU taken from SLAM for no reason.
    """

    def __init__(self, source, enabled=True):
        self.source = source
        self.lcd = None
        self.error = ""
        self.frames = 0
        self.ms = 0.0
        self.on = True
        self._last = None
        self._img = None
        self._test_until = 0.0
        self._lock = threading.Lock()
        if not enabled:
            self.error = "disabled (--no-display)"
            return
        try:
            self.lcd = ST7789()
        except Exception as e:                                 # noqa: BLE001
            self.error = self._explain(e)
            return
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def _explain(e):
        msg = str(e) or e.__class__.__name__
        if isinstance(e, ImportError):
            return f"{msg} — sudo apt install -y python3-spidev python3-pil python3-numpy"
        if "busy" in msg.lower():
            return (f"GPIO{LCD_DC} busy — the SPI driver owns it. Add "
                    "dtoverlay=spi0-1cs to /boot/firmware/config.txt and reboot")
        return msg

    def _loop(self):
        period = 1.0 / max(0.2, LCD_FPS)
        while self.lcd is not None:
            t0 = time.monotonic()
            try:
                if self.on:
                    if t0 < self._test_until:
                        img = render_test(ip=self.source().get("ip"))
                    else:
                        img = render_status(self.source())
                    buf = ST7789.rgb565(img)
                    if buf != self._last:
                        self.lcd.frame(buf)
                        self._last = buf
                        self.frames += 1
                    with self._lock:
                        self._img = img
                    self.ms = (time.monotonic() - t0) * 1000.0
                    self.error = ""
            except Exception as e:                             # noqa: BLE001
                self.error = self._explain(e)
                self._last = None       # resend in full once it recovers
            time.sleep(max(0.05, period - (time.monotonic() - t0)))

    def png(self):
        """What the panel is showing, as PNG bytes, for the web page. Rendered
        from the source even without a panel, so the layout can be checked
        before the display is wired."""
        import io                                              # noqa: PLC0415
        with self._lock:
            img = self._img
        if img is None:
            img = render_status(self.source())
        out = io.BytesIO()
        img.save(out, "PNG")
        return out.getvalue()

    def test(self, seconds=4.0):
        self._test_until = time.monotonic() + seconds

    def set_power(self, on):
        self.on = bool(on)
        if self.lcd:
            self.lcd.power(self.on)
            self._last = None

    @property
    def state(self):
        return {
            "present": self.lcd is not None,
            "on": self.on,
            "error": self.error,
            "frames": self.frames,
            "ms": round(self.ms, 1),
            "spi_hz": self.lcd.actual_hz if self.lcd else None,
            "fps": LCD_FPS,
            "rotation": self.lcd.rotation if self.lcd else LCD_ROTATION,
            "backlight": self.lcd is not None and self.lcd.blk is not None,
        }

    def close(self):
        lcd, self.lcd = self.lcd, None       # stops the loop
        if lcd:
            time.sleep(0.1)
            lcd.close()
