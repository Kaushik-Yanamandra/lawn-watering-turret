"""Hardware layer.

Real system (two units joined by RS-485):
  SensorUnit   on the Raspberry Pi in the Sensor Unit: Camera Module 3 Wide
               (picamera2) and the MLX90640 thermal camera (I2C) on a
               micro-servo pan stage.
  TurretLink   talks to the ESP32 Turret Node over a USB-RS485 adapter using
               the checksummed line protocol in firmware/turret_node.
  Rig          joins the two into the interface the controller expects.

SimTurret stands in for all of it on a desktop (Windows is fine): a hidden
"true" jet, a rendered lawn seen through a perspective camera, and a
synthetic thermal field. The controller can't tell the difference.

All expose: move(pan, tilt), valve(on), capture(), capture_thermal(), sleep(s).
The real Rig also has dose(litres), status(), pressure_psi() and clear_faults().
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .ballistics import JetModel


# =============================================================================
# Line protocol shared with the firmware
# =============================================================================
def checksum(text: str) -> int:
    c = 0
    for ch in text.encode("ascii"):
        c ^= ch
    return c


def frame(text: str) -> bytes:
    return f"{text}*{checksum(text):02X}\n".encode("ascii")


def parse(line: bytes | str) -> str | None:
    """Return the frame text if the checksum is good, else None."""
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="replace")
    line = line.strip()
    star = line.rfind("*")
    if star <= 0 or len(line) != star + 3:
        return None
    text = line[:star]
    try:
        return text if int(line[star + 1:], 16) == checksum(text) else None
    except ValueError:
        return None


@dataclass
class NodeStatus:
    pan: float
    tilt: float
    valve: bool
    lpm: float
    total_ml: float
    psi: float
    vin: float
    probe_mv: tuple
    faults: int
    uptime_s: int

    FAULT_NAMES = {1: "LINK", 2: "NO_FLOW", 4: "LOW_PSI", 8: "STUCK_OPEN", 16: "VALVE_TIMEOUT"}

    def fault_names(self):
        return [n for b, n in self.FAULT_NAMES.items() if self.faults & b]


class NodeError(RuntimeError):
    pass


# =============================================================================
# Real hardware
# =============================================================================
class TurretLink:
    """RS-485 link to the ESP32 Turret Node (firmware rev B).

    The node only ever speaks in reply to a command, so the Pi owns the bus.
    The node closes the valves if it hears nothing for 3 s, so every wait in
    this class keeps the link alive with a PING at least every 0.8 s, and no
    caller may block for long while a valve is open (see Controller).
    """

    RETRYABLE = ("ERR CHECKSUM",)

    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.05, ser=None):
        if ser is None:
            import serial
            ser = serial.Serial(port, baud, timeout=timeout, exclusive=True)   # no second process on the bus
        self.ser = ser
        self._buf = b""
        self._last_tx = 0.0
        self._dose_id = int(time.time()) % 1_000_000     # fresh ids after every Pi restart
        self.last_uptime = None

    def _read_line(self, deadline):
        """Next complete line, keeping any partial tail for the next call."""
        while True:
            if b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                return line
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(64)
            if chunk:
                self._buf += chunk

    def cmd(self, text: str, retries: int = 2, wait: float = 0.5) -> str:
        """Send one command, return the OK reply text. Retries on silence, a
        corrupted reply, or the node reporting a corrupted command; raises
        NodeError on any other ERR or after the retries."""
        last = "no reply"
        for _ in range(retries + 1):
            self._buf = b""
            self.ser.write(frame(text))
            self._last_tx = time.monotonic()
            deadline = time.monotonic() + wait
            while True:
                line = self._read_line(deadline)
                if line is None:
                    break
                body = parse(line)
                if body is None:
                    last = "corrupted reply"
                    continue
                if body.startswith(self.RETRYABLE):
                    last = body
                    break
                if body.startswith("ERR"):
                    raise NodeError(f"{text!r} -> {body}")
                if body.startswith("OK"):
                    return body
        raise NodeError(f"{text!r}: {last} after {retries + 1} tries")

    def ping(self):
        return self.cmd("PING")

    def move(self, pan, tilt, settle=0.35):
        eta_ms = int(self.cmd(f"MOVE {pan:.1f} {tilt:.1f}").split()[-1])
        self.sleep(eta_ms / 1000 + settle)

    def valve(self, on: bool, max_s: float = 30.0):
        self.cmd(f"VALVE 1 {int(max_s * 1000)}" if on else "VALVE 0")

    def dose_start(self, litres: float, max_s: float) -> int:
        """Start a metered dose; returns its id. The node closes the valves
        itself when the flow meter has counted `litres` (or on a fault)."""
        self._dose_id += 1
        did = self._dose_id
        self.cmd(f"DOSE {did} {litres * 1000:.0f} {int(max_s * 1000)}")
        return did

    def dose_poll(self, did: int):
        """(running, ml_so_far, ms, reason) for dose `did`."""
        _, _, sid, state, ml, ms, reason = self.cmd("DOSE?").split()
        if int(sid) != did:
            raise NodeError(f"dose id mismatch: sent {did}, node reports {sid} (node rebooted?)")
        return state == "RUN", float(ml), int(ms), reason

    def dose(self, litres: float, max_s: float):
        """Blocking dose: start it and poll until the node reports the end.
        Returns (ml, ms, reason)."""
        did = self.dose_start(litres, max_s)
        deadline = time.monotonic() + max_s + 3
        while time.monotonic() < deadline:
            self.sleep(0.4)
            running, ml, ms, reason = self.dose_poll(did)
            if not running:
                return ml, ms, reason
        self.cmd("VALVE 0")
        raise NodeError("dose did not finish")

    def status(self) -> NodeStatus:
        f = self.cmd("STAT").split()[2:]
        st = NodeStatus(float(f[0]), float(f[1]), f[2] == "1", float(f[3]), float(f[4]), float(f[5]),
                        float(f[6]), tuple(int(x) for x in f[7:10]), int(f[10]), int(f[11]))
        if self.last_uptime is not None and st.uptime_s < self.last_uptime:
            self.last_uptime = st.uptime_s
            raise NodeError("Turret Node rebooted during the session")
        self.last_uptime = st.uptime_s
        return st

    def clear_faults(self, stuck: bool = False):
        self.cmd("CLR STUCK" if stuck else "CLR")

    def sleep(self, s: float):
        """Wait while keeping the node's link watchdog fed (a PING at least
        every 0.8 s)."""
        end = time.monotonic() + s
        while True:
            now = time.monotonic()
            if now - self._last_tx >= 0.8:
                self.cmd("PING")
                now = time.monotonic()
            if now >= end:
                return
            time.sleep(min(end - now, 0.2))

    def close(self):
        try:
            self.cmd("PARK")
        finally:
            self.ser.close()


class SensorUnit:
    """Cameras on the Sensor Unit Pi. The MLX90640 (55 deg version) sits on an
    MG90S micro servo on GPIO18 that pans it between fixed stops."""

    def __init__(self, size=(2304, 1296), pan_gpio=18):
        from picamera2 import Picamera2
        import board
        import busio
        import adafruit_mlx90640

        self.cam = Picamera2()
        self.cam.configure(self.cam.create_still_configuration(main={"size": size}))
        self.cam.start()
        time.sleep(2.0)
        # Lock exposure and white balance so frames can be differenced.
        md = self.cam.capture_metadata()
        self.cam.set_controls({"AeEnable": False, "AwbEnable": False,
                               "ExposureTime": md["ExposureTime"],
                               "AnalogueGain": md["AnalogueGain"],
                               "ColourGains": md["ColourGains"]})
        i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
        self.mlx = adafruit_mlx90640.MLX90640(i2c)
        self.mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ
        from gpiozero import AngularServo
        self.pan = AngularServo(pan_gpio, min_angle=-90, max_angle=90,
                                min_pulse_width=0.0005, max_pulse_width=0.0025)

    def capture(self):
        return self.cam.capture_array()[:, :, :3]

    def capture_thermal_stops(self, stops, settle=1.5):
        """{stop: averaged frame} with the thermal camera panned to each stop."""
        out = {}
        for a in stops:
            self.pan.angle = float(a)
            time.sleep(settle)                # servo move plus two sensor refreshes
            out[a] = self.capture_thermal()
        self.pan.angle = 0.0
        time.sleep(0.5)
        self.pan.detach()                     # stop holding torque (and buzzing) between scans
        return out

    def capture_thermal(self, frames: int = 4):
        """Average a few frames: the MLX90640 is noisy (about 0.5 deg C per frame)."""
        acc, got = np.zeros((24, 32)), 0
        buf = [0.0] * 768
        for _ in range(frames * 3):
            try:
                self.mlx.getFrame(buf)
            except ValueError:            # dropped frame, retry
                time.sleep(0.3)
                continue
            acc += np.array(buf).reshape(24, 32)
            got += 1
            if got == frames:
                return acc / got
        raise RuntimeError("thermal camera read failed")

    def close(self):
        self.cam.stop()


class Rig:
    """Sensor Unit + Turret Node, presented as one device to the controller."""

    def __init__(self, sensor: SensorUnit, link: TurretLink):
        self.sensor, self.link = sensor, link

    def move(self, pan, tilt):
        self.link.move(pan, tilt)

    def valve(self, on):
        self.link.valve(on)

    def dose(self, litres, max_s):
        return self.link.dose(litres, max_s)

    def dose_start(self, litres, max_s):
        return self.link.dose_start(litres, max_s)

    def dose_poll(self, did):
        return self.link.dose_poll(did)

    def capture(self):
        return self.sensor.capture()

    def capture_thermal(self):
        return self.sensor.capture_thermal()

    def capture_thermal_stops(self, stops):
        return self.sensor.capture_thermal_stops(stops)

    def sleep(self, s):
        self.link.sleep(s)

    def status(self):
        return self.link.status()

    def pressure_psi(self):
        return self.link.status().psi

    def clear_faults(self, stuck=False):
        self.link.clear_faults(stuck)

    def close(self):
        self.link.close()
        self.sensor.close()


# =============================================================================
# Simulator
# =============================================================================
@dataclass
class SimWorld:
    true_jet: JetModel                          # hidden truth
    gust_std: float = 0.0                       # m/s, random gust per shot
    splash_std: float = 0.05                    # m, scatter of the impact point
    angle_std: float = 0.25                     # deg, servo deadband + residual backlash, per move and axis
    v0_std: float = 0.006                       # fraction, shot-to-shot exit speed (about 0.25 psi)
    flow_lpm: float = 7.0
    # ground canvas: x in [x0, x1], y in [y0, y1] at ppm pixels per metre
    x0: float = -2.0
    x1: float = 14.0
    y0: float = -12.0
    y1: float = 12.0
    ppm: int = 40
    # camera: 4 ground points and where they appear in a 1280x720 image
    # (the real Camera Module 3 is 2304x1296; the simulator stays coarser on purpose)
    cam_ground: list = field(default_factory=lambda: [(1.0, -5.0), (1.0, 5.0), (12.0, -10.0), (12.0, 10.0)])
    cam_pixels: list = field(default_factory=lambda: [(160.0, 705.0), (1120.0, 705.0), (80.0, 60.0), (1200.0, 60.0)])
    cam_size: tuple = (1280, 720)
    # thermal: same idea, 32x24 sensor
    th_pixels: list = field(default_factory=lambda: [(4.0, 23.0), (28.0, 23.0), (2.0, 1.0), (30.0, 1.0)])
    seed: int = 1


class SimTurret:
    def __init__(self, world: SimWorld, temp_field=None):
        import cv2
        self.cv2 = cv2
        self.w = world
        self.rng = np.random.default_rng(world.seed)
        W = int((world.x1 - world.x0) * world.ppm)
        Hh = int((world.y1 - world.y0) * world.ppm)
        self.canvas_size = (W, Hh)
        # Grass texture: green-ish with fine noise and blotches.
        base = 110 + 18 * self.rng.standard_normal((Hh, W))
        base = cv2.GaussianBlur(base.astype(np.float32), (0, 0), 1.2)
        blot = cv2.GaussianBlur(self.rng.standard_normal((Hh, W)).astype(np.float32), (0, 0), 25) * 120
        self.texture = np.clip(base + blot, 30, 220).astype(np.float32)
        self.wet = np.zeros((Hh, W), np.float32)       # mm of water applied
        self.temp_field = temp_field                   # callable (x, y) -> deg C

        def canvas_px(pts):
            return [((x - world.x0) * world.ppm, (y - world.y0) * world.ppm) for x, y in pts]
        src = np.float32(canvas_px(world.cam_ground))
        self.H_cam = cv2.getPerspectiveTransform(src, np.float32(world.cam_pixels))

        self.pan = 0.0
        self.tilt = 0.0
        self.pan_err = self.tilt_err = 0.0
        self.valve_open_t = None
        self.t = 0.0
        self.wind_now = (0.0, 0.0)
        self.shots = []          # (pan, tilt, true_x, true_y, seconds)

    # ------------------------------------------------------------ helpers
    def _true_landing(self):
        j = self.w.true_jet
        g = self.w.gust_std
        self.wind_now = (j.wind_x + g * self.rng.standard_normal(), j.wind_y + g * self.rng.standard_normal())
        jj = j.copy(wind_x=self.wind_now[0], wind_y=self.wind_now[1],
                    v0=j.v0 * (1 + self.w.v0_std * self.rng.standard_normal()))
        lx, ly, _ = jj.landing(self.pan + self.pan_err, self.tilt + self.tilt_err)
        s = self.w.splash_std
        return lx + s * self.rng.standard_normal(), ly + s * self.rng.standard_normal()

    def _paint(self, wet, x, y, mm_on_spot):
        """Add water as a Gaussian splash about 0.25 m across."""
        w = self.w
        cx, cy = (x - w.x0) * w.ppm, (y - w.y0) * w.ppm
        sig = 0.12 * w.ppm
        r = int(4 * sig)
        x_lo, x_hi = max(int(cx) - r, 0), min(int(cx) + r, wet.shape[1])
        y_lo, y_hi = max(int(cy) - r, 0), min(int(cy) + r, wet.shape[0])
        if x_lo >= x_hi or y_lo >= y_hi:
            return
        yy, xx = np.mgrid[y_lo:y_hi, x_lo:x_hi]
        wet[y_lo:y_hi, x_lo:x_hi] += mm_on_spot * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * sig ** 2))

    def _mm_peak(self, seconds):
        litres = self.w.flow_lpm / 60 * seconds
        area = 2 * math.pi * 0.12 ** 2                # Gaussian footprint, m^2
        return litres / area

    # ------------------------------------------------------------ interface
    def move(self, pan_cmd, tilt_cmd):
        self.pan, self.tilt = pan_cmd, tilt_cmd
        # where the servo actually stops: deadband and residual backlash, new each move
        self.pan_err = self.w.angle_std * self.rng.standard_normal()
        self.tilt_err = self.w.angle_std * self.rng.standard_normal()

    def valve(self, on: bool):
        if on and self.valve_open_t is None:
            self.valve_open_t = self.t
            self.open_landing = self._true_landing()
        elif not on and self.valve_open_t is not None:
            dur = self.t - self.valve_open_t
            x, y = self.open_landing
            self._paint(self.wet, x, y, self._mm_peak(dur))
            self.shots.append((self.pan, self.tilt, x, y, dur))
            self.valve_open_t = None

    def last_true_landing(self):
        return self.shots[-1][2], self.shots[-1][3]

    def capture(self):
        wet = self.wet
        if self.valve_open_t is not None:               # spray in progress
            wet = wet.copy()
            x, y = self.open_landing
            self._paint(wet, x, y, self._mm_peak(self.t - self.valve_open_t))
        img = self.texture * (1.0 - 0.45 * np.clip(wet / 2.0, 0, 1))
        cam = self.cv2.warpPerspective(img, self.H_cam, self.w.cam_size)
        cam = cam * (1 + 0.03 * self.rng.standard_normal())          # exposure flicker
        cam = cam + 2.0 * self.rng.standard_normal(cam.shape)          # sensor noise
        return np.clip(cam, 0, 255).astype(np.uint8)

    def capture_thermal(self):
        import cv2
        H = cv2.getPerspectiveTransform(np.float32(self.w.cam_ground), np.float32(self.w.th_pixels))
        Hinv = np.linalg.inv(H)
        out = np.zeros((24, 32))
        for v in range(24):
            for u in range(32):
                p = Hinv @ np.array([u, v, 1.0])
                x, y = p[0] / p[2], p[1] / p[2]
                out[v, u] = self.temp_field(x, y) + 0.25 * self.rng.standard_normal()
        return out

    def sleep(self, s: float):
        self.t += s
