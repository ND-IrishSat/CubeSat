
"""
Simulated 3-axis magnetometer with realistic error sources.
 
Matches the generic interface (begin, reset, chip_id, set_mode, set_data_rate,
set_range, data_ready, read_raw, read_field, calibrate, heading), so code
written against it can later be pointed at a real driver.
 
Error model (applied in this order):
    true field (Earth, in the sensor's body frame)
      -> soft-iron distortion (3x3 matrix)
      -> per-axis gain error
      -> hard-iron offset (constant bias from nearby magnets/steel)
      -> slow bias drift (random walk) + temperature drift
      -> white Gaussian noise
      -> range clipping (saturation) and quantization to signed 16-bit counts
 
Body frame: x forward, y right, z down. Heading = 0 deg is magnetic north,
increasing clockwise.
 
Requires: numpy

Used codex to make this, so it's not all mine. I just added the docstrings and some comments to help me understand it. Not completely finished yet tho. 
"""
 
import math
import numpy as np
 
 
class MagnetometerSim:
    MODES = ("power_down", "single", "continuous")
    CHIP_ID = 0x48  # arbitrary fake ID
 
    def __init__(
        self,
        seed=None,
        earth_field_ut=50.0,          # total field magnitude, microtesla
        inclination_deg=65.0,         # dip angle (positive = field points down)
        declination_deg=0.0,          # magnetic vs. true north
        noise_std_ut=0.3,             # white noise per axis, uT RMS
        hard_iron_ut=(12.0, -8.0, 5.0),
        soft_iron=None,               # 3x3 matrix; None -> mild default distortion
        gain_error=(1.00, 0.98, 1.02),
        bias_walk_ut_per_sqrt_s=0.002,
        temp_coeff_ut_per_c=0.01,     # offset drift per deg C away from 25 C
        temperature_c=25.0,
    ):
        self.rng = np.random.default_rng(seed)
        self.earth_field_ut = earth_field_uth
        self.inclination = math.radians(inclination_deg)
        self.declination = math.radians(declination_deg)
        self.noise_std = noise_std_ut
        self.hard_iron = np.array(hard_iron_ut, dtype=float)
        self.soft_iron = (
            np.array(soft_iron, dtype=float)
            if soft_iron is not None
            else np.array([[1.00, 0.05, 0.02],
                           [0.05, 0.92, 0.03],
                           [0.02, 0.03, 1.08]])
        )
        self.gain = np.array(gain_error, dtype=float)
        self.bias_walk = bias_walk_ut_per_sqrt_s
        self.temp_coeff = temp_coeff_ut_per_c
        self.temperature_c = temperature_c
 
        # Simulated clock (seconds). Advance it yourself with advance(dt).
        self._t = 0.0
        self._bias = np.zeros(3)
        self._orientation_fn = lambda t: (0.0, 0.0, 0.0)  # yaw, pitch, roll (deg)
 
        # Software calibration (filled in by calibrate())
        self.cal_offset = np.zeros(3)
        self.cal_matrix = np.eye(3)
 
        self.begin()
 
    # ------------------------------------------------------------------ setup
    def begin(self):
        self.reset()
        self.set_mode("continuous")
        return self.chip_id() == self.CHIP_ID
 
    def reset(self):
        self._mode = "power_down"
        self._odr = 10.0
        self._lsb = 100.0 / 32768.0
        self._range = 100.0
        self._next_ready = 0.0
        self._single_pending = False
        self._last_raw = (0, 0, 0)
        self.saturated = False
 
    def chip_id(self):
        return self.CHIP_ID
 
    def set_mode(self, mode):
        if mode not in self.MODES:
            raise ValueError(f"mode must be one of {self.MODES}")
        self._mode = mode
        if mode == "single":
            self._single_pending = True
            self._next_ready = self._t + 1.0 / self._odr
        elif mode == "continuous":
            self._next_ready = self._t + 1.0 / self._odr
 
    def set_data_rate(self, hz):
        if hz <= 0:
            raise ValueError("data rate must be positive")
        self._odr = float(hz)
 
    def set_range(self, full_scale_ut):
        """Symmetric full-scale range in uT; 16-bit signed output."""
        self._range = float(full_scale_ut)
        self._lsb = self._range / 32768.0
 
    # --------------------------------------------------------------- sim control
    def advance(self, dt):
        """Move the simulated clock forward and update slow drift."""
        self._t += dt
        self._bias += self.rng.normal(0.0, self.bias_walk * math.sqrt(dt), 3)
 
    def set_orientation(self, yaw_deg, pitch_deg=0.0, roll_deg=0.0):
        """Hold the sensor at a fixed orientation."""
        o = (yaw_deg, pitch_deg, roll_deg)
        self._orientation_fn = lambda t: o
 
    def set_motion(self, fn):
        """fn(t) -> (yaw, pitch, roll) in degrees, for a moving sensor."""
        self._orientation_fn = fn
 
    @property
    def time(self):
        return self._t
 
    # --------------------------------------------------------------- measurement
    def data_ready(self):
        if self._mode == "power_down":
            return False
        if self._mode == "single" and not self._single_pending:
            return False
        return self._t >= self._next_ready
 
    def read_raw(self):
        """Signed integer counts (x, y, z). Returns stale data if not ready."""
        if not self.data_ready():
            return self._last_raw
        b = self._measure_ut()
        counts = np.round(b / self._lsb)
        self.saturated = bool(np.any(np.abs(counts) >= 32767))
        counts = np.clip(counts, -32768, 32767).astype(int)
        self._last_raw = tuple(int(c) for c in counts)
 
        period = 1.0 / self._odr
        if self._mode == "single":
            self._single_pending = False
            self._mode = "power_down"
        else:
            self._next_ready = self._t + period
        return self._last_raw
 
    def read_field(self):
        """Field in uT (uncalibrated: includes hard/soft-iron errors)."""
        return tuple(c * self._lsb for c in self.read_raw())
 
    def read_field_calibrated(self):
        b = np.array(self.read_field())
        return tuple(self.cal_matrix @ (b - self.cal_offset))
 
    def read_temperature(self):
        return self.temperature_c + self.rng.normal(0.0, 0.2)
 
    def sample(self):
        """Convenience: wait one sample period in sim time, then read."""
        self.advance(1.0 / self._odr)
        return self.read_field()
 
    # --------------------------------------------------------------- calibration
    def calibrate(self, samples):
        """
        Simple min/max calibration from samples (N x 3, uT) collected while
        rotating the sensor through many orientations.
        Hard iron: center of the bounding box.
        Soft iron: per-axis scale so every axis has the same average radius
        (diagonal only; an ellipsoid fit would correct the off-diagonals too).
        """
        s = np.asarray(samples, dtype=float)
        lo, hi = s.min(axis=0), s.max(axis=0)
        self.cal_offset = (hi + lo) / 2.0
        radii = (hi - lo) / 2.0
        self.cal_matrix = np.diag(radii.mean() / radii)
 
    def heading(self, calibrated=True, apply_declination=False):
        """Heading in degrees [0, 360), assuming the sensor is level."""
        x, y, _ = self.read_field_calibrated() if calibrated else self.read_field()
        h = math.degrees(math.atan2(-y, x))
        if apply_declination:
            h += math.degrees(self.declination)
        return h % 360.0
 
    # ------------------------------------------------------------------ internals
    def _true_body_field(self, t):
        yaw, pitch, roll = (math.radians(a) for a in self._orientation_fn(t))
        # Earth's field in the north-east-down frame
        b_ned = self.earth_field_ut * np.array([
            math.cos(self.inclination) * math.cos(self.declination),
            math.cos(self.inclination) * math.sin(self.declination),
            math.sin(self.inclination),
        ])
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        # Body -> NED rotation (yaw, pitch, roll = Z-Y-X)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ])
        return R.T @ b_ned
 
    def _measure_ut(self):
        b = self._true_body_field(self._t)
        b = self.gain * (self.soft_iron @ b)
        b = b + self.hard_iron + self._bias
        b = b + self.temp_coeff * (self.temperature_c - 25.0)
        b = b + self.rng.normal(0.0, self.noise_std, 3)
        return np.clip(b, -self._range, self._range)
 
 
# ====================================================================== demo
def _angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0
 
 
if __name__ == "__main__":
    mag = MagnetometerSim(seed=42)
    mag.set_data_rate(50)
    print(f"chip id: {mag.chip_id():#04x}, begin ok: {mag.begin()}")
 
    # 1) Sample while tumbling through random orientations (calibration run)
    rng = np.random.default_rng(1)
    mag.set_mode("continuous")
    cal_samples = []
    for _ in range(2000):
        mag.set_orientation(rng.uniform(0, 360),
                            rng.uniform(-90, 90),
                            rng.uniform(-180, 180))
        cal_samples.append(mag.sample())
    cal_samples = np.array(cal_samples)
    print(f"raw min/max per axis (uT): {cal_samples.min(0).round(1)} / "
          f"{cal_samples.max(0).round(1)}")
 
    mag.calibrate(cal_samples)
    print(f"estimated hard iron (uT): {mag.cal_offset.round(2)}  "
          f"(true: {mag.hard_iron})")
 
    # 2) Level heading sweep: error before vs. after calibration
    errs_raw, errs_cal = [], []
    for yaw in range(0, 360, 5):
        mag.set_orientation(yaw)
        for _ in range(10):
            mag.sample()
        errs_raw.append(_angle_diff(mag.heading(calibrated=False), yaw))
        errs_cal.append(_angle_diff(mag.heading(calibrated=True), yaw))
 
    rms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))
    print(f"heading RMS error, uncalibrated: {rms(errs_raw):6.2f} deg")
    print(f"heading RMS error, calibrated:   {rms(errs_cal):6.2f} deg")
 
    # 3) Noise check: hold still and look at the scatter
    mag.set_orientation(90)
    still = np.array([mag.sample() for _ in range(2000)])
    print(f"stationary std per axis (uT): {still.std(0).round(3)} "
          f"(configured white noise: {mag.noise_std})")
 
    # 4) Saturation behaviour with a too-small range
    mag.set_range(50)
    mag.sample()
    print(f"saturated at +/-50 uT range: {mag.saturated}")
