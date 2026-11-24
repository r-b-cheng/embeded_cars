"""Strict configuration parsing. All public angles are radians except explicit *_deg fields."""
from __future__ import annotations
from dataclasses import dataclass, asdict
import copy
import math


@dataclass(frozen=True)
class RobotConfig:
    radius_m: float = 0.10
    wheel_base_m: float = 0.135
    wheel_radius_m: float = 0.0325
    max_wheel_speed_m_s: float = 0.30
    motor_tau_s: float = 0.0
    left_gain: float = 1.0
    right_gain: float = 1.0
    left_slip: float = 0.0
    right_slip: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("robot parameters must be finite")
        for k in ("radius_m","wheel_base_m","wheel_radius_m","max_wheel_speed_m_s","left_gain","right_gain"):
            if getattr(self,k) <= 0:
                raise ValueError(f"{k} must be positive")
        if self.motor_tau_s < 0 or not (0 <= self.left_slip <= 1 and 0 <= self.right_slip <= 1):
            raise ValueError("tau >= 0; slip must lie in [0,1]")


def tick_count(seconds, dt, label):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError(f"{label} must be finite and nonnegative")
    count = round(seconds / dt)
    if abs(count * dt - seconds) > 1e-9:
        raise ValueError(f"{label} must be an integer multiple of physics_dt_s={dt}")
    return count


_COMMON = {"enabled":True, "frequency_hz":10.0, "latency_s":0.0, "mount":[0.0,0.0,0.0]}
_RANGE = {"min_range_m":0.02, "max_range_m":4.0, "noise_std_m":0.0, "dropout_probability":0.0}
_DEFAULTS = {
    "ultrasonic": {**_RANGE, "beam_width_deg":15.0,"servo":True,"servo_min_rad":-math.pi/2,
                   "servo_max_rad":math.pi/2,"servo_speed_rad_s":math.pi,
                   "servo_settle_s":0.06,"sound_speed_m_s":343.0},
    "lidar": {**_RANGE,"max_range_m":12.0,"min_range_m":0.05,
              "angles_rad":[math.radians(i) for i in range(-180,180,4)]},
    "infrared": {**_RANGE,"min_range_m":0.05,"max_range_m":0.8,"beam_width_deg":0.0},
    "imu": {"frequency_hz":100.0,"gyro_noise_std_rad_s":0.0,"accel_noise_std_m_s2":0.0,
            "gyro_bias_rad_s":0.0,"gyro_bias_walk_rad_s_sqrt_s":0.0,"gravity_m_s2":9.80665},
    "encoder": {"frequency_hz":50.0,"ticks_per_rev":1440},
    "bumper": {"frequency_hz":100.0},
}


def sensor_config(name, raw, dt, radius):
    raw = copy.deepcopy(raw)
    kind = raw.pop("type", name)
    if kind not in _DEFAULTS:
        raise ValueError(f"unsupported sensor type {kind!r}")
    defaults = {**_COMMON, **_DEFAULTS[kind]}
    unknown = set(raw) - set(defaults)
    if unknown:
        raise ValueError(f"{name}: unknown parameters {sorted(unknown)}")
    c = {**defaults, **raw, "type":kind}
    if not isinstance(c["enabled"], bool):
        raise ValueError("enabled must be bool")
    if not c["enabled"]:
        return None
    if not isinstance(name,str) or not name:
        raise ValueError("sensor name must be nonempty str")
    for key,value in c.items():
        if isinstance(value,(int,float)) and not math.isfinite(value):
            raise ValueError(f"{name}.{key} must be finite")
    hz = c["frequency_hz"]
    if hz <= 0 or tick_count(1/hz, dt, "sensor period") < 1:
        raise ValueError("sensor rate exceeds physics rate")
    tick_count(c["latency_s"],dt,"latency_s")
    mount = c["mount"]
    if not isinstance(mount,(tuple,list)) or len(mount)!=3 or not all(math.isfinite(v) for v in mount):
        raise ValueError("mount must be [x_m, y_m, yaw_rad]")
    if math.hypot(*mount[:2]) > radius + 1e-10:
        raise ValueError("sensor mount must lie inside the circular robot footprint")
    c["mount"] = list(mount)
    if kind in ("ultrasonic","infrared","lidar"):
        if not 0 <= c["min_range_m"] < c["max_range_m"] or c["noise_std_m"] < 0 or not 0 <= c["dropout_probability"] <= 1:
            raise ValueError("invalid sensor range/noise/dropout")
    if kind in ("ultrasonic","infrared") and not 0 <= c["beam_width_deg"] < 180:
        raise ValueError("beam_width_deg must lie in [0,180)")
    if kind == "ultrasonic":
        if not isinstance(c["servo"], bool) or not c["servo_min_rad"] <= 0 <= c["servo_max_rad"] or c["servo_max_rad"] <= c["servo_min_rad"] or c["servo_speed_rad_s"] <= 0 or c["sound_speed_m_s"] <= 0:
            raise ValueError("invalid ultrasonic servo/sound speed settings")
        tick_count(c["servo_settle_s"],dt,"servo_settle_s")
    if kind == "lidar":
        aa = c["angles_rad"]
        if not isinstance(aa,(tuple,list)) or not aa or len(aa)>10000 or not all(math.isfinite(a) for a in aa):
            raise ValueError("angles_rad needs 1..10000 finite values")
        c["angles_rad"] = list(aa)
    if kind == "imu":
        for key in ("gyro_noise_std_rad_s","accel_noise_std_m_s2","gyro_bias_walk_rad_s_sqrt_s","gravity_m_s2"):
            if c[key]<0:
                raise ValueError(f"{key} must be nonnegative")
    if kind == "encoder" and (isinstance(c["ticks_per_rev"],bool) or not isinstance(c["ticks_per_rev"],int) or c["ticks_per_rev"] <= 0):
        raise ValueError("ticks_per_rev must be a positive integer, including quadrature multiplication")
    if kind in ("encoder","bumper") and c["mount"] != [0,0,0]:
        raise ValueError("encoder/bumper are whole-robot sensors; mount must be [0,0,0]")
    return c
