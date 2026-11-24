"""Independently seeded, cached, timestamped, periodically sampled sensors."""
from __future__ import annotations
from collections import deque
import copy
import hashlib
import math
import random
from .config import tick_count


def independent_rng(seed: int, name: str):
    token = hashlib.sha256(f"robot-sim-v1:{seed}:{name}".encode()).digest()
    return random.Random(int.from_bytes(token,"big"))


def nearest_tick(value):
    return math.copysign(math.floor(abs(value) + 0.5), value) if value else 0


class Sensor:
    def __init__(self, name, config, dt, seed):
        self.name, self.config, self.dt = name, copy.deepcopy(config), dt
        self.period = tick_count(1/config["frequency_hz"], dt, "period")
        self.latency = tick_count(config["latency_s"],dt,"latency")
        self.rng = independent_rng(seed,"sensor/"+name)
        self.queue = deque()
        self.latest = None
        self.sequence = 0
        self.servo_angle = self.servo_target = 0.0
        self.settle = tick_count(config.get("servo_settle_s",0),dt,"settle")
        self.last_move_tick = -self.settle
        self.bias = config.get("gyro_bias_rad_s",0.0)
        self.previous_encoder = (0,0)
        self.previous_collision_count = 0
        self.debug_origin = None

    def command_servo(self, angle):
        c = self.config
        if c["type"] != "ultrasonic" or not c["servo"]:
            raise ValueError(f"{self.name} has no steerable servo")
        if not math.isfinite(angle) or not c["servo_min_rad"] <= angle <= c["servo_max_rad"]:
            raise ValueError("servo command outside configured limits")
        self.servo_target = float(angle)

    def update_actuator(self,tick):
        if self.config["type"] != "ultrasonic" or not self.config["servo"]:
            return
        error = self.servo_target - self.servo_angle
        if abs(error)>1e-12:
            change = min(abs(error),self.config["servo_speed_rad_s"] * self.dt)
            self.servo_angle += math.copysign(change,error)
            self.last_move_tick = tick

    def _range(self, world, origin, angle, width=0.0):
        c = self.config
        d = world.range(origin,angle,c["max_range_m"],width)
        if not math.isfinite(d):
            return None,"no_echo"
        if d < c["min_range_m"] - 1e-10:
            return None,"below_min"
        if c["dropout_probability"] and self.rng.random() < c["dropout_probability"]:
            return None,"dropout"
        if c["noise_std_m"]:
            d += self.rng.gauss(0,c["noise_std_m"])
        return min(c["max_range_m"],max(c["min_range_m"],d)),"hit"

    def sample(self,tick,world,state,robot_config):
        c = self.config
        kind = c["type"]
        x,y,theta = state.pose
        ox,oy,phi = c["mount"]
        ct,st = math.cos(theta),math.sin(theta)
        origin = (x + ct*ox-st*oy,y+st*ox+ct*oy)
        bearing = phi + self.servo_angle
        self.debug_origin = (origin,theta+bearing)
        if kind in ("ultrasonic","infrared"):
            ready = (abs(self.servo_angle-self.servo_target)<1e-12 and tick-self.last_move_tick >= self.settle)
            if kind == "ultrasonic" and not ready:
                d,status = None,"servo_unsettled"
            else:
                d,status = self._range(world,origin,theta+bearing,math.radians(c["beam_width_deg"]))
            data = {"distance_m":d,"status":status,"bearing_rad":phi+self.servo_target}
            if kind == "ultrasonic":
                data["echo_time_s"] = None if d is None else 2*d/c["sound_speed_m_s"]
        elif kind == "lidar":
            measured = [self._range(world,origin,theta+phi+a) for a in c["angles_rad"]]
            data = {"angles_rad":list(c["angles_rad"]),"ranges_m":[p[0] for p in measured],"statuses":[p[1] for p in measured]}
        elif kind == "imu":
            omega,alpha = state.omega,state.alpha
            # Body x forward, y left, z up. Specific force includes +g at rest.
            ax = state.accel_forward - alpha*oy - omega*omega*ox
            ay = state.v*omega + alpha*ox - omega*omega*oy
            cp,sp = math.cos(phi),math.sin(phi)
            force = [cp*ax+sp*ay,-sp*ax+cp*ay,c["gravity_m_s2"]]
            if tick and c["gyro_bias_walk_rad_s_sqrt_s"]:
                self.bias += self.rng.gauss(0,c["gyro_bias_walk_rad_s_sqrt_s"]*math.sqrt(self.period*self.dt))
            if c["accel_noise_std_m_s2"]:
                force = [a+self.rng.gauss(0,c["accel_noise_std_m_s2"]) for a in force]
            gyro = omega+self.bias
            if c["gyro_noise_std_rad_s"]:
                gyro += self.rng.gauss(0,c["gyro_noise_std_rad_s"])
            data = {"angular_velocity_rad_s":[0.0,0.0,gyro],"specific_force_m_s2":force}
        elif kind == "encoder":
            scale = c["ticks_per_rev"]/(2*math.pi)
            counts = tuple(int(nearest_tick(a*scale)) for a in state.wheel_angles)
            data = {"left_ticks":counts[0],"right_ticks":counts[1],
                    "left_delta":counts[0]-self.previous_encoder[0],"right_delta":counts[1]-self.previous_encoder[1]}
            self.previous_encoder = counts
        else: # Whole-body contact switch, latched since last acquisition.
            data = {"contact":state.collision_count>self.previous_collision_count}
            self.previous_collision_count = state.collision_count
        frame = {"sensor":self.name,"type":kind,"sequence":self.sequence,
                 "timestamp_s":tick*self.dt,"available_at_s":(tick+self.latency)*self.dt,"data":data}
        self.sequence += 1
        self.queue.append((tick+self.latency,frame))

    def deliver(self,tick):
        while self.queue and self.queue[0][0] <= tick:
            self.latest = self.queue.popleft()[1]

    def read(self):
        return copy.deepcopy(self.latest)
