"""Public experiment administration and a separate robot-only capability interface."""
from __future__ import annotations
from dataclasses import dataclass, asdict
import copy
import json
import math
from pathlib import Path
from typing import Protocol
from .config import RobotConfig, sensor_config, tick_count
from .geometry import integrate, wrap
from .sensors import Sensor, independent_rng
from .world import WorldMap, finite_values


@dataclass
class _State:
    pose: tuple[float,float,float]
    wheel_speeds: tuple[float,float] = (0.0,0.0)
    wheel_angles: tuple[float,float] = (0.0,0.0)
    v: float = 0.0
    omega: float = 0.0
    accel_forward: float = 0.0
    alpha: float = 0.0
    collided: bool = False
    collision_count: int = 0


class RobotPort(Protocol):
    """The only dependency a controller should have; no true pose/map access."""
    @property
    def time_s(self) -> float: ...
    @property
    def sensor_names(self) -> tuple[str,...]: ...
    def set_motor_pwm(self, left: float, right: float) -> None: ...
    def set_servo(self, sensor: str, angle_rad: float) -> None: ...
    def read_sensor(self, sensor: str) -> dict | None: ...
    def get_observations(self) -> dict: ...
    def stop(self) -> None: ...


class RobotInterface:
    """Architectural separation, not a security sandbox against Python introspection."""
    __slots__ = ("__env",)

    def __init__(self, env):
        self.__env = env

    @property
    def time_s(self):
        return self.__env.time_s

    @property
    def sensor_names(self):
        return tuple(sorted(self.__env._sensors))

    def set_motor_pwm(self, left, right):
        self.__env._set_motor_pwm(left,right)

    def stop(self):
        self.set_motor_pwm(0.0,0.0)

    def set_servo(self, sensor, angle_rad):
        self.__env._set_servo(sensor,angle_rad)

    def read_sensor(self, sensor):
        self.__env._require_reset()
        return self.__env._sensors[sensor].read()

    def get_observations(self):
        return {name:self.read_sensor(name) for name in self.sensor_names}


class RobotEnvironment:
    """A fixed-tick, differential-drive 2D experiment with static polygon maps.

    Commands: left/right PWM in [-100,100]. Distances: metres. Time: seconds.
    Pose yaw and servo commands: radians, positive counterclockwise.
    Explicit reset is required after construction or load_map().
    """
    def __init__(self, map_path=None, *, sensors=None, robot=None, physics_dt_s=0.01, seed=0):
        if not math.isfinite(physics_dt_s) or physics_dt_s <= 0:
            raise ValueError("physics_dt_s must be positive and finite")
        if not isinstance(seed,int) or isinstance(seed,bool):
            raise ValueError("seed must be an integer")
        self.dt = float(physics_dt_s)
        self.seed = seed
        self.config = robot if isinstance(robot,RobotConfig) else RobotConfig(**(robot or {}))
        raw = {"ultrasonic":{},"imu":{},"encoder":{},"bumper":{}} if sensors is None else copy.deepcopy(sensors)
        self.sensor_configs = {}
        for name,c in raw.items():
            parsed = sensor_config(name,c,self.dt,self.config.radius_m)
            if parsed is not None:
                self.sensor_configs[name] = parsed
        self._sensors = {}
        self._state = None
        self._tick = 0
        self._command = (0.0,0.0)
        self._record = None
        self._figure = None
        self._trajectory = []
        self._world = None
        self.robot = RobotInterface(self)
        if map_path is not None:
            self.load_map(map_path)

    @property
    def time_s(self):
        return self._tick*self.dt

    def _require_reset(self):
        if self._state is None:
            raise RuntimeError("call load_map() and reset() first")

    def load_map(self, source):
        if self._record is not None:
            raise RuntimeError("stop_recording() before changing the map")
        if isinstance(source,WorldMap):
            world = WorldMap(source.to_dict())
        elif isinstance(source,dict):
            world = WorldMap(source)
        else:
            world = WorldMap.load(source)
        self._world = world
        self._state = None
        self._sensors = {}
        self._tick = 0
        self._command = (0.0,0.0)
        self._trajectory = []

    def reset(self, pose=None, *, random_spawn=False, seed=None):
        if self._world is None:
            raise RuntimeError("load a map before reset")
        if self._record is not None:
            raise RuntimeError("stop_recording() before reset")
        if pose is not None and random_spawn:
            raise ValueError("choose either pose or random_spawn")
        if seed is not None:
            if not isinstance(seed,int) or isinstance(seed,bool):
                raise ValueError("seed must be integer")
            self.seed = seed
        if pose is None:
            if not random_spawn:
                raise ValueError("provide pose=(x,y,yaw) or random_spawn=True")
            rng = independent_rng(self.seed,"spawn")
            x0,y0,x1,y1 = self._world.bounds
            for _ in range(10000):
                candidate = (rng.uniform(x0,x1),rng.uniform(y0,y1),rng.uniform(-math.pi,math.pi))
                if self._world.is_free(candidate[:2],self.config.radius_m):
                    pose = candidate
                    break
            else:
                raise ValueError("no valid spawn found in 10000 attempts")
        p = finite_values(pose,3,"pose")
        if not self._world.is_free(p[:2],self.config.radius_m):
            raise ValueError("robot footprint collides at the requested pose")
        self._state = _State((float(p[0]),float(p[1]),wrap(float(p[2]))))
        self._tick = 0
        self._command = (0.0,0.0)
        self._trajectory = [self._state.pose]
        self._sensors = {name:Sensor(name,c,self.dt,self.seed) for name,c in self.sensor_configs.items()}
        for sensor in self._sensors.values():
            sensor.sample(0,self._world,self._state,self.config)
            sensor.deliver(0)
        return self.robot.get_observations()

    def _set_motor_pwm(self,left,right):
        self._require_reset()
        if not all(math.isfinite(v) and -100 <= v <= 100 for v in (left,right)):
            raise ValueError("PWM must be finite and in [-100,100]; no silent clipping")
        self._command = (float(left),float(right))
        self._event({"event":"motor","left":float(left),"right":float(right)})

    def _set_servo(self,name,angle):
        self._require_reset()
        self._sensors[name].command_servo(angle)
        self._event({"event":"servo","sensor":name,"angle_rad":float(angle)})

    def _physics_tick(self):
        s,c,dt = self._state,self.config,self.dt
        gains = (c.left_gain,c.right_gain)
        target = tuple(p/100*c.max_wheel_speed_m_s*g for p,g in zip(self._command,gains))
        decay = math.exp(-dt/c.motor_tau_s) if c.motor_tau_s else 0.0
        wheels = tuple(a+(b-a)*decay for a,b in zip(target,s.wheel_speeds))
        s.wheel_speeds = wheels
        # Encoder counts axle motion, even if a wheel slips or the body is blocked.
        s.wheel_angles = tuple(a+v*dt/c.wheel_radius_m for a,v in zip(s.wheel_angles,wheels))
        vl,vr = wheels[0]*(1-c.left_slip),wheels[1]*(1-c.right_slip)
        v,omega = (vl+vr)/2,(vr-vl)/c.wheel_base_m
        proposal = integrate(*s.pose,v,omega,dt)
        # The exact arc is within sagitta of its chord. Subdivide large rotations.
        n = max(1,math.ceil(abs(omega*dt)/0.1)) if abs(v)>1e-15 else 0
        p = s.pose
        collision = False
        for _ in range(n):
            q = integrate(*p,v,omega,dt/n)
            sagitta = abs(v/omega)*(1-math.cos(abs(omega*dt/n)/2)) if abs(omega)>1e-12 else 0
            if not self._world.swept_free(p[:2],q[:2],c.radius_m+sagitta):
                collision = True
                break
            p = q
        old_v,old_omega = s.v,s.omega
        if collision:
            # Conservative stop at previous tick: no penetration, no sliding solver.
            s.v=s.omega=0.0
            s.collision_count += 1
        else:
            s.pose=proposal
            s.v,s.omega=v,omega
        s.collided=collision
        s.accel_forward=(s.v-old_v)/dt
        s.alpha=(s.omega-old_omega)/dt
        self._tick += 1
        for sensor in self._sensors.values():
            sensor.update_actuator(self._tick)
            if self._tick % sensor.period == 0:
                sensor.sample(self._tick,self._world,s,c)
            sensor.deliver(self._tick)

    def step(self,duration_s=None):
        self._require_reset()
        duration = self.dt if duration_s is None else duration_s
        count = tick_count(duration,self.dt,"duration_s")
        for _ in range(count):
            self._physics_tick()
        if count:
            self._trajectory.append(self._state.pose)
        obs = self.robot.get_observations()
        self._event({"event":"step","ticks":count,"truth":self.ground_truth(),"observations":obs})
        return obs

    def ground_truth(self):
        """Evaluator-only state; never pass this return value to a controller."""
        self._require_reset()
        s = self._state
        return {"time_s":self.time_s,"pose":list(s.pose),"wheel_angles_rad":list(s.wheel_angles),
                "wheel_speeds_m_s":list(s.wheel_speeds),"v_m_s":s.v,"omega_rad_s":s.omega,
                "accel_forward_m_s2":s.accel_forward,"alpha_rad_s2":s.alpha,
                "collided":s.collided,"collision_count":s.collision_count}

    def start_recording(self,path):
        self._require_reset()
        if self._record is not None or self._tick != 0:
            raise RuntimeError("recording must start immediately after reset, at tick zero")
        if self._command != (0.0,0.0) or any(s.servo_target for s in self._sensors.values()):
            raise RuntimeError("start recording before sending any command")
        self._record = open(path,"w",encoding="utf-8")
        self._event({"event":"header","format_version":1,"engine_version":"0.1.0",
                     "seed":self.seed,"physics_dt_s":self.dt,"robot":asdict(self.config),
                     "sensors":self.sensor_configs,"map":self._world.to_dict(),
                     "map_sha256":self._world.sha256,"initial_pose":list(self._state.pose)})

    def _event(self,event):
        if self._record is not None:
            self._record.write(json.dumps(event,ensure_ascii=False,sort_keys=True,allow_nan=False)+"\n")
            self._record.flush()

    def stop_recording(self):
        if self._record is not None:
            self._event({"event":"end","ticks":self._tick,"truth":self.ground_truth(),
                         "observations":self.robot.get_observations()})
            self._record.close()
            self._record=None

    def render(self, *, save_path=None, show=False, estimated_pose=None):
        self._require_reset()
        from .render import render_environment
        self._figure = render_environment(self, self._figure,save_path,show,estimated_pose)
        return self._figure

    def close(self):
        self.stop_recording()
        if self._figure is not None:
            import matplotlib.pyplot as plt
            plt.close(self._figure)
            self._figure=None

    def __enter__(self):
        return self

    def __exit__(self,*args):
        self.close()


def replay(path, *, verify=True):
    """Re-simulate a command log; fail on mismatching output or corrupted headers.

    Exact replay is supported with the same engine/Python/NumPy versions. Returning
    the reconstructed environment lets an evaluator inspect or render the result.
    """
    env=None
    try:
        with open(path,encoding="utf-8") as f:
            header=json.loads(next(f))
            if header.get("event")!="header" or header.get("format_version")!=1 or header.get("engine_version")!="0.1.0":
                raise ValueError("unsupported replay header")
            world=WorldMap(header["map"])
            if world.sha256 != header["map_sha256"]:
                raise ValueError("map hash mismatch")
            env=RobotEnvironment(world,sensors=header["sensors"],robot=header["robot"],
                                 physics_dt_s=header["physics_dt_s"],seed=header["seed"])
            env.reset(pose=header["initial_pose"])
            finished=False
            for number,line in enumerate(f,2):
                if finished:
                    raise ValueError("trailing data after replay end marker")
                event=json.loads(line)
                kind=event["event"]
                if kind=="motor":
                    env.robot.set_motor_pwm(event["left"],event["right"])
                elif kind=="servo":
                    env.robot.set_servo(event["sensor"],event["angle_rad"])
                elif kind=="step":
                    if not isinstance(event["ticks"],int) or event["ticks"]<0:
                        raise ValueError("invalid replay tick count")
                    obs=env.step(event["ticks"]*env.dt)
                    if verify and (obs!=event["observations"] or env.ground_truth()!=event["truth"]):
                        raise AssertionError(f"replay diverged at line {number}")
                elif kind=="end":
                    if event["ticks"] != env._tick:
                        raise ValueError("replay end tick mismatch")
                    if verify and (env.robot.get_observations()!=event["observations"] or env.ground_truth()!=event["truth"]):
                        raise AssertionError(f"replay diverged at end marker, line {number}")
                    finished=True
                else:
                    raise ValueError(f"unknown replay event: {kind}")
            if not finished:
                raise ValueError("truncated replay: missing end marker")
        return env
    except Exception:
        if env is not None:
            env.close()
        raise
