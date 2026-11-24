# Embedded Cars Gym — 二维小车仿真环境

面向 TM4C123G 双轮差速小车的 **Python 算法实验环境（v0.1.0）**。在真实硬件接入前，可以加载地图、放置机器人、控制左右轮、扫描超声波，以及比较定位和建图算法。

> **接口约定**：本项目目前是一个自定义的仿真 Gym，**尚未实现 Gymnasium / OpenAI Gym 标准接口**（例如 `observation, reward, terminated, truncated, info` 的五元组）。实际 API 为 `RobotEnvironment` + `RobotPort`。不要直接把它传给需要 `gymnasium.Env` 的训练库。

## 0. 五分钟上手

环境要求：**Python ≥ 3.10**。从整个仓库根目录运行：

```bash
git clone https://github.com/r-b-cheng/embeded_cars.git
cd embeded_cars/envs

python -m venv .venv
# Linux / macOS
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -e ".[visual,test]"

# 打开带键盘控制的二维练习场（需要桌面图形界面）
python -m robot_sim.demo --map maps/01_empty_room.json --pose 1 1 0 --interactive

# 没有图形界面：运行脚本并保存最终画面
python -m robot_sim.demo --map maps/08_irregular.json --steps 300 --save output.png

# 检查环境
python -m pytest -q
```

Windows CMD 使用 `.venv\Scripts\activate.bat`；如果 PowerShell 禁止执行激活脚本，可以直接用 `.venv\Scripts\python.exe -m pip ...`。

交互键位：

| 按键 | 动作 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 原地左转 / 右转 |
| 松开 `W/A/S/D` | 停车 |
| 空格 | 停车 |
| `[` / `]` | 将默认超声波舵机向右 / 左调节 |
| `P` | 暂停或恢复仿真 |
| `Esc` | 关闭窗口 |

**坐标与单位：**世界 X 轴向右、Y 轴向上；`yaw=0` 朝 X 正向，逆时针为正。位置和距离用**米**，时间用**秒**，角度和角速度用**弧度 / rad/s**；只有名称带 `_deg` 的参数使用角度制。

## 1. 第一个 Python 程序：加载、驱动、读取传感器

在 `envs/` 下新建 `my_experiment.py`：

```python
import math
from robot_sim import RobotEnvironment

# 字典的键是传感器实例名；名称与类型不相同时需提供 type。
sensors = {
    "front": {
        "type": "ultrasonic",
        "frequency_hz": 10,
        "min_range_m": 0.02,
        "max_range_m": 4.0,
        "beam_width_deg": 15,
        "noise_std_m": 0.0,       # 先用无噪声验证算法
        "mount": [0.06, 0, 0],   # 相对车体中心：向前 6 cm
        "servo": True,
    },
    "imu": {"type": "imu", "frequency_hz": 100},
}

with RobotEnvironment(
    map_path="maps/01_empty_room.json",
    sensors=sensors,
    physics_dt_s=0.01,
    seed=42,
) as env:
    # 明确设置初始位置 x=1m, y=1m, yaw=0rad
    env.reset(pose=(1.0, 1.0, 0.0))
    robot = env.robot

    # 设置左右轮有符号 PWM，取值范围 [-100, 100]
    robot.set_motor_pwm(left=40, right=40)
    env.step(0.20)                       # 推进 0.20 秒
    robot.stop()

    sonar = robot.read_sensor("front")
    imu = robot.read_sensor("imu")
    print("sonar:", sonar)
    print("imu:", imu)

    # 停车后把超声波舵机指向车身左前方 30°
    robot.set_servo("front", math.radians(30))
    env.step(0.30)                       # 留出舵机运动、稳定与采样时间
    print("new sonar:", robot.read_sensor("front"))

    env.render(save_path="my_experiment.png")
    # 仅用于实验评估；不要把真值传给算法：
    print("ground truth:", env.ground_truth())
```

然后执行：

```bash
python my_experiment.py
```

注意：

- `set_motor_pwm(...)` 设置**持续保持**的驱动命令；必须明确调用 `stop()` 才会停车。
- `step()` 默认推进一个物理步（默认 **0.01 s**）；`step(0.20)` 推进 20 步。时长必须是物理步长的整数倍。
- `reset(pose=(x, y, yaw))` 的位姿必须让**整个圆形车体**不与障碍物相交。
- `read_sensor()` 返回最近已交付的缓存观测；有采样延迟时可能暂时返回 `None`，多次读取不会重新抽噪声。
- 如果要同时拿到所有已启用传感器，调用 `robot.get_observations()`。

### 从随机位置生成小车

```python
env.reset(random_spawn=True, seed=73)
```

随机起点会检查车体与障碍物的净空；相同地图、参数和种子可重现结果。**每次 `load_map()` 后都需要重新 `reset()`**。运行中不可把 `reset()` 当成控制小车瞬移的方法。

## 2. 运行时 API：算法只依赖 RobotPort

环境专门区分了**实验管理 API**和**机器人运行 API**：

| API 所属对象 | 方法或属性 | 用途 |
|---|---|---|
| `env` | `load_map(path_or_dict)` | 加载 / 更换真值地图 |
| `env` | `reset(pose=...)`、`reset(random_spawn=True)` | 初始化机器人、时钟与传感器 |
| `env` | `step(duration_s=None)` | 推进仿真时间；返回所有当前观测 |
| `env` | `render(save_path=..., estimated_pose=...)` | 显示地图、轨迹及可选的估计位姿 |
| `env` | `ground_truth()` | **仅评估程序**可读真实位姿及速度 |
| `env.robot` | `set_motor_pwm(left, right)`、`stop()` | 控制左右电机 |
| `env.robot` | `set_servo(sensor, angle_rad)` | 调整支持舵机的超声波实例 |
| `env.robot` | `read_sensor(name)` | 读取单个传感器观测 |
| `env.robot` | `get_observations()` | 获取传感器观测字典 |
| `env.robot` | `sensor_names`、`time_s` | 查看已启用传感器和仿真时间 |

**你们的 SLAM、定位、A* 和控制算法应该只接收 `RobotPort`。** 主程序负责创建环境与调用 `env.step()`，算法内部只能发送电机/舵机命令和读取传感器：

```python
from robot_sim import RobotEnvironment, RobotPort

class ObstacleStopController:
    def update(self, robot: RobotPort):
        frame = robot.read_sensor("ultrasonic")
        if frame is None:
            robot.stop()
            return
        # 过期、缺失的距离不能当作安全通道
        if robot.time_s - frame["timestamp_s"] > 0.25:
            robot.stop()
        elif frame["data"]["status"] != "hit":
            robot.stop()
        elif frame["data"]["distance_m"] < 0.5:
            robot.stop()
        else:
            robot.set_motor_pwm(35, 35)

controller = ObstacleStopController()
with RobotEnvironment(
    "maps/02_central_box.json",
    sensors={"ultrasonic": {}},
) as env:
    env.reset(pose=(0.6, 0.7, 0.0))
    for _ in range(300):
        controller.update(env.robot)
        env.step()
    env.render(save_path="controller_example.png")
```

这个控制器只展示接口使用方法，**不构成可靠避障或导航算法**。完整代码见 [算法模板](examples/algorithm_template.py)。

> 目前隔离是 Python 软件架构约定，不能防止故意通过反射访问内部属性。未来连接真实小车时，可以实现相同的 `RobotPort`，把驱动指令和观测读写映射到 TM4C/ESP 的通信协议；若想让算法在 MCU 上原生运行，需要另行移植。

## 3. 传感器怎么选、怎么调参数？

`sensors=None` 使用默认的**超声波 + IMU + 编码器 + 碰撞开关**；`sensors={}` 表示全部不启用。传感器实例的键是名称，`type` 是类型，同种传感器可以出现多次。

| `type` | 读数 `data` 核心字段 | 常用配置 |
|---|---|---|
| `ultrasonic` | `distance_m`、`status`、`bearing_rad`、`echo_time_s` | `min_range_m`、`max_range_m`、`beam_width_deg`、`servo` |
| `imu` | `angular_velocity_rad_s`、`specific_force_m_s2` | `gyro_noise_std_rad_s`、`gyro_bias_rad_s`、`accel_noise_std_m_s2` |
| `encoder` | `left_ticks`、`right_ticks`、`left_delta`、`right_delta` | `ticks_per_rev`（已包含 AB 相解码倍频及减速比） |
| `lidar` | `angles_rad`、`ranges_m`、`statuses` | `angles_rad`、`max_range_m`、`noise_std_m` |
| `infrared` | `distance_m`、`status`、`bearing_rad` | `min_range_m`、`max_range_m`、`beam_width_deg` |
| `bumper` | `contact` | `frequency_hz` |

所有类型支持 `enabled`、`frequency_hz`、`latency_s`、`mount=[x_m,y_m,yaw_rad]`，其中编码器和碰撞开关的安装位姿固定为零。另有传感器专属参数，请看 [配置校验代码](robot_sim/config.py)。

推荐的**可对比实验配置**：

```python
sonar_only = {
    "ultrasonic": {"noise_std_m": 0.01, "dropout_probability": 0.02}
}

sonar_and_imu = {
    **sonar_only,
    "imu": {"frequency_hz": 100, "gyro_noise_std_rad_s": 0.005}
}

sonar_and_encoder = {
    **sonar_only,
    "encoder": {"frequency_hz": 50, "ticks_per_rev": 1440}
}
```

在不同传感器组合下复用相同地图、运动序列和种子，比较定位算法的输出。更完整的例子见 [传感器对照实验](examples/sensor_ablation.py)。

### 一帧传感器数据长什么样？

```json
{
  "sensor": "ultrasonic",
  "type": "ultrasonic",
  "sequence": 3,
  "timestamp_s": 0.3,
  "available_at_s": 0.3,
  "data": {
    "distance_m": 1.25,
    "status": "hit",
    "bearing_rad": 0.0,
    "echo_time_s": 0.00728863
  }
}
```

上述数字仅用于展示数据结构。`timestamp_s` 是采样时间，`available_at_s` 是数据可供读取的时间。使用 `sequence` 可识别重复观测。

超声波可能返回 `hit`、`no_echo`、`below_min`、`dropout`、`servo_unsettled`。**只有 `hit` 的 `distance_m` 是有效数值**，其他状态下为 `None`；不要把无回波解释为无遮挡。超声波 `bearing_rad` 是车身相对指向，**不包含车身的世界航向**。

### 调整差速小车的物理参数

```python
env = RobotEnvironment(
    "maps/01_empty_room.json",
    robot={
        "radius_m": 0.10,
        "wheel_base_m": 0.135,
        "wheel_radius_m": 0.0325,
        "max_wheel_speed_m_s": 0.30,
        "motor_tau_s": 0.08,
        "left_gain": 1.0,
        "right_gain": 0.98,
        "left_slip": 0.02,
        "right_slip": 0.02
    },
)
```

左右轮速度、转角和小车位姿通过差速运动学更新；可模拟电机响应滞后、增益不一致、固定打滑比例和碰撞。没有轮胎接触力、真实电机电流、三维姿态等完整物理模型。

**频率约束：**物理步长默认 0.01 s，因此支持每 1、2、5、10 个物理步等整数周期的采样，如 100 Hz、50 Hz、20 Hz、10 Hz；30 Hz 不能直接配合 0.01 s 使用，会抛出异常。采样延迟也必须是物理步长的整数倍。

## 4. 加载自定义地图；导入后续真实扫图结果

地图使用**版本化 JSON**，并支持多边形、墙段和二维占据栅格。如下示例代表 4m × 4m 的场地，以及一个矩形障碍物：

```json
{
  "version": 1,
  "units": "m",
  "name": "my_room",
  "bounds": [0, 0, 4, 4],
  "polygons": [
    [[1.5, 1.5], [2.0, 1.5], [2.0, 2.2], [1.5, 2.2]]
  ],
  "walls": []
}
```

将它保存为 `maps/my_room.json`，随后：

```python
env = RobotEnvironment("maps/my_room.json")
env.reset(pose=(0.6, 0.6, 0))
```

地图的四条边自动视为实体边界；`polygons` 是有面积的实体，`walls` 为线段障碍。拒绝自交多边形、越界障碍和不合法的几何；完整结构参见 [地图 Schema](docs/map.schema.json)。

**真实超声波扫图的推荐输出：Occupancy Grid。** 当你们的 SLAM 已生成具有实际坐标和分辨率的栅格时，可以转成相同的 JSON 地图：

```python
import numpy as np
from robot_sim import WorldMap

# -1 = 未知，0..100 = 占用概率百分数
cells = np.array([
    [100, 100, 100, 100],
    [100,   0,   0, 100],
    [100,   0,   0, 100],
    [100, 100, 100, 100],
])
world = WorldMap.from_occupancy(
    cells,
    resolution_m=0.10,
    origin_m=(0, 0),
    unknown_policy="occupied"
)
world.save("measured_map.json")
```

`cells[0]` 是地图**最底部**一行，列向右、行向上；`origin_m` 是左下角。采用阈值 65（0..100）决定实体障碍。带未知格（-1）的地图必须显式指定 `unknown_policy="occupied"` 或 `"free"`；前者保守，后者会假设未探测区域可通行。

CSV、NPY、PNG、PGM、BMP 也可以使用命令行转换：

```bash
# 从二维整数占用概率栅格（-1, 0..100）导入
python -m robot_sim.convert recorded.npy maps/from_robot.json \
  --resolution 0.05 --unknown-policy occupied

# 二值栅格（-1,0,1）
python -m robot_sim.convert recorded.csv maps/from_robot.json \
  --resolution 0.05 --encoding binary --unknown-policy occupied

# PNG 图像：黑色为占用、白色为空闲，灰色中间区间为未知
python -m robot_sim.convert recorded.png maps/from_image.json \
  --resolution 0.05 --unknown-policy occupied
```

CSV/NPY 来源如果按图片行序（上→下）存储，添加 `--top-to-bottom`；图片输入会自动翻转。原始超声波日志只包含测距，**必须先经过你们的建图算法生成空间占用栅格**，格式转换器不会自动做 SLAM。详见 [地图转换实现](robot_sim/convert.py)。

项目附带 10 张测试地图，位于 [maps/](maps/)：空房间、中央障碍、L 形走廊、T 路口、斜墙、门洞、折线迷宫、不规则场景、杂物区和栅格导入场景。

## 5. 停车扫描、记录与复现

最贴近你们 HC-SR04 + SG90 的例子：

```bash
python examples/scan_stationary.py
```

它会停车、逐角度转动单个超声波模块，等待稳定后的有效采样，再把测距结果打印出来，最后保存 `stationary_scan.png`。

记录和重放一次实验：

```python
from robot_sim import RobotEnvironment, replay

with RobotEnvironment("maps/01_empty_room.json", seed=42) as env:
    env.reset(pose=(1, 1, 0))
    env.start_recording("trial.jsonl")   # reset 后、任何控制命令前

    env.robot.set_motor_pwm(40, 40)
    env.step(0.5)
    env.robot.stop()
    env.step(0.1)
    env.stop_recording()

replayed = replay("trial.jsonl", verify=True)
replayed.render(save_path="replay.png")
replayed.close()
```

录制内容包括地图、配置、种子、运动命令、观测数据和评估真值；重放时重新计算并逐项比对。相同代码及依赖版本下可以得到确定性结果。运行对照示例：

```bash
python examples/sensor_ablation.py
```

它会执行相同的预设命令并生成不同传感器组合的日志；它本身**不会输出定位算法优劣结论**。

评估接口在 `robot_sim.metrics`（也可从 `robot_sim` 直接导入）：

- `pose_errors(truth, estimate)`：对齐的 `N×3` 位姿数组，返回位置 RMSE、最大位置误差和朝向 RMSE。
- `occupancy_iou(truth, estimate)`：对齐的二维占据栅格，返回已知区域的占用 IoU 和已知覆盖比例。

真值只能在**实验评估程序**中读取。你的算法应只从 `RobotPort` 获取传感器数据，否则测出来的定位精度没有意义。

## 6. 测试和故障排查

```bash
# 从 envs/ 运行所有测试
python -m pytest -q

# 生成覆盖率结果
python -m coverage run -m pytest -q
python -m coverage report
```

随仓库提交的 [测试报告](reports/TEST_REPORT.md) 记录了初版在 10 张地图 × 每图 10 个起点 × 每条轨迹 20 段运动下的验证，以及测距的独立 Shapely/GEOS 数学核验。历史记录显示 **90 项 pytest 通过、92.58% 代码语句覆盖率**；这些数字属于初版构建时的本地测试记录，后续更改代码后应重新运行。

常见报错：

| 现象 | 检查 |
|---|---|
| `ModuleNotFoundError: robot_sim` | 是否在 `envs/` 安装了 `pip install -e .` |
| 起始位姿无效 | `reset` 所选位置和整个车身半径是否有足够净空 |
| `step` 或采样频率报错 | 时长 / 采样周期是否为 `physics_dt_s` 的整数倍 |
| `read_sensor(...)` 返回 `None` | 传感器延迟尚未到达，或还未产生已交付样本 |
| 测距 `distance_m=None` | 检查 `status`（超量程、盲区、丢测、舵机尚未稳定） |
| 交互窗口打不开 | 需要桌面显示服务与可用的 Matplotlib GUI 后端；使用 `--save` 无窗口运行 |
| 更换地图后不能运动 | `load_map` 后必须再次 `reset` |

## 7. 边界与源码结构

本项目目前支持静态二维场景、差速小车、简化声呐扇区模型、简化平面 IMU、轮编码器、红外距离、LiDAR 和碰撞开关。**没有完整真实声学反射、材料吸收、多径、三维姿态、轮胎动力学，也没有内置可直接评价的 SLAM / A* 算法。** 数值核验精度不代表实际 HC-SR04 的硬件精度。

```text
envs/
├── README.md                  本文
├── robot_sim/                 核心环境、RobotPort、地图、运动、传感器、评估
├── maps/                      10 张场景地图
├── configs/                   ideal / noisy / 验收初始位置
├── examples/                  算法模板、停车扫描、传感器对照、地图生成
├── docs/                      地图 Schema、模型推导、参考项目调查
├── tests/                     几何、传感器、环境、验收与工具测试
└── reports/                   历史验收记录、可视化和可回放日志
```

技术细节可以继续阅读 [数学与传感器模型](docs/MODELS.md) 和 [参考项目调研](docs/REFERENCE_REVIEW.md)。