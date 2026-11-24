# 小车二维算法实验环境 · v0.1.0

Python 差速小车仿真平台，面向 TM4C123G 小车的建图、定位与导航算法开发。
**加载地图、放置小车、发送左右轮 PWM、读取传感器。算法通过 `RobotPort` 接口运行。**

已经包含 10 张地图、可配置的 6 类传感器、固定时钟、碰撞检测、噪声、记录与确定性回放。
核心依赖只有 NumPy；Matplotlib 用于可视化，Shapely 仅用于独立测试。

> 精度承诺的范围：验证的是已定义二维数学模型的数值正确性。超声波采用“扇区内最近反射物”模型，未模拟真实声场、材料、镜面反射和多径。不要把数值测试中的浮点误差当作 HC-SR04 的实物精度。

## 1. 安装与运行

建议 Python 3.10+；本次实际测试版本见 [测试报告](reports/TEST_REPORT.md)。在仓库根目录执行：

```bash
cd envs
python -m venv .venv
# Linux / macOS
source .venv/bin/activate
# Windows PowerShell 使用：.venv\Scripts\Activate.ps1
python -m pip install -e ".[visual,test]"

# 无窗口演示：保存图片和可回放日志
python -m robot_sim.demo --map maps/08_irregular.json --pose 0.5 0.5 0 \
  --config configs/noisy.json --steps 350 --save preview.png --record run.jsonl

# 桌面练习场；须在有图形显示和可用 Matplotlib GUI 后端的本机执行
python -m robot_sim.demo --map maps/01_empty_room.json --pose 1 1 0 --interactive

# 逐值验证回放，保存最终画面
python -m robot_sim.demo --replay run.jsonl --save replay.png

# 完整验收（包含 10×10×20 测试）
python -m pytest -q
```

交互键：`W/S` 前后、`A/D` 原地转向，松开方向键停车；空格停车，`[` / `]` 调舵机，`P` 暂停，`Esc` 关闭。模拟时间由固定步长推进，界面卡顿不会扩大物理步长。

在无显示服务器上使用无窗口命令即可。实机键盘窗口没有在本次 Linux 无显示环境中打开；绘图输出与键盘回调经过自动测试。

## 2. 最小调用示例

```python
import math
from robot_sim import RobotEnvironment

sensors = {
    "front": {
        "type": "ultrasonic",
        "frequency_hz": 10,
        "max_range_m": 4.0,
        "min_range_m": 0.02,
        "beam_width_deg": 15,
        "noise_std_m": 0.0,
        "mount": [0.06, 0.0, 0.0],  # 车身坐标：前方 6 cm
        "servo": True,
    },
    "imu": {"frequency_hz": 100, "gyro_noise_std_rad_s": 0.005},
    "encoder": {"enabled": False},
}

with RobotEnvironment("maps/01_empty_room.json", sensors=sensors, seed=42) as env:
    env.reset(pose=(1.0, 1.0, 0.0))
    bot = env.robot
    bot.set_motor_pwm(left=40, right=40)  # [-100,100]，持续保持
    env.step(0.20)                       # 前进 20 个 10 ms 物理步
    print(bot.read_sensor("front"))
    print(bot.read_sensor("imu"))

    bot.stop()
    bot.set_servo("front", math.radians(30))
    env.step(0.30)                       # 舵机需要转动、稳定和等待采样
    print(bot.read_sensor("front"))
    env.render(save_path="frame.png")
```

`reset(random_spawn=True, seed=73)` 会随机选择**整个圆形车体都不碰撞**的位置；同一个种子重复 reset 会得到相同位置。更换种子才能得到不同随机场景。

`env.load_map("maps/06_doorway.json")` 更换场景后，必须再次 `reset(...)`。`reset` 清零时间、运动状态、传感器缓存和累计编码器计数；录制期间禁止 reset 或换图。

## 3. 接口分工

| 实验程序 `env` | 算法接收的 `env.robot` |
|---|---|
| `load_map(source)`，支持路径、字典、WorldMap | `set_motor_pwm(left, right)` |
| `reset(pose=...)` / `reset(random_spawn=True)` | `stop()` |
| `step(duration_s)`，省略参数推进一个物理步 | `set_servo(name, angle_rad)` |
| `ground_truth()`，仅用于评估 | `read_sensor(name)` |
| `render(save_path=..., show=False, estimated_pose=...)` | `get_observations()` |
| `start_recording(path)` / `stop_recording()` | `sensor_names` / `time_s` |
| `close()` / 上下文管理器 | 无真实位姿、真值地图和瞬移接口 |

控制器接收 `RobotPort`，仿真器由外层实验程序管理，见 [算法模板](examples/algorithm_template.py)。Python 私有属性隔离是工程约定，无法抵御刻意的运行时反射。

真实小车接入时，另写实现同一接口的硬件适配器，将 PWM/传感器读写转换为通信协议。Python 算法可继续放在上位机；在 TM4C 上执行算法仍需要对应的 C 实现。本版本没有连接任何真实电机，也没有修改原仓库固件。

## 4. 传感器可自由组合

以字典的键作为传感器实例名，`type` 表示类型；同一类型可创建多份，例如前、左、右三个声呐。`sensors=None` 启用默认超声波、IMU、编码器、碰撞开关；`sensors={}` 全部禁用。`enabled=False` 单独禁用。

| 类型 | 输出 | 主要参数 |
|---|---|---|
| `ultrasonic` | 距离、回波时间、状态、名义朝向 | 范围、波束宽度、噪声、丢测、舵机范围/速度/稳定时间 |
| `imu` | 三轴比力，平面 Z 轴角速度 | 采样率、角速度噪声/零偏/随机游走、加速度噪声、安装位置/角度 |
| `encoder` | 左右累计计数和相邻采样增量 | 每转总计数 `ticks_per_rev`，已经包含 AB 相四倍频 |
| `lidar` | 多方向测距与逐束状态 | `angles_rad`、量程、噪声、丢测 |
| `infrared` | 简化近距距离与状态 | 量程、波束角、噪声、丢测；不输出器件模拟电压 |
| `bumper` | 整车接触开关 | 采样率；锁存采样之间的碰撞事件 |

通用配置：`frequency_hz`、`latency_s`、`mount=[x_m,y_m,yaw_rad]`。安装位置必须在车体圆形轮廓内。编码器、碰撞开关按整车定义，安装参数固定为零。

参数默认值与检查在 [config.py](robot_sim/config.py)。未知参数、无效范围、NaN、不可支持的采样率都会报错，避免悄悄忽略拼写错误。

### 观测格式与时间

```json
{
  "sensor": "front", "type": "ultrasonic", "sequence": 2,
  "timestamp_s": 0.2, "available_at_s": 0.23,
  "data": {"distance_m": 1.24, "status": "hit", "bearing_rad": 0.0, "echo_time_s": 0.00723032}
}
```

这是格式示例；`echo_time_s` 在实际运行时由同一次距离样本计算。

`timestamp_s` 是采集时间，`available_at_s` 是延迟后的可读时间。`sequence` 用于去重。第一次数据尚未到达时返回 `None`。重复读取返回同一份样本的独立副本，**不会重新抽取噪声**。

测距状态：`hit`、`no_echo`、`below_min`、`dropout`、`servo_unsettled`。除 `hit` 外距离均为 `None`，不能把它们写成“前方空闲”或“最大距离处有墙”。

声呐 `bearing_rad` 表示安装方向加舵机指令角；不包含车体真实世界朝向，不提供反射点在声束内的真实角度。

默认物理步长 0.01 s。采样周期和延迟必须是其整数倍，例如 100/50/20/10 Hz。30 Hz 在该步长下会明确报错；可改用支持它的物理步长，例如 1/300 s，同时重新检查所有周期。

## 5. 地图格式和实测图导入

采用有版本号的 JSON。地图可包含实体多边形、零厚度墙段或占据栅格。`bounds` 的四条边始终是实体外墙。

```json
{
  "version": 1,
  "units": "m",
  "name": "my_room",
  "bounds": [0, 0, 4, 4],
  "polygons": [[[1.5, 1.5], [2.0, 1.5], [2.0, 2.2], [1.5, 2.2]]],
  "walls": [[[3.0, 0.5], [3.4, 1.5]]]
}
```

多边形末尾不要重复首点。支持凹多边形；拒绝自交、多边形面积为零、越界坐标和无效墙段。几何/尺寸约束由运行时进一步检查，结构定义见 [map.schema.json](docs/map.schema.json)。

**实测建图最直接的入口：**

```python
from robot_sim import WorldMap
# cells[row, col]：-1 未知；0..100 占用概率百分数。
world = WorldMap.from_occupancy(cells, resolution_m=0.05,
                                origin_m=(-1.0, -1.0),
                                unknown_policy="occupied")
world.save("measured_map.json")
```

栅格第 0 行位于地图底部，列向右、行向上。原点为左下角，单位为米；默认占用阈值 65。保留原始栅格值，模拟碰撞和回波时按阈值生成实体。已知概率低于阈值的格子按空闲处理；未实现概率占据随机采样。

未知格默认拒绝进入真值仿真，调用者必须显式选择 `occupied` 或 `free`。选择 `free` 是实验假设，会掩盖未探测障碍，必须在实验记录中说明。输入数据中的 -1 仍保留，方便地图评估。

```bash
# -1 / 0..100 百分数矩阵
python -m robot_sim.convert measured.npy measured.json --resolution 0.05 --unknown-policy occupied
# -1 / 0 / 1 二值矩阵
python -m robot_sim.convert measured.csv measured.json --resolution 0.05 --encoding binary
# -1 或 [0,1] 概率矩阵
python -m robot_sim.convert measured.npy measured.json --resolution 0.05 --encoding probability --unknown-policy occupied
# 图片：黑色为障碍，白色为空闲，中间灰度未知；自动翻转图片行坐标
python -m robot_sim.convert measured.pgm measured.json --resolution 0.05 --unknown-policy occupied
```

CSV/NPY 的行若来自图片坐标，添加 `--top-to-bottom`；PNG/PGM 已自动翻转，不需要该选项。图片导入只有比例尺与平移原点，不自动识别照片透视或地图旋转。

**原始超声波距离日志需要先经过你们的建图算法，得到带坐标的栅格。转换工具不承担 SLAM。**

## 6. 可重复实验与评估

```python
from robot_sim import replay, pose_errors, occupancy_iou

env.reset(pose=(1, 1, 0), seed=42)
env.start_recording("trial.jsonl")  # 必须在任何运动/舵机指令前
# ... 通过 env.robot 发指令，通过 env.step 推进 ...
env.stop_recording()
restored = replay("trial.jsonl", verify=True)
```

日志包含地图内容和哈希、配置、种子、初始位姿、控制命令、观测和评估真值。回放重新执行指令并逐值比对输出。确定性保证限定于相同代码和运行依赖版本。

每个传感器的随机流由“全局种子 + 实例名”独立生成；更换传感器组合不会扰动原有实例的噪声序列。随机放置使用另一条随机流。

`pose_errors(truth, estimated)` 计算位置 RMSE、最大位置误差和包角后的航向 RMSE；要求估计和真值在同一坐标系、同一时间点。`occupancy_iou` 仅比较双方都已知的对齐栅格，并返回已知覆盖率。

[三组合对照示例](examples/sensor_ablation.py) 执行同一指令序列并生成日志。它没有预设“编码器一定更好”的结果；定位质量需要你们把算法估计值接入评估函数。

## 7. 模型边界

本版本采用平地、静态二维环境、圆形车身和差速模型。可以配置左右电机增益、响应时间和固定打滑比例；没有完整轮胎动力学、悬架或电机电流模型。

碰撞使用扫掠检测和圆弧弦误差保护，防止高速穿墙。接触时保守停在上一个物理步的位置；不处理接触滑动、弹跳、推箱子。编码器仍记录轴转动，便于测试“轮子转了但车没走”。

IMU 不返回完美航向或位置；水平静止时加速度计比力为 `[0,0,+g]`。角速度仅模拟平面 Z 轴，俯仰、横滚及对应噪声未建模。加速度采用离散时间差分和转动项，碰撞冲击波形不具备物理保真度。

LiDAR 一帧按同一时刻采集，不模拟一圈扫描中的运动畸变。超声波传播时长作为输出给出，场景几何在采样时瞬时求值；`latency_s` 为固定交付延迟，不自动等于往返声时。

本版本提供实验环境和评估工具，不包含已经验证的 SLAM、A* 导航系统或硬件固件。导航成功率须在接入相应算法后单独评估。

## 8. 目录

```text
robot_sim/     环境、传感器、几何、地图、转换、可视化、指标
maps/          10 张实际 JSON 地图
configs/       理想/噪声配置、100 个验收起点
examples/      控制器模板、停车扫描、传感器组合对照、地图生成
 tests/        数学基准、GEOS 独立校验、运动/时序/回放、完整验收
reports/       实际测试输出、验收明细、逐段观测日志、示例图
 docs/         地图 Schema、数学模型、调研依据
```

调研依据与实现取舍见 [REFERENCE_REVIEW.md](docs/REFERENCE_REVIEW.md)。
