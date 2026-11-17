# 建图 / 导航开源项目清单（超声波建图口径）

**平台**：TM4C123G（Cortex-M4F 80MHz，256KB Flash / **32KB RAM**）
**口径**：建图用 **HC-SR04 超声波测距**（配舵机云台逐角度扫描），红外循迹/避障作局部避障；不用摄像头图像 / 二维码 / 视觉识别
**核实**：2026-11-12 走 GitHub REST API + LICENSE 文件直读；2026-11-14 精简，当晚按超声波建图口径订正

---

## 一、起步只用这三个

| 仓库 | 许可证 | 拿什么 | 落到 TM4C 要改什么 |
|---|---|---|---|
| [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) | MIT | `PathPlanning/AStar`、`BreadthFirstSearch`、`Dijkstra`、`DynamicWindowApproach`、`Localization/extended_kalman_filter` 独立脚本 | 只作算法蓝本与测试用例来源；numpy / matplotlib 不进 MCU，矩阵运算手写静态数组 |
| [BigZaphod/AStar](https://github.com/BigZaphod/AStar) | BSD-3-Clause | `AStar.c` + `AStar.h` 共约 29KB 纯 C：二叉堆 open list、索引数组查重、3 个回调解耦地图 | `malloc` 换静态内存池，启发式换曼哈顿距离；栅格与坐标系自己写 |
| [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) | MIT | Madgwick / Mahony AHRS 算 yaw，纯 C、无堆、无 HAL 依赖 | 可直接编进工程，配合编码器做航位推算 |

许可证全宽松、无 ROS 依赖、无动态分配硬依赖。
**RAM 账**：100×100 `uint8` 栅格本体 10KB，加上 A* 的 `g`/`f`/`parent` 数组轻松破 32KB；地图降到 **64×64 或 50×50**，本体放 Flash，RAM 只放搜索窗口。只用单精度 `float`（M4F 无双精度 FPU），编译加 `-fsingle-precision-constant`。

---

## 二、两条架构概念（只看思路，不抄代码）

1. **导航栈分层** —— 来自 [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) 的 costmap：静态地图 / 动态障碍 / 全局规划 / 局部避障**分开存、分开更新**，planner 与 controller 接口解耦。这不只是架构洁癖：32KB RAM 下，不常变的地图放 Flash、RAM 里只维护障碍层，正是省内存的关键。
2. **多层栅格** —— 来自 [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map)（BSD-3-Clause）：占用、代价、访问时间各存一层，用统一索引；比给每个格塞一个大 struct 省得多，也方便只更新被扫到的那一层。

其余 ROS2 / SLAM 完整栈一律不列：navigation2 本体依赖 DDS + Eigen + Boost + 大量 `new`，搬不上 MCU；cartographer 已停更，slam_toolbox 是 LGPL-2.1，betaflight / crazyflie-firmware 是 GPL-3.0（抄进产品有传染风险）。

---

## 三、必须自己补的定位闭环 ⭐

**超声波解决了「看见环境」，但没解决「知道自己在哪」——这正是仍然要买硬件的原因。**

- **观测这环已经有了**：HC-SR04 给的是**距离**（不是红外那种 0/1 开关量），配舵机云台逐角度扫一圈就能得到距离剖面，作为栅格建图的观测输入够用。**测距传感器不用再买**，也不用摄像头 / K210 视觉 / 激光雷达。
- **位姿这环还是空的**：每帧扫描要投进地图坐标系，必须有 `(x, y, θ)`。超声波只回答「障碍离我多远」，不回答「我移动了多少、转了多少」。原厂 `makerobo_run(speed, time)` 是开环占空比 + 定时近似距离，地面摩擦、电池电压、左右轮差异都会让「以为走了 10cm」变成实际 8cm 或 15cm —— 扫几帧地图就糊成一团。
- **必买**：左右轮 **AB 相霍尔编码器** → 正好接 TM4C 的 **QEI0 / QEI1**（硬件正交解码，`QEIConfigure` / `QEIPositionGet` / `QEIVelocityGet`）。
  **建议买**：IMU（MPU6050，I2C）给航向；双轮差分航向在打滑、地面不均时会飘。
  **待确认**：套件是否带舵机云台（资料库有 `9.机器人超声波避障（带舵机摇头）` 案例与 `舵机控制教程.rar`，大概率配了，看一眼发货清单实物）。
- **软件**：`Odom_Update(dt, dL, dR)` 轮速积分 → 互补滤波 / EKF 融合 yaw（Fusion 给 AHRS，[simondlevy/TinyEKF](https://github.com/simondlevy/TinyEKF) 或 [sunsided/kalman-clib](https://github.com/sunsided/kalman-clib) 给矩阵运算，均 MIT 纯 C）→ 栅格地图 → A* → 航点跟踪 + 速度环 PID。
  **编码器驱动和里程计模型没有现成能直接搬的**：轮径、轮距、减速比、每转脉冲数全和车体绑定，只能自己写、自己标定。
- **超声波建图的实际约束**（会反过来加重对里程计的依赖）：波束角约 15°、镜面反射会丢回波、软材料吸收、盲区约 2cm；单次测距几十 ms，扫 180° 每 5° 一测约 36 次要 1~2 秒，这期间**车必须停住**，否则扫描中位姿在变、地图扭曲。所以流程只能是「停 → 扫 → 走一小段 → 再停 → 扫」，而每一小段走多远全靠编码器。
- **资料缺口**：资料库里编码器 / IMU / 里程计 / 栅格地图资料为 **0**；`organized/library/` 只做外设通信，也没有定位模块。

> **先做这一步再谈建图**：装好编码器，让车走 1 米直线，用编码器实测位移反推每转脉冲数与有效轮径，把左右轮偏差标定掉。里程计不准，A* 算得再漂亮，车也只是"看起来在算、其实在瞎跑"——这块我见过太多人跳过了。

---

## 已裁掉（口径：不用视觉）

`dlbeer/quirc`、`sipeed/MaixPy-v1`、`ZBar/ZBar`、`mchehab/zbar`、`opencv/opencv`（视觉 / 二维码）；`FreeRTOS/FreeRTOS-Kernel`（裸机超级循环下每个任务栈都在啃 32KB，等真上多任务再说）；全部 ROS2 / SLAM 完整栈。
