# 车端控制协议（v1、v2 和 v3）

公开演示固件使用车载热点。手机通过车载 ESP-01S 的 WPA2 热点打开 TCP 连接。默认地址 `192.168.4.1:7070`。TM4C 通过 UART1 管理热点、解析 ESP 的 `+IPD` 长度帧，使用 `AT+CIPSEND` 回传遥测。第一个连接占用控制权；其他连接的数据被忽略。ESP 需使用正常启动的 AT 固件，默认 AP 地址与 DHCP 配置应可用。[ESP8266 AT TCP 服务端示例](https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Examples/TCP-IP_AT_Examples.html)

每条请求都是以 LF 结尾的 ASCII 行，字段间单空格。新连接第一条发 `0 S\n`。客户端一次发一个请求，等待相同序号的响应后再发下一条；App 约每 100 ms 加一个往返延时刷新。序号为 0–65535，循环使用。车端按最新有效请求执行，650 ms 未续约时撤销运动；停止后重新操作。无效请求撤销运动，不产生正常响应。App 收不到完整遥测会关闭连接。

| 请求例子（行尾 LF） | 用途 |
|---|---|
| `1 S` | 停车，同时读取前、左、右测距及估计位姿 |
| `2 F` | 受避障保护的低速前进 |
| `3 L`、`4 R` | 受限原地左转、右转 |
| `5 G 800 -200` | 追踪一个车端坐标系中的目标，单位 mm，示例为初始朝向前方 800 mm、右侧 200 mm。两项均为整数，绝对值上限 1000000 |

手机保存整条路线，把地图坐标经当前定位修正逆变换成车端坐标，逐段发送 G。车端按目标方位控制左右轮，进入目标附近停车。手机可以随新障碍更换下一目标；断线后需要重新启动任务。手动按住方向键时也持续续约，松开即发 S。

```text
T seq state x y heading   tF mmF xF yF aF   tL mmL xL yL aL   tR mmR xR yR aR
```

上面空列仅用于分组，线上为单空格。state 依次为 0 待命、1 正在驱动轮子、2 障碍停车、3 回波缺失或过期、4 指令超时停车、5 SW1 停车。state 反映最近控制周期，不能视为电机运动反馈。每束的 t 为测量完成时板端毫秒计数（32 位回绕），mm 为距离，−1 表示无有效回波；x/y 和 a 是采样时该束的估计原点 mm 与绝对朝向度。车端 x 轴沿上电时的车头，y 轴向左，正角逆时针。App 利用时间标记去重，超时回波不会被写成“前方畅通”。

端到端演示使用同一套请求解析、遥测序列化及运动控制 C 代码。PC 模拟器直接提供 TCP 端口，UART 的分包、提示符、应答及断连另由 AT 对话测试覆盖。

## 扩展请求与日志（v2）

同一帧在动作前插入 `V ` 即选择扩展格式。握手示例为 `1 V S`，续约示例为 `2 V F`。手机在旧固件上首次握手超时会尝试普通停车握手。专用固件让 ESP 以 Station 身份加入手机热点，通过 `AT+CIPSTA?` 取得网关，主动连往网关 TCP 7071；同网段的手机也可以主动连 ESP 的 7070。两种连接使用同一套请求和响应。[ESP-AT Station 地址查询](https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/Wi-Fi_AT_Commands.html#at-cipsta-query-set-the-ip-address-of-an-esp-station)

| 命令 | 单位与行为 |
|---|---|
| `17 V D 9 500` | 任务 9，从本次起点前进 500 mm（双轮固件按车体中心路程判定），支持 10–10000 mm |
| `18 V A 10 -90` | 任务 10，从当前航向右转 90°，支持 ±1–360° |
| `19 V S` | 撤销任务、停车并读取遥测 |

任务号为 1–65535。同一次任务使用相同动作、任务号和目标，仅递增帧序号；连续续约不会重置起点，达到目标后保持停车。新任务使用新任务号。超时、SW1、反馈故障后先发 S，再发新的动作。状态 6 表示定量任务完成，7 表示反馈故障（IMU 未就绪、码盘无进展或定量任务超时等）。状态 8 表示停车输出复写后仍读到高电平，锁定驱动直至复位。v1 将状态 6 映射为待命，将状态 7、8 映射为测量无效。

扩展响应保留原 21 个字段，在换行符前追加 16 个整数：

```text
board_ms pulses wheel_mm speed_mm_s imu_ready task_id ax ay az gx gy gz event_id event_ms event_code log_failed
```

- 板上时间和脉冲计数为 32 位无符号数。wheel_mm 为自启动累计的左轮路程，speed_mm_s 为约 250 ms 窗口的无符号平均左轮速度。航向字段 heading 为相对启动方向的整度数，左转为正。位姿在原地转弯时按车体中心近似不动。
- 六轴字段按加速度 XYZ、角速度 XYZ 排列，均为有符号原始读数。量程 ±2 g 和 ±250°/s，分别除以 16384 和 131 得到 g 和 °/s。imu_ready 为 0 时，原始值可能未更新。[MPU-6050 寄存器说明](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)
- 每次连接从 Flash 中最旧的有效事件开始补传，随后传新事件。event_id 为 0 表示本帧无事件。可按同一车辆的事件号去重。event_ms 以该事件发生时的那次上电为起点。事件码 1 启动、2 控制连接关闭、3 续约超时、4 SW1 停车、5 运动反馈故障、6 遇障停车、7 停车输出复写后仍异常。log_failed 为 1 表示本次启动中持久化写入失败。
- 手机 TSV 每行依次为手机 Unix 毫秒、帧序号、下发命令和完整响应，用制表符分隔。原始遥测保留单空格分隔，便于按协议重放。

## 双轮遥测（v3）

握手及后续请求使用 `W` 前缀，例如 `1 W S`、`2 W D 1 100`。每次只保留一笔未确认请求。App 先以 W 握手，旧固件无响应时依次尝试 V 和 v1。旧客户端可继续使用原 21/37 个字段。W 的前 37 个字段与 V 相同，随后依次追加 27 项：

```text
right_pulses right_mm right_mm_s left_rpm10 right_rpm10 center_mm_s center_mm
pwm_left pwm_right applied_action target imu_ms bearing_front bearing_left bearing_right servo_target
fault uart_errors radio_resets invalid_requests max_control_interval_ms
trace_id trace_ms trace_kind trace_a trace_b trace_skipped
```

- 左轮脉冲、里程、速度沿用 V 字段。每轮 RPM 为码盘所在车轮轴速率幅值，两个 rpm10 字段除以 10 得 RPM。编码器每轮独立标定。左右同向时中心路程按两轮增量的均值，命令原地旋转时中心近似不动；车轮方向由驱动指令推断。无驱动方向信息时手推只累加无符号路程。上电清零。
- PWM 是最近控制周期的有符号百分比，applied_action 为车端当前命令。target 为最近定量任务的参数，D 用 mm，A 用度。手机 TSV 中包含完整下发参数。IMU 时间戳为板端毫秒。bearing 和 servo_target 是经安装标定的车体系指令角，实际舵轴角度无独立传感器。每束原有 heading 字段为采样时世界系波束朝向。
- fault 依次为 0 正常、1 IMU 未就绪、2 左轮无脉冲、3 右轮无脉冲、4 任务超时或缺少反馈、5 转角反向、6 停车电平异常。超声、SW1 和失联停车见 state。uart_errors 计 UART/缓冲异常恢复次数；其后为无线链路重置（含启动）、无效请求数和启动以来控制间隔最大值。
- RAM 环保存最近 128 条事件，每帧从旧到新回传 1 条，trace_id 为 0 表示无事件。类型 1 命令变更（a 为 ASCII），2 控制状态（a 为 state，b 为 fault），3 IMU 就绪变化（a 为 0/1），4 无线重置（阶段、步骤），5 无效请求（长度、连接号），6 UART 恢复，7 TCP 连接变化（连接号、开闭）。trace_skipped 标记覆盖的条数，新连接可能重放同一批事件。Flash 事件仍有独立 ID。
- 手机正常遥测行保留原 TSV 格式；以 `#` 开头的命令列为手机事件，包括 TX 请求、STATUS、SMART、DECISION、COMPLETE 和 GAP。文件完整保留已记录的历史，日志写入失败或缓冲溢出后报告缺口。

手机智能模式使用[系统提示词](android/assets/system-prompt.txt)。ADB 工具输出包含单位的命名 JSON 字段、决策票据和局部地图，API 模式使用相同观测。手机给 LLM 的六轴数据已换算为 g 与 °/s，导出的原始帧保留芯片读数。
