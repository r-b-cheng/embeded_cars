# clib — 小车外设通信函数库 v1.0.0

> **这个库解决的问题**：以后不用再重复写红外解码、舵机控制，以及"怎么给外设发控制消息 / 怎么收外设送来的数据"这些板级代码。
> 从厂商 STM32 案例里抽出的 **A 级（纯 C 逻辑）+ B 级（换 HAL 即可复用的通信逻辑）**，统一封装成跨平台库。
> **换主控到 TM4C123G 时，只改 `ports/` 下那一个函数指针表，`include/` 和 `src/` 一行都不用动。**
>
> 本库经过四路子代理并行代码审查并逐项修复，详见 [`代码审查报告.md`](代码审查报告.md)。

---

## 1. 目录结构

```
organized/library/
├── README.md                    ← 本文件
├── 代码审查报告.md               ← 四路并行审查的发现、修复状态与不采纳理由
├── Makefile                     ← make test（PC 端编译 + 跑 200 个断言）
├── include/                     ← 对外头文件（芯片无关，只依赖 C 标准库，含 C++ 包装）
│   ├── clib_common.h            公共类型 / 错误码 / 逻辑引脚 / 动作枚举
│   ├── clib_port.h              ★ 平台适配层接口（唯一需要随芯片改的地方）
│   ├── clib_frame.h             通用串口帧：组帧 / 发送 / 逐字节解析 / 串口轮询
│   ├── clib_servo.h             舵机：角度→脉宽、发送控制消息、非阻塞多方向扫描
│   ├── clib_motor.h             L298N：动作→4 路占空比、持续速度指令
│   ├── clib_ir_remote.h         红外遥控：脉宽解码、两套键码表、GPIO 采样降级方案
│   ├── clib_ir_sensor.h         红外循迹/避障：2bit 状态、极性、去抖、决策查表
│   ├── clib_ultrasonic.h        HC-SR04：触发脉冲、回波状态机、距离换算
│   ├── clib_vision_k210.h       K210 视觉：5 字节帧组帧/解析、丢线判定、阈值决策
│   └── clib_cmd.h               蓝牙/串口文本指令：解析、应答、GOTO 坐标
├── src/                         ← 与上面一一对应的实现（纯 C99）
├── ports/
│   ├── mock/                    PC 端假硬件（单元测试用，已完整实现）
│   ├── tm4c123g/README.md       TM4C123G 移植说明 + TivaWare 代码骨架
│   └── stm32f103/README.md      回迁 STM32 的对照说明
├── tests/test_all.c             10 组测试 / 200 个断言，覆盖全部模块与端到端链路
└── build/                       make 生成的测试程序（可随时删）
```

## 2. 三层架构

```
┌─────────────────────────────────────────────────────────┐
│ app（你的导航程序）                                       │
│   调度、状态机、里程计、A*、航点跟踪  ← 本库不负责        │
├─────────────────────────────────────────────────────────┤
│ device 层 = include/ + src/                              │
│   servo / motor / ir_remote / ir_sensor / ultrasonic /    │
│   vision_k210 / frame / cmd                              │
│   ★ 纯 C99，零芯片依赖，换板子一行不改                   │
│   职责：把「动作/角度」变成控制消息，把「电平/字节」变成数据 │
├─────────────────────────────────────────────────────────┤
│ port 层 = clib_port_t 函数指针表                          │
│   ★ 唯一随主控改动的一层                                 │
│   millis/micros/delay_us, gpio_read/write,               │
│   motor_set_duty, servo_set_pulse_us, uart_write/read     │
└─────────────────────────────────────────────────────────┘
```

库代码里只认**逻辑引脚**（`CLIB_PIN_TRACK_L`…）和**逻辑串口**（`CLIB_UART_BT`…），
物理管脚号（PB6 / PE4 / PC6…）全部由 port 层映射，所以原理图换了也不影响库。

## 3. 快速开始（3 步）

```c
#include "clib_port.h"
#include "clib_servo.h"
#include "clib_motor.h"
#include "clib_ir_remote.h"

/* 1) 实现/引入 port（TM4C 版本见 ports/tm4c123g/README.md） */
extern const clib_port_t my_board_port;

int main(void) {
    /* 2) 注册一次，之后全库可用 */
    clib_init(&my_board_port);

    /* 3) 直接发控制消息，不用再写 PWM / 红外 / 串口驱动 */
    clib_servo_set_angle(90.0f);              /* 舵机转正中：自动换算 1500us */
    clib_motor_set(CLIB_ACT_FORWARD, 70);     /* 四路 PWM 自动映射并输出 */
    /* ... 主循环里用 clib_irdec_feed_pulse() / clib_k210_poll() 收数据 ... */
}
```

**端到端示例：红外遥控 → 差速电机**（`tests/test_all.c` 里已验证）

```c
clib_irdec_t dec;  uint32_t raw = 0;
clib_irdec_init(&dec);
/* 输入捕获中断里拿到一个高电平脉宽，直接喂： */
if (clib_irdec_feed_pulse(&dec, high_us, &raw) == 1) {
    clib_act_t act = clib_ir_lookup(raw, CLIB_IR_KEYMAP_V1, CLIB_IR_KEYMAP_V1_LEN);
    clib_motor_set(act, 70);
}
```

**端到端示例：舵机三向扫描 + 超声波测距**（替代原案例的阻塞写法）

```c
static const float angles[3] = { 90.0f, 175.0f, 5.0f };
clib_servo_scan_t scan;  clib_ultra_t ultra;  float angle, cm;
clib_servo_scan_begin(&scan, angles, 3, 300 /*ms 稳定时间*/);

/* 在主循环里非阻塞 tick，绝不用 Delay_ms 卡住控制环 */
int r = clib_servo_scan_tick(&scan, &angle);
if (r == 1)        { clib_ultra_start(&ultra, 30000); }  /* 切好角度 → 发触发脉冲 */
else if (r == CLIB_SERVO_SCAN_FINISHED) { /* 三向扫完，开始规划 */ }
if (clib_ultra_poll(&ultra, &cm) == 1) { /* 拿到该方向距离 cm */ }
```

## 4. 移植到 TM4C123G（只需实现 `clib_port_t`）

| 库要的回调 | TivaWare driverlib 对应写法 |
|---|---|
| `millis` | SysTick 中断维护毫秒计数 |
| `micros` | **直读 GPTM 32 位自由计数**（80MHz 下 1us 中断不可承受，别这么做） |
| `delay_us` | `ROM_SysCtlDelay(us * (SysCtlClockGet()/3000000UL))`，3 时钟/循环；是 80MHz 不是 72MHz |
| `gpio_read` | `GPIOPinRead(port, pin)` + `GPIOPadConfigSet(... GPIO_PIN_TYPE_STD_WPU)`（上拉输入） |
| `gpio_write` | `GPIOPinWrite(port, pin, ...)` |
| `motor_set_duty` | `SysCtlPWMClockSet(SYSCTL_PWMDIV_1)`（**必须显式设，复位值不是 /1**）+ `PWMGenConfigure`、`PWMGenPeriodSet`（20kHz）+ `PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0..3, ...)` |
| `servo_set_pulse_us` | 舵机单独占 Gen2（**与电机 20kHz 不能共用时基**）：`PWMGenPeriodSet(..., SysCtlClockGet()/50)`，按 `us` 折算比较值 |
| `uart_write` | 建议 FIFO/中断 + 环形缓冲**非阻塞**；`UARTCharPut` 阻塞版在 9600 下会拖垮控制环 |
| `uart_read` | `UARTCharGetNonBlocking` + 环形缓冲；中断开 `UART_INT_RX \| UART_INT_RT`（RT 是 **FIFO 接收超时**，需先使能 FIFO，作用近似但不等同于 STM32 的总线 IDLE） |
| `log_putc` | 可为 NULL；或接到 UART0（PA0/PA1 板载 ICDI 虚拟串口） |

**逻辑引脚 → 建议管脚**（务必对照底板原理图 + TI PinMux Utility 确认复用号，这是建议不是定论）

| 功能 | 原 STM32 | 建议 TM4C123G |
|---|---|---|
| 电机 PWM×4 | TIM4 CH1~4 / PB6~9 | PB6/PB7(M0PWM0/1 Gen0)、PB4/PB5(M0PWM2/3 Gen1) |
| 舵机 50Hz | TIM3_CH2 / PB5 | PE4(M0PWM4 Gen2) |
| Trig / Echo | PB15 / PB14 | PA2(out) / PB2(T3CCP0 输入捕获) |
| 循迹 L/R、避障 L/R | PB13/12、PA11/12 | PE2/PE3、PE0/PE1 |
| 红外遥控 | PA8 | PA3（双边沿中断） |
| 蓝牙 / K210 | 都挤在 USART3 | 分开：蓝牙 UART1、K210 UART3(PC6/PC7) |

> 详见 `ports/tm4c123g/README.md`，含完整代码骨架、16 位捕获溢出处理、GPIO 解锁（PF0/PD7 需 `HWREG(base+GPIO_O_LOCK)`，PC0~3 是 JTAG 复用另一回事）、5V 电平等踩坑清单。

## 5. API 速查

### 5.1 向外设发送控制消息

| 函数 | 作用 |
|---|---|
| `clib_servo_set_angle(float)` / `_cal()` | 角度→脉宽→port 层 PWM（自动钳位；自定义标定支持 min/max_us） |
| `clib_servo_set_pulse(uint16_t)` | 直接给脉宽，带 500~2500us 全局安全钳位 |
| `clib_servo_scan_begin/tick` | 非阻塞多方向扫描，扫完返回 `CLIB_SERVO_SCAN_FINISHED` |
| `clib_motor_set(act, speed)` / `clib_motor_stop()` | 动作→4 路占空比→port 层 PWM（**持续输出，不自动停车**） |
| `clib_ultra_trigger()` | 发 20us TRIG 脉冲 |
| `clib_frame_build/send` | 组帧（头+负载+可选校验+尾）并经串口发出 |
| `clib_k210_build()` | 组 K210 5 字节帧（K210 侧参考 / 测试造数据） |
| `clib_cmd_send/send_ack` | 发送文本控制消息与应答（**手写拼接，不链入 snprintf**） |

### 5.2 从外设接收数据

| 函数 | 作用 |
|---|---|
| `clib_irdec_feed_pulse/tick` | 喂高电平脉宽，解出 32bit 红外码（推荐由输入捕获中断调用） |
| `clib_ir_sampler_poll` | 无捕获中断时的 GPIO 电平采样降级方案 |
| `clib_ir_lookup` | 码→动作查表（两套码表 `V1`/`V2` 按实物选） |
| `clib_ir_sensor_read/read_at` | 循迹/避障 2bit 状态，支持极性与去抖 |
| `clib_track_decide` / `clib_avoid_decide` | 状态→动作查表（纯函数） |
| `clib_ultra_poll` | 回波状态机，返回 cm（带超时，永不卡死） |
| `clib_echo_us_to_cm/mm`、`clib_echo_count_to_cm` | 脉宽→距离（标准物理公式） |
| `clib_echo_count_to_legacy` / `clib_echo_legacy_measure` | 复现原厂两个中间量，仅作兼容对照（**单位不是 cm**） |
| `clib_k210_feed/poll` | 逐字节 / 从串口解析 5 字节视觉帧 |
| `clib_frame_feed/feed_buf/poll` | 通用帧逐字节 / 批量 / 串口轮询 |
| `clib_cmd_parse` / `parse_goto` | 蓝牙/串口指令与 GOTO 坐标解析 |

> ⚠️ `clib_frame_feed/feed_buf/poll` 多了一个 `payload_cap` 参数：库**不会**写超过调用方给的容量，
> 容量不足返回 `CLIB_ERR_OVERFLOW`。请传 `sizeof(你的缓冲)` 或 `CLIB_FRAME_PAYLOAD_MAX`。
> 三个 `poll`/`feed_buf` 相关函数单次最多处理 16 字节（`CLIB_FRAME_POLL_BUDGET` /
> `CLIB_K210_POLL_BUDGET`），串口持续来噪声时也能按时把 CPU 还给控制环。

### 5.3 并发 / ISR 约定（重要）

- **一个状态实例只归一个上下文**。`clib_irdec_t`、`clib_k210_t`、`clib_frame_t`、
  `clib_ultra_t`、`clib_ir_sensor_t`、`clib_servo_scan_t` 内部都是非原子的读-改-写，
  库没有锁；同一个实例不要同时在 ISR 和主循环里操作。
- **上电后单线程 `clib_init()` 一次，运行期不再改 port**（`s_port` 已加 volatile）。
- ISR **可用**：所有纯函数（`clib_diff_map`、`clib_*_decide`、`clib_servo_angle_to_pulse`、
  `clib_echo_us_to_cm`、`clib_k210_build`、`clib_ir_lookup`、`clib_cmd_to_act` …）、
  `millis`/`micros`/`gpio_read`/`gpio_write`，以及**由该 ISR 独占实例**的
  `clib_irdec_feed_pulse` 与 `clib_k210_feed`。
- ISR **禁止**：`delay_us`、`uart_write`、`log_putc`（阻塞）；
  `clib_ultra_trigger`（含 20us 忙等）、所有 `*_poll` / `clib_frame_feed*`（调 millis 回调、有 memset）、
  `clib_motor_set*` / `clib_servo_set*`（改输出、含除法/浮点）。
- 唯一可变全局量是 `clib_port.c` 里的 `s_port`（4 字节指针），其余全是 `const` 表或调用方实例。

## 6. 与原案例的逐项对照

| 库模块 | 原文件（相对 `3.案例程序/ZY10A-STM32/`） | 抽出的东西 |
|---|---|---|
| `clib_servo` | `9.../Hardware/Servo.c`、`ServoPWM.c`；`2-38 角度舵机实验/Hardware/PWM.c` | `angle/180*2000+500` 换算、50Hz 周期、三向扫描 |
| `clib_motor` | `1.机器人基本运动/Hardware/robot.c`、`PWM.c` | 6 种动作→4 路占空比映射表、0~100 限幅、20kHz 时基（交给 port） |
| `clib_ir_remote` | **案例 6** `User/main.c` 的 `Irscan()` / `ControlCar_Ircontrol()`、`Hardware/Incontrol.c` 的 `IRremote_Counttime()`；**案例 4** `User/main.c` 提供另一套按键码表（其 `Incontrol.c` 的中断解码体整段被注释、不可用） | 20us tick 判定阈值、32bit 拼装、两套键码表、`code>>8` 拆键 |
| `clib_ir_sensor` | `6.../Hardware/Irtracking.c`、`Irobstacle.c` 与 `main.c` 的 `Robot_Traction/Avoidance` | GPIO 读取、2bit 状态、两组决策查表 |
| `clib_ultrasonic` | `9.../Hardware/UltrasonicWave.c`、`timer.c` | 20us TRIG、Echo 脉宽计数、测距换算（改为标准公式） |
| `clib_frame` | `10.../Hardware/Uart3.c`、K210 `STM32 巡线程序/Hardware/Uart3.c` | RXNE+IDLE 收帧思路（改为软件状态机）、串口发送 |
| `clib_cmd` | `10.../User/main.c` | `ONA/ONB/ONC/OND/ONE/ONF` 指令表 |
| `clib_vision_k210` | `K210.../User/main.c` 的 `canmv_Receive_Data()`、`K210程序/robot_LinePatrol_4wd.py` | 5 字节帧格式、状态机、丢线=53、阈值决策 |

## 7. 相对原厂代码修掉的问题

| # | 原问题 | 库里的处理 |
|---|---|---|
| 1 | `makerobo_run(speed,time)` 延时后自动停车 + `Delay_ms` 阻塞，导航控制环跑不起来 | 改成**持续速度指令**，停车必须显式 `clib_motor_stop()`；舵机扫描也改成非阻塞 tick |
| 2 | 原厂仅 `makerobo_run()` 在 `robot.h` 声明 `uint8_t` 而 `robot.c` 定义 `int8_t`（其余 5 个动作两处都是 int8_t），>127 会隐式截断 | 统一 `int16_t`，限幅在函数内部完成（编译时 `-Werror` 抓出来并已修） |
| 3 | 超声波中断里 `while(GPIO_Read...)` 忙等无超时，没回波直接卡死 | 非阻塞状态机 + `timeout_us`，返回 `CLIB_ERR_TIMEOUT` |
| 4 | 原测距 `cnt*5*34/200.0`（=`cnt*0.85`），而 `timer.c` 的 TIM2 实际是 72MHz/7200=10kHz → **1 计数 ≈ 100us**；按此物理正确值应 ≈ `cnt*1.715cm`。原式常数 5 与时基对不上，后面 `StartMeasure()` 又 `*10` 返回 `cnt*8.5`，案例 9 阈值 60 判的是这个量纲不明的数 | 改标准公式 `us*0.0343/2`；另留 `clib_echo_count_to_legacy()` 与 `clib_echo_legacy_measure()` 复现原厂两个中间量供对照，**调用方须自行确认单位** |
| 5 | 红外 `Irscan()` 局部变量 `Data` 未初始化，无效脉宽沿用上次 bit → 误码 | 无效脉宽直接忽略，不推进 bit |
| 6 | 红外移位在 `Num>=32` 判断之前，实际收 33 位 | 严格 32 位 |
| 7 | `ControlCar_Ircontrol()` 的 `buf[2]` 未初始化 → 没遥控时误动作 | 解码器显式状态，未知键一律 `CLIB_ACT_STOP` |
| 8 | K210 解析 `RxBuffer1[5]` 却循环写 10 字节（越界） | 固定 5 字节缓冲，`pos` 严格边界检查 |
| 9 | K210 `struct.pack("<bbhb")` 用有符号 `b` 承载 `0xAA/0xAE`(>127)，标准 struct 会报错 | `clib_k210_build()` 按无符号帧头 `<BBhB` 语义组帧 |
| 10 | `Uart3.c` 的 `buff[100]` 无越界保护、依赖 STM32 IDLE 中断 | 通用帧解析器带容量上限、输出缓冲 cap 检查、超时复位、可选 XOR/SUM8 校验，不依赖任何芯片中断 |
| 11 | 传感器同一循环重复读 4 次、无防抖 | 一次采样 + 可配置去抖 + 极性反转（`active_low=false` 才与原案例两张决策表一致） |
| 12 | `UltrasonicWave_Distance` 中断写主循环读，无 volatile / 临界区 | 状态机内部状态，无跨上下文共享裸变量 |
| 13 | 原 `Servo_SetAngle()` float→uint16 截断；库用四舍五入 | 库更精确，非整数角度最多差 1us（已在源码注明不是逐位等价） |
| 14 | 原避障 (0,0) 用 `brake(400)+Spin_Right(70,900)` 阻塞 | 合并成持续 `SPIN_RIGHT`，时长交给调用方调度 |

## 8. 编译与测试

```bash
cd organized/library
make test
# 或从任意目录： make -f <本 Makefile 绝对路径> test
```

要求：任意 C99 编译器（验证环境 MinGW-w64 gcc 13.1.0）。
编译参数：`-std=c99 -Wall -Wextra -Werror -pedantic -O2`，**零警告**。
当前结果：**200 passed, 0 failed**（10 组测试：common/port、servo、motor、ir_remote、ir_sensor、ultrasonic、k210、frame、cmd、端到端）。

测试用 `ports/mock/` 的假硬件，能断言"发给外设的控制消息"（4 路 PWM / 舵机脉宽 / 串口字节）是否正确，也能注入"外设送来的数据"（GPIO 电平序列 / 串口字节流）验证解析，并覆盖输出缓冲容量、串口噪声活锁、整数溢出饱和等边界。**以后改库，先跑 `make test`。**

> 注：MinGW 的链接器写不了含中文的绝对路径，Makefile 已改为 `cd` 到库目录后用相对路径编译；
> 该环境的 AddressSanitizer/UBSan 不可用（缺 `libasan`/`libubsan`），故 sanitizer 检查未执行，见审查报告。

## 9. 已知限制 / 下一步

- 本库只做**外设通信**，不含建图、里程计、路径规划、PID —— 那部分要接 `organized/github_repo/` 里筛的开源项目，且必须先补编码器硬件。
- K210 现行 5 字节帧**无校验、无 ACK**；库保持兼容，但建议你们在 K210 侧把 `struct.pack` 改成 `<BBhB` 并加一个 XOR 校验字节，然后用 `clib_frame` 的带校验配置收发。
- `clib_ir_sampler_poll()` 是 GPIO 轮询降级方案，分辨率受采样频率限制；**正式项目请用 TM4C 的 GPTM 输入捕获**，把捕获到的脉宽喂 `clib_irdec_feed_pulse()`。注意 16 位拆分定时器在 80MHz 下约 819us 就溢出，测 HC-SR04 的 30ms 回波必须加预分频或溢出补偿。
- `clib_frame_cfg_t` 在 `frame_t` 里是裸指针，必须是 `static const` 或生命周期长于解析器实例，别用栈上临时变量。
- port 层的 `tm4c123g` / `stm32f103` 目录目前是**说明与代码骨架**，尚未接真机验证；需要在 CCS + TivaWare 里落地后逐项调试极性、PWM 频率与电平。
- 资源占用：库自身静态 RAM < 0.5KB、Flash 约 4~6KB，32KB RAM / 256KB Flash 毫无压力；瓶颈在栅格地图（100×100 uint8 就要 10KB）和 FreeRTOS 任务栈。
