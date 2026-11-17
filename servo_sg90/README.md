# TM4C123G LaunchPad + SG90：位置保持与两轮试摆

2026-11-17 实机验证：使用 PC7 控制 SG90，最终接受的试摆范围为 **1100–1900µs**。每次执行两轮，结束后回到 **1500µs** 并保持。1500µs 是信号中位，舵臂需要按实际车头方向安装；本次没有角度传感器或示波器测量，不能把这些脉宽直接当成精确角度。

## 接线入口

完整针脚位置、降压模块识别、上电顺序及故障排查见 **[接线与供电说明](WIRING.md)**。

| 导线 | 接到 |
| --- | --- |
| SG90 红线 | 外部降压模块已测量确认的固定 5V 输出 |
| SG90 棕/黑线 | 外部降压模块 GND |
| SG90 橙/黄信号线 | LaunchPad **PC7 / J4.7** |
| 另加共地线 | 降压模块 **GND → LaunchPad GND / J2.1** |

LaunchPad 单独使用 **DEBUG USB** 供电。外部 5V 只接舵机红线；按这套方案，不把外部 5V 接入开发板 VBUS、3.3V 或 GPIO。

## 当前程序

- **`hold.c`：当前推荐版本。**复位后默认关闭脉冲输出。位置命令存放在 `volatile` 控制变量中，主循环读取这些变量并平滑改变目标，不会自行循环扫动。
- **SW1** 切换保持/停止；首次上电后按 SW1 保持 1500µs。暂停只是停止脉冲，不会切断舵机 5V。
- `tools/command.js sweep 400` 由电脑发出有限序列：`1500 → 1100 → 1500 → 1900 → 1500`，重复两轮，最后保持 1500µs。各端点到达后至少停留约 0.5 秒。
- 需要停止时按 SW1，或执行 `tools/command.js stop`；如卡住、持续嗡响或发热，直接关闭舵机 5V。
- `main.c` 保留旧版自动循环程序，同时为 `hold.c` 提供已实测的底层驱动。旧版 `build/servo_sg90.bin` 按 SW1 后会持续扫动，**不是当前保持版本**。

驱动使用 **PC7 / WT1CCP1 / Wide Timer 1B**，复位后的标称 16MHz 内部时钟，20ms 一帧。每 5ms 最多改变 2µs，避免一次大幅改变目标。软件目标限制为 1000–2200µs；这只是软件边界，不代表每台 SG90 在此范围内都无机械干涉。当前两轮试摆脚本幅度限制为中位两侧各 50–400µs，默认各 400µs。

SG90 三线接口没有返回角度或接线状态的通道。调试读数表示**下发脉宽及定时器状态**，实际朝向、左右方向与安装有关，需要现场观察。更换或重新安装舵臂后应重新标定。

## 编译

自带启动代码和链接脚本，不依赖 TivaWare。Linux 安装 Arm GNU `arm-none-eabi` 工具链、Python 3 和 Make 后，在本目录运行：

```bash
make
```

生成 `build/servo_sg90_hold.bin` 和 `build/servo_sg90_hold.symbols.json`。后者绑定二进制 SHA-256 与控制变量地址，重编译后两者必须一起更新。`make legacy` 可重建旧版循环程序。

Windows 使用 PowerShell 7 和 Arm GNU 工具链：

```powershell
.\build.ps1
# 未加入 PATH 时：
.\build.ps1 -ToolchainBin 'C:\工具链目录\bin'
# 仅在确实需要旧版持续循环时：
.\build.ps1 -LegacySweep
```

本次执行并验证了 Linux 构建；更新后的 PowerShell 脚本尚未在 Windows 上执行。

## Linux：烧录与位置命令

使用已安装的 **TI CCS Scripting**，本次实测版本为 CCS 21.0.1。脚本只接受一个连接中的 ICDI，并要求显式指定目标板序列号。使用正常板，故障板保持断开。先在所有电源关闭时断开舵机和车轮驱动，只用 DEBUG USB 连接正常板。

```bash
ls -l /dev/serial/by-id
export TM4C_SERIAL='填写正常板的USB序列号'
export CCS_ROOT='/填写CCS安装位置/ccs'

# 先备份完整 256KiB Flash，再烧录、读回校验，并检查默认待机状态
"$CCS_ROOT/scripting/run.sh" tools/flash.js
```

按 [接线说明](WIRING.md)复核外部电源后再接舵机。在本目录运行：

```bash
# 回中并保持
"$CCS_ROOT/scripting/run.sh" tools/command.js 1500
# 当前接受的幅度：1100–1900µs，两轮后回中
"$CCS_ROOT/scripting/run.sh" tools/command.js sweep 400
# 较小幅度：1300–1700µs
"$CCS_ROOT/scripting/run.sh" tools/command.js sweep 200
# 停止发脉冲 / 仅读取状态
"$CCS_ROOT/scripting/run.sh" tools/command.js stop
"$CCS_ROOT/scripting/run.sh" tools/command.js read
```

命令脚本会核对板上 Flash 与本地保持固件一致，通过 SRAM 控制变量下发目标，并检查定时器与目标是否稳定；不依赖旧版程序的 CPU 寄存器临时修改。结束后保留最后的保持/停止状态。脚本中途退出时应检查现场状态，必要时按 SW1 或关闭舵机电源。

所有原程序备份、带板号的日志和历史记录只写入被 Git 忽略的 `local/`。本仓库不包含实验板的原 Flash 备份。

## Windows：烧录保持固件

使用 [TI LM Flash Programmer](https://www.ti.com/tool/LMFLASHPROGRAMMER) 的 ICDI 接口，选择 `EK-TM4C123GXL`，烧录 **`build/servo_sg90_hold.bin`**，地址 `0x00000000`，选择 `Erase Necessary Pages`，勾选烧录后校验和复位。烧录会替换现有程序，需要时先备份。完成后默认待机，按 SW1 可保持中位。

电脑命令脚本目前只验证了 Linux；Windows 的两轮试摆控制没有在本次环境验证。

## 实测记录

- Arm GCC `-Wall -Wextra -Werror` 构建通过，保持固件为 1308 字节。
- 当前提交的固件与现场使用的固件 SHA-256 一致：`5df8790fa8cada0eab49c879a0e4d83635c2c1d0dc62a0d814f256b7df7cb965`。
- ICDI 烧录及读回校验通过；定时器与控制变量多次读取一致，CPU 故障寄存器为 0。
- 发布整理时开发板已断开，未重复烧录或驱动；发布目录脚本已通过语法检查，并确认无目标板时会在连接前拒绝执行。
- 用户现场确认新版本在两轮试摆后停止来回摆动；逐步扩大到 1100–1900µs 后确认当前幅度合适。
- 未对实际脉宽、电源纹波及精确左右角度做仪器标定，尚不能保证车头两侧角度完全对称。

参考：[TI LaunchPad 用户手册](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)、[TM4C123GH6PM 数据手册](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)、[TowerPro SG90 Analog 规格](https://towerpro.com.tw/product/sg90-analog/)。
