# SG90 舵机扫动：TM4C123G LaunchPad

## 接线

先关闭黑色多路电源模块、拔掉 LaunchPad 的 USB，让 L298N 车轮驱动保持断电。沿着蓝色 SG90 外壳找出舵机原装三芯插头，用公对母杜邦线逐根转接，按下表接。白色转接板上带 `Trig/Echo` 的四针区属于 HC-SR04；`P2.7` 是转接板的信号标记，LaunchPad 上应找的是 `PC7`。单独试舵机时绕过这块转接板的舵机排针即可。

| SG90 原装线色 | 接到 |
| --- | --- |
| 红色（电源正，通常在插头中间） | 图 1 黑色电源模块的**红色排针**，找丝印 `5V` 的那一列 |
| 棕色（有的舵机为黑色，电源负） | 同一红色排针上标地符号的那一列 |
| 橙色（有的舵机为黄色或白色，控制信号） | LaunchPad 丝印 **`PC7`**（`J4.7`，靠着 `PC6` 和 `PD6`） |
| 另拉一根共地线 | 模块红色排针的 **GND** → LaunchPad 的 **GND**（如 `J2.1`，也可用 `J3.2`） |

接舵机前先空载用万用表直流档量红排 `5V` 对 `GND`，确认极性和约 5 V，再断电接线。不要把红色螺丝接线端误当成 5 V 排针，也不要用黄色 `VADJ` 或蓝色 `3V3` 供电。SG90 原厂标称 4.8 V；单舵机台架测试用稳定的约 5 V，电源留约 1 A 或更多输出余量。LaunchPad 继续由电脑的 **DEBUG USB** 供电，外部 5 V **只进舵机红线**，不接 LaunchPad 的 `VBUS/5V`、`3.3V` 或 GPIO。

```text
黑色电源模块 5V ───────── SG90 红
黑色电源模块 GND ───┬──── SG90 棕 / 黑
                    └──── LaunchPad GND
LaunchPad PC7 ────────── SG90 橙 / 黄 / 白
电脑 USB ─────────────── LaunchPad DEBUG USB
```

一般先用 PC7 输出的 3.3 V 信号试验。如果一次都不动，先确认已按 SW1、绿灯亮、三方共地、舵机插头接法和运行时 5 V 仍稳定。若 PC7 对地测到正确的约 50 Hz、3.3 V 脉冲而这个舵机仍不识别，可用由 5 V 供电的 74AHCT125 作单向电平转换；不要往 PC7 送 5 V。

## 烧录和试验

使用 Windows 上的 [TI LM Flash Programmer](https://www.ti.com/tool/LMFLASHPROGRAMMER)，通过板载 **ICDI** 烧录（与 `COM13` 虚拟串口是不同接口）。先断开舵机和车轮驱动供电。选择 `Configuration → Quick Set → EK-TM4C123GXL`，在 `Program` 选 `build/servo_sg90.bin`、地址 `0x00000000`、`Erase Necessary Pages`，勾选 `Verify After Program` 和 `Reset MCU After Program`，再点击 `Program`。烧录会替换当前测试应用；如需要保留旧应用，先备份对应固件。

烧录完成后，按上面的接法连接，先打开 LaunchPad 的 USB 电源，再开舵机 5 V。让舵臂和超声波线束周围留出空间，最好先卸下云台负载。程序上电后等待；按下板上 **SW1** 后绿灯亮，舵机先到中位约 1 秒，再每约 0.8 秒切换一个位置：**一侧 → 中位 → 另一侧 → 中位**，自动循环。再次按 SW1 停止发脉冲；如果舵臂卡住或持续嗡响，直接关舵机 5 V。位置版 SG90 不做连续转圈，实际角度和左右方向以这台舵机和舵臂安装为准。

有示波器时可先在未接舵机的情况下，把地夹接 LaunchPad GND，探头接 PC7，按 SW1。应看到约 20 ms 一帧，高电平约 1.30、1.50 或 1.70 ms。测试用 Wide Timer 1B，独立于 `wheel_drive` 的 20 kHz 电机 PWM；第一次试机特意缩小脉宽范围。后续要接到小车外设库，可把该定时器的脉宽更新接到 `servo_set_pulse_us` 回调，按实际系统时钟改计数，再为超声波扫描标定脉宽和等待时间。

## 重新编译

自带启动代码和链接脚本，无须安装 TivaWare。使用 [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 的 Windows `arm-none-eabi` 版，在 PowerShell 7 中运行：

```powershell
.\build.ps1
# 如未加到 PATH，显式指定工具链：
.\build.ps1 -ToolchainBin 'C:\工具链目录\bin'
```

参考：[TI LaunchPad 用户手册（引脚与烧录）](https://www.ti.com/lit/ug/spmu296/spmu296.pdf)、[TM4C123GH6PM 数据手册（PC7 复用与定时器 PWM）](https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf)、[TowerPro SG90 Analog 规格](https://towerpro.com.tw/product/sg90-analog/)。
