# STM32F103 回迁对照（备查）

> 本项目主目标是 TM4C123G，这个目录只是记录"如果哪天要回到 STM32，port 层怎么填"，
> 采用上一轮评估里的 **C 类资料一律作废** 原则：只用逻辑，不搬 SPL 代码进库。

## 逻辑到原厂实现的对照

| `clib_port_t` 回调 | 原 STM32F103 案例里的实现 | 改法 |
|---|---|---|
| `millis` / `micros` | `System/Delay.c` 直接写 `SysTick->LOAD = 72*xus` 轮询 | 改成 SysTick 中断维护单调计数；72MHz 常数保留但别再忙等 |
| `delay_us` | `Delay_us()`，`SysTick->CTRL=0x5` | 只在 TRIG 这类短时序用，别用于舵机稳定等待 |
| `gpio_read` | `GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)` 等 | 按 `clib_pin_t` 建映射表，`GPIOB->IDR` 位读也可以 |
| `gpio_write` | `GPIO_SetBits/ResetBits` | `GPIO_WriteBit` 或 BSRR/BRR |
| `motor_set_duty` | `TIM_SetCompare1~4(TIM4, duty)`，TIM4 CH1~4 / PB6~9，ARR=99 PSC=35 → 20kHz | 直接调 SPL 即可，占空比 0~100 正好对应 CCR |
| `servo_set_pulse_us` | `TIM_SetCompare2(TIM3, us)`，TIM3_CH2/PB5，`TIM3_PWM_Init(19999,71)` → 1 计数 = 1us | 脉宽微秒直接写 CCR2；注意原代码 `PWM2 模式 + OCPolarity_Low`，极性要实测 |
| `uart_write` | `Serial_SendByte` / `usatr3_send_data`，`USART_GetFlagStatus(TXE)` | 加超时，别无限等标志位 |
| `uart_read` | `USART3_IRQHandler` 用 RXNE+IDLE 填 `MyUsart3.buff[100]` | 换成环形缓冲 + 容量上限；或干脆把字节喂给 `clib_frame_feed` / `clib_k210_feed`，不依赖 IDLE |
| `log_putc` | `fputc` 重定向到 USART1 PA9/PA10 | 保持 |

## 关键提醒

- 不要把 `Library/`（StdPeriph Lib）、`Start/`（CMSIS）、`stm32f10x_it.c` 拷进 `organized/library/`——那是 C 类，本库刻意做到零芯片依赖，混进来就白抽了。
- `robot.h` 声明 `uint8_t speed` 而 `robot.c` 定义 `int8_t speed` 的类型不一致，库里已统一为 `int16_t`，别把老声明带回来。
- 红外遥控**两套码表**对应案例 4 与案例 6，按实物选 `CLIB_IR_KEYMAP_V1` / `V2`。
- 超声波原公式 `cnt*5*34/200.0` 与 `timer.c` 里 TIM2 的实际时基（72MHz/7200 = 10kHz，即 100us/计数）对不上，量纲混乱；库改成标准公式 `us*0.0343/2`。原案例阈值 60 判的是 `UltrasonicWave_StartMeasure()` 返回的 `cnt*8.5`（不是 cm），要复现用 `clib_echo_legacy_measure()`；`clib_echo_count_to_legacy()` 复现的是中间量 `cnt*0.85`。
