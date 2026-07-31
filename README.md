# 三模式 AC-AC 变换器控制固件

本工程面向 STM32F407V(E-G)Tx（LQFP100），实现单相有源整流、三相逆变以及两级联动三种运行模式。当前快速控制频率和 PWM 频率均为 20 kHz，整流级使用单极性 SPWM，逆变级使用连续 SVPWM。

> **安全状态说明**
>
> 当前代码可通过主机算法测试。`App/Src/app_config.c` 中 Ia/Ib/Vab/Vbc/Vdc
> 五路已写入实测标定，Iin/Vin **临时复用同型传感器（TA/TV）通道的标定参数，
> 更换前端必须重新标定**；整流 PI/QPR 参数为保守可运行值，实机仍须按实测
> 前端和负载重新整定。预充完成和相位就绪在 MODE1/MODE0 中可由测量安全推导
> （Vdc 达到运行欠压阈值、输入 RMS 超过最小输入），也可由板级 GPIO 经
> `App_SetPrechargeDone()`/`App_SetPhaseReady()` 提交。完成死区、保护阈值
> 和桥臂极性的实机校核前，不应连接满电压功率回路。

## 1. 工程结构

```text
App/
├─ Inc/                    控制算法、状态机和应用接口头文件
└─ Src/
   ├─ app_config.c         集中式参数与上电安全锁
   ├─ app_control.c        三模式调度和对 Core 层公开的应用接口
   ├─ measurement.c        ADC 标定、三相电压/电流重构
   ├─ inverter_control.c   三相电压外环、电流内环和 SVPWM
   ├─ rectifier_control.c  母线电压外环、输入电流内环和软启动
   ├─ svpwm.c              连续 SVPWM
   ├─ rectifier_pwm.c      单相全桥单极性 SPWM
   ├─ qpr_controller.c     Tustin 离散准 PR 控制器
   ├─ pi_controller.c      带抗饱和的离散 PI
   ├─ notch_filter.c       Tustin 离散陷波器
   ├─ phase_recurrence.c   无查表正交相位递推
   ├─ protection.c         软件保护判断与故障锁存
   └─ power_state.c        上电、运行、停机和故障状态机
Core/
├─ Inc/
└─ Src/
   ├─ main.c               HAL/外设初始化和后台循环入口
   ├─ board_runtime.c      ADC DMA、PWM 寄存器、BKIN、按键和 OLED 适配
   ├─ adc.c/dma.c          双 ADC 同步采样与循环 DMA
   ├─ tim.c                TIM1/TIM8 PWM、同步和 Break 配置
   ├─ key_board.c          4×4 矩阵键盘扫描
   └─ oled.c/spi.c         OLED 和 SPI3 驱动
host_tests/               不依赖 HAL 的纯 C 算法测试
```

旧 `icode` 控制代码只作历史参考，不参与当前 Debug/Release 构建。`TIM4`、`DAC` 和 `SPI2` 虽仍保留 CubeMX 生成文件，但 `main()` 不初始化它们，也不参与当前控制链路。

## 2. 系统运行链路

### 2.1 初始化

`main()` 按以下顺序启动：

```text
HAL_Init
  → 168 MHz 系统时钟
  → GPIO / DMA
  → ADC1 / ADC2
  → TIM1 / TIM8
  → SPI3
  → BoardRuntime_Init
  → BoardRuntime_BackgroundTask 循环
```

`BoardRuntime_Init()` 的主要工作：

1. 清除 TIM1、TIM8 的 `MOE` 和输出通道使能，使所有功率管保持关闭。
2. 把五路 PWM 比较值置于 50% 中性值，但此时仍不开放门极。
3. 启动 DWT 周期计数器，用于测量 20 kHz 快速环最坏执行时间。
4. 初始化应用层、矩阵键盘和 OLED。
5. 清除并启用 TIM1/TIM8 Break 中断；若 BKIN 已有效，立即锁存故障。
6. 先启动 ADC2，再启动 ADC1 双 ADC DMA。
7. 先启动 TIM8，再启动作为同步主定时器的 TIM1。

### 2.2 20 kHz 快速控制

TIM8 CH4 每个 PWM 周期产生一次 ADC 触发。ADC1 和 ADC2 同步完成 4 个 Rank 后，DMA 得到一帧数据。8 个 32 位 DMA 字组成双帧缓冲区：

```text
DMA 半满中断：处理第 1 帧
DMA 全满中断：处理第 2 帧
```

每帧的处理顺序如下：

```text
ADC CDR 数据
  → App_ApplyAdcPairs
  → ADC 标定和 Vin 重复采样诊断
  → App_FastStep20kHz
  → 保护判断和状态机
  → 整流/逆变控制算法
  → 归一化占空比
  → 延迟一周期写入 CCR
  → 最终 BKIN/故障复查
  → 根据模式设置 CCER 和 MOE
```

快速环为固定时间、固定内存算法，不动态分配内存；各核心算法的单次时间复杂度和额外空间复杂度均为 `O(1)`。

### 2.3 后台任务

`BoardRuntime_BackgroundTask()` 不直接修改 PWM，仅执行：

- 每 10 ms 扫描一次矩阵键盘；
- 连续 3 次一致后确认按键状态；
- 每 100 ms 更新 OLED；
- 调用预留的 `App_BackgroundTask()`。

因此 OLED 和按键操作不会进入 20 kHz 中断。

## 3. 运行模式和状态机

### 3.1 三种模式

| 模式 | 功能 | PWM 行为 |
|---|---|---|
| `MODE0_FULL` | 单相整流与三相逆变联动 | 先运行整流级；滤波 `Vdc` 连续满足容差后，再开放逆变级；逆变使用实测 `Vdc` |
| `MODE1_RECTIFIER` | 单独测试整流级 | 只开放 TIM8，TIM1 保持关闭；三相输出量固定为 0，不参与保护 |
| `MODE2_INVERTER` | 单独测试逆变级 | 只开放 TIM1，TIM8 保持关闭；需外接隔离、限流直流源；SVPWM 归一化和限幅使用 PC4 实测 `Vdc`，键盘设定只作为期望母线/输出参考 |

模式只能在 `OFF` 状态切换。切换模式不会绕过预充、相位就绪、测量有效、参数有效或故障联锁；切换瞬间板级显式关闭两级 PWM 并作废延迟一周期命令，避免旧模式 CCR 被装到新定时器。

### 3.2 状态机

```text
OFF
 └─ 启动请求 → SELF_TEST
                  └─ 连续有效采样 → PRECHARGE
                                      └─ precharge_done → READY
                                                            └─ phase_ready 连续确认 → RUN
RUN / READY / PRECHARGE
 └─ 停机或条件丢失 → DISCHARGE
                         └─ Vdc 降至安全阈值并连续确认 → OFF

任意状态出现有效或锁存故障 → FAULT_LATCH
```

只有同时满足下列条件时 `PowerStateMachine` 才允许 PWM：

- 状态为 `RUN`；
- `BoardPower_ConfigValid()` 返回真；
- 当前测量帧有效；
- 预充完成；
- 相位就绪；
- 没有有效或锁存故障；
- 没有停机请求。

`MODE2_INVERTER` 使用外接直流源，预充和相位联锁内部固定视为已满足。`MODE1/MODE0` 中：

- 板级可通过 `App_SetPrechargeDone()`、`App_SetPhaseReady()` 提交真实联锁信号；
- 若没有外部信号，则由测量安全推导，**不会默认虚假置真**：`precharge_done` 要求实测 `Vdc` 达到运行欠压阈值（被动/外部预充已把母线充起），`phase_ready` 要求整流慢环测得的输入 RMS 超过最小输入电压。

## 4. 输入采样引脚

所有 ADC 引脚均配置为模拟输入、无上下拉。**输入电压使用 PC5；PA6 已改作 TIM8_BKIN，不是 ADC 输入。**

| 物理量 | MCU 引脚 | ADC 通道 | Rank | 含义和符号 |
|---|---:|---:|---:|---|
| 输入电流 `Iin` | PA5 | ADC1_IN5 | ADC1 Rank 1 | 单相整流输入电流 |
| 输入电压 `Vin` | PC5 | ADC2_IN15 | ADC2 Rank 1 | 单相交流输入电压 |
| 输出电流 `Ia` | PC0 | ADC1_IN10 | ADC1 Rank 2 | 三相 A 相电流 |
| 输出电流 `Ib` | PC1 | ADC2_IN11 | ADC2 Rank 2 | 三相 B 相电流 |
| 线电压 `Vab` | PC2 | ADC1_IN12 | ADC1 Rank 3 | `Va−Vb` |
| 线电压 `Vbc` | PC3 | ADC2_IN13 | ADC2 Rank 3 | `Vb−Vc`，不是 `Vcb` |
| 直流母线 `Vdc` | PC4 | ADC1_IN14 | ADC1 Rank 4 | 直流母线电压 |
| `Vin` 诊断重复采样 | PC5 | ADC2_IN15 | ADC2 Rank 4 | 与 Rank 1 对比，检查采集建立时间/帧错位 |

双 ADC 的 `CDR` 帧格式固定如下，每个 32 位字低 16 位为 ADC1、高 16 位为 ADC2：

| CDR 字 | 低 16 位 ADC1 | 高 16 位 ADC2 |
|---:|---|---|
| Rank 1 | `Iin` | `Vin` |
| Rank 2 | `Ia` | `Ib` |
| Rank 3 | `Vab` | `Vbc` |
| Rank 4 | `Vdc` | `Vin_diag` |

ADC 当前配置：

- ADC1 + ADC2 双规则同步模式；
- 12 位右对齐；
- ADC 时钟为 PCLK2/4；
- 每通道采样时间 28 个 ADC 周期；
- ADC1 由 TIM8 TRGO 上升沿触发；
- DMA2 Stream0、Channel 0，32 位外设/内存宽度，循环双帧，高优先级；
- `Vin` 两次采样默认允许相差 9 个 ADC 码，实际阈值可通过配置修改。

ADC 物理量换算统一使用：

```text
physical_value = (raw_count - offset_count) × gain
```

其中 `gain` 包含传感器、运放、分压及 ADC 每码比例，并允许用正负号描述采样极性。

只采两相电流和两路线电压，其余量按下式重构：

```text
Ic  = -Ia - Ib
Vca = -Vab - Vbc

Va = ( 2Vab + Vbc) / 3
Vb = (- Vab + Vbc) / 3
Vc = (- Vab - 2Vbc) / 3
```

## 5. PWM 输出和硬件保护引脚

### 5.1 三相逆变级：TIM1

TIM1 使用 AF1、中心对齐 PWM1 和互补输出。奇数引脚为高侧，偶数引脚为低侧。

| 相位 | 高侧 HI | 低侧 LI | 定时器通道 | 占空比字段 |
|---|---|---|---|---|
| A 相 | PE9 | PE8 | TIM1_CH1 / CH1N | `inverter_a` → CCR1 |
| B 相 | PE11 | PE10 | TIM1_CH2 / CH2N | `inverter_b` → CCR2 |
| C 相 | PE13 | PE12 | TIM1_CH3 / CH3N | `inverter_c` → CCR3 |

### 5.2 单相整流级：TIM8

TIM8 使用 AF3、中心对齐 PWM1 和互补输出。

| 桥臂 | 高侧 HI | 低侧 LI | 定时器通道 | 占空比字段 |
|---|---|---|---|---|
| 桥臂 A | PC7 | PB0 | TIM8_CH2 / CH2N | `rectifier_a` → CCR2 |
| 桥臂 B | PC6 | PA7 | TIM8_CH1 / CH1N | `rectifier_b` → CCR1 |

整流调制为两桥臂反相的单极性 SPWM：

```text
duty_a = 0.5 + 0.5 × modulation
duty_b = 0.5 - 0.5 × modulation
```

最终占空比受 `duty_min`、`duty_max` 限制，实际施加调制量会反馈给 QPR 进行抗饱和处理。

### 5.3 PWM 时基和同步

TIM1、TIM8 的高级定时器时钟均为 168 MHz：

```text
PSC = 0
ARR = 4199
RCR = 1
中心对齐频率 = 168 MHz / (2 × 4200) = 20 kHz
```

- TIM1 的 Update TRGO 是两只定时器的同步基准；
- TIM8 以 ITR0、Reset Slave Mode 跟随 TIM1；
- TIM8 CH4 不输出到引脚，仅以 PWM2、CCR4=4198 在载波峰值附近产生 ADC TRGO；
- TIM8 CH3 完全未使用；
- TIM1/TIM8 死区当前均为 `DTG=148`（168 MHz、CKD=DIV1 下编码为 168 个 tDTS，约 1.000 us）；上功率前仍必须结合驱动传播延时和功率管实测复核；
- `AOE=0`，Break 后不会自动恢复输出。

### 5.4 Break 输入

| 保护对象 | MCU 引脚 | 外设 | 有效电平 | 作用 |
|---|---|---|---|---|
| 三相逆变级 | PE15 | TIM1_BKIN，AF1 | 高有效，内部下拉 | 硬件立即清 TIM1 MOE |
| 单相整流级 | PA6 | TIM8_BKIN，AF3 | 高有效，内部下拉 | 硬件立即清 TIM8 MOE |

任意一路 Break 回调都会调用全局跳闸逻辑，同时关闭 TIM1 和 TIM8，清通道使能并锁存 `PROTECTION_FAULT_EXTERNAL`。建议外部比较器或栅极驱动 `FAULT` 同时直达：

1. 对应定时器 BKIN；
2. 栅极驱动器 EN/SD；
3. MCU 可读取的诊断信号。

这样关断速度不依赖 ADC、DMA 或软件中断。

### 5.5 尚未分配的关键板级信号

以下信号目前没有确定 GPIO，也没有板级实现：

- `EN_INV`：逆变驱动使能；
- `EN_RECT`：整流驱动使能；
- 预充继电器控制及外部 `precharge_done`；
- 输入相位/锁相完成信号及外部 `phase_ready`；
- 独立的驱动器故障读回（除 BKIN 外）。

预充/相位联锁当前由测量推导（见 3.2 节）。确定原理图后，应在板级适配层实现这些信号，并调用 `App_SetPrechargeDone()`、`App_SetPhaseReady()`；不能直接在算法层把标志常置为真。

## 6. 人机接口和其他引脚

### 6.1 OLED：SPI3

| 功能 | 引脚 | 配置 |
|---|---|---|
| OLED 时钟 D0 | PC10 | SPI3_SCK，AF6 |
| OLED 数据 D1 | PC12 | SPI3_MOSI，AF6 |
| OLED DC | PC11 | 推挽输出 |
| OLED Reset | PD0 | 推挽输出 |
| OLED CS | PD1 | 推挽输出 |

SPI3 外设配置为 2 线主机模式，当前 OLED 驱动只执行发送；数据为 8 位、MSB 先行、CPOL=High、第一边沿采样、波特率分频 2。OLED 每 100 ms 显示模式、状态、30/60 Hz、`Vdc`、首要故障码、配置锁状态和快速环最大周期数。

### 6.2 4×4 矩阵键盘

| 类型 | 引脚 | 配置 |
|---|---|---|
| 行 R1/R2/R3/R4 | PE1、PE3、PE4、PD11 | 推挽输出；扫描时逐行置高 |
| 列 C1/C2/C3/C4 | PD3、PD4、PD7、PD10 | 下拉输入 |

按键布局：

```text
1  2  3  A
4  5  6  B
7  8  9  C
*  0  #  D
```

当前功能与操作结果：

| 按键 | 操作条件 | 功能及 OLED 预期变化 |
|---|---|---|
| `0` 短按 | 仅 `OFF` 状态接受 | 选择 `MODE0_FULL`，OLED 显示 `FULL`；这是上电默认模式，再按一次不会发生可见变化 |
| `1` 短按 | 仅 `OFF` 状态接受 | 选择单独整流模式，OLED 显示 `RECT` |
| `2` 短按 | 仅 `OFF` 状态接受 | 选择单独逆变模式，OLED 显示 `INV` |
| `A` 短按 | 仅 `OFF` 状态接受（`MODE1` 下拒绝） | 在 30 Hz 和 60 Hz 间切换；上电默认 60 Hz，第一次按下应切换为 30 Hz |
| `B` 短按 | `OFF` 且无启动请求，或 PWM 已禁止的 `FAULT_LATCH` | Vdc 目标 −1 V（30～65 V）；`MODE1/0` 同时更新整流母线目标 |
| `C` 短按 | 同上 | Vdc 目标 +1 V（30～65 V） |
| `#` 持续按住至少 2 s | 已选择模式，且启动安全条件满足 | 提交启动请求；它不是点按键，未达到 2 s 不启动 |
| `D` 短按 | 任意状态 | 立即撤销启动请求，并在状态机处理前先行关闭 TIM1/TIM8 两级 PWM |
| `*` 持续按住至少 2 s | 当前为 `FAULT_LATCH`，且 PA6、PE15 两路 BKIN 均为低 | 清除硬件 Break 标志并请求复位故障；正常 `OFF` 状态按下不会发生可见变化 |

其余数字键当前未分配控制功能。

#### 6.2.1 按 `#` 不启动时的排查

默认配置已写入五路实测标定和保守整流参数，`BoardPower_ConfigValid()` 为真。长按 `#` 仍不能进入 `RUN` 时，按以下顺序排查：

1. 模式与测量是否匹配：`MODE1/0` 需要 Iin/Vin/Vdc 通道有效、输入 RMS 超过最小输入电压且母线已被被动/外部预充到运行欠压阈值以上；`MODE2` 需要 Ia/Ib/Vab/Vbc/Vdc 有效且实测母线不低于欠压阈值。
2. OLED `FLT:` 故障码：先排除故障源，再在 PA6、PE15 均为低时长按 `*` 复位。
3. Vin 冗余诊断（`MODE0/1`）：两次 Vin 采样差值超过 9 码会锁存测量故障；检查 PC5 前端是否悬空、噪声是否过大。
4. Debug 构建的快速环超时：实时验收请使用 Release 构建。

验证键盘是否工作的最直接方法：

1. 确认 MCU 正在运行。若通过 CubeIDE 的 Debug 启动并停在 `main()`，先按 `F8`（Resume）；CPU 停住时不会扫描键盘。
2. OLED 显示 `OFF` 时短按 `1`，模式应在 100 ms 左右更新为 `RECT`。
3. 再短按 `2`，应更新为 `INV`。
4. 短按 `A`，频率应在 30 Hz 与 60 Hz 之间切换（`MODE1` 下拒绝切频）。

不要使用 `0`、`#` 或 `*` 作为首次按键测试：`0` 是默认模式，`#` 需要满足全部启动条件，`*` 只在故障锁存时有效，三者都可能在键盘正常时表现为“没有变化”。

如果 OLED 状态已经是 `FAULT`，`0/1/2/A` 也会被拒绝，因为模式和频率只能在 `OFF` 修改。应先排除故障源，再在 PA6、PE15 均为低时长按 `*` 复位；如果故障源仍存在，状态机会立即回到 `FAULT`。

2026-07-30 对当时 Debug 固件的在线检查结果如下（历史记录，对应旧的逆变测试固件）：

- `g_app.initialized=true`，`s_last_key_scan` 随 `HAL_GetTick()` 推进，说明后台扫描正在运行；
- `s_key_candidate/s_key_stable` 能从键值 `1` 恢复为无键 `0`，说明矩阵扫描能够识别按下与释放；
- 当前状态为 `FAULT_LATCH`，锁存故障掩码为 `0x301`；
- 两次 Vin 原始采样为 921 和 894，相差 27 码，超过当前 9 码诊断阈值，触发测量故障；
- Debug 构建实测快速环最坏为 6686 个 CPU 周期，在 168 MHz 下约为 39.8 µs，超过当时 20 µs 门限，触发快速环超时（当前代码门限为 50 µs，即一个 20 kHz 控制周期）。

因此这次“按键无反应”的直接原因是故障状态拒绝控制请求，不是键盘驱动未执行。用于实时运行时间验收时应烧录 Release 构建；同时必须先检查 PC5 输入电压采样是否悬空、噪声是否过大、两次采样的时序和诊断阈值是否与实测前端匹配。不要仅通过放宽阈值来掩盖采样链问题。

#### 6.2.2 键盘接线和扫描方式

4×4 薄膜键盘只是 16 个触点组成的无源矩阵，没有 VCC/GND 电源脚。键盘排线顺序并不统一，必须按万用表通断档确认实际 R1～R4、C1～C4，不能只按排线从左到右直接假定。

| 键盘信号 | STM32 引脚 |
|---|---|
| R1 | PE1 |
| R2 | PE3 |
| R3 | PE4 |
| R4 | PD11 |
| C1 | PD3 |
| C2 | PD4 |
| C3 | PD7 |
| C4 | PD10 |

`Read_Key()` 每 10 ms 执行一次：先把所有行拉低，再逐行短暂置高，同时读取带下拉的四列。按键稳定约 30 ms 后才被确认，以消除机械抖动。由于行高电平是很窄的扫描脉冲，万用表通常只会显示接近 0 V；检查扫描波形应使用示波器或逻辑分析仪。键盘端不得外接 5 V。

若 `1/2/A` 均无响应，按以下顺序检查：

1. OLED 是否每 100 ms 正常刷新，以及 CubeIDE 是否仍停在断点。
2. 八根键盘线是否确实接到上表引脚，尤其核对排线的行列定义。
3. 按住某键时，用通断档确认对应的一根行线和一根列线确实导通。
4. 用示波器确认 PE1、PE3、PE4、PD11 存在扫描脉冲；再确认按键按住时对应 PD3、PD4、PD7 或 PD10 出现高脉冲。
5. 在 CubeIDE Live Expressions 中观察 `s_key_candidate` 和 `s_key_stable`：无键为 0，按键确认后应得到 1～16。两者是 `board_runtime.c` 内的静态变量。

### 6.3 预留或未参与当前控制的引脚

| 引脚 | 名称/功能 | 当前状态 |
|---|---|---|
| PC13 | USER_KEY，下降沿 EXTI、上拉 | 已配置，但未接入当前模式/启停逻辑 |
| PB8 | LED1 | 普通输出，未用于功率控制 |
| PC9 | RGB | 普通输出，未用于功率控制 |
| PB12 | SPI_IMU_CS | 普通输出；SPI2 当前未初始化 |
| PA15 | SPI_FLASH_CS | 普通输出；SPI2 当前未初始化 |
| PB2 | 通用输出 | 未用于功率控制 |
| PA13 / PA14 | SWDIO / SWCLK | 调试下载接口 |

## 7. 重要函数

### 7.1 Core 层

| 函数 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `main()` | 无 | 不返回 | 初始化 HAL、时钟和已使用外设，然后永久执行后台任务 |
| `SystemClock_Config()` | 无 | 无 | 使用 HSE 和 PLL 配置 168 MHz SYSCLK；APB1=42 MHz、APB2=84 MHz |
| `Error_Handler()` | 无 | 不返回 | 直接清 TIM1/TIM8 MOE、禁止中断并停机 |
| `BoardRuntime_Init()` | 无 | 无 | 以门极关闭状态初始化应用、Break、ADC DMA、定时器、键盘和 OLED |
| `BoardRuntime_BackgroundTask()` | 无 | 无 | 运行低速键盘、OLED 和应用后台逻辑 |
| `BoardRuntime_ProcessAdcFrame()` | 一帧 4 个双 ADC CDR 字 | 无 | 提交采样、执行 20 kHz 控制、延迟并施加 PWM、统计执行周期 |
| `BoardRuntime_ApplyDelayedPwm()` | 归一化 PWM 命令 | 是否成功 | 最终检查故障/BKIN，在临界区写 CCR、CCER 和 MOE |
| `BoardRuntime_LatchTrip()` | 故障位 | 无 | 锁存跳闸、关闭两级 PWM，并通知应用保护层 |
| `BoardRuntime_TryResetFault()` | 无 | 是否复位成功 | 只在 `FAULT_LATCH` 且两路 BKIN 均为低时清 Break 标志并请求复位 |
| `HAL_ADC_ConvHalfCpltCallback()` | ADC 句柄 | 无 | 处理 DMA 双缓冲前半帧 |
| `HAL_ADC_ConvCpltCallback()` | ADC 句柄 | 无 | 处理 DMA 双缓冲后半帧 |
| `HAL_ADC_ErrorCallback()` | ADC 句柄 | 无 | ADC1/ADC2 错误时锁存测量故障 |
| `HAL_TIMEx_BreakCallback()` | TIM 句柄 | 无 | TIM1/TIM8 Break 后执行全局故障锁存 |

NMI、HardFault、MemManage、BusFault 和 UsageFault 处理函数也会直接清 TIM1/TIM8 MOE，避免 CPU 异常时门极继续输出。

### 7.2 应用层公开接口

公开接口位于 `App/Inc/app_control.h`。

| 函数 | 输入 | 返回/输出 | 作用和限制 |
|---|---|---|---|
| `App_Init()` | 无 | 无 | 初始化全局应用实例和安全 PWM 命令 |
| `App_RequestMode(mode)` | 三种 `app_mode_t` | 是否接受 | 仅在 `OFF` 接受模式切换 |
| `App_RequestStart()` | 无 | 无 | 设置启动请求；不会绕过配置、预充、相位和故障条件 |
| `App_RequestStop()` | 无 | 无 | 撤销启动请求，状态机转入放电 |
| `App_ApplyAdcPairs(pairs, sequence)` | 4 个 CDR 字、帧序号 | 帧是否通过诊断 | 解包双 ADC 并检查两次 `Vin` 采样差值 |
| `App_ApplyAdcSamples(samples, sequence)` | 8 个交错 ADC 样本 | 是否有效 | 供主机测试使用的打包辅助接口 |
| `App_FastStep20kHz()` | 使用已提交帧 | 无 | 执行标定、保护、状态机和两级控制，更新公开状态和 PWM 命令 |
| `App_TripFromISR(faults)` | 故障位掩码 | 无 | 可从中断安全调用，原子锁存故障并撤销使能 |
| `App_GetPwmCommand()` | 无 | `AppPwmCommand*` | 读取五路归一化占空比及两级使能请求 |
| `App_GetStatus()` | 无 | `AppStatus*` | 读取模式、状态、频率、Vdc、故障和配置状态；后台读取时应短暂关中断复制 |
| `App_SetOutputFrequency(hz)` | 30 或 60 | 是否接受 | 仅在 `OFF` 接受，并重建 QPR 系数和相位递推步长；`MODE1` 返回 false |
| `App_SetOutputLineVoltageReference(line_rms)` | 三相线电压 RMS | 是否接受 | 比例跟随校验：仅当等于 `0.60 × Vdc` 设定值时接受，不允许独立改写 |
| `App_SetDcVoltageReference(volts)` | 母线目标电压 | 是否接受 | `MODE2` 推导输出线电压参考；`MODE1/0` 同时更新整流 DC 目标；SVPWM 归一化始终用实测 Vdc |
| `App_ResetFault()` | 无 | 无 | 撤销使能、清 ISR 故障暂存并发出单拍复位请求；目标板应通过 BKIN 安全检查后调用 |
| `App_SetPrechargeDone(done)` | 布尔值 | 无 | 板级适配层提交预充完成状态 |
| `App_SetPhaseReady(ready)` | 布尔值 | 无 | 板级适配层提交输入相位/同步就绪状态 |

`AppControl_Init()`、`AppControl_SetMode()` 和 `AppControl_Step()` 是不依赖 HAL 的底层实例接口，主要用于模块复用和纯 C 测试；目标板通常使用上述 `App_*` 单例接口。

### 7.3 测量、控制和调制

| 函数 | 作用 |
|---|---|
| `BoardPower_GetConfig()` | 返回集中式板级配置。GCC 下为弱符号，可由板级文件覆盖，但仍必须通过集中安全校验 |
| `BoardPower_ValidateModeConfig(cfg, mode)` | 按模式校验：`MODE2` 逆变五通道+逆变参数，`MODE1` 整流三通道+整流参数，`MODE0` 全部七通道+两级参数；PI/QPR 零增益视为无效 |
| `BoardPower_ValidateConfig()` | 兼容接口，按最严格的 `MODE0_FULL` 校验 |
| `Measurement_UnpackDualAdcPairs()` | 按固定 Rank 从四个 CDR 字解出八个 12 位采样结果 |
| `Measurement_Convert()` | 应用偏置/增益，检查原始范围，并重构 `Ic/Vca/Va/Vb/Vc` |
| `PhaseRecurrence_Init/SetFrequency()` | 只在初始化或切频时用 `sinf/cosf` 生成递推系数 |
| `PhaseRecurrence_Step()` | 20 kHz 中只做正交乘加，每 256 拍归一化一次，不使用正弦查表 |
| `PhaseRecurrence_GetThreePhase()` | 生成标准正相序 `A=sinθ`、`B=sin(θ−120°)`、`C=sin(θ+120°)` |
| `QprController_Init()` | 用 Tustin 法离散 `Kp + Kr·2ωc·s/(s²+2ωc·s+ωr²)` |
| `QprController_Step()` | 更新准 PR 状态、执行输出限幅和内部抗饱和 |
| `QprController_TrackOutput()` | 把 PWM 实际施加量回写控制器，避免过调制后积分式积累 |
| `PiController_Step()` | 梯形积分 PI，支持回算或条件积分抗饱和 |
| `NotchFilter_Init/Step()` | 建立并执行 Tustin 离散二阶陷波器；整流母线环使用 100 Hz 陷波 |
| `RectifierControl_Step()` | 20 kHz 输入电流 QPR；每 20 拍更新 1 kHz 的 Vin RMS、Vdc 陷波和母线 PI；执行 Vdc 软启动 |
| `RectifierPwm_Generate()` | 把整流调制量转换为两桥臂反相占空比并限幅 |
| `InverterControl_Step()` | 相电压 αβ 外环 QPR 产生电流参考，电流内环 QPR 产生桥电压命令，再调用 SVPWM |
| `Svpwm_Generate()` | 将 αβ 实际电压命令换成三相量，超限时等比例缩放，再注入连续零序并输出占空比 |

STM32F407 没有 TI C2000 的 TMU。本工程不使用查表：`sinf/cosf` 只在初始化或切频的低频路径执行，20 kHz 快速环使用 FPU 乘加递推。

SVPWM 的核心处理为：

```text
Va = Vα
Vb = -0.5Vα + √3/2·Vβ
Vc = -0.5Vα - √3/2·Vβ

若 max(Vabc)-min(Vabc) 超过 (duty_max-duty_min)·Vdc：
    三相矢量等比例缩小

Vcommon = -0.5·(max(Vabc)+min(Vabc))
dabc = duty_center + (Vabc+Vcommon)/Vdc
```

零序注入不会改变线电压；矢量缩放保持方向和三相对称性。

## 8. 软件保护和故障码

OLED 显示最低有效故障位的序号，`0` 表示无故障：

| OLED 码 | 故障位 | 含义 |
|---:|---|---|
| 1 | `PROTECTION_FAULT_MEASUREMENT` | 测量无效、ADC 解包/标定失败 |
| 2 | `PROTECTION_FAULT_SAMPLE_STALE` | ADC 帧序号停滞 |
| 3 | `PROTECTION_FAULT_INPUT_OVERCURRENT` | 输入电流绝对值超限 |
| 4 | `PROTECTION_FAULT_PHASE_OVERCURRENT` | 任一重构相电流绝对值超限 |
| 5 | `PROTECTION_FAULT_INPUT_OVERVOLTAGE` | 输入电压绝对值超限 |
| 6 | `PROTECTION_FAULT_DC_OVERVOLTAGE` | 直流母线过压 |
| 7 | `PROTECTION_FAULT_DC_UNDERVOLTAGE` | `RUN` 时直流母线欠压 |
| 8 | `PROTECTION_FAULT_OUTPUT_OVERVOLTAGE` | 任一重构相电压 `Va/Vb/Vc` 绝对值超限 |
| 9 | `PROTECTION_FAULT_EXTERNAL` | BKIN、驱动或外部故障 |
| 10 | `PROTECTION_FAULT_LOOP_OVERRUN` | 20 kHz 快速环执行时间超过 50 µs（一个控制周期） |
| 11 | `PROTECTION_FAULT_CONTROL` | 控制器参数、状态或数值计算失败 |
| 12 | `PROTECTION_FAULT_DC_MISMATCH` | 预留故障位，当前代码不置位（OLED 文本为 `DC SET MISMATCH`） |

另有 ADC 帧看门狗：首帧到达后，若 TIM8/ADC/DMA 中断链停摆导致超过 5 ms 没有新的 ADC 帧，后台任务会锁存 `PROTECTION_FAULT_MEASUREMENT`（OLED 码 1）并关断两级 PWM，避免功率级保持最后一次 CCR 输出。

中断优先级为：

| 中断 | 抢占优先级 |
|---|---:|
| TIM1/TIM8 Break | 0 |
| ADC DMA | 1 |
| ADC | 2 |
| TIM4 | 3（当前未启动） |
| SysTick | 15 |

软件过流依赖 20 kHz ADC 帧，最迟在一个 PWM 周期内动作；亚微秒至数微秒级关断必须依靠比较器、BKIN 和驱动器 EN/SD 硬件链路。

## 9. 上电前必须完成的配置

不要只把 `config_valid` 改为 `true`。`BoardPower_ValidateConfig()` 会继续检查以下内容：

1. 七路 ADC 的零点、每码比例、极性和合法原始范围；
2. 逆变电压/电流 QPR 参数；
3. 整流 Vdc PI、输入电流 QPR 和 100 Hz 陷波参数；
4. 输入/相电流、输入/母线/输出电压保护阈值；
5. 功率管、驱动器、桥臂极性和整流电流参考极性；
6. 实测安全死区；
7. 预充、相位就绪、驱动 EN/SD 和外部 FAULT 板级逻辑。

建议新增一个板级配置源文件，以强符号覆盖弱函数 `BoardPower_GetConfig()`，集中放置实测配置；不要把标定常数散落在 ADC 中断或控制算法中。

六个确认字段必须在对应实测完成后才置真：

```c
config_valid
adc_calibrated
gate_driver_pins_valid
power_stage_valid
deadtime_valid
protection_thresholds_valid
```

TA1015/TV1013 等互感器次级不得开路。PC4 的母线采样前端必须用分压、串联限流和硬件钳位确保 MCU 引脚始终处于 0～3.3 V，软件保护不能替代输入钳位。

## 10. 编译和测试

### 10.1 纯 C 主机测试

在工程根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1
```

测试不依赖 HAL 或 STM32 寄存器，覆盖：

- ADC 标定和相量重构，逆变/整流两种测量换算路径；
- 30/60 Hz 无查表相位递推和标准相序；
- PI、Tustin QPR 和 100 Hz 陷波；
- SVPWM 线电压关系、零序注入和过调制缩放；
- SVPWM 对固定及 100 Hz 纹波 Vdc 的 1/vdc 归一化（无长周期幅值包络回归）；
- MODE2 固定 Vdc 闭环多周期包络回归（RL 负载模型、延迟一周期装载）；
- 整流单极性 SPWM、慢环和 Vdc 软启动；
- 逆变电压/电流双闭环；
- 故障锁存和完整状态机；
- 三模式配置校验（`BoardPower_ValidateModeConfig`）和配置无效时的 RUN/PWM 硬锁止；
- MODE1 整流单级、MODE0 整流→逆变顺序开放的完整应用流程。

### 10.2 STM32CubeIDE

使用 STM32CubeIDE 导入本目录工程，可选择 `Debug` 或 `Release` 完整构建。修改 `.ioc` 并重新生成代码后，应重点复核：

- ADC 双模式、Rank 和 DMA 数据宽度；
- TIM1/TIM8 的中心对齐、RCR、同步和 CH4 ADC 触发；
- BKIN 极性、`AOE=0` 和中断优先级；
- `main()` 是否仍只初始化当前使用的外设；
- 自定义 `App` 源文件是否仍在两个构建配置中。

## 11. 建议的硬件验证顺序

1. 母线断电、驱动 EN/SD 关闭，下载固件并确认 OLED 显示 `FULL OFF`、无故障码。
2. 用信号源向 ADC 前端注入已知小信号，完成七路偏置、增益和极性标定。
3. 不接功率母线，用示波器检查 TIM1/TIM8 的 20 kHz、互补极性、同步关系和死区。
4. 分别拉高 PE15、PA6，确认对应 BKIN 立即清 MOE，且全局故障锁存后不会自动恢复。
5. 实现并测试 EN/SD、预充和相位就绪板级逻辑，再解除配置锁。
6. `MODE2_INVERTER` 使用约 30 V 隔离限流直流源（不低于运行欠压阈值 27 V）、小电压参考和阻性负载调试。
7. `MODE1_RECTIFIER` 使用隔离低压交流源及可吸能直流负载调试。
8. 最后进行 `MODE0_FULL` 两级联调，并逐步升压、升载。

禁止通过满母线硬短路验证保护，也不能依赖实验电源限流代替 MOSFET 的硬件过流保护。

## 12. 本次修改说明（三模式接入与 DCAC 包络修复）

### 12.1 DCAC 长周期包络根因与修复

20 kHz/60 Hz 载波比约 333，本身不会造成长周期包络。实际根因是：
`AppControl_Step()` 把逆变测量中的 `vdc` 强制替换成键盘设定值，且
`Measurement_ConvertInverter()` 直接丢弃 PC4 实测 Vdc。真实母线的
100/120 Hz 纹波通过 SVPWM 的 `1/vdc` 直接对 60 Hz 输出做 AM 调制，
与输出电压设定大小无关。修复内容：

1. `Measurement_ConvertInverter()` 按 `cal->vdc` 换算实测 Vdc；
   `Measurement_InverterCalibrationValid()` 要求 Ia/Ib/Vab/Vbc/Vdc 五通道。
2. MODE2 的 SVPWM 归一化和输出限幅一律使用实测 Vdc；键盘设定只作为
   期望母线/输出参考来源（推导输出线电压 RMS），不再替代调制分母。
3. 实测 Vdc 无效、欠压（RUN 时）或过压时按保护锁存并由状态机安全关断。
4. TIM1 CH1-3 与 TIM8 CH1/CH2 启用 OC preload：CCR 在 ADC ISR 写入后
   统一在下一个 update 事件转移，与“延迟一周期装载”一致；每帧只写一次
   CCR，不存在双缓冲错拍。TIM8 CH4（ADC 触发基准）保持原配置。
5. 新增宿主机回归测试：固定 Vdc 下多个 60 Hz 周期的幅值包络变化小于
   5%；100 Hz 纹波 Vdc 下 1/vdc 归一化重建电压与命令一致。

### 12.2 第一级 ACDC 三模式接入

1. 三种模式全部有效：`MODE0_FULL`（默认）、`MODE1_RECTIFIER`、
   `MODE2_INVERTER`；键盘 `0/1/2` 恢复模式选择，仅在 `OFF` 切换，
   切换时显式关闭两级 PWM 并作废延迟命令；模式写入使用短临界区，
   ADC 解包/换算不会跨模式错帧。
2. `AppControl_Init()` 同时初始化整流级并设置 DC 目标；Reset 同步复位。
3. ADC 路径按模式分发：MODE2 用逆变解包/五通道换算；MODE1 用含
   Vin/Vin_diag 校验的双 ADC 解包 + 仅 Iin/Vin/Vdc 的整流换算（三相量
   固定为 0）；MODE0 用全七通道换算。
4. 新增 `BoardPower_ValidateModeConfig(cfg, mode)`；原
   `BoardPower_ValidateConfig()` 保持兼容并按 MODE0 最严格校验；
   PI/QPR 零增益视为无效配置。
5. 默认配置：Iin 暂用与 Ia/Ib 相同的已标定 TA 参数、Vin 暂用与
   Vab/Vbc 相同的已标定 TV 参数（**同型传感器通道临时复用，换前端必须
   重标定**）；整流参数为保守可运行值（目标 60 V、dc max 65、
   slew 20 V/s、容差 1.5 V、确认 1000 拍、输入 RMS min 20 V、
   电流 RMS max 6 A、电感限压 60 V、duty 0.02～0.98）；保护阈值与
   32 Vrms/60 V 题目匹配（输入 10 A/60 V、母线 27～66 V）。
6. 控制调度：整流 RMS/陷波在所有状态下以 `enabled=false` 预热；
   RUN 且模式为 MODE1/0 时使能整流；MODE0 中滤波 Vdc 连续在容差内
   （默认 1000 拍）后才开放逆变；任一控制器失败锁存 CONTROL 并关两级。
7. TIM8 映射严格为 `rectifier_a→CCR2`、`rectifier_b→CCR1`；使能时先
   CCER 后 MOE，禁用时清 MOE/CCER 但保持 TIM8 计数以维持 ADC 触发。
8. OLED 首行显示 FULL/RECT/INV；VDC 行显示实测母线；整流模式显示
   整流 ready（`R:RDY/R:---`）。
9. `App_SetDcVoltageReference()` 在 MODE1/0 同时更新整流 DC 目标；
   `App_SetOutputFrequency()` 在 MODE1 返回 false；`App_SetPrechargeDone()`/
   `App_SetPhaseReady()` 在 MODE1/0 真实生效，无外部信号时按测量安全
   推导，绝不默认虚假置真。

### 12.3 仍需实机标定/校核的项目

- Iin（PA5）电流互感器、Vin（PC5）电压互感器的实际偏置、增益和极性
  （当前为同型通道临时复用值）；
- 整流桥臂极性 `bridge_polarity`、电流参考极性 `current_reference_sign`；
- 整流 PI/QPR 增益按实测前端和负载重新整定；
- 死区时间（TIM1/TIM8 当前均约 1 us，需按驱动器和功率管实测复核）、驱动传播延时；
- 各项保护阈值、BKIN 链路、预充和相位就绪的板级硬件逻辑。
