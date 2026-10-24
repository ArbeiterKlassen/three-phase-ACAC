# 单半桥 ACDC · STM32F407 独立工程

本项目实现：

```text
单相 AC → 4×肖特基不控全桥 → Lf/Cf → Lb → 单半桥 Boost → Cbus
```

目标母线为 62 V。最终 ACAC 工程原有的四路输出采样全部保留；ACDC 控制只使用
`Iin / Vcf / Vdc`，不假设另有一路 `Lb` 电流传感器。

> 这是根据平均模型和现场问题新写的独立工程，不复用旧版“双半桥有源 PFC”
> 的 PWM 和控制入口。旧工程仍保留在仓库其他分支。

## 1. 当前安全状态

当前实验台构建已允许启动，但上电仍**不会自动开管**。`App/Inc/acdc_config.h` 中：

```c
#define ACDC_HARDWARE_COMMISSIONED (1)
```

启动仍由人工按键触发。正式复测前建议确认：

1. 确认半桥 HI=PC7、LI=PB0，且没有与另一组 PA7/PC6 混接；
2. 标定 `Iin / Vcf / Vdc`，并确认原 PC0..PC3 四路没有被改动；
3. 用限流电源、预充电阻/NTC 验证零母线合闸；
4. 示波器确认 1 us 死区、互补关系和自举高侧供电。

完整验收见 [COMMISSIONING.md](docs/COMMISSIONING.md)。

## 2. 硬件接口

### 2.1 半桥

功率半桥固定为：

| 高侧 HI | 低侧 LI | TIM8 通道 |
|---|---|---|
| **PC7** | **PB0** | CH2 / CH2N，PWM1 |

> 现场曾把驱动板 HI/LI 对调。此错误会让低侧指令实际驱动高侧，启动时出现
> 电感蜂鸣、母线快速冲高并报 `faults=0x8`。接线必须逐根核对，不能仅凭
> 排线方向判断；对调修复后，RUN 稳态母线实测均值约 62.4 V。

另一组 PA7/PC6 不参与本工程，CCER 始终关闭。

### 2.2 ADC 与保护

双 ADC 由 TIM8 CH4 的 TRGO 在 20 kHz 中心对齐周期内触发，DMA 使用两块交替
帧，避免 CPU 与 DMA 同时访问同一帧。

| 物理量 | 引脚 | ADC | rank |
|---|---|---|---|
| `Iin`（有符号交流电流） | PA5 | ADC1_IN5 | 1 低 16 位 |
| `Vcf`（桥后 LC 电容电压） | **PC5** | ADC2_IN15 | 1 高 16 位 |
| `Vdc`（母线电压） | **PC4** | ADC1_IN14 | 4 低 16 位 |
| 原 Vab 输出采样 | PC2 | ADC1_IN12 | 3 低 16 位，**保留不动** |
| 外部 Break | PA6 | TIM8_BKIN，高有效 | — |
| 启动/停止/清故障 | PC13 | USER_KEY，低有效 | — |
| 矩阵键盘 `#` | PE4 / PD7 | 行 / 列，低/高电平扫描 | — |
| 状态 LED | PB8 | GPIO | — |
| OLED | PC10/PC12 SPI3，PC11 DC，PD0 RES，PD1 CS | 128×64 | — |

原有的两路电流、两路电压采样 `PC0/PC1/PC2/PC3` 全部保留，不能挪作 ACDC
用途。整流侧只使用 **PC5 / ADC2_IN15** 这一路 Vcf；新前端采用：

```text
Vcf ── 48.4 kΩ ──┬── PC5/ADC2_IN15
                  │
                1.9 kΩ
                  │
                 GND
```

- 分压比：`1.9 / (1.9 + 48.4) = 0.037773`
- ADC 引脚达到 3.3 V 时，高压侧理论值：`87.36 V`
- ADC LSB：`3.3 / 4095 = 0.00080586 V/码`
- 换算系数：`0.00080586 / 0.037773 = 0.0213341 V/码`
- 有效窗口取 `0..4031` 码时，Vcf 测量上限约 `86.0 V`

该量程覆盖 41 V RMS 输入的 58 V 峰值。1.9 kΩ 节点的源阻抗对当前 28-cycle
采样时间足够低；若实际增加了较大的 RC 滤波，应同步增加 ADC 采样周期。

Vcf 是单端、从 0 V 向上取的脉动直流电压，不使用交流采样的 1.65 V 中点偏置；
换算出的轻微负值会直接钳到 0 V。

PB1 保持 TIM8_CH3N 用途，不配置为 ADC；PA0/PA1 保持未使用。

## 3. 控制结构

```text
1 kHz 母线 PI + 100 Hz 陷波
        │
        └─► Iamp（输入电流 RMS 指令）
                │
                └─► Iref = Iamp · Vcf / Vcf_rms
                        │
                        └─► 20 kHz 输入电流 PI → vL
                                │
                                └─► d_low = 1 − (vCf − vL) / Vbus
```

关键实现：

- 400 点、20 ms 滑动窗口计算 `Vcf_rms`，与 50 Hz 严格对应；
- 电流反馈使用 `fabs(Iin)`，不需要 `Lb` 电流传感器；
- 下管占空比限制为 5%–95%，最低 5% 用来刷新高侧自举电容；
- 高压侧比较值使用 `1-d_low`，死区由 TIM8 硬件插入；
- 软启动斜率 60 V/s；
- 母线参考从 0 V 起坡；参考尚未爬过当前母线时保持门极关闭；
- 只有母线 PI 产生正电流指令后才开放 PWM，避免启动前馈提前抢能量；
- 外环程序限幅 5.5 A RMS，但硬件瞬时过流阈值仍是 5.6 A；
- 过流、输入过压、母线过压、运行中测量失效、控制失败、BKIN 全部锁存，
  不自动恢复。

## 4. 状态与操作

| 状态 | 行为 | LED |
|---|---|---|
| `LOCKED` | 未 commissioning，永不开管 | 100 ms 闪 |
| `OFF` | 测量正常，等待按键 | 500 ms 闪 |
| `WAIT` | 等 Vcf RMS 和预充稳定 100 ms | 500 ms 闪 |
| `RUN` | 闭环运行 | 常亮 |
| `FAULT` | 门极锁存关闭 | 100 ms 闪 |

矩阵键盘 `#` 长按 2 s 启动/停止；PC13 用户键仍保留为备用启停/清故障。

PC13 按键消抖 30 ms：

- `OFF/WAIT/RUN`：切换启动请求；
- `FAULT`：清除锁存并回到 `OFF`，需要再按一次才重新请求启动。

默认 `ACDC_AUTO_START=0`，即每次上电后必须按键启动。

`#` 只负责发出启动请求，不会绕过 `WAIT` 条件。若长按后停在 `WAIT`，先看 OLED：

- `Vcf RMS` 必须至少 10 V；
- `Bus` 必须在 30..63 V 并连续保持 100 ms。

例如输入已经整流到约 45 V、但 `Bus` 显示约 14 V 时，固件会拒绝开管。这不是
按键失效，而是 PC4 母线采样/分压或共地仍有问题；不要通过降低母线阈值来绕过。

OLED 每 200 ms 刷新一次，显示状态、Iin、Vcf、Vdc、占空比、故障码和门极
是否开启。

## 5. 保护阈值

| 项目 | 阈值 | 行为 |
|---|---:|---|
| 输入电流瞬时值 | >5.6 A | 立即锁存 |
| 输入电压瞬时值 | >65 V 连续 3 拍 | 当拍关门，150 us 后仍越限才锁存 |
| 母线过压 | >66 V 连续 3 拍 | 当拍关门，150 us 后仍越限才锁存 |
| 母线运行欠压 | <30 V 连续 40 ms | 立即关门，随后锁存 |
| 运行中测量无效 | 一旦发生 | 当拍关门并锁存 |
| 启动前测量无效 | 连续 100 ms | 锁存 |
| TIM8 BKIN | 高有效 | 硬件清 MOE + 软件锁存 |
| 控制结果无效 | 任一拍 | 立即锁存 |

## 6. 仿真结果（交流电流 + 单路 Vcf 反馈）

本工程的控制规律在独立平均模型
[`host_tests/sim_input_current.c`](host_tests/sim_input_current.c) 中验证：

| 输入 | 负载 | 母线均值 | PF | 电流 THD |
|---|---:|---:|---:|---:|
| 31 V RMS | 1.8 A | 61.36 V | 0.977 | 16.7% |
| 36 V RMS | 1.8 A | 61.89 V | 0.988 | 12.8% |
| 41 V RMS | 1.8 A | 61.90 V | 0.991 | 12.1% |
| 36 V RMS | 1.0 A | 61.95 V | 0.994 | 10.3% |

这说明“只测网侧电流 + 一路桥后 Vcf”仍可闭环，但：

- THD 仍在约 10%–17%，不能宣传满足 THD<5%；
- 31 V 满载时输入电流已到 3.78 A RMS，峰值约 5.4 A，距离 5.6 A OCP
  只有约 3%；
- 仿真效率未计开关、磁芯、死区和驱动损耗。

## 7. 编译与测试

宿主机算法测试：

```powershell
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1
```

编译功率级参考仿真：

```powershell
gcc -std=c11 -O2 -Wall -Wextra host_tests\sim_input_current.c -lm -o sim_acdc.exe
.\sim_acdc.exe 36 1.8 10 2 --ilimit=5.5
```

CubeIDE 2.2.0 已实际构建通过（下面是默认 `COMMISSIONED=0` 安全锁定构建的尺寸）：

```text
Debug:   text 37716 B, data 92 B, bss 5452 B
Release: text 25856 B, data 92 B, bss 5452 B
```

命令行复编（把工作区路径换成自己的临时目录）：

```powershell
& "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\stm32cubeidec.exe" `
  -nosplash -consoleLog `
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data "$env:TEMP\acdc_cubeide_ws" `
  -import "D:\stm32proj\three-phase-inverter-readable\standalone\acdc_halfbridge_f407" `
  -cleanBuild "acdc_halfbridge_f407/Release"
```

构建产物位于 `Debug/` 和 `Release/`，主文件：

```text
Debug/acdc_halfbridge_f407.elf
Release/acdc_halfbridge_f407.elf
```

命令行构建不保证同步刷新 `.hex/.bin`。烧录前应从本次 ELF 重新生成，避免把
上一次构建的旧固件烧进芯片：

```powershell
$objcopy = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-objcopy.exe"
& $objcopy -O ihex .\Release\acdc_halfbridge_f407.elf .\Release\acdc_halfbridge_f407.hex
& $objcopy -O binary .\Release\acdc_halfbridge_f407.elf .\Release\acdc_halfbridge_f407.bin
```

## 8. 已知限制

- `Lf=2 mH, Cf=10 uF, Lb=1 mH, Cbus=1000 uF` 仍是模型假设，必须替换成
  实物值并重跑；
- 输入电流量程决定 31 V 满载只能“擦线通过”，换传感器或降低输出功率更稳；
- 小于 0.5 A 的轻载 PF/THD 会明显变差；
- 零母线直接合闸会产生很大的 LC 涌流，软件预充流程不能替代硬件预充；
- 原板没有分配驱动器的 `EN_RECT`/`SD` 使能脚；如果所用驱动板需要使能，
  必须把该脚接成上电即安全关闭、只有 MCU 明确拉高后才允许驱动的逻辑；
- 若驱动器的 HI/LI 有效电平不是“高有效”，必须在 GPIO/驱动侧反相，并重新
  验证 MOE、死区和 Break。
