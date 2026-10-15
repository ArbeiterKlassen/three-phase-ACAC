# 交接说明（2026 省赛 A 题 · 三相 AC-AC 变换器固件）

> 这份是**接手入口**。设计细节看 [README.md](README.md)，问题记录看
> [docs/ERROR_LOG.md](docs/ERROR_LOG.md)。

---

## 1. 这是什么

STM32F407V(E-G)Tx 上的三模式 AC-AC 变换器固件，用于 2026 省赛 A 题：

| 模式 | 功能 | 用哪级 |
|---|---|---|
| `MODE0_FULL` | 单相整流 + 三相逆变联动 | TIM8 + TIM1 |
| `MODE1_INVERTER` | 只测三相逆变级 | 只 TIM1 |
| `MODE2_RECTIFIER` | 只测单相整流级 | 只 TIM8 |

上电默认 `MODE1_INVERTER`，20 kHz 快环，SVPWM 调制，20 kHz/1 kHz 双环。

**当前分支是交接版**：8/1 版本 + 本轮修复（见第 4 节）。

---

## 2. 快速开始

```powershell
# ① 离线回归测试（30 项，不需要硬件）
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1

# ② ACDC 闭环仿真（真实控制器 + 母线电容模型）
gcc -std=c11 -I App\Inc host_tests\sim_rectifier.c App\Src\*.c -lm -o sim_rect.exe
.\sim_rect.exe

# ③ DCAC 闭环仿真（三相桥 + LC + 负载，MODE1）
gcc -std=c11 -I App\Inc host_tests\sim_inverter.c App\Src\*.c -lm -o sim_inv.exe
.\sim_inv.exe 1        # 参数 1 = PC4 接通，0 = PC4 悬空

# ④ 编译固件（CubeIDE）
#    导入本目录 → Debug → Build；或命令行见 README 第 10.2 节
```

改代码后**先跑 ①**。30 项里挂了任何一项，别烧板子。

---

## 3. 引脚接口（已三方核对）

`.ioc` / README / 代码三处逐条对齐过，结论：**一致**。

### 3.1 三相逆变级（TIM1，AF1，中心对齐 20 kHz，带死区）

| 相位 | 高侧 HI | 低侧 LI | 通道 | 占空比来源 |
|---|---|---|---|---|
| A | **PE9** | **PE8** | TIM1_CH1 / CH1N | `inverter_a` → `TIM1->CCR1` |
| B | **PE11** | **PE10** | TIM1_CH2 / CH2N | `inverter_b` → `TIM1->CCR2` |
| C | **PE13** | **PE12** | TIM1_CH3 / CH3N | `inverter_c` → `TIM1->CCR3` |

### 3.2 单相整流级（TIM8，AF3）

| 桥臂 | 高侧 HI | 低侧 LI | 通道 | 占空比来源 |
|---|---|---|---|---|
| A | **PC7** | **PB0** | TIM8_CH2 / CH2N（PWM1） | `rectifier_a` → `TIM8->CCR2` |
| B | **PA7** | **PC6** | TIM8_CH1N / CH1（**CH1 用 PWM2**） | `rectifier_b` → `TIM8->CCR1` |

> 桥臂 B 的实物 HI/LI 与定时器主/互补顺序相反，所以 CH1 配成 PWM2 让相位对齐。
> **改这里之前先想清楚**，配错就是上下管直通。

### 3.3 ADC（双 ADC 同步，TIM8 CH4 → TRGO 触发）

| Rank | ADC1（CDR 低 16 位） | ADC2（CDR 高 16 位） |
|---|---|---|
| 1 | `Iin` = PA5 / IN5 | `Vin` = PC5 / IN15 |
| 2 | `Ib` = PC0 / IN10 | `Ia` = PC1 / IN11 |
| 3 | `Vab` = PC2 / IN12 | `Vbc` = PC3 / IN13 |
| 4 | `Vdc` = PC4 / IN14 | `Vin` 诊断重复采样 = PC5 |

**注意**：MODE1 只解包 Rank2/Rank3（`Measurement_UnpackInverterAdcPairs`），
Vdc 用用户 SET 值代替实测——不是 bug，是设计（见 README 3.1）。

### 3.4 硬件保护与触发

| 信号 | 引脚 | 说明 |
|---|---|---|
| 逆变级 BKIN | **PE15** | TIM1_BKIN，高有效，硬件立即清 MOE |
| 整流级 BKIN | **PA6** | TIM8_BKIN，高有效 |
| ADC 触发 | TIM8_CH4 | 不输出到引脚，内部 TRGO |

### 3.5 HMI

| 功能 | 引脚 |
|---|---|
| OLED SPI3 | PC10 SCK / PC12 MOSI / PC11 DC / PD0 RST / PD1 CS |
| 键盘行 R1-R4 | PE1 / PE3 / PE4 / PD11 |
| 键盘列 C1-C4 | PD3 / PD4 / PD7 / PD10 |

---

## 4. 本轮改了什么

详细根因与证据见 [docs/ERROR_LOG.md](docs/ERROR_LOG.md)。

| # | 问题 | 修复 |
|---|---|---|
| E1 | `host_tests` 在 Windows 上链接失败，离线验证从未跑过 | `_WIN32` 下弱符号退化为强定义 |
| E2 | README 四处仍写 3.0 A / 5.0 A，与代码的 3.8 A / 5.6 A 矛盾 | 统一为 3.8 A / 5.6 A |
| E3 | **整流级上电 0.3 ms 即锁死、毫无输出**（充电浪涌越过电流量程被判"测量无效"） | 测量无效确认窗口 2 拍 → 2000 拍（100 ms） |

E3 的验证：仿真里母线从 0 V 起充 → `state=RUN`、`faults=0`、母线稳在 62.07 V。

---

## 5. 还没解决的问题（接手第一优先）

### DCAC 输出只有 3 V 多（MODE1，母线直入 >60 V）

**已排除**：控制器算法。同一份代码在闭环仿真里输出 **45.20 V 峰值**
（= 32 Vrms × √2，完全达标），软启动约 1 s，无故障。

**所以重点在"信号有没有真的到 MOSFET"**，按可疑度：

1. **栅极驱动的 EN/SD 使能脚**——README 5.5 节把 `EN_INV` 列为"尚未分配"。
   量驱动板 EN/SD 电平；示波器看**栅极**（不是 MCU 引脚）有无 20 kHz。
2. 母线实际电压与 SET 值是否一致（MODE1 用 SET 当调制分母）。
3. 衰减发生在哪一级：桥臂中点 → 滤波后，逐级量。
4. 软启动 1 s 内是否被动重启（`inverter_enable` 周期性丢失）。

现场如果 OLED 报**故障码 1**，先怀疑过流/浪涌，别先拆线——见 ERROR_LOG 的说明。

### 其它已知薄弱点

- **电流量程余量只有约 6%**：3.8 A RMS 限幅对应 5.37 A 峰值，TA 上限 5.68 A。
  效率不达标就会顶到量程边界。**不要靠调高限幅解决。**
- **过流被归类成"测量故障"**：超过 5.68 A 时采样先被判量程超限，走不到过流判断，
  故障码会是 1 而不是 3。建议后续修正分类。
- **无预充电路**：板子上没有继电器，也没有实现预充控制（`EN_RECT` 未分配）。
  靠回路阻抗限制上电浪涌——固件只是能扛过去，替代不了硬件预充。
- **测试覆盖盲区**：现有 30 项用的软启动参数比生产值快 1200~20000 倍，
  真实斜坡时序靠新的两个闭环仿真补。

---

## 6. 交接注意

- 改 `.ioc` 重新生成会**重写 USER CODE 标记之外的一切**，动手前先提交。
- `App/` 是手写代码，`Core/` 里 CubeMX 生成的部分不要手改。
- `icode/` 是历史参考代码，不参与构建。
- 仿真里的电感/电容/负载是**假设值**，有原理图后应替换重跑。
