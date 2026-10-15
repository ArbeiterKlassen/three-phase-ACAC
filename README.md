# three-phase-inverter · 三相 AC-AC 变换器固件（重构版）

STM32F407VGT6 上的**单相有源整流 + 三相逆变**两级变换器固件，用于 2026 省赛 A 题。
20 kHz 快速控制环，整流级用单极性 SPWM，逆变级用连续 SVPWM，三种运行模式可切换。

> 本仓库是在原工程基础上的**可读性重构**：代码行为不变（30 项主机测试全部通过），
> 但模块划分、命名、注释风格按统一规范重写，并删除了不参与构建的历史代码。

---

## 运行模式

| 模式 | 功能 | 用哪一级 | PWM 行为 |
|---|---|---|---|
| `MODE_FULL` | 单相整流 + 三相逆变联动 | TIM8 + TIM1 | 整流先建立母线，就绪后才开放逆变 |
| `MODE_INVERTER` | 只测三相逆变级 | TIM1 | TIM8 计数但门极关闭；Vdc 用设定值 |
| `MODE_RECTIFIER` | 只测单相整流级 | TIM8 | TIM1 门极关闭 |

上电默认 `MODE_INVERTER`。模式只能在 `OFF` 状态切换。

---

## 功能特性

### 三相逆变级（DCAC）
- **SVPWM**：αβ 指令 → 三相 → 注入零序 → 归一化占空比，含过调制等比例缩放
- **双闭环**：相电压外环 QPR 给出电流参考，电流内环 QPR 给出桥电压指令
- **开环诊断通道**：绕过两个环，把电压参考直接送 SVPWM
- **软启动**：参考按设定斜率爬升，避免上电冲击
- **无查表相位生成**：正交递推 + 每 256 拍归一化，`sinf/cosf` 只在上电/切频时用

### 单相整流级（ACDC）
- **单极性 SPWM**：两桥臂反相调制，占空比绕 50% 对称
- **输入电流 QPR 内环 + 母线电压 PI 外环**，外环 1 kHz、内环 20 kHz
- **100 Hz 陷波器**抑制母线二倍频纹波
- **400 点真有效值**输入电压窗口，用于建立启动条件
- **门极门控**：外环未请求电流时桥臂保持高阻（仅二极管通路），
  避免占空比被限幅后把母线泵升

### 测量
- 双 ADC 同步采样，TIM8 CH4 触发，DMA 双缓冲
- 四路 Rank 覆盖两路电流、两路线电压、母线电压与一路诊断重复采样
- 只采两相电流与两路线电压，其余量按基尔霍夫关系重构
- 通道标定统一为 `物理量 = (原始码 − 零点) × 每码比例`

### 保护
- 硬件：两路 BKIN（逆变 PE15 / 整流 PA6），高有效，直接清 MOE
- 软件：过流 / 过压 / 欠压 / 测量 / 采样陈旧 / 快环超时，全部带确认窗口
- 故障锁存，需人工确认后复位
- CPU 异常（HardFault 等）与 `Error_Handler` 都会清两级 MOE

### 人机接口
- OLED（SPI3）显示状态、模式、频率、电压电流与故障码
- 4×4 矩阵键盘：选模式、切频率、输出参考微调、启停、复位故障

---

## 项目结构

```
App/
├── Inc/  Src/
│   ├── app_config.*        集中式参数与"上电安全锁"：标定、限幅、保护阈值
│   ├── app_control.*       三模式调度、20 kHz 快环、对外单例接口
│   ├── app_types.*         跨模块共享的类型定义（状态、测量、配置）
│   ├── measurement.*       ADC 解包、通道标定、三相量重构
│   ├── inverter_control.*  三相逆变：电压外环 + 电流内环 + 软启动 + SVPWM 调用
│   ├── svpwm.*             连续 SVPWM：零序注入与过调制缩放
│   ├── rectifier_control.* 单相整流：母线外环 + 电流内环 + 陷波 + RMS + 软启动
│   ├── rectifier_pwm.*     单极性 SPWM：调制量 → 两桥臂占空比
│   ├── qpr_controller.*    Tustin 离散准 PR 控制器（带输出跟踪抗饱和）
│   ├── pi_controller.*     带抗饱和的离散 PI
│   ├── notch_filter.*      Tustin 离散二阶陷波器
│   ├── phase_recurrence.*  无查表正交相位递推
│   ├── protection.*        故障判断、确认窗口与锁存
│   └── power_state.*       上电 → 自检 → 预充 → 就绪 → 运行 → 放电 → 停机
Core/
├── Inc/  Src/
│   ├── main.c              HAL/时钟初始化 + 后台循环入口
│   ├── board_runtime.*     板级适配：ADC DMA、PWM 寄存器、BKIN、键盘、OLED
│   ├── adc.c / dma.c       双 ADC 同步采样与循环 DMA
│   ├── tim.c               TIM1/TIM8 的 PWM、同步、死区与 Break 配置
│   ├── gpio.c              引脚复用与初值
│   ├── key_board.c         4×4 矩阵键盘扫描
│   └── oled.c / spi.c      OLED 驱动与 SPI3
host_tests/
├── run_tests.ps1           回归测试入口
├── test_main.c             30 项算法与状态机测试（不依赖 HAL）
├── sim_rectifier.c         ACDC 闭环仿真（真实控制器 + 母线电容模型）
└── sim_inverter.c          DCAC 闭环仿真（三相桥 + LC + 负载）
docs/
└── ERROR_LOG.md            问题记录：已修 / 未决 / 判别方法
```

`Drivers/` 与 `Middlewares/` 是 ST 官方 HAL 与 CMSIS-DSP，不修改。

---

## 构建

**STM32CubeIDE**：导入本目录 → 选择 `Debug` 或 `Release` → Build → 下载。

**命令行无头编译**（需 STM32CubeIDE，把路径换成自己的）：

```
stm32cubeidec.exe --launcher.suppressErrors -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data <工作区目录> -import <本目录> -build "bianliuqifuzainibian/Debug"
```

目标芯片 STM32F407VGT6（LQFP100），HSE 8 MHz，SYSCLK 168 MHz。

---

## 测试与仿真

改代码后**先跑回归测试**，30 项里挂了任何一项都不要烧板子：

```powershell
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1
```

两个闭环仿真把**真实的控制器源码**接到简化被控对象上，用于验证开环/闭环行为：

```powershell
# ACDC：单相全桥 + 升压电感 + 母线电容
gcc -std=c11 -I App\Inc host_tests\sim_rectifier.c App\Src\*.c -lm -o sim_rect.exe
.\sim_rect.exe

# DCAC：三相全桥 + LC 滤波器 + 星形负载
gcc -std=c11 -I App\Inc host_tests\sim_inverter.c App\Src\*.c -lm -o sim_inv.exe
.\sim_inv.exe 1        # 1 = PC4 接通，0 = PC4 悬空
```

被控对象参数（电感、电容、负载）是**假设值**，拿到原理图后应替换重跑。

---

## 引脚分配

### 三相逆变级（TIM1，AF1，中心对齐 20 kHz，带死区）

| 相位 | 高侧 | 低侧 | 通道 | 占空比来源 |
|---|---|---|---|---|
| A | PE9 | PE8 | CH1 / CH1N | `inverter_a` → `CCR1` |
| B | PE11 | PE10 | CH2 / CH2N | `inverter_b` → `CCR2` |
| C | PE13 | PE12 | CH3 / CH3N | `inverter_c` → `CCR3` |

### 单相整流级（TIM8，AF3）

| 桥臂 | 高侧 | 低侧 | 通道 | 占空比来源 |
|---|---|---|---|---|
| A | PC7 | PB0 | CH2 / CH2N（PWM1） | `rectifier_a` → `CCR2` |
| B | PA7 | PC6 | CH1N / CH1（**CH1 用 PWM2**） | `rectifier_b` → `CCR1` |

> 桥臂 B 的实物高低侧与定时器主/互补顺序相反，CH1 配成 PWM2 让相位对齐。
> **改动这里之前先想清楚**——配错就是上下管直通。

### ADC（双 ADC 同步，TIM8 CH4 → TRGO 触发）

| Rank | ADC1（CDR 低 16 位） | ADC2（CDR 高 16 位） |
|---|---|---|
| 1 | `Iin` = PA5 | `Vin` = PC5 |
| 2 | `Ib` = PC0 | `Ia` = PC1 |
| 3 | `Vab` = PC2 | `Vbc` = PC3 |
| 4 | `Vdc` = PC4 | `Vin` 诊断重复采样 |

### 保护与其它

| 信号 | 引脚 | 说明 |
|---|---|---|
| 逆变 BKIN | PE15 | 高有效，硬件立即清 MOE |
| 整流 BKIN | PA6 | 高有效 |
| OLED SPI3 | PC10 SCK / PC12 MOSI / PC11 DC / PD0 RST / PD1 CS | |
| 键盘行 | PE1 / PE3 / PE4 / PD11 | 推挽输出，扫描时逐行置高 |
| 键盘列 | PD3 / PD4 / PD7 / PD10 | 下拉输入 |

---

## 关键参数

全部集中在 `App/Src/app_config.c`，改动前先读该文件顶部的注释。

| 参数 | 值 | 说明 |
|---|---|---|
| 快环频率 | 20 kHz | TIM1/TIM8 中心对齐，ARR=4199 |
| 死区 | 1.000 μs | DTG=148，168 MHz 下 168 个 tDTS |
| 母线目标 | 62 V | 41 V 输入峰值 58 V 时调制比 0.935，避开限幅 |
| 输出线电压 | 32 Vrms | 与母线解耦，可在 28~34 V 现场微调 |
| 整流电流限幅 | 3.8 A RMS | 满载所需，**峰值距采样量程顶仅约 6%** |
| 输入过流阈值 | 5.6 A | 勿再调高，见下方风险说明 |
| 逆变电压/电流 QPR | 0.05+30 / 2.5+30 | Kp + Kr，Wc = 1 / 5 rad/s |
| 整流 PI / QPR | 1+1 / 5+60 | 母线外环 / 输入电流内环 |

---

## 已知风险（接手必读）

1. **输入电流量程余量只有约 6%**：3.8 A RMS 对应 5.37 A 峰值，TA 可测上限 5.68 A。
   效率或功率因数不达标就会顶到量程边界。**不要把限幅继续调高。**
2. **过流被归类成"测量故障"**：电流超过 5.68 A 时采样先被判量程超限，
   走不到过流判断，OLED 故障码显示 1 而不是 3。排查时注意这个误导。
3. **无预充电路**：板子没有继电器，也没有实现预充控制。
   上电浪涌靠回路阻抗限制，固件只能扛过去，不能替代硬件预充。
4. **DCAC 现场输出只有 3 V**（本仓库未复现）：同一份代码在闭环仿真里输出
   45.20 V 峰值完全达标。排查顺序见 [docs/ERROR_LOG.md](docs/ERROR_LOG.md)——
   第一优先是**栅极驱动的 EN/SD 使能脚**。

---

## 代码规范

见 [docs/STYLE.md](docs/STYLE.md)。要点：

- 模块一对 `.h/.c`，对外函数用 `模块_动词` 命名
- 头文件里**每个声明上方一行中文注释**，说清做什么、单位是什么
- 结构体**每个字段行尾注释**，标明单位与含义
- 文件头用简短中文说明职责，不写版权套话
- 函数保持单一职责；超过 100 行先考虑拆分
- 版式由根目录的 `.clang-format` 统一（大括号跟随语句、单语句分支可省大括号、
  行宽 120），改完代码跑一次 `clang-format -i` 即可：
  `python -m pip install clang-format`
