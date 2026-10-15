# three-phase-inverter · 三相逆变（DCAC）独立工程

STM32F407VGT6 上的三相逆变固件：**连续 SVPWM + 电压外环/电流内环双闭环（QPR）**，
输出线电压 32 Vrms（峰值 45.25 V）、基波 60 Hz；直流母线由**外部直流电源**提供。

> 本工程是从 2026 省赛 A 题“三相 AC-AC 变换器”整机工程里**拆出来的单级工程**：
> 只保留三相逆变级，整流级（ACDC）的模块、参数、测试全部移除。
> 整机工程在同仓库的 `refactor/readable` 分支，需要时对照着看。

---

## 1. 它做什么

控制结构（一个 20 kHz 快环走一遍）：

```
软启动参考 ──► 电压外环 QPR ──► 电流参考 ──► 电流内环 QPR ──► 桥压指令
    ▲              ▲                              ▲
  斜坡 1 s     Clarke(实测相电压)            Clarke(实测相电流)
                                    └──► SVPWM ──► TIM1 三相占空比
```

两个环都在 αβ 静止坐标系里做，不需要 Park 变换与角度同步——固定频率下
QPR 在谐振频率上的高增益已经保证稳态精度。

- **调制**：连续 SVPWM，20 kHz 中心对齐三角载波，TIM1 三相互补 + 死区 1.000 us
- **过调制**：整体等比例缩矢量（不逐相削顶，否则会产生低次谐波）
- **软启动**：相电压参考按 19.09 V/s 从 0 爬到额定 18.48 Vrms（约 1 s）
- **限幅**：占空比 2%~98%，并按当前母线值对参考降额
- **保护**：测量无效、采样停滞、相过流、母线过压、相电压过压、BKIN 外部故障、快环超时、控制环失败
- **状态机**：`OFF → SELF_TEST → PRECHARGE → READY → RUN → DISCHARGE`，故障一律进 `FAULT_LATCH`，必须人工复位
- **人机**：OLED 状态页 + 输出过压故障快照（5 页，D 键翻页）；键盘 `A` 切 30/60 Hz、`B`/`C` 输出线电压 ∓0.1 V、`D` 停机、`#` 长按 2 s 启动、`*` 长按 2 s 清故障

## 2. 目录

```
App/Inc, App/Src      纯算法（不碰寄存器，可在 PC 上编译运行）
  constants.h         工程常用数学常数（π、√2、√3 …）唯一真源
  app_types.h         跨模块类型：采样帧 / 测量值 / PWM 指令 / 状态快照
  app_config.*        集中式板级参数与“上电安全锁”（每个数都标了来历）
  measurement.*       ADC 解包、通道标定、三相量重构
  inverter_control.*  电压外环 + 电流内环 + 软启动
  svpwm.*             空间矢量调制
  qpr_controller.*    准比例谐振控制器（Tustin 离散）
  phase_recurrence.*  无查表相位发生器
  protection.*        故障判据、确认窗口与锁存
  power_state.*       电源状态机（唯一放行开关 pwm_allowed）
  app_control.*       单例门面：一次快环 = 换算 → 保护 → 状态机 → 双闭环 → 占空比
Core/Src, Core/Inc    板级：TIM1/ADC/DMA/SPI 配置 + board_runtime.c（唯一碰寄存器的地方）
host_tests/           宿主机测试与闭环仿真（只依赖 App/）
docs/                 代码规范与问题记录
```

## 3. 构建与运行

```powershell
# ① 离线回归测试（18 项，不需要硬件）
powershell -ExecutionPolicy Bypass -File .\host_tests\run_tests.ps1

# ② 闭环仿真（三相桥 + LC + 星形负载，跑 3 秒）
gcc -std=c11 -I App\Inc host_tests\sim_inverter.c App\Src\*.c -lm -o sim.exe
.\sim.exe 1        # 参数 1 = PC4 接通，0 = PC4 悬空（本工程不采母线，两者等价）

# ③ 编译固件：CubeIDE 导入本目录 → Debug → Build
```

改代码后**先跑 ①**，再跑 ②：仿真的稳态数字（45.20 V 线电压峰值）是这套控制
参数的基线，对不上就别上板。

## 4. 引脚（本工程用到的部分）

| 用途 | 引脚 | 说明 |
|---|---|---|
| 逆变 A 相 | PE9(高侧) / PE8(低侧) | TIM1_CH1 / CH1N |
| 逆变 B 相 | PE11(高侧) / PE10(低侧) | TIM1_CH2 / CH2N |
| 逆变 C 相 | PE13(高侧) / PE12(低侧) | TIM1_CH3 / CH3N |
| 逆变 BKIN | PE15 | 高有效，硬件立即清 MOE |
| 电流采样 | PC0(IN10)=Ib、PC1(IN11)=Ia | ADC1/ADC2 同步 |
| 线电压采样 | PC2(IN12)=Vab、PC3(IN13)=Vbc | ADC1/ADC2 同步 |
| 母线采样 | PC4(IN14)=Vdc | **本工程未使用**（见第 6 节） |
| OLED | PC10 SCK / PC12 MOSI / PC11 DC / PD0 RST / PD1 CS | SPI3 |
| 键盘 | 行 PE1/PE3/PE4/PD11，列 PD3/PD4/PD7/PD10 | 4×4 扫描 |

ADC 由 TIM8 CH4 的比较事件（CCR4 = 3447，即载波峰值前 752 个计数）触发，
双 ADC 同步转换、DMA 搬运。**TIM8 只当触发源用**：它的通道输出使能位一直
为 0，不会驱动任何功率管（原整流桥的引脚保持静默）。

## 5. 关键参数（全部集中在 App/Src/app_config.c）

| 参数 | 值 | 说明 |
|---|---|---|
| 快环频率 | 20 kHz | TIM1 中心对齐，ARR=4199 |
| 死区 | 1.000 μs | DTG=148（168 MHz 下 168 个 tDTS） |
| 输出线电压 | 32 Vrms | 28~34 V 现场微调，与母线解耦 |
| 母线目标 | 62 V | 调制分母（SET 值，非实测） |
| 电流参考限幅 | 3.0 A | 相电流峰值上限（额定 2.83 A） |
| 电压/电流 QPR | 0.05+30 / 2.5+30 | Kp + Kr，Wc = 1 / 5 rad/s |
| 相过流阈值 | 3.5 A | 高于参考限幅，触发即说明电流环失控 |
| 输出过压阈值 | 32 V | 相电压瞬时值（额定峰值 26.1 V） |

改任何一个数之前，先读 `app_config.c` 里该字段的来历注释。

## 6. 母线为什么用 SET 值而不是实测值

台架上 PC4（Vdc 采样）没有接线，实测恒为 0 V，当调制分母会直接除零。
所以调制分母用**用户设定的母线值**（`App_SetDcVoltageReference`，30~65 V），
它只影响调制深度，不影响输出电压目标（两者已解耦）。

要改成用实测母线：把 `app_control.c` 里 `inverter_measurement.vdc = ...` 那行
换成 `measurement.vdc`，并把同一函数里传给 `Protection_Evaluate` 的
`run_expected` 改成 `(app->state.state == RUN)`，两处必须一起改。

## 7. 已知问题

**DCAC 现场输出只有 3 V 多**（母线 >60 V 直入、频率正确、每隔几秒一次频率
震荡）：本仓库未复现——同一份控制代码在闭环仿真里输出 45.20 V 峰值完全达标。
排查顺序见 [docs/ERROR_LOG.md](docs/ERROR_LOG.md)，第一优先是**栅极驱动的
EN/SD 使能脚**（原理图上还没分配）。

## 8. 代码规范

见 [docs/STYLE.md](docs/STYLE.md)。版式由根目录 `.clang-format` 统一
（大括号跟随语句、单语句分支可省大括号、行宽 120），改完代码跑一次：

```powershell
python -m pip install clang-format
clang-format -i App\Inc\*.h App\Src\*.c Core\Src\main.c Core\Src\board_runtime.c
```
