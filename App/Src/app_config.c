#include "app_config.h"

#include <math.h>

/*
 * 板级配置的默认实现声明为弱符号，允许板级文件提供强定义覆盖。
 *
 * 注意：弱符号在 ELF（arm-none-eabi，即单片机目标）上是一等公民，链接器
 * 直接支持；但 PE/COFF 没有弱绑定，GCC 只能用 `.weak.<符号>.<目标>` 别名
 * 段模拟，而 MinGW 的 binutils 不解析该别名段——结果是 host_tests 在
 * Windows 上必然报 “undefined reference to BoardPower_GetConfig”。
 * 因此在 PE 上退回普通强定义：板级覆盖只在真实目标（ELF）上成立，
 * 主机测试本来也不需要覆盖。
 */
#if defined(__GNUC__) && !defined(_WIN32)
#define APP_WEAK __attribute__((weak))
#else
#define APP_WEAK
#endif

/*
 * ═══════════════════════════════════════════════════════════════════════
 *  g_board_power_config —— 整块板子的"唯一真源"
 * ═══════════════════════════════════════════════════════════════════════
 *
 * 读法：每个字段按"含义 → 来历 → 改动风险"写注释。
 *   - 来历 = 实测标定数据 / 仿真（SLX 模型 + host_tests）整定值 /
 *     按公式反算；凡是没写来历的数一律当"待确认"，别当真；
 *   - 单位约定：电压 [V]、电流 [A]、时间 [s]、频率 [Hz]、
 *     归一化占空比 0~1、原始采样 [ADC 码 0~4095]。
 *
 * 改动纪律（按顺序，一步都别跳）：
 *   ① 改完先跑 host_tests\run_tests.ps1（30 项离线回归）；
 *   ② 再跑 host_tests\sim_inverter.c 与 sim_rectifier.c，与 README 的
 *      验证数字逐字比对（DCAC 稳态 45.20 V 峰值、ACDC 母线 62.08 V）；
 *   ③ 最后才上板：先空载、再低压、最后满压。
 *
 * 写这张表的前提（硬件哥已确认，未推翻前不用再问）：
 *   1) Iin 与 Ia 用同型 TA 模块、Vin 与 Vab 用同型 TV 模块 → 标定可
 *      直接复用（见 .measurement 各通道注释）；
 *   2) 功率级与栅极驱动关断路径按 README 第 3 节逐脚核对过；
 *   3) 死区实测 1.000 us（tim.c 里 DTG=148，168 MHz 下 168 个 tDTS）。
 * 注意：模式校验（本文件下半部分）仍会逐项检查复用后的可测量程是否
 * 盖得住控制上限和过流阈值——"复用"不等于"不检查"。
 */
const BoardPowerConfig g_board_power_config = {
    /*
     * 七个安全标志 = "这块板做过这些确认"的集中声明。
     * 它们只是入场券：下面每个具体参数仍会被校验函数逐条检查，
     * 所以这里的 true 不是"跳过校验"的后门（见 app_config.h）。
     */
    .config_valid = true,                /* 总开关：false → 任何模式都进不了 RUN */
    .adc_calibrated = true,              /* 三相通道 Ia/Ib/Vab/Vbc/Vdc 已实测标定 */
    .rectifier_adc_calibrated = true,    /* 整流通道 Iin/Vin 已按"同型复用"确认 */
    .gate_driver_pins_valid = true,      /* 驱动引脚与 README 第 3 节逐脚一致 */
    .power_stage_valid = true,           /* 功率级（桥式拓扑、母线电容）确认 */
    .deadtime_valid = true,              /* 死区已实测：1.000 us（tim.c DTG=148） */
    .protection_thresholds_valid = true, /* 保护阈值已按量程核算，见 .protection */
    /*
     * 输出线电压参考 [Vrms]。
     * 来历：赛题要求 Uo = 32 V ± 0.1 V；本工程把它与母线**解耦**独立
     *       设定，28~34 V 是现场微调窗口（app_config.h）。
     *       这里填线电压 RMS，内部换算相电压 RMS = 线/√3；闭环仿真
     *       输出 45.20 V 线电压峰值 = 32×√2，与指标一致。
     * 风险：填 ≤0 会退回旧的"按母线参考 32/60 比例推导"兼容路径——
     *       别误填 0 还以为只是"没设置"。
     * 现场微调走 App_SetOutputLineVoltageReference（仅停机/锁存时可改）。
     */
    .output_line_voltage_rms = APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V,
    /*
     * 母线目标 [V]。
     * 来历：41 V 输入峰值 58 V 时升压调制比 m = 58/62 = 0.935，仍低于
     *       0.96 占空比限幅；若用 60 V 母线，41 V 输入会撞限幅失控
     *       （推导见 app_config.h 顶部）。ACDC 闭环仿真稳态 62.08 V ✓。
     * 风险：抬高母线**不会**抬高输出目标（两者已解耦）；但超过 65 V
     *       会被校验直接拒绝（> APP_INVERTER_DC_VOLTAGE_MAX_V）。
     */
    .dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V,
    /*
     * 两次 Vin 采样（PC5 在 ADC1、ADC2 各采一次）允许的最大码差。
     * 来历：9 码 ≈ 0.465 V（按 TV 标定 0.0517 V/码）；给两路采样开关
     *       时序差 + ADC 噪声留余量。MODE0 用它做冗余校验，超差即判
     *       测量故障（PROTECTION_FAULT_MEASUREMENT，故障码 1）。
     * 风险：调到 1~2 码会在正常噪声下频繁误报；调得太大则失去诊断意义。
     */
    .vin_diagnostic_tolerance_counts = 9U,
    /*
     * ══ 七路 ADC 标定 ═══════════════════════════════════════════════════
     * 统一换算：物理量 = (原始码 − offset_count) × gain
     *   offset_count —— "零物理量"对应的 ADC 码（实测点拟合值）
     *   gain         —— 每码代表的物理量
     *   raw_min/max  —— 有效窗口：码落窗外 → 本帧判"测量无效"，保护记
     *                   PROTECTION_FAULT_MEASUREMENT（故障码 1）。
     *                   注意窗口裁的是"测量"不是"过流"：电流先撞量程
     *                   就会显示故障码 1 而不是 3（README 已知风险 2）。
     *
     * 下面是 2025 现场实测标定；硬件不换就不要动。换采样模块必须整
     * 通道重标（offset/gain/窗口一起改），并同步复核 .protection 阈值。
     */
    .measurement =
        {/*
         * 整流输入电流 Iin [A]（TA 同型通道）。
         * 来历：Iin 与已标定的 Ia 是同型 TA 模块（硬件已确认），复用其
         *       实测偏置/增益；折算可测量程 −5.90 … +5.68 A。
         * 风险：单通道复标只需替换本项，但别只改 gain 不改窗口——
         *       校验函数算量程用的就是 offset/gain/raw_min/max 四项。
         */
         .iin =
             {
                 .offset_count = 2084.7273f, /* 零点 [码]：两个实测点 + 零点约束最小二乘 */
                 .gain = 0.002919044f,       /* 灵敏度 [A/码]：折合前端 ≈ 0.276 V/A */
                 .raw_min = 64U,             /* 窗口下限：避开 ADC 轨底 ~1.5% 非线性区 */
                 .raw_max = 4031U            /* 窗口上限：避开 ADC 轨顶 ~1.5% 非线性区 */
             },
         /*
         * 整流输入电压 Vin [V]（TV 同型通道）。
         * 来历：Vin 与已标定的 Vab 是同型 TV 模块（硬件已确认），复用其
         *       实测偏置/增益；折算可测量程 −105.7 … +99.3 V。
         * 用途扩展：MODE0 下 PC5 被采两次，第二次当诊断冗余（见
         *       vin_diagnostic_tolerance_counts）。
         */
         .vin = {.offset_count = 2109.5455f, /* 零点 [码] */
                 .gain = 0.051657744f,       /* 灵敏度 [V/码]：前端 15.6 mV/V（分压比 1/64） */
                 .raw_min = 64U,             /* 同 TA：避开 ADC 两端非线性区 */
                 .raw_max = 4031U},
         /*
         * 三相电流 Ia / Ib [A]（TA 通道）。
         * 来历：两个实测量点 + "零点必对应零电流"约束，做零点约束最小
         *       二乘，拟合出 offset = 2084.7273 码、gain = 0.002919044
         *       A/码；折算可测量程 −5.90 … +5.68 A。
         * 连带影响：整流输入电流沿用同一量程，所以"输入 3.8 A RMS 余量
         *       只剩约 6%"说的就是这个窗口（见 .rectifier 注释）。
         */
         .ia = {.offset_count = 2084.7273f, /* 零点 [码] */
                .gain = 0.002919044f,       /* 灵敏度 [A/码] */
                .raw_min = 64U,
                .raw_max = 4031U},
         .ib = {.offset_count = 2084.7273f, /* B 相与 A 相同型同批 → 同一实测标定 */
                .gain = 0.002919044f,
                .raw_min = 64U,
                .raw_max = 4031U},
         /*
         * 线电压 Vab / Vbc [V]（TV 通道）。
         * 来历：10 Vpp→156 mVpp 与 20 Vpp→312 mVpp 两个实测量点给出同一
         *       比例（15.6 mV/V）→ gain = (3.3/4095)/0.0156 = 0.051657744
         *       V/码；offset 由零点实测量点给出。量程 −105.7 … +99.3 V。
         * 注意：只采两路线电压，第三路 vca = −(vab+vbc) 由软件重构。
         */
         .vab = {.offset_count = 2109.5455f, /* 零点 [码] */
                 .gain = 0.051657744f,       /* 灵敏度 [V/码] */
                 .raw_min = 64U,
                 .raw_max = 4031U},
         .vbc = {.offset_count = 2109.5455f, /* 与 Vab 同型同批 → 同一实测标定 */
                 .gain = 0.051657744f,
                 .raw_min = 64U,
                 .raw_max = 4031U},
         /*
         * 直流母线电压 Vdc [V]（单极性：分压 + 缓冲）。
         * 来历：四个实测量点线性拟合，引脚电压 Vout = 0.04779169×Vdc +
         *       0.00365764 V；换算到 ADC 码：
         *         gain   = (3.3 V / 4095) ÷ 0.04779169 = 0.016861944 [V/码]
         *         offset = 0.00365764 V ÷ (3.3 V / 4095) = 4.5388 [码]
         *       折算可测量程 −0.08 … +67.9 V，**盖住 66 V 过压阈值**——
         *       过压一定先被"测到"再判，不会因量程被裁掉而错报故障码。
         * 单极性通道故窗口取 0…4031；MODE1 不接 PC4 时读数 ≈0 V，
         *       属预期（此时母线值由用户 SET 值代替，见 README 3.1）。
         */
         .vdc = {.offset_count = 4.5388f, /* 零点 [码] */
                 .gain = 0.016861944f,    /* 灵敏度 [V/码]（分压比 0.0478 V/V） */
                 .raw_min = 0U,
                 .raw_max = 4031U}},
    /*
     * ══ 三相逆变级（DCAC）═══════════════════════════════════════════════
     * 结构：电压外环 QPR → 电流内环 QPR → SVPWM（见 inverter_control.h）。
     * 量纲约定：所有 *_rms 都指**相电压/相电流有效值**，线电压 = 相×√3；
     *   线电压 32 Vrms ↔ 相电压 18.48 Vrms ↔ SVPWM 里的相电压峰值 26.13 V。
     */
    .inverter =
        {/*
         * 快环周期 [s]：20 kHz 中心对齐 PWM（tim.c ARR=4199）。
         * 改这里等于换整个控制频率：QPR 系数、相位步长、RMS 窗口、
         * 死区占比全部跟着变——没有重新整定过就别动。MODE0 校验还要求
         * 逆变/整流两级 sample_time 完全相等（共用 ADC 触发的前提）。
         */
         .sample_time_s = 50.0e-6f,
         /*
         * 输出基波频率 [Hz]，默认 60。
         * 来历：现场按 60 Hz 输出验证；UI 的 A 键在 30/60 Hz 间切换
         *       （board_runtime.c），切换走 InverterControl_SetFrequency
         *       会重算四个 QPR 的谐振系数。
         * 风险：校验白名单只放行 30/60——其他频率（如 50）未经闭环
         *       验证，要用就得同时改这里、改校验白名单并重跑 sim_inverter。
         */
         .output_frequency_hz = 60.0f,
         /*
         * 相电压 RMS 上限 [V] = 65 V 母线 × (32/60) ÷ √3 = 20.0 V。
         * 来历：最大母线 65 V 按 32/60 比例外推出的线电压上限 34.67 V，
         *       再 ÷√3 换成相电压——正好等于 28~34 V 微调窗的上沿。
         * 用途：① InverterControl_SetReference 的内部限幅；
         *       ② App_SetOutputLineVoltageReference 的合法性门限
         *       （要求相电压 ≤ 20.0 V，即线电压 ≤ 34.67 V）。
         */
         .voltage_rms_max = APP_INVERTER_DC_VOLTAGE_MAX_V * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_INV_SQRT3_F,
         /*
         * 相电压 RMS 软启动斜率 [V/s] = 62 V 母线 × (32/60) ÷ √3 = 19.09。
         * 来历：同一比例外推；从 0 爬到额定 18.48 V 约 0.97 s，仿真里
         *       1 s 内到位、无过冲（sim_inverter.c：1.0 s 参考到 32.00 V）。
         * 风险：调大起机更猛（可能顶到过流），调小则到额定更慢——改了
         *       要重看仿真里前 1 s 的母线/电流波形。
         */
         .reference_slew_v_per_s =
             APP_INVERTER_DC_VOLTAGE_DEFAULT_V * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_INV_SQRT3_F,
         /*
         * 电压前馈系数：桥压指令 = 前馈×参考 + 电流内环 QPR 输出。
         * 0 = 纯反馈（与 SLX 一致，也是唯一验证过的整定）；非零相当于
         * 让电流环"提前给量"。没有新仿真/实测依据前别给非零值。
         */
         .feedforward_gain = 0.0f,
         /*
         * 电压外环 QPR [Kp, Kr, Wc]。
         * 来历：SLX（Simulink）模型的整定值原样搬入；host_tests\
         *       sim_inverter.c 用同一组参数做闭环仿真（45.20 V 峰值、
         *       faults=0），所以这组数在离线侧是"验证过的"。
         * 形状：Kp 很小、Kr 大 → 靠基波处的谐振高增益实现稳态无差；
         *       Wc = 1 rad/s 谐振峰很窄，对基波频率漂移敏感——换频率
         *       必须走 InverterControl_SetFrequency（会重算系数）。
         */
         .voltage_qpr_kp = 0.05f,
         .voltage_qpr_kr = 30.0f,
         .voltage_qpr_bandwidth_rad_s = 1.0f,
         /*
         * 电压外环输出的电流参考限幅 [A]（αβ 每轴 ±）。
         * 来历：赛题满载相电流 2.0 A RMS → 峰值 2.83 A，取 3.0 A 留约
         *       6% 余量；同时是 QprController_Init 的输出上下限。
         * 风险：调小会削顶（带不动满载），调大要留意相电流保护阈值
         *       3.5 A 仍在它上面（.protection），别把阈值压到限幅下面。
         */
         .current_reference_limit_a = 3.0f,
         /*
         * 电流内环 QPR [Kp, Kr, Wc]：同样来自 SLX 模型。
         * 带宽 5 rad/s 比电压环(1 rad/s)宽，保证内环快于外环。
         */
         .current_qpr_kp = 2.5f,
         .current_qpr_kr = 30.0f,
         .current_qpr_bandwidth_rad_s = 5.0f,
         /*
         * 桥压指令限幅 [V]（αβ 每轴 ±200 V）。
         * 来历：防爆常量——62 V 母线满调制线电压峰值也只有 62 V，正常
         *       工况永远触不到 ±200。幅值的真约束在 SVPWM：过调制时按
         *       比例缩矢量（applied_scale），不是靠这个限幅。
         */
         .command_limit_v = 200.0f,
         /*
         * 母线欠压下限 [V]：母线 < 10 V 时 InverterControl_Step 直接
         * 返回失败（不给门极命令）。只会出现在没上电/没预充的时候。
         * 校验要求它 < SET 允许下限 30 V，否则正常停机点会被误判。
         */
         .dc_voltage_min_v = 10.0f,
         /*
         * SVPWM 占空比区间（归一化）。
         * 0.02 / 0.98：两端各留 2% 给死区、驱动上升沿与最小脉宽。
         * 有效调制跨度 = (0.98−0.02)×Vdc = 59.5 V @62 V，比额定线电压
         *   峰值 45.25 V 大 31%——这是"母线够高才不削顶"的余量来源。
         * 过调制时 SVPWM 整体缩矢量、不动区间；区间本身别乱动。
         */
         .duty_min = 0.02f,
         .duty_max = 0.98f},
    /*
     * ══ 单相整流级（ACDC）═══════════════════════════════════════════════
     * 结构：母线电压 PI（1 kHz + 100 Hz 陷波）→ 输入电流 QPR（20 kHz）
     *       → 单极性 SPWM（rectifier_pwm.c）。见 rectifier_control.h。
     * 方向约定：只允许 AC→DC（PI 输出下限固定 0 A + current_reference_sign
     *       = +1）；这也是校验函数强制 current_reference_sign ≥ 0.5 的原因。
     */
    .rectifier =
        {/* 快环周期 [s]：同逆变级 20 kHz；MODE0 校验要求与逆变严格相等 */
         .sample_time_s = 50.0e-6f,
         /*
         * 输入交流频率 [Hz]（现场为 50 Hz 市电/信号源）。
         * 它决定两件事：① 输入 RMS 滑动窗长度 = 400 点 = 20 ms = 一个
         * 周期（RECTIFIER_VIN_RMS_WINDOW_SAMPLES）；② 100 Hz 陷波中心
         * 频率 = 2×50。改成 60 Hz 要同步改窗口长度与陷波中心，别只改这里。
         */
         .line_frequency_hz = 50.0f,
         /*
         * 外环分频：每 20 拍跑一次母线 PI + 陷波 → 1 kHz（20 kHz/20）。
         * 电流内环仍每拍 20 kHz 跑（rectifier_control.c 用
         * sample_time_s × outer_loop_divider 作外环离散周期）。
         * 改它等于改外环带宽，PI 需要重新整定。
         */
         .outer_loop_divider = 20U,
         /*
         * **保留字段（当前实现不读它）**：输入 RMS 用的是
         * rectifier_control.h 里 RECTIFIER_VIN_RMS_WINDOW_SAMPLES = 400
         * 的滑动窗口，与这个时间常数无关。校验仍要求 > 0，别填 0；
         * 也不要以为改它会影响 RMS 响应速度。
         */
         .rms_filter_time_constant_s = 0.05f,
         /*
         * 输入就绪下限 [V]：Vin 的 400 点 RMS < 10 V 时稳态控制器拒绝
         * 工作（Step 返回失败），防止空载/噪声下误起机。
         * 起机时机另有开关：app_control.c 的
         * APP_RECTIFIER_START_VIN_RMS_FRACTION = 0.10——等 vin 瞬时值
         * 落到 RMS 的 ±10% 内（过零点附近）才开管，避开合闸冲击。
         */
         .input_voltage_rms_min_v = 10.0f,
         /*
         * 整流侧母线上限 [V]：= 过压保护阈值 66 V，避免"外环目标贴着
         * 保护线"造成跳闸抖动。校验要求 ≤ protection.dc_voltage_max_v。
         */
         .dc_voltage_max_v = APP_INVERTER_DC_OVERVOLTAGE_V,
         /*
         * 母线参考软启动斜率 [V/s]：0 → 62 V 约 1.03 s。
         * 来历：参考工程同值；仿真记录 0.2 s 进 READY、1.2 s 母线到
         *       62.07 V（docs/ERROR_LOG.md）。调大 = 冲击电流更大。
         */
         .dc_reference_slew_v_per_s = 60.0f,
         /*
         * 母线"已就绪"容差 [V]：|实测 − 目标| ≤ 5 V 才算到位（配合
         * dc_ready_confirm_samples 连续确认）。
         */
         .dc_ready_tolerance_v = 5.0f,
         /* 与参考工程一致：20 kHz 下连续 1000 拍（50 ms）确认母线就绪。 */
         .dc_ready_confirm_samples = 1000U,
         /*
         * 输入电流 RMS 限幅 [A]（母线 PI 输出的电流幅值指令上限）。
         * 赛题满载 Po=√3×32V×2.0A≈110.9W，η≥95% 时 Pin≈116.7W：
         * 36V 输入需 3.24A RMS，31V 输入需 3.77A RMS。
         * 限幅取 3.8A RMS（5.37A 峰值）。TA 实测量程约 -5.90..+5.68A，
         * 峰值余量仅剩 ~6%，效率不达标时 31V 满载会贴量程顶。
         * 校验链（BoardPower_RectifierConfigValid）：本限幅的峰值
         * 1.414×3.8 = 5.37 A 必须 < 过流阈值 5.6 A，且两者都要落在
         * iin 可测量程内——一起看，别单改一个数。
         */
         .current_rms_max_a = 3.8f,
         /* SLX 外环 PI 增益保持 Kp=1、Ki=1（外环按 1 kHz 离散）。 */
         .dc_pi_kp = 1.0f,
         .dc_pi_ki = 1.0f,
         /* 抗饱和增益 0 = 条件积分（见 pi_controller.c 的 kaw 语义），
         * 不做额外回灌；要改非零先读那段实现。 */
         .dc_pi_kaw = 0.0f,
         /* SLX 输入电流 PR/QPR 参数：Kp=5、Kr=60、Wc=5 rad/s。 */
         .current_qpr_kp = 5.0f,
         .current_qpr_kr = 60.0f,
         .current_qpr_bandwidth_rad_s = 5.0f,
         /* 电感电压指令限幅 [V]（内环 QPR 输出 ±）：正常只有几十伏
         * （桥压 ≈ Vin − 电感压降），1200 V 是防爆常量。真正的约束在
         * 占空比区间——桥压被钳在 [0.02, 0.98]×Vdc 内。 */
         .inductor_voltage_limit_v = 1200.0f,
         /* 整流侧母线欠压下限 [V]：低于它 Step 直接失败（同逆变侧）。
         * 校验要求 SET 目标 ≥ 它。 */
         .dc_voltage_min_v = 10.0f,
         /* 正电流幅值和 PI 的 0 A 下限共同禁止直流向交流回馈。 */
         .current_reference_sign = 1.0f,
         /* 桥臂极性：+1 = 与原理图一致；实物高低侧接反（等价电流方向
         * 反号）才改 −1。**改错就是正反馈**，会把母线泵到过压——
         * 改前先看 README 3.2 节桥臂 B 的接线说明。 */
         .bridge_polarity = 1.0f,
         /* 陷波器零点阻尼 0 → 理想零点：100 Hz 处全陷（坑最深）。 */
         .notch_zero_damping = 0.0f,
         /* 极点阻尼 ζp = 50/(4π×100) ≈ 0.0398，满足 2ζpω0 = 50 rad/s，
         * 即陷波半带宽 ≈ 8 Hz——坑足够窄：摘掉母线二倍频纹波又不吃掉
         * 外环相位裕度。改大 = 坑变宽、外环相位裕度下降。 */
         .notch_pole_damping = 50.0f / (4.0f * APP_PI_F * 100.0f),
         /* 同逆变级：占空比区间，避开最小脉宽。桥压上限 = 0.98×Vdc；
         * 41 V 输入峰值 58 V 时需要 m = 0.935 < 0.96，余量就在这两个数里。 */
         .duty_min = 0.02f,
         .duty_max = 0.98f},
    .protection =
        {/* 输入电流瞬时绝对值阈值 [A]。
         * 31V 满载输入电流峰值约 5.33A；阈值需高于它且不超过 TA 可测
         * 上限（+5.68A），取 5.6A——余量很薄，勿再调高。
         * 校验还要求它 > 1.414×current_rms_max_a = 5.37 A（否则满载即
         * 误跳闸）。 */
         .input_current_abs_max_a = 5.6f,
         /* 逆变相电流瞬时绝对值阈值 [A]：相电流额定峰值 2.83 A、电流
         * 参考限幅 3.0 A，3.5 A 高于两者——真触发说明电流环已失控。
         * 也低于 TA 可测 5.68 A，保证先"测到"再判。 */
         .phase_current_abs_max_a = 3.5f,
         /* 输入交流电压瞬时绝对值阈值 [V]：41 V RMS 峰值 58 V，留 12%
         * 余量；且远低于 TV 可测上限 99.3 V，不会先撞量程。 */
         .input_voltage_abs_max_v = 65.0f,
         /* RUN 期间母线欠压下限 [V] = SET 下限 30 − 一致性容差 3 = 27，
         * 比"设定未达标"松一档，避免母线纹波/负载跳变时误报停车。 */
         .dc_voltage_min_run_v = APP_INVERTER_DC_VOLTAGE_MIN_V - APP_INVERTER_DC_MATCH_TOLERANCE_V,
         /* 母线过压阈值 [V]：SET 上限 65 + 1 V；≤ Vdc 可测上限 67.9 V，
         * 保证过压先被"测到"再判，不会因量程超限走错故障码。 */
         .dc_voltage_max_v = APP_INVERTER_DC_OVERVOLTAGE_V,
         /* 三相相电压瞬时绝对值阈值 [V]：额定相电压峰值 = 32/√3×√2 =
         * 26.1 V → 余量 22%。
         * 注意比的是"相电压瞬时值"，不是线电压 32 Vrms——别混。 */
         .output_voltage_abs_max_v = 32.0f,
         /* 连续 2 拍（100 us @20 kHz）没有新 ADC 帧 → 采样停滞故障
         * （PROTECTION_FAULT_SAMPLE_STALE）。只有 DMA 断了才会到这步。 */
         .stale_limit_samples = 2U},
    .state = {
        /* 状态机"连续确认"拍数（power_state.c 的 PowerState_Confirmed）：
         * 条件连续成立 N 拍才允许迁移，滤掉单拍毛刺。 */
        .self_test_confirm_samples = 4U, /* SELF_TEST：测量连续有效 4 拍（200 us）→ PRECHARGE */
        .ready_confirm_samples = 20U,    /* READY：相位就绪+测量有效 20 拍（1 ms）→ RUN */
        .discharge_confirm_samples = 20U /* DISCHARGE：放电完成 20 拍（1 ms）→ OFF */
    }};

/*
 * 取当前板级配置。默认实现是弱符号：板级适配文件可用强定义覆盖它，
 * 换板不必改本文件（ELF 目标有效；PE/COFF 退化说明见文件头）。
 */
APP_WEAK const BoardPowerConfig *BoardPower_GetConfig(void) {
    return &g_board_power_config;
}

/*
 * 所有模式共用的第一道校验：总开关 + 六个安全标志 + 保护阈值/状态机
 * 拍数的有限性与范围。返回 false = 这份配置根本不允许进 RUN。
 */
static bool BoardPower_CommonConfigValid(const control_config_t *cfg) {
    if ((cfg == NULL) || !cfg->config_valid || !cfg->adc_calibrated || !cfg->gate_driver_pins_valid ||
        !cfg->power_stage_valid || !cfg->deadtime_valid || !cfg->protection_thresholds_valid) {
        return false;
    }

    return isfinite(cfg->protection.dc_voltage_max_v) && isfinite(cfg->protection.input_current_abs_max_a) &&
           isfinite(cfg->protection.phase_current_abs_max_a) && isfinite(cfg->protection.input_voltage_abs_max_v) &&
           isfinite(cfg->protection.dc_voltage_min_run_v) && isfinite(cfg->protection.output_voltage_abs_max_v) &&
           (cfg->protection.input_current_abs_max_a > 0.0f) && (cfg->protection.phase_current_abs_max_a > 0.0f) &&
           (cfg->protection.input_voltage_abs_max_v > 0.0f) && (cfg->protection.dc_voltage_min_run_v >= 0.0f) &&
           (cfg->protection.output_voltage_abs_max_v > 0.0f) &&
           (cfg->protection.dc_voltage_max_v > cfg->protection.dc_voltage_min_run_v) &&
           (cfg->protection.stale_limit_samples > 0U) && (cfg->state.self_test_confirm_samples > 0U) &&
           (cfg->state.ready_confirm_samples > 0U) && (cfg->state.discharge_confirm_samples > 0U);
}

/*
 * 逆变模式（MODE1）校验：逐项对应 .inverter 段每个数——
 *   标定有效；线电压参考 ∈ [28, 34] 且 ≤ voltage_rms_max×√3；
 *   母线目标 ∈ [30, 65] 且 < 过压阈值；母线下限 < SET 下限；
 *   频率白名单（只放行 30 / 60 Hz）；QPR 增益非负且不同时为 0；
 *   限幅为正、占空比区间 0 ≤ min < max ≤ 1。
 * 缺一不可——这些不是"提醒"，是进 RUN 的必要条件。
 */
static bool BoardPower_InverterConfigValid(const control_config_t *cfg) {
    return Measurement_InverterCalibrationValid(&cfg->measurement) && isfinite(cfg->output_line_voltage_rms) &&
           isfinite(cfg->dc_voltage_ref) && (cfg->output_line_voltage_rms >= APP_INVERTER_OUTPUT_LINE_RMS_MIN_V) &&
           (cfg->output_line_voltage_rms <= APP_INVERTER_OUTPUT_LINE_RMS_MAX_V) &&
           (cfg->output_line_voltage_rms <= cfg->inverter.voltage_rms_max * APP_SQRT3_F) &&
           (cfg->dc_voltage_ref >= APP_INVERTER_DC_VOLTAGE_MIN_V) &&
           (cfg->dc_voltage_ref <= APP_INVERTER_DC_VOLTAGE_MAX_V) &&
           (cfg->dc_voltage_ref < cfg->protection.dc_voltage_max_v) &&
           (APP_INVERTER_DC_VOLTAGE_MIN_V > cfg->inverter.dc_voltage_min_v) && isfinite(cfg->inverter.sample_time_s) &&
           isfinite(cfg->inverter.output_frequency_hz) && isfinite(cfg->inverter.voltage_rms_max) &&
           isfinite(cfg->inverter.reference_slew_v_per_s) && isfinite(cfg->inverter.feedforward_gain) &&
           isfinite(cfg->inverter.voltage_qpr_kp) && isfinite(cfg->inverter.voltage_qpr_kr) &&
           isfinite(cfg->inverter.voltage_qpr_bandwidth_rad_s) && isfinite(cfg->inverter.current_reference_limit_a) &&
           isfinite(cfg->inverter.current_qpr_kp) && isfinite(cfg->inverter.current_qpr_kr) &&
           isfinite(cfg->inverter.current_qpr_bandwidth_rad_s) && isfinite(cfg->inverter.command_limit_v) &&
           isfinite(cfg->inverter.dc_voltage_min_v) && isfinite(cfg->inverter.duty_min) &&
           isfinite(cfg->inverter.duty_max) && (cfg->inverter.sample_time_s > 0.0f) &&
           ((cfg->inverter.output_frequency_hz == 30.0f) || (cfg->inverter.output_frequency_hz == 60.0f)) &&
           (cfg->inverter.voltage_rms_max > 0.0f) && (cfg->inverter.reference_slew_v_per_s > 0.0f) &&
           (cfg->inverter.voltage_qpr_kp >= 0.0f) && (cfg->inverter.voltage_qpr_kr >= 0.0f) &&
           (cfg->inverter.voltage_qpr_bandwidth_rad_s > 0.0f) &&
           ((cfg->inverter.voltage_qpr_kp + cfg->inverter.voltage_qpr_kr) > 0.0f) &&
           (cfg->inverter.current_qpr_kp >= 0.0f) && (cfg->inverter.current_qpr_kr >= 0.0f) &&
           (cfg->inverter.current_qpr_bandwidth_rad_s > 0.0f) &&
           ((cfg->inverter.current_qpr_kp + cfg->inverter.current_qpr_kr) > 0.0f) &&
           (cfg->inverter.current_reference_limit_a > 0.0f) && (cfg->inverter.command_limit_v > 0.0f) &&
           (cfg->inverter.dc_voltage_min_v > 0.0f) && (cfg->inverter.duty_min >= 0.0f) &&
           (cfg->inverter.duty_max <= 1.0f) && (cfg->inverter.duty_min < cfg->inverter.duty_max);
}

/*
 * 整流模式（MODE2）校验。关键的一条是"量程必须盖住两个上限"：
 *   把 iin 的标定反算成可测量程 [measurable_min, measurable_max]，
 *   要求同时盖住 ①电流指令峰值 1.414×current_rms_max_a = 5.37 A
 *   ②软件过流阈值 input_current_abs_max_a = 5.6 A；并额外要求
 *   1.414×current_rms_max_a < input_current_abs_max_a。
 * 这条就是 README"已知风险 2"的由来：量程一旦不够，超限电流会先被判
 * 成"测量无效"（故障码 1）而不是过流（故障码 3）——现场排查别被误导。
 * 其余检查：Vin 诊断容差 > 0；母线目标在 [min,max] 内且 < 过压阈值；
 * 外环 PI / 内环 QPR 参数非负且不同时为 0；陷波阻尼 0 ≤ zero ≤ pole；
 * 极性 +1（禁止 AC←DC）；占空比区间合法。
 */
static bool BoardPower_RectifierConfigValid(const control_config_t *cfg) {
    float current_at_raw_min;
    float current_at_raw_max;
    float measurable_min;
    float measurable_max;
    float current_command_peak;
    float required_peak;

    current_at_raw_min =
        ((float)cfg->measurement.iin.raw_min - cfg->measurement.iin.offset_count) * cfg->measurement.iin.gain;
    current_at_raw_max =
        ((float)cfg->measurement.iin.raw_max - cfg->measurement.iin.offset_count) * cfg->measurement.iin.gain;
    measurable_min = fminf(current_at_raw_min, current_at_raw_max);
    measurable_max = fmaxf(current_at_raw_min, current_at_raw_max);
    current_command_peak = 1.41421356237f * cfg->rectifier.current_rms_max_a;
    required_peak = fmaxf(current_command_peak, cfg->protection.input_current_abs_max_a);

    return cfg->rectifier_adc_calibrated && Measurement_RectifierCalibrationValid(&cfg->measurement) &&
           isfinite(measurable_min) && isfinite(measurable_max) && isfinite(current_command_peak) &&
           isfinite(required_peak) && (measurable_min <= -required_peak) && (measurable_max >= required_peak) &&
           (cfg->vin_diagnostic_tolerance_counts > 0U) && isfinite(cfg->dc_voltage_ref) &&
           (cfg->dc_voltage_ref >= cfg->rectifier.dc_voltage_min_v) &&
           (cfg->dc_voltage_ref <= cfg->rectifier.dc_voltage_max_v) &&
           (cfg->dc_voltage_ref < cfg->protection.dc_voltage_max_v) && isfinite(cfg->rectifier.sample_time_s) &&
           isfinite(cfg->rectifier.line_frequency_hz) && isfinite(cfg->rectifier.rms_filter_time_constant_s) &&
           isfinite(cfg->rectifier.input_voltage_rms_min_v) && isfinite(cfg->rectifier.dc_voltage_max_v) &&
           isfinite(cfg->rectifier.dc_reference_slew_v_per_s) && isfinite(cfg->rectifier.dc_ready_tolerance_v) &&
           isfinite(cfg->rectifier.current_rms_max_a) && isfinite(cfg->rectifier.dc_pi_kp) &&
           isfinite(cfg->rectifier.dc_pi_ki) && isfinite(cfg->rectifier.dc_pi_kaw) &&
           isfinite(cfg->rectifier.current_qpr_kp) && isfinite(cfg->rectifier.current_qpr_kr) &&
           isfinite(cfg->rectifier.current_qpr_bandwidth_rad_s) && isfinite(cfg->rectifier.inductor_voltage_limit_v) &&
           isfinite(cfg->rectifier.dc_voltage_min_v) && isfinite(cfg->rectifier.current_reference_sign) &&
           isfinite(cfg->rectifier.bridge_polarity) && isfinite(cfg->rectifier.notch_zero_damping) &&
           isfinite(cfg->rectifier.notch_pole_damping) && isfinite(cfg->rectifier.duty_min) &&
           isfinite(cfg->rectifier.duty_max) && (cfg->rectifier.sample_time_s > 0.0f) &&
           (cfg->rectifier.line_frequency_hz > 0.0f) && (cfg->rectifier.outer_loop_divider > 0U) &&
           (cfg->rectifier.rms_filter_time_constant_s > 0.0f) && (cfg->rectifier.input_voltage_rms_min_v > 0.0f) &&
           (cfg->rectifier.dc_voltage_min_v > 0.0f) &&
           (cfg->rectifier.dc_voltage_max_v > cfg->rectifier.dc_voltage_min_v) &&
           (cfg->rectifier.dc_voltage_max_v <= cfg->protection.dc_voltage_max_v) &&
           (cfg->rectifier.dc_reference_slew_v_per_s > 0.0f) && (cfg->rectifier.dc_ready_tolerance_v > 0.0f) &&
           (cfg->rectifier.dc_ready_confirm_samples > 0U) && (cfg->rectifier.current_rms_max_a > 0.0f) &&
           (current_command_peak < cfg->protection.input_current_abs_max_a) && (cfg->rectifier.dc_pi_kp >= 0.0f) &&
           (cfg->rectifier.dc_pi_ki >= 0.0f) && (cfg->rectifier.dc_pi_kaw >= 0.0f) &&
           ((cfg->rectifier.dc_pi_kp + cfg->rectifier.dc_pi_ki) > 0.0f) && (cfg->rectifier.current_qpr_kp >= 0.0f) &&
           (cfg->rectifier.current_qpr_kr >= 0.0f) && (cfg->rectifier.current_qpr_bandwidth_rad_s > 0.0f) &&
           ((cfg->rectifier.current_qpr_kp + cfg->rectifier.current_qpr_kr) > 0.0f) &&
           (cfg->rectifier.inductor_voltage_limit_v > 0.0f)
           /*
         * PI 输出下限在 RectifierControl_Init 中固定为 0 A；这里禁止
         * 负的电流参考极性，确保任何有效配置都只能 AC -> DC。
         */
           && (cfg->rectifier.current_reference_sign >= 0.5f) && (fabsf(cfg->rectifier.bridge_polarity) >= 0.5f) &&
           (cfg->rectifier.notch_zero_damping >= 0.0f) && (cfg->rectifier.notch_pole_damping > 0.0f) &&
           (cfg->rectifier.notch_zero_damping <= cfg->rectifier.notch_pole_damping) &&
           (cfg->rectifier.duty_min >= 0.0f) && (cfg->rectifier.duty_max <= 1.0f) &&
           (cfg->rectifier.duty_min < cfg->rectifier.duty_max);
}

/*
 * 按运行模式分派校验：
 *   MODE0_FULL       —— 两级都查，且要求两级 sample_time 完全相等
 *                       （两级共用同一个 ADC 触发源的前提）；
 *   MODE1_INVERTER   —— 只查逆变级（整流标定没做完也能跑）；
 *   MODE2_RECTIFIER  —— 只查整流级。
 * "只查用到的通道"是刻意设计，别改成永远全查——那会让单级调试无法进行。
 */
bool BoardPower_ValidateModeConfig(const control_config_t *cfg, app_mode_t mode) {
    if (!BoardPower_CommonConfigValid(cfg)) return false;

    switch (mode) {
    case MODE1_INVERTER:
        return BoardPower_InverterConfigValid(cfg);

    case MODE2_RECTIFIER:
        return BoardPower_RectifierConfigValid(cfg);

    case MODE0_FULL:
        return Measurement_CalibrationValid(&cfg->measurement) && BoardPower_InverterConfigValid(cfg) &&
               BoardPower_RectifierConfigValid(cfg) && (cfg->rectifier.sample_time_s == cfg->inverter.sample_time_s);

    default:
        return false;
    }
}

/* 全量校验的旧接口（等价 MODE0_FULL），保留给既有调用方 */
bool BoardPower_ValidateConfig(const control_config_t *cfg) {
    return BoardPower_ValidateModeConfig(cfg, MODE0_FULL);
}

/* 便捷封装：直接校验 BoardPower_GetConfig() 返回的当前配置 */
bool BoardPower_ConfigValid(void) {
    return BoardPower_ValidateConfig(BoardPower_GetConfig());
}
