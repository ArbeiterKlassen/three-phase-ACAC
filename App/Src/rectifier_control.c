#include "rectifier_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 单相整流级控制（ACDC）实现。
 *
 * 结构（快慢双环，方向只允许 AC→DC）：
 *   1 kHz 慢环：母线电压 PI（输出 = 输入电流幅值指令 [A]）+ 100 Hz 陷波
 *   20 kHz 快环：输入电流 QPR（谐振在输入频率）→ 桥压指令 → 单极性 SPWM
 *
 * 关键约定：
 *   - 电流参考 Iref = Iamp × vin / Vin_rms —— 与输入电压同相（单位功率因数）；
 *   - PI 输出下限固定 0 A + current_reference_sign = +1 → 禁止向交流侧回馈；
 *   - 外环未请求电流时输出 gates_enabled=false（桥臂高阻），此时**不能**
 *     输出 m=0——单极性 SPWM 下 m=0 等于把交流侧经桥臂短路到电感上。
 */

/* 把 value 限制到 [low, high]。 */
static float RectifierControl_Clamp(float value, float low, float high) {
    return value<low?low:(value>high?high:value);
}

/* 输出清零 + PWM 置中性（生成安全命令；真正关门极在 board_runtime.c）。 */
static void RectifierControl_ClearOutput(RectifierControlOutput *out) {
    *out = (RectifierControlOutput){0};
    RectifierPwm_SetNeutral(&out->pwm);
}

/*
 * 把输入 RMS 窗口置为"无效"：清累计和与有效标志。
 * 用在两处：测量突然失效（重新积累），以及复位。
 */
static void RectifierControl_InvalidateInputRms(RectifierControl *control) {
    control->vin_square_sum = 0.0f;
    control->vin_rms_v = 0.0f;
    control->vin_square_index = 0U;
    control->vin_square_count = 0U;
    control->outer_count = 0U;
    control->vin_rms_valid = false;
}

/* 彻底复位 RMS 窗口：先清环形缓冲，再置无效。 */
static void RectifierControl_ResetInputRms(RectifierControl *control) {
    memset(control->vin_square_window, 0, sizeof(control->vin_square_window));
    RectifierControl_InvalidateInputRms(control);
}

/*
 * 用"滑动窗口"维护输入电压真有效值：每拍把 vin² 推进环形缓冲，
 * 减去最旧的一项，再除以窗口长度开方。
 *
 * 为什么不用一阶低通：RMS 要求"过去一个完整工频周期的均方值"才是
 * 真有效值，低通会带来相位滞后与纹波，Iref 的相位就跟着歪，
 * 功率因数直接变差。窗口长度固定 400 点 = 20 ms @20 kHz = 一个 50 Hz
 * 周期（Init 里会强制校验这一点）。
 *
 * 窗口没填满前（vin_rms_valid=false）控制器不工作；任何一步算出非有限
 * 值都立刻整体置无效，绝不把 NaN 漏给下游。
 */
static void RectifierControl_UpdateInputRms(RectifierControl *control, float vin) {
    float vin_square;
    float oldest_square = 0.0f;
    vin_square = vin * vin;
    if (!isfinite(vin_square)) {
        RectifierControl_InvalidateInputRms(control);
        return;
    }
    if (control->vin_square_count >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES) oldest_square = control->vin_square_window[control->vin_square_index];
    else control->vin_square_count++;
    control->vin_square_window[control->vin_square_index] = vin_square;
    control->vin_square_sum += vin_square - oldest_square;
    control->vin_square_index++;
    if (control->vin_square_index >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES) control->vin_square_index = 0U;
    if (!isfinite(control->vin_square_sum)) {
        RectifierControl_InvalidateInputRms(control);
        return;
    }
    if (control->vin_square_count >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES) {
        control->vin_rms_v = sqrtf(fmaxf(control->vin_square_sum, 0.0f) / (float)RECTIFIER_VIN_RMS_WINDOW_SAMPLES);
        control->vin_rms_valid = isfinite(control->vin_rms_v);
        if (!control->vin_rms_valid) RectifierControl_InvalidateInputRms(control);
    }
}

/*
 * 初始化：校验配置 → 建母线 PI、电流 QPR、100 Hz 陷波器。
 *
 * 配置检查里最"怪"但最重要的一条是最后那个式子：它要求
 *   Ts × f_line × 400 ≈ 1（容差半拍，即 0.5×Ts×f_line）
 * 也就是**RMS 窗口必须正好是一个完整工频周期**（20 kHz、50 Hz、400 点）。
 * 改了 sample_time_s 或 line_frequency_hz 而没改窗口长度，这里会直接
 * 拒绝初始化——这是刻意的，别为了让它过而放宽容差。
 *
 * 三处周期/限幅的来历：
 *   w0               = 2π·f_line   → 电流内环谐振在**输入频率**（50 Hz）；
 *   slow_sample_time = Ts × outer_loop_divider = 1 ms → PI 与陷波的离散周期；
 *   PI 输出限幅 [0, current_rms_max_a] → 下限 0 就是"禁止回馈"的实现。
 * 陷波中心取 2×w0（100 Hz，母线二倍频纹波）。
 */
bool RectifierControl_Init(RectifierControl *control, const RectifierControlConfig *cfg) {
    float w0;
    float slow_sample_time;
    if ((control == NULL) || (cfg == NULL)
        || !isfinite(cfg->sample_time_s)
        || !isfinite(cfg->line_frequency_hz)
        || !isfinite(cfg->rms_filter_time_constant_s)
        || !isfinite(cfg->input_voltage_rms_min_v)
        || !isfinite(cfg->dc_voltage_max_v)
        || !isfinite(cfg->dc_reference_slew_v_per_s)
        || !isfinite(cfg->dc_ready_tolerance_v)
        || !isfinite(cfg->current_rms_max_a)
        || !isfinite(cfg->inductor_voltage_limit_v)
        || !isfinite(cfg->dc_voltage_min_v)
        || !isfinite(cfg->current_reference_sign)
        || !isfinite(cfg->bridge_polarity)
        || (cfg->sample_time_s <= 0.0f)
        || (cfg->line_frequency_hz <= 0.0f)
        || (cfg->outer_loop_divider == 0U)
        || (cfg->rms_filter_time_constant_s <= 0.0f)
        || (cfg->input_voltage_rms_min_v <= 0.0f)
        || (cfg->dc_voltage_max_v <= cfg->dc_voltage_min_v)
        || (cfg->dc_reference_slew_v_per_s <= 0.0f)
        || (cfg->dc_ready_tolerance_v <= 0.0f)
        || (cfg->dc_ready_confirm_samples == 0U)
        || (cfg->current_rms_max_a <= 0.0f)
        || (cfg->inductor_voltage_limit_v <= 0.0f)
        || (fabsf(cfg->current_reference_sign) < 0.5f)
        || (fabsf(cfg->bridge_polarity) < 0.5f)
        || (cfg->duty_min < 0.0f)
        || (cfg->duty_max > 1.0f)
        || (cfg->duty_min >= cfg->duty_max)
        || (fabsf(cfg->sample_time_s * cfg->line_frequency_hz
                  * (float)RECTIFIER_VIN_RMS_WINDOW_SAMPLES - 1.0f)
            > (0.5f * cfg->sample_time_s
               * cfg->line_frequency_hz)))
    {
        return false;
    }

    *control = (RectifierControl){0};
    control->cfg = *cfg;
    w0 = 2.0f * APP_PI_F * cfg->line_frequency_hz;  /* 谐振角频率 [rad/s] */
    slow_sample_time = cfg->sample_time_s * (float)cfg->outer_loop_divider;

    if (!PiController_Init(&control->dc_pi,
                           cfg->dc_pi_kp,
                           cfg->dc_pi_ki,
                           cfg->dc_pi_kaw,
                           slow_sample_time,
                           0.0f,
                           cfg->current_rms_max_a)
        || !QprController_Init(&control->current_qpr,
                               cfg->current_qpr_kp,
                               cfg->current_qpr_kr,
                               w0,
                               cfg->current_qpr_bandwidth_rad_s,
                               cfg->sample_time_s,
                               -cfg->inductor_voltage_limit_v,
                               cfg->inductor_voltage_limit_v)
        || !NotchFilter_Init(&control->dc_notch,
                             2.0f * w0,
                             cfg->notch_zero_damping,
                             cfg->notch_pole_damping,
                             slow_sample_time))
    {
        *control = (RectifierControl){0};
        return false;
    }
    control->initialized = true;
    return true;
}

/*
 * 复位全部动态状态：PI / QPR / 陷波器、软启动参考与标志、RMS 窗口。
 * 静态配置与母线目标保持不变；复位后软启动会从 0 V 重爬（见 Step）。
 * NULL 安全。
 */
void RectifierControl_Reset(RectifierControl *control) {
    if (control != NULL) {
        PiController_Reset(&control->dc_pi, 0.0f);
        QprController_Reset(&control->current_qpr);
        NotchFilter_Reset(&control->dc_notch, 0.0f);
        control->dc_reference_v = 0.0f;
        control->current_rms_a = 0.0f;
        control->dc_voltage_filtered = 0.0f;
        RectifierControl_ResetInputRms(control);
        control->soft_start_initialized = false;
    }
}

/*
 * 设置母线目标 [V]，内部限幅 [0, dc_voltage_max_v]（66 V）。
 * 实际 dc_reference_v 由 Step 按 dc_reference_slew_v_per_s 逐拍爬升；
 * 未初始化或非有限值静默忽略。
 */
void RectifierControl_SetDcReference(RectifierControl *control, float dc_voltage) {
    if ((control != NULL) && control->initialized && isfinite(dc_voltage)) {
        control->dc_reference_target_v = RectifierControl_Clamp(dc_voltage, 0.0f, control->cfg.dc_voltage_max_v);
    }
}

/*
 * 慢环（1 kHz）：陷波 + 母线 PI。
 *
 * 用 outer_count 数到 outer_loop_divider 才真正执行一次——但**陷波器
 * 即使控制器没使能也照跑**，这样状态机在 SELF_TEST/PRECHARGE 预热期间
 * 就把滤波器和 RMS 窗口"喂"到稳态，进 RUN 时没有暂态。
 * 未使能时 PI 被复位且输出 0 A：既不控制也不积累。
 */
static void RectifierControl_UpdateSlow(RectifierControl *control, const Measurements *measurement, bool enabled) {
    float dc_error;

    /* 100 Hz 陷波与母线 PI 固定 1 kHz 更新（每 20 个 20 kHz 拍一次）。 */
    if (control->outer_count + 1U < control->cfg.outer_loop_divider) {
        control->outer_count++;
        return;
    }
    control->outer_count = 0U;
    control->dc_voltage_filtered = NotchFilter_Step(&control->dc_notch, measurement->vdc);
    if (enabled) {
        dc_error = control->dc_reference_v - control->dc_voltage_filtered;
        control->current_rms_a = PiController_Step(&control->dc_pi, dc_error);
    } else {
        PiController_Reset(&control->dc_pi, 0.0f);
        control->current_rms_a = 0.0f;
    }
}
/*
 * 一次控制周期。
 *
 * 返回语义（和逆变级一样容易踩坑）：
 *   true  —— 本周期走完（可能是正常命令，也可能是"中性 + 未使能"）；
 *   false —— **不能给门极命令**：输入测量无效（且使能中）、母线或输入
 *            RMS 不到下限、或 PWM 生成失败。上层按故障处理。
 *
 * 两个"不使能≠错误"的细节：
 *   - 测量无效但 enabled=false → 返回 true（只是在攒数据）；
 *   - 输入 RMS 窗口没填满时 out->valid 为 false 但返回 true——
 *     状态机预热阶段会看到这个组合，属正常。
 */
bool RectifierControl_Step(RectifierControl *control, const Measurements *measurement, bool enabled, RectifierControlOutput *out) {
    float dc_step;
    float error;
    float inductor_voltage;
    float inductor_voltage_applied;
    float modulation;
    bool control_enabled;
    if ((control == NULL) || (out == NULL) || !control->initialized) return false;
    RectifierControl_ClearOutput(out);
    out->input_voltage_rms = control->vin_rms_v;
    out->input_rms_valid = control->vin_rms_valid;
    if ((measurement == NULL) || !measurement->valid
        || !isfinite(measurement->vin)
        || !isfinite(measurement->iin)
        || !isfinite(measurement->vdc))
    {
        /* 测量失效：RMS 窗口作废重攒；未使能时不算错误（见函数头）。 */
        RectifierControl_InvalidateInputRms(control);
        out->input_voltage_rms = 0.0f;
        out->input_rms_valid = false;
        return !enabled;
    }

    /*
     * 严格"一个工频周期"的滑动 RMS，按 20 kHz 快环节奏推进。
     * 未使能的控制器也照样填窗口——状态机预热要在这期间把窗口喂满，
     * 否则一进 RUN 就会因 vin_rms_valid=false 报控制失败。
     */
    RectifierControl_UpdateInputRms(control, measurement->vin);
    control_enabled = enabled && control->vin_rms_valid;

    if (control_enabled && !control->soft_start_initialized) {
        /*
         * 从0 V参考完整起坡；若母线已被动充电，高于参考时PI下限为0，
         * 不会命令反向灌电，也不会把1 s软启动缩短为余量起坡。
         */
        control->dc_reference_v = 0.0f;
        control->soft_start_initialized = true;
    }
    if (control_enabled) {
        /* 软启动步长 = 斜率 × 快环周期（60 V/s × 50 us = 3 mV/拍，
         * 62 V 约需 1.03 s），逐拍逼近目标。 */
        dc_step = control->cfg.dc_reference_slew_v_per_s * control->cfg.sample_time_s;
        control->dc_reference_v = RectifierControl_Clamp(control->dc_reference_v + dc_step, 0.0f, control->dc_reference_target_v);
    }
    /*
     * The 100 Hz notch and DC PI update every 20 fast samples at 1 kHz.
     * The current reference still uses measured vin/vrms every fast sample.
     */
    RectifierControl_UpdateSlow(control, measurement, control_enabled);
    out->dc_voltage_filtered = control->dc_voltage_filtered;
    out->dc_voltage_reference = control->dc_reference_v;
    out->input_voltage_rms = control->vin_rms_v;
    out->input_rms_valid = control->vin_rms_valid;
    if (!control_enabled) {
        /* 没使能（含 RMS 窗口未满）：清内环状态，输出保持中性。 */
        QprController_Reset(&control->current_qpr);
        control->soft_start_initialized = false;
        return true;
    }
    if ((measurement->vdc < control->cfg.dc_voltage_min_v) || (control->vin_rms_v < control->cfg.input_voltage_rms_min_v)) {
        /* 母线或输入电压不到下限：拒绝给门极命令（让上层报故障/等待）。 */
        return false;
    }

    /* 与输入电压同相的正弦电流参考：Iref = 幅值 × vin / Vin_rms。
     * 同相 → 单位功率因数；符号由 current_reference_sign 定死为 +1。 */
    out->current_reference = control->cfg.current_reference_sign
                           * control->current_rms_a
                           * measurement->vin / control->vin_rms_v;
    /*
     * 外环未请求电流（软启动爬坡段，或母线已高于参考）时只关断门极
     *（gates_enabled=false，桥臂高阻、仅剩二极管通路），调制仍按
     * vin/vdc 生成。不能输出 m=0：单极性 SPWM 下 m=0 等于把交流侧
     * 经桥臂短接到电感上，会产生数十安培的工频短路电流。
     */
    out->gates_enabled = control->current_rms_a > 0.0f;
    if (!out->gates_enabled) {
        QprController_Reset(&control->current_qpr);
        out->current_reference = 0.0f;
    }
    error = out->current_reference - measurement->iin;
    inductor_voltage = QprController_Step(&control->current_qpr, error);
    /* 桥压指令 = 极性 × (Vin − 电感电压)：V_L 由内环给出，
     * 桥臂需要产生 Vin − V_L 才能在电抗器上得到想要的电流斜率。 */
    out->bridge_voltage_command = control->cfg.bridge_polarity * (measurement->vin - inductor_voltage);
    /* 归一化调制比 m = 桥压/Vdc，交给单极性 SPWM 生成两桥臂占空比。 */
    modulation = out->bridge_voltage_command / measurement->vdc;
    out->valid = RectifierPwm_Generate(modulation,
                                       control->cfg.duty_min,
                                       control->cfg.duty_max,
                                       &out->pwm);
    if (out->valid && out->gates_enabled) {
        /* 门极关断时桥压由二极管决定，不按 PWM 公式反灌电流环状态。
         * 反推"实际施加的电感电压"：Vin − m·Vdc/极性（PWM 被限幅时
         * m 与指令不再相等，必须用实际值回灌，否则谐振器积累虚账）。 */
        inductor_voltage_applied = measurement->vin - (out->pwm.modulation * measurement->vdc) / control->cfg.bridge_polarity;
        QprController_TrackOutput(&control->current_qpr, error, inductor_voltage_applied);
    }
    return out->valid;
}
