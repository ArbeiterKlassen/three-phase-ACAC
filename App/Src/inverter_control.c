#include "inverter_control.h"

#include <math.h>
#include <stddef.h>

/*
 * 三相逆变级控制（DCAC）双闭环实现。
 *
 * 量纲约定（本文件每个 float 的含义地图，读代码前先过一遍）：
 *   reference_rms / reference_target_rms —— 相电压有效值 [Vrms]（不是线电压）
 *   peak = √2 × reference_rms             —— 相电压峰值 [V]
 *   αβ / 三相瞬时量                        —— [V] 或 [A] 的瞬时值
 *   线电压 = 相电压 × √3（额定：线 32 Vrms ↔ 相 18.48 Vrms ↔ 峰值 26.13 V）
 *
 * 正相序约定：A = sinθ、B = sin(θ−120°)、C = sin(θ+120°)，
 *   Clarke 之后 α = peak·sinθ、β = −peak·cosθ（推导见 StepInternal）。
 *
 * 每拍执行顺序（任一环失效就输出中性占空比，不给门极命令）：
 *   软启动参考 → 电压外环 QPR → 电流参考 → 电流内环 QPR → 桥压指令(+前馈)
 *   → SVPWM → 三相占空比；最后把"实际施加量"回灌电流环抗饱和。
 */
/* 把 value 限制到 [low, high]，三段式无分支逻辑（编译后是条件移动）。 */
static float InverterControl_Clamp(float value, float low, float high) {
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

/*
 * 输出结构清零，并把 PWM 置成"三相 50% 中性 + 未使能"。
 * 注意这只是**生成安全命令**：真正的门极关断在 board_runtime.c 清
 * CCER/MOE 完成，别以为调了它功率管就关了。
 */
static void InverterControl_ClearOutput(InverterControlOutput *out) {
    *out = (InverterControlOutput){0};
    Svpwm_SetNeutral(&out->pwm);
}

/*
 * 初始化：校验配置 → 建相位发生器 → 建四个 QPR（电压/电流 × α/β）。
 *
 * 参数检查分两层：先查有限性（isfinite），再查范围——周期/频率/限幅
 * 必须为正；电压上限与参考斜率允许为 0；占空比区间要求
 * 0 ≤ duty_min < duty_max ≤ 1。任一不满足返回 false，**且不留下半个
 * 控制器**（失败分支把实例整体清零）。
 *
 * QPR 输出限幅的来历：
 *   电压外环输出的是**电流参考** → 限幅 ±current_reference_limit_a；
 *   电流内环输出的是**桥压指令** → 限幅 ±command_limit_v。
 * 这四个 QPR 的谐振角频率都取 w0 = 2π·output_frequency_hz（基波处增益最大）。
 */
bool InverterControl_Init(InverterControl *control, const InverterControlConfig *cfg) {
    float w0;

    if ((control == NULL) || (cfg == NULL) || !isfinite(cfg->sample_time_s) || !isfinite(cfg->output_frequency_hz) ||
        !isfinite(cfg->voltage_rms_max) || !isfinite(cfg->reference_slew_v_per_s) || !isfinite(cfg->feedforward_gain) ||
        !isfinite(cfg->current_reference_limit_a) || !isfinite(cfg->command_limit_v) ||
        !isfinite(cfg->dc_voltage_min_v) || (cfg->sample_time_s <= 0.0f) || (cfg->output_frequency_hz <= 0.0f) ||
        (cfg->voltage_rms_max < 0.0f) || (cfg->reference_slew_v_per_s < 0.0f) ||
        (cfg->current_reference_limit_a <= 0.0f) || (cfg->command_limit_v <= 0.0f) || (cfg->dc_voltage_min_v <= 0.0f) ||
        (cfg->duty_min < 0.0f) || (cfg->duty_max > 1.0f) || (cfg->duty_min >= cfg->duty_max)) {
        return false;
    }

    *control = (InverterControl){0};
    control->cfg = *cfg;
    w0 = 2.0f * APP_PI_F * cfg->output_frequency_hz; /* 谐振角频率 ωr [rad/s] */

    /* 相位发生器：初相 0，每拍步进 2πf·Ts；与 QPR 共用同一个频率。 */
    if (!PhaseRecurrence_Init(&control->phase, cfg->output_frequency_hz, cfg->sample_time_s, 0.0f) ||
        !QprController_Init(&control->voltage_alpha_qpr, cfg->voltage_qpr_kp, cfg->voltage_qpr_kr, w0,
                            cfg->voltage_qpr_bandwidth_rad_s, cfg->sample_time_s, -cfg->current_reference_limit_a,
                            cfg->current_reference_limit_a) ||
        !QprController_Init(&control->voltage_beta_qpr, cfg->voltage_qpr_kp, cfg->voltage_qpr_kr, w0,
                            cfg->voltage_qpr_bandwidth_rad_s, cfg->sample_time_s, -cfg->current_reference_limit_a,
                            cfg->current_reference_limit_a) ||
        !QprController_Init(&control->current_alpha_qpr, cfg->current_qpr_kp, cfg->current_qpr_kr, w0,
                            cfg->current_qpr_bandwidth_rad_s, cfg->sample_time_s, -cfg->command_limit_v,
                            cfg->command_limit_v) ||
        !QprController_Init(&control->current_beta_qpr, cfg->current_qpr_kp, cfg->current_qpr_kr, w0,
                            cfg->current_qpr_bandwidth_rad_s, cfg->sample_time_s, -cfg->command_limit_v,
                            cfg->command_limit_v)) {
        *control = (InverterControl){0};
        return false;
    }

    control->initialized = true;
    return true;
}

/*
 * 复位动态状态：清四个 QPR 的积分/谐振历史，并把软启动参考拉回 0 Vrms。
 * 目标参考、频率、静态配置**保持不变**——所以复位后是"从 0 重新爬到
 * 当前目标"，这正是重上电/退出故障后想要的行为。NULL 安全。
 */
void InverterControl_Reset(InverterControl *control) {
    if (control != NULL) {
        QprController_Reset(&control->voltage_alpha_qpr);
        QprController_Reset(&control->voltage_beta_qpr);
        QprController_Reset(&control->current_alpha_qpr);
        QprController_Reset(&control->current_beta_qpr);
        control->reference_rms = 0.0f;
    }
}

/*
 * 设置相电压 RMS 目标 [Vrms]，内部限幅到 [0, voltage_rms_max]。
 * 只改"目标"：实际输出 reference_rms 由 Step 按软启动斜率逐拍逼近。
 * 未初始化或传入非有限值时静默忽略（返回 void，没有失败通道）。
 */
void InverterControl_SetReference(InverterControl *control, float voltage_rms) {
    if ((control != NULL) && control->initialized && isfinite(voltage_rms))
        control->reference_target_rms = InverterControl_Clamp(voltage_rms, 0.0f, control->cfg.voltage_rms_max);
}

/*
 * 重配基波频率 [Hz]：更新相位步长，并按新 ωr 重建四个 QPR 的谐振系数。
 *
 * 先全部算进临时变量、全部成功才整体替换——避免"改一半"留下系数与
 * 频率不匹配的控制器。失败时原配置原样保留并返回 false。
 * UI 的 30/60 Hz 切换就是走这里（见 board_runtime.c）。
 */
bool InverterControl_SetFrequency(InverterControl *control, float frequency_hz) {
    float w0;
    QprController voltage_alpha;
    QprController voltage_beta;
    QprController current_alpha;
    QprController current_beta;

    if ((control == NULL) || !control->initialized || !isfinite(frequency_hz) || (frequency_hz <= 0.0f)) return false;

    w0 = 2.0f * APP_PI_F * frequency_hz;
    if (!QprController_Init(&voltage_alpha, control->cfg.voltage_qpr_kp, control->cfg.voltage_qpr_kr, w0,
                            control->cfg.voltage_qpr_bandwidth_rad_s, control->cfg.sample_time_s,
                            -control->cfg.current_reference_limit_a, control->cfg.current_reference_limit_a) ||
        !QprController_Init(&voltage_beta, control->cfg.voltage_qpr_kp, control->cfg.voltage_qpr_kr, w0,
                            control->cfg.voltage_qpr_bandwidth_rad_s, control->cfg.sample_time_s,
                            -control->cfg.current_reference_limit_a, control->cfg.current_reference_limit_a) ||
        !QprController_Init(&current_alpha, control->cfg.current_qpr_kp, control->cfg.current_qpr_kr, w0,
                            control->cfg.current_qpr_bandwidth_rad_s, control->cfg.sample_time_s,
                            -control->cfg.command_limit_v, control->cfg.command_limit_v) ||
        !QprController_Init(&current_beta, control->cfg.current_qpr_kp, control->cfg.current_qpr_kr, w0,
                            control->cfg.current_qpr_bandwidth_rad_s, control->cfg.sample_time_s,
                            -control->cfg.command_limit_v, control->cfg.command_limit_v) ||
        !PhaseRecurrence_SetFrequency(&control->phase, frequency_hz)) {
        return false;
    }

    control->voltage_alpha_qpr = voltage_alpha;
    control->voltage_beta_qpr = voltage_beta;
    control->current_alpha_qpr = current_alpha;
    control->current_beta_qpr = current_beta;
    control->cfg.output_frequency_hz = frequency_hz;
    return true;
}

/*
 * 一次控制周期的公共实现（Step 与 StepOpenLoop 都走这里）。
 *
 * closed_loop = true  → 正常双闭环；
 * closed_loop = false → 开环诊断：桥压指令直接取软启动后的参考，
 *                       两个 QPR 全部旁路（软启动/相位/SVPWM 限幅照旧）。
 *
 * 返回值语义容易踩坑，记清楚：
 *   true  —— 本周期流程走完（out 里可能是正常命令，也可能是中性值）；
 *   false —— **不能给门极命令**（测量无效或母线低于下限），上层必须按
 *            故障处理；enabled=false 时走"复位+相位继续"分支，返回的是
 *            true 而不是 false——控制器歇着不是错误。
 */
static bool InverterControl_StepInternal(InverterControl *control, const Measurements *measurement, bool enabled,
                                         bool closed_loop, InverterControlOutput *out) {
    float max_step;
    float peak;
    float error_alpha;
    float error_beta;
    float current_error_alpha = 0.0f;
    float current_error_beta = 0.0f;

    if ((control == NULL) || (out == NULL) || !control->initialized) return false;
    InverterControl_ClearOutput(out);

    if (!enabled) {
        /* 未使能：控制器复位，但**相位照旧推进**——这样再次使能时角度
         * 连续，不会从 0 跳变造成输出相位突跳。 */
        InverterControl_Reset(control);
        PhaseRecurrence_Step(&control->phase);
        return true;
    }
    /* 测量无效或母线低于配置下限（10 V）→ 拒绝输出，上层按故障处理。 */
    if ((measurement == NULL) || !measurement->valid || !isfinite(measurement->vdc) ||
        (measurement->vdc < control->cfg.dc_voltage_min_v)) {
        return false;
    }

    /* 软启动步长 [Vrms/拍] = 斜率 × 控制周期；目标调低时也按同一斜率下爬。 */
    max_step = control->cfg.reference_slew_v_per_s * control->cfg.sample_time_s;
    if (control->reference_rms < control->reference_target_rms) {
        control->reference_rms =
            InverterControl_Clamp(control->reference_rms + max_step, 0.0f, control->reference_target_rms);
    } else {
        control->reference_rms = InverterControl_Clamp(control->reference_rms - max_step, control->reference_target_rms,
                                                       control->cfg.voltage_rms_max);
    }

    /* 相峰值 = √2·相RMS；αβ 矢量幅值就等于相峰值（线电压 = 相×√3）。 */
    peak = APP_SQRT2_F * control->reference_rms;
    /* 正相序约定：A=sinθ，B=sin(θ-120°)，C=sin(θ+120°)
     * → Clarke 后 α = peak·sinθ、β = −peak·cosθ（β 的负号来自 B/C 相
     * 的正交投影方向，写反会变成负相序）。 */
    out->reference.alpha = peak * control->phase.sin_value;
    out->reference.beta = -peak * control->phase.cos_value;
    if (closed_loop) {
        /* ── 闭环：电压外环 → 电流内环，各管 αβ 一轴 ── */
        out->measured = Measurement_VoltageAlphaBeta(measurement);
        error_alpha = out->reference.alpha - out->measured.alpha;
        error_beta = out->reference.beta - out->measured.beta;
        out->current_reference.alpha = QprController_Step(&control->voltage_alpha_qpr, error_alpha);
        out->current_reference.beta = QprController_Step(&control->voltage_beta_qpr, error_beta);
        out->current_measured = Measurement_CurrentAlphaBeta(measurement);
        current_error_alpha = out->current_reference.alpha - out->current_measured.alpha;
        current_error_beta = out->current_reference.beta - out->current_measured.beta;

        /* 电压外环给出i*，电流内环再给出逆变桥电压指令。 */
        out->voltage_command.alpha = control->cfg.feedforward_gain * out->reference.alpha +
                                     QprController_Step(&control->current_alpha_qpr, current_error_alpha);
        out->voltage_command.beta = control->cfg.feedforward_gain * out->reference.beta +
                                    QprController_Step(&control->current_beta_qpr, current_error_beta);
        /* 前馈项会抬高指令，加完前馈后再兜底限幅一次
         * （QPR 自身已按 command_limit_v 限过一次）。 */
        out->voltage_command.alpha = InverterControl_Clamp(out->voltage_command.alpha, -control->cfg.command_limit_v,
                                                           control->cfg.command_limit_v);
        out->voltage_command.beta = InverterControl_Clamp(out->voltage_command.beta, -control->cfg.command_limit_v,
                                                          control->cfg.command_limit_v);
    } else {
        /*
         * 开环诊断通道：桥压指令 = 软启动后的参考，两个 QPR 全部旁路。
         * 软启动、相位发生器与 SVPWM 限幅仍然生效——它测的是"功率级 +
         * 调制"的裸能力，不含闭环修正（OLED 的 OL 模式用它）。
         */
        out->voltage_command = out->reference;
    }

    /* 生成三相占空比；applied_scale < 1 表示本拍发生过调制等比例缩放。 */
    out->valid = Svpwm_Generate(out->voltage_command.alpha, out->voltage_command.beta, measurement->vdc,
                                control->cfg.duty_min, control->cfg.duty_max, &out->pwm);
    if (out->valid && closed_loop) {
        /*
         * 抗饱和回灌：把"实际施加的桥压"告诉电流环——即过调制缩放后的
         * 指令、扣掉前馈项（前馈不是 QPR 发出的量，混进去会算错）。
         * 不回灌的话 QPR 会继续按"以为发出了 V、实际只有 0.8V"积累，
         * 解除限幅后长时间过冲。
         */
        QprController_TrackOutput(&control->current_alpha_qpr, current_error_alpha,
                                  out->pwm.applied_scale * out->voltage_command.alpha -
                                      control->cfg.feedforward_gain * out->reference.alpha);
        QprController_TrackOutput(&control->current_beta_qpr, current_error_beta,
                                  out->pwm.applied_scale * out->voltage_command.beta -
                                      control->cfg.feedforward_gain * out->reference.beta);
    }
    /* 相位最后推进：本拍用的 θ 是推进前的，保证参考/QPR/输出同相。 */
    PhaseRecurrence_Step(&control->phase);
    return out->valid;
}

/* 正常闭环入口：closed_loop = true */
bool InverterControl_Step(InverterControl *control, const Measurements *measurement, bool enabled,
                          InverterControlOutput *out) {
    return InverterControl_StepInternal(control, measurement, enabled, true, out);
}

/* 开环诊断入口：closed_loop = false（OLED 的 OL 模式/联调用） */
bool InverterControl_StepOpenLoop(InverterControl *control, const Measurements *measurement, bool enabled,
                                  InverterControlOutput *out) {
    return InverterControl_StepInternal(control, measurement, enabled, false, out);
}
