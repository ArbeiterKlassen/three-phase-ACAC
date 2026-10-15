/*
 * 应用层门面：三模式调度、20 kHz 快环，以及对板级层公开的单例接口。
 *
 * ── 阅读导引（本文件 1390 行，按下面的顺序读最省力）────────────────────
 *
 *   ① 临界区辅助        与 ISR 共享状态时的进出临界区
 *   ② 模式与参考值工具  模式判定、母线→输出参考换算、有效性检查
 *   ③ 安全输出与限幅    出错时输出什么、参考被什么限制
 *   ④ 故障记录器        跳闸前滚动保存现场，供事后定位
 *   ⑤ 实例接口          AppControl_Init / Reset / SetMode（不含 Step）
 *   ⑥ ★ AppControl_Step 核心：一次快环的全部流程（约 360 行）
 *   ⑦ 单例发布          把控制结果搬进对外快照
 *   ⑧ 单例接口          App_* 系列，板级层调用的就是这些
 *
 * ── 两条必须守住的设计约束 ───────────────────────────────────────────
 *
 *   1. 没有任何软件路径能绕过状态机的 pwm_allowed。
 *      硬件 Break 更快——它由定时器直接清 MOE，不经本文件。
 *
 *   2. MODE1 用用户 SET 值代替未接入的 Vdc 作为调制分母（见 ⑥ 的开头）。
 *      这是单级测试台架的设计，不是疏漏；MODE0/MODE2 一律用实测值。
 */

#include "app_control.h"

#include <math.h>
#include <stddef.h>

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#endif

#define APP_INV_SQRT6_F (0.40824829046386301637f)
#define APP_RECTIFIER_START_VIN_RMS_FRACTION (0.10f)
#define APP_RECTIFIER_START_IIN_MAX_A (0.50f)
#define APP_RECTIFIER_START_CONFIRM_SAMPLES (2U)

typedef struct {
    AppControl control;
    AdcRawFrame raw;
    AppCommand command;
    AppOutput output;
    AppPwmCommand pwm;
    AppStatus status;
    volatile uint32_t isr_faults;
    bool frame_pending;
    bool initialized;
} AppRuntime;

static AppRuntime g_app;

/* 保护 ISR 与主循环共享的故障位及使能命令，临界区必须短且可恢复原中断状态。 */
/* ══ ① 临界区辅助 ═════════════════════════════════════════════════════ */

static uint32_t AppRuntime_EnterCritical(void) {
#if defined(STM32F407xx)
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
#else
    return 0U;
#endif
}

static void AppRuntime_ExitCritical(uint32_t primask) {
#if defined(STM32F407xx)
    __DMB();
    if ((primask & 1U) == 0U) __enable_irq();
#else
    (void)primask;
#endif
}

/* ══ ② 模式与参考值工具 ═══════════════════════════════════════════════ */

static bool AppControl_ModeValid(app_mode_t mode) {
    return (mode == MODE0_FULL) || (mode == MODE1_INVERTER) || (mode == MODE2_RECTIFIER);
}

static bool AppControl_ModeHasInverter(app_mode_t mode) {
    return (mode == MODE0_FULL) || (mode == MODE1_INVERTER);
}

static bool AppControl_ModeHasRectifier(app_mode_t mode) {
    return (mode == MODE0_FULL) || (mode == MODE2_RECTIFIER);
}

static bool AppControl_ModeReady(const AppControl *app, app_mode_t mode) {
    if ((app == NULL) || !AppControl_ModeValid(mode) || !BoardPower_ValidateModeConfig(app->cfg, mode)) return false;
    if (AppControl_ModeHasInverter(mode) && !app->inverter.initialized) return false;
    return !AppControl_ModeHasRectifier(mode) || app->rectifier.initialized;
}

static bool AppControl_InverterRunModeValid(inverter_run_mode_t mode) {
    return (mode == INVERTER_CLOSED_LOOP) || (mode == INVERTER_OPEN_LOOP);
}

static bool AppControl_DcReferenceValid(float dc_voltage_v) {
    return isfinite(dc_voltage_v) && (dc_voltage_v >= APP_INVERTER_DC_VOLTAGE_MIN_V) &&
           (dc_voltage_v <= APP_INVERTER_DC_VOLTAGE_MAX_V);
}

static bool AppControl_DeratingDcValid(float dc_voltage_v) {
    return isfinite(dc_voltage_v) &&
           (dc_voltage_v >= (APP_INVERTER_DC_VOLTAGE_MIN_V - APP_INVERTER_DC_MATCH_TOLERANCE_V));
}

static float AppControl_LineReferenceFromDc(float dc_voltage_v) {
    return dc_voltage_v * APP_INVERTER_OUTPUT_LINE_RMS_RATIO;
}

static float AppControl_PhaseReferenceFromDc(float dc_voltage_v) {
    return AppControl_LineReferenceFromDc(dc_voltage_v) * APP_1DIVSQRT3_F;
}

/* ══ ③ 安全输出与参考限幅 ═════════════════════════════════════════════ */

/* 参考上限 = (duty_max − duty_min) × Vdc × (1/√6) × 0.95
 * 推导：SVPWM 线性区最大线电压峰值 = Vdc，对应相电压 RMS = Vdc/√6；
 * 再乘占空比区间与 5% 余量。母线低时参考会被这里削掉——这是正常限幅。 */
static float AppControl_LimitPhaseReference(const InverterControlConfig *cfg, float requested_phase_rms,
                                            float dc_voltage_v) {
    float max_phase_rms;

    if ((cfg == NULL) || !isfinite(requested_phase_rms) || (requested_phase_rms <= 0.0f) ||
        !AppControl_DeratingDcValid(dc_voltage_v)) {
        return 0.0f;
    }

    /*
     * SVPWM允许的最大线电压峰值为(duty_max-duty_min)*Vdc，
     * 因而相电压RMS上限为该值/sqrt(6)。
     */
    max_phase_rms = (cfg->duty_max - cfg->duty_min) * dc_voltage_v * APP_INV_SQRT6_F * APP_INVERTER_VOLTAGE_MARGIN;
    return (requested_phase_rms < max_phase_rms) ? requested_phase_rms : max_phase_rms;
}

static void AppControl_SetSafeCommands(AppOutput *out) {
    out->inverter.a = 0.5f;
    out->inverter.b = 0.5f;
    out->inverter.c = 0.5f;
    out->inverter.enabled = false;
    out->rectifier.leg_a = 0.5f;
    out->rectifier.leg_b = 0.5f;
    out->rectifier.enabled = false;
    out->pwm_allowed = false;
}

static float AppControl_MaxPhaseMagnitude(const Measurements *measurement) {
    float maximum = fabsf(measurement->va);
    float value = fabsf(measurement->vb);

    if (value > maximum) maximum = value;
    value = fabsf(measurement->vc);
    return (value > maximum) ? value : maximum;
}

/* ══ ④ 故障记录器 ═════════════════════════════════════════════════════ */

/* 跳闸前滚动保存三类证据，供事后区分"算错了"还是"量错了"：
 *   fast_history   触发前逐拍原始码（看采样本身有没有问题）
 *   previous_control 触发前最后一拍的控制量（看控制器输出）
 *   cycle_peak_v   历次工频周期的峰值（看是否逐步恶化） */
static void AppControl_ResetFaultRecorder(AppFaultRecorder *recorder) {
    recorder->previous_control.valid = false;
    recorder->previous_control_sequence = 0U;
    recorder->partial_cycle_peak_v = 0.0f;
    recorder->partial_cycle_samples = 0U;
    recorder->cycle_target_samples = 0U;
    recorder->fast_write_index = 0U;
    recorder->fast_history_count = 0U;
    recorder->cycle_write_index = 0U;
    recorder->cycle_peak_count = 0U;
    recorder->active = false;
}

static void AppControl_ClearFaultDiagnostics(AppControl *app) {
    app->fault_snapshot.valid = false;
    AppControl_ResetFaultRecorder(&app->fault_recorder);
}

static void AppControl_BeginFaultRecording(AppControl *app) {
    AppFaultRecorder *recorder = &app->fault_recorder;
    float samples_per_cycle;

    AppControl_ResetFaultRecorder(recorder);
    samples_per_cycle = 1.0f / (app->inverter.cfg.sample_time_s * app->inverter.cfg.output_frequency_hz);
    if (samples_per_cycle < 1.0f) recorder->cycle_target_samples = 1U;
    else if (samples_per_cycle > 65535.0f) recorder->cycle_target_samples = UINT16_MAX;
    else recorder->cycle_target_samples = (uint16_t)(samples_per_cycle + 0.5f);
    recorder->active = true;
}

static void AppControl_RecordFaultDiagnostics(AppControl *app, const AdcRawFrame *raw, const Measurements *measurement,
                                              const InverterControlOutput *inverter) {
    AppFaultRecorder *recorder;
    AppFaultRawSample *sample;
    float phase_max;

    if ((app == NULL) || (raw == NULL) || (measurement == NULL) || (inverter == NULL) || !measurement->valid ||
        !inverter->valid) {
        return;
    }

    if (!app->fault_recorder.active) AppControl_BeginFaultRecording(app);
    recorder = &app->fault_recorder;

    sample = &recorder->fast_history[recorder->fast_write_index];
    sample->sample_sequence = measurement->sequence;
    sample->pc0_raw = raw->ib;
    sample->pc1_raw = raw->ia;
    sample->pc2_raw = raw->vab;
    sample->pc3_raw = raw->vbc;
    recorder->fast_write_index = (uint8_t)((recorder->fast_write_index + 1U) % APP_FAULT_FAST_HISTORY_DEPTH);
    if (recorder->fast_history_count < APP_FAULT_FAST_HISTORY_DEPTH) recorder->fast_history_count++;

    phase_max = AppControl_MaxPhaseMagnitude(measurement);
    if (phase_max > recorder->partial_cycle_peak_v) recorder->partial_cycle_peak_v = phase_max;
    recorder->partial_cycle_samples++;
    if (recorder->partial_cycle_samples >= recorder->cycle_target_samples) {
        recorder->cycle_peak_v[recorder->cycle_write_index] = recorder->partial_cycle_peak_v;
        recorder->cycle_write_index = (uint8_t)((recorder->cycle_write_index + 1U) % APP_FAULT_CYCLE_HISTORY_DEPTH);
        if (recorder->cycle_peak_count < APP_FAULT_CYCLE_HISTORY_DEPTH) recorder->cycle_peak_count++;
        recorder->partial_cycle_peak_v = 0.0f;
        recorder->partial_cycle_samples = 0U;
    }

    recorder->previous_control = *inverter;
    recorder->previous_control_sequence = measurement->sequence;
}

static void AppControl_CaptureOutputOvSnapshot(AppControl *app, const AdcRawFrame *raw, const AppCommand *command,
                                               const Measurements *measurement) {
    AppFaultSnapshot *snapshot;
    const AppFaultRecorder *recorder;
    uint8_t count;
    uint8_t start;
    uint8_t i;
    uint8_t index;

    if ((app == NULL) || (raw == NULL) || (command == NULL) || (measurement == NULL) || !measurement->valid ||
        app->fault_snapshot.valid || ((app->protection.active_faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL)) {
        return;
    }

    snapshot = &app->fault_snapshot;
    recorder = &app->fault_recorder;
    snapshot->valid = false;
    snapshot->fast_history_count = 0U;
    snapshot->cycle_peak_count = 0U;
    snapshot->partial_cycle_peak_v = 0.0f;
    snapshot->partial_cycle_samples = 0U;
    snapshot->previous_control.valid = false;
    snapshot->inverter_run_mode = app->inverter_run_mode;
    snapshot->faults = app->protection.active_faults;
    snapshot->sample_sequence = measurement->sequence;
    snapshot->dc_voltage_reference_v = command->dc_voltage_ref;
    snapshot->vab_v = measurement->vab;
    snapshot->vbc_v = measurement->vbc;
    snapshot->va_v = measurement->va;
    snapshot->vb_v = measurement->vb;
    snapshot->vc_v = measurement->vc;
    snapshot->ia_a = measurement->ia;
    snapshot->ib_a = measurement->ib;
    snapshot->trigger_raw.sample_sequence = measurement->sequence;
    snapshot->trigger_raw.pc0_raw = raw->ib;
    snapshot->trigger_raw.pc1_raw = raw->ia;
    snapshot->trigger_raw.pc2_raw = raw->vab;
    snapshot->trigger_raw.pc3_raw = raw->vbc;
    snapshot->pc2_raw = raw->vab;
    snapshot->pc3_raw = raw->vbc;

    /*
     * Inactive data belongs to an earlier stopped run. Preserve the trigger
     * frame, but never attach stale pre-trigger history to a later OFF fault.
     */
    if (recorder->active) {
        count = recorder->fast_history_count;
        snapshot->fast_history_count = count;
        start = (uint8_t)((recorder->fast_write_index + APP_FAULT_FAST_HISTORY_DEPTH - count) %
                          APP_FAULT_FAST_HISTORY_DEPTH);
        for (i = 0U; i < count; i++) {
            index = (uint8_t)((start + i) % APP_FAULT_FAST_HISTORY_DEPTH);
            snapshot->fast_history[i] = recorder->fast_history[index];
        }

        count = recorder->cycle_peak_count;
        snapshot->cycle_peak_count = count;
        for (i = 0U; i < count; i++) {
            index = (uint8_t)((recorder->cycle_write_index + APP_FAULT_CYCLE_HISTORY_DEPTH - 1U - i) %
                              APP_FAULT_CYCLE_HISTORY_DEPTH);
            snapshot->cycle_peak_v[i] = recorder->cycle_peak_v[index];
        }
        snapshot->partial_cycle_peak_v = recorder->partial_cycle_peak_v;
        snapshot->partial_cycle_samples = recorder->partial_cycle_samples;

        if (recorder->previous_control.valid) {
            snapshot->previous_control.voltage_reference = recorder->previous_control.reference;
            snapshot->previous_control.voltage_measured = recorder->previous_control.measured;
            snapshot->previous_control.current_reference = recorder->previous_control.current_reference;
            snapshot->previous_control.current_measured = recorder->previous_control.current_measured;
            snapshot->previous_control.voltage_command = recorder->previous_control.voltage_command;
            snapshot->previous_control.sample_sequence = recorder->previous_control_sequence;
            snapshot->previous_control.duty_a = recorder->previous_control.pwm.duty_a;
            snapshot->previous_control.duty_b = recorder->previous_control.pwm.duty_b;
            snapshot->previous_control.duty_c = recorder->previous_control.pwm.duty_c;
            snapshot->previous_control.applied_scale = recorder->previous_control.pwm.applied_scale;
            snapshot->previous_control.saturated = recorder->previous_control.pwm.saturated;
            snapshot->previous_control.valid = true;
        }
    }
    snapshot->valid = true;
}

static void AppControl_CaptureInputOvSnapshot(AppControl *app, const AdcRawFrame *raw, const AppCommand *command,
                                              const Measurements *measurement) {
    AppFaultSnapshot *snapshot;

    if ((app == NULL) || (raw == NULL) || (command == NULL) || (measurement == NULL) || !measurement->valid ||
        app->fault_snapshot.valid || ((app->protection.active_faults & PROTECTION_FAULT_INPUT_OVERVOLTAGE) == 0UL)) {
        return;
    }

    snapshot = &app->fault_snapshot;
    *snapshot = (AppFaultSnapshot){0};
    snapshot->inverter_run_mode = app->inverter_run_mode;
    snapshot->faults = app->protection.active_faults;
    snapshot->sample_sequence = measurement->sequence;
    snapshot->dc_voltage_reference_v = command->dc_voltage_ref;
    snapshot->vin_v = measurement->vin;
    snapshot->vdc_v = measurement->vdc;
    snapshot->vin_raw = raw->vin;
    snapshot->previous_vin_valid = app->previous_vin_valid;
    snapshot->previous_vin_sequence = app->previous_vin_sequence;
    snapshot->previous_vin_raw = app->previous_vin_raw;
    snapshot->previous_vin_v = app->previous_vin_v;
    snapshot->valid = true;
}

static void AppControl_CaptureInputOcSnapshot(AppControl *app, const AdcRawFrame *raw, const AppCommand *command,
                                              const Measurements *measurement) {
    AppFaultSnapshot *snapshot;

    if ((app == NULL) || (raw == NULL) || (command == NULL) || (measurement == NULL) || !measurement->valid ||
        app->fault_snapshot.valid || ((app->protection.active_faults & PROTECTION_FAULT_INPUT_OVERCURRENT) == 0UL)) {
        return;
    }

    snapshot = &app->fault_snapshot;
    *snapshot = (AppFaultSnapshot){0};
    snapshot->inverter_run_mode = app->inverter_run_mode;
    snapshot->faults = app->protection.active_faults;
    snapshot->sample_sequence = measurement->sequence;
    snapshot->dc_voltage_reference_v = command->dc_voltage_ref;
    snapshot->iin_a = measurement->iin;
    snapshot->vdc_v = measurement->vdc;
    snapshot->iin_raw = raw->iin;
    snapshot->previous_iin_valid = app->previous_iin_valid;
    snapshot->previous_iin_sequence = app->previous_iin_sequence;
    snapshot->previous_iin_raw = app->previous_iin_raw;
    snapshot->previous_iin_a = app->previous_iin_a;
    snapshot->valid = true;
}

/* ══ ⑤ 实例接口 ═══════════════════════════════════════════════════════ */

bool AppControl_Init(AppControl *app) {
    const BoardPowerConfig *cfg;

    if (app == NULL) return false;

    *app = (AppControl){0};
    cfg = BoardPower_GetConfig();
    if (cfg == NULL) return false;

    app->cfg = cfg;
    app->mode = MODE1_INVERTER;
    app->inverter_run_mode = INVERTER_CLOSED_LOOP;
    Protection_Init(&app->protection);
    PowerState_Init(&app->state);
    if (!InverterControl_Init(&app->inverter, &cfg->inverter)) return false;

    /*
     * MODE1 is the proven default path and must not be coupled to a future
     * invalid rectifier parameter set. Rectifier modes remain hard-gated by
     * their mode-specific board validation and control initialized flag.
     */
    if (RectifierControl_Init(&app->rectifier, &cfg->rectifier))
        RectifierControl_SetDcReference(&app->rectifier, cfg->dc_voltage_ref);
    app->board_config_valid = AppControl_ModeReady(app, app->mode);
    if (!app->board_config_valid) return false;
    app->validated_cfg = app->cfg;
    app->validated_mode = app->mode;
    app->initialized = true;
    return true;
}

void AppControl_Reset(AppControl *app) {
    if ((app != NULL) && app->initialized) {
        InverterControl_Reset(&app->inverter);
        RectifierControl_Reset(&app->rectifier);
        RectifierControl_SetDcReference(&app->rectifier, app->cfg->dc_voltage_ref);
        Protection_Init(&app->protection);
        PowerState_Init(&app->state);
        AppControl_ClearFaultDiagnostics(app);
        app->previous_iin_valid = false;
        app->previous_vin_valid = false;
        app->mode0_dc_ready_count = 0U;
        app->mode0_inverter_ready = false;
        app->rectifier_dc_ready = false;
        app->rectifier_start_count = 0U;
        app->rectifier_start_ready = false;
    }
}

bool AppControl_SetMode(AppControl *app, app_mode_t mode) {
    bool board_config_valid;
    bool state_allows_change;

    if ((app == NULL) || !app->initialized || !AppControl_ModeValid(mode)) return false;

    state_allows_change = ((app->state.state == OFF) || (app->state.state == FAULT_LATCH)) && !app->state.pwm_allowed;
    if (!state_allows_change) return false;

    board_config_valid = AppControl_ModeReady(app, mode);
    if (!board_config_valid) return false;

    app->mode = mode;
    app->validated_cfg = app->cfg;
    app->validated_mode = mode;
    app->board_config_valid = board_config_valid;
    InverterControl_Reset(&app->inverter);
    RectifierControl_Reset(&app->rectifier);
    RectifierControl_SetDcReference(&app->rectifier, app->cfg->dc_voltage_ref);
    app->mode0_dc_ready_count = 0U;
    app->mode0_inverter_ready = false;
    app->rectifier_dc_ready = false;
    app->rectifier_start_count = 0U;
    app->rectifier_start_ready = false;
    app->previous_iin_valid = false;
    app->previous_vin_valid = false;
    return true;
}

bool AppControl_SetInverterRunMode(AppControl *app, inverter_run_mode_t mode) {
    bool state_allows_change;

    if ((app == NULL) || !app->initialized || !AppControl_InverterRunModeValid(mode)) return false;

    state_allows_change = ((app->state.state == OFF) || (app->state.state == FAULT_LATCH)) && !app->state.pwm_allowed;
    if (!state_allows_change) return false;

    app->inverter_run_mode = mode;
    InverterControl_Reset(&app->inverter);
    return true;
}

/* ══ ⑥ ★ 核心：一次 20 kHz 快环 ═══════════════════════════════════════ */

/*
 * 执行顺序（每一段的边界都有注释标出）：
 *
 *   A. 参数与实例校验，准备本拍的基础量
 *   B. MODE1 特例：用 SET 值替代未接入的 Vdc
 *   C. 逆变参考：解耦设定 → 回退比例推导 → 按母线降额限幅
 *   D. 整流使能与就绪条件（含 MODE0 的母线就绪计数）
 *   E. 保护判断：本拍故障 + 锁存
 *   F. 状态机推进
 *   G. 按模式运行逆变/整流控制
 *   H. 组装输出：占空比 + 两级使能请求
 *   I. 发布状态快照
 *
 * 任何一步失败都不会"跳过"，而是走安全输出（0.5 中性占空比 + 撤销使能），
 * 由后面的使能请求统一裁决是否真的输出。
 */
bool AppControl_Step(AppControl *app, const AdcRawFrame *raw, const AppCommand *command, AppOutput *out) {
    Measurements measurement;
    Measurements inverter_measurement;
    PowerStateInput state_input;
    InverterControlOutput inverter = {0};
    app_mode_t mode;
    bool board_valid;
    bool measurement_valid;
    bool dc_reference_valid;
    bool precharge_done;
    bool phase_ready;
    bool run;
    bool has_rectifier;
    bool rectifier_enable;
    bool inverter_enable;
    bool rectifier_ok = true;
    bool rectifier_pwm_valid = false;
    bool rectifier_gates_enabled = false;
    bool inverter_ok;
    bool dc_in_tolerance;
    float rectifier_duty_a = 0.5f;
    float rectifier_duty_b = 0.5f;
    float effective_reference_rms;
    float inverter_dc_voltage;

    if ((app == NULL) || (raw == NULL) || (command == NULL) || (out == NULL) || !app->initialized) return false;

    AppControl_SetSafeCommands(out);
    mode = app->mode;
    board_valid = app->board_config_valid && (app->validated_cfg == app->cfg) && (app->validated_mode == mode) &&
                  AppControl_ModeValid(mode);
    has_rectifier = AppControl_ModeHasRectifier(mode);
    switch (mode) {
    case MODE2_RECTIFIER:
        measurement_valid = Measurement_ConvertRectifier(&app->cfg->measurement, raw, &measurement);
        break;
    case MODE0_FULL:
        measurement_valid = Measurement_Convert(&app->cfg->measurement, raw, &measurement);
        break;
    case MODE1_INVERTER:
        measurement_valid = Measurement_ConvertInverter(&app->cfg->measurement, raw, &measurement);
        break;
    default:
        measurement = (Measurements){0};
        measurement_valid = false;
        break;
    }
    measurement.valid = measurement_valid;
    dc_reference_valid = AppControl_DcReferenceValid(command->dc_voltage_ref);
    inverter_measurement = measurement;
    if (mode == MODE1_INVERTER) {
        /*
         * 保留已完成实测的逆变单级行为：该夹具未接 PC4 Vdc，
         * 调制分母和降额继续使用经过范围校验的用户设定值。
         * MODE0/MODE2 必须使用真实 Vdc，不进入此特例。
         */
        inverter_measurement.vdc = dc_reference_valid ? command->dc_voltage_ref : 0.0f;
    }

    if (board_valid) {
        /*
         * dc_voltage_min_run_v is the inverter-side minimum. MODE2 and the
         * rectifier-only portion of MODE0 may legitimately boost from the
         * lower passive bus; RectifierControl enforces dc_voltage_min_v.
         */
        (void)Protection_Evaluate(&app->protection, &app->cfg->protection, &measurement,
                                  (mode == MODE0_FULL) && app->mode0_inverter_ready && (app->state.state == RUN),
                                  command->external_fault, command->loop_overrun);
        if (command->enable && !dc_reference_valid) Protection_ForceFault(&app->protection, PROTECTION_FAULT_CONTROL);
    } else {
        app->protection.active_faults = PROTECTION_FAULT_NONE;
        if (command->external_fault) Protection_ForceFault(&app->protection, PROTECTION_FAULT_EXTERNAL);
        if (command->loop_overrun) Protection_ForceFault(&app->protection, PROTECTION_FAULT_LOOP_OVERRUN);
    }

    AppControl_CaptureInputOcSnapshot(app, raw, command, &measurement);
    AppControl_CaptureInputOvSnapshot(app, raw, command, &measurement);
    if (AppControl_ModeHasInverter(mode)) AppControl_CaptureOutputOvSnapshot(app, raw, command, &measurement);

    if (command->reset_fault && (app->protection.active_faults == PROTECTION_FAULT_NONE)) {
        if (Protection_ClearLatched(&app->protection)) AppControl_ClearFaultDiagnostics(app);
    }

    if (mode == MODE1_INVERTER) {
        precharge_done = true;
        phase_ready = true;
    } else {
        /*
         * 整流/全级只能在真实母线已被动或外部预充、且完整 20 ms
         * 输入 RMS 窗口有效后进入 RUN。外部板级联锁仍可显式提交。
         */
        precharge_done =
            command->precharge_done || (measurement_valid && (measurement.vdc >= app->cfg->rectifier.dc_voltage_min_v));
        phase_ready =
            command->phase_ready || (measurement_valid && app->rectifier.vin_rms_valid &&
                                     (app->rectifier.vin_rms_v >= app->cfg->rectifier.input_voltage_rms_min_v));
    }

    state_input.start_request = command->enable;
    state_input.stop_request = !command->enable;
    state_input.reset_fault_request = command->reset_fault;
    state_input.precharge_done = precharge_done;
    state_input.phase_ready = phase_ready;
    state_input.measurement_valid = measurement_valid;
    state_input.discharge_done =
        (mode == MODE1_INVERTER) || (measurement_valid && (measurement.vdc <= app->cfg->rectifier.dc_voltage_min_v));
    state_input.board_config_valid = board_valid;
    state_input.active_faults = app->protection.active_faults;
    state_input.latched_faults = app->protection.latched_faults;
    PowerState_Step(&app->state, &app->cfg->state, &state_input);

    run = app->state.pwm_allowed;
    inverter_dc_voltage =
        (mode == MODE1_INVERTER) ? command->dc_voltage_ref : (measurement_valid ? measurement.vdc : 0.0f);
    /*
     * 输出参考与母线解耦：优先使用独立设定的 inverter_vrms_ref；
     * 未设置（<=0 或非法）时回退为按母线参考 32/60 比例推导，
     * 兼容旧的纯逆变比例跟随用法。
     */
    effective_reference_rms = (isfinite(command->inverter_vrms_ref) && (command->inverter_vrms_ref > 0.0f))
                                  ? command->inverter_vrms_ref
                                  : AppControl_PhaseReferenceFromDc(command->dc_voltage_ref);
    effective_reference_rms =
        AppControl_LimitPhaseReference(&app->inverter.cfg, effective_reference_rms, inverter_dc_voltage);
    InverterControl_SetReference(&app->inverter, effective_reference_rms);

    /*
     * 整流控制在 SELF_TEST/PRECHARGE/READY 中以 enabled=false 预热
     * 一周期 RMS 与母线陷波；只有 RUN 才允许产生门极命令。
     */
    rectifier_enable = false;
    dc_in_tolerance = false;
    if (has_rectifier) {
        RectifierControlOutput rectifier = {0};
        float start_vin_limit = APP_RECTIFIER_START_VIN_RMS_FRACTION * app->rectifier.vin_rms_v;

        if (!run) {
            app->rectifier_start_count = 0U;
            app->rectifier_start_ready = false;
        } else if (!app->rectifier_start_ready) {
            if (measurement_valid && app->rectifier.vin_rms_valid && (fabsf(measurement.vin) <= start_vin_limit) &&
                (fabsf(measurement.iin) <= APP_RECTIFIER_START_IIN_MAX_A)) {
                if (app->rectifier_start_count < APP_RECTIFIER_START_CONFIRM_SAMPLES) app->rectifier_start_count++;
                app->rectifier_start_ready = app->rectifier_start_count >= APP_RECTIFIER_START_CONFIRM_SAMPLES;
            } else {
                app->rectifier_start_count = 0U;
            }
        }

        rectifier_enable = run && app->rectifier_start_ready;
        rectifier_ok = RectifierControl_Step(&app->rectifier, &measurement, rectifier_enable, &rectifier);
        dc_in_tolerance = rectifier_ok && app->rectifier.vin_rms_valid &&
                          (fabsf(rectifier.dc_voltage_reference - app->rectifier.dc_reference_target_v) <=
                           app->cfg->rectifier.dc_ready_tolerance_v) &&
                          (fabsf(rectifier.dc_voltage_filtered - app->rectifier.dc_reference_target_v) <=
                           app->cfg->rectifier.dc_ready_tolerance_v);
        rectifier_pwm_valid = rectifier.pwm.valid;
        rectifier_gates_enabled = rectifier.gates_enabled;
        rectifier_duty_a = rectifier.pwm.duty_a;
        rectifier_duty_b = rectifier.pwm.duty_b;
    }
    if ((mode == MODE0_FULL) && rectifier_enable && dc_in_tolerance) {
        if (app->mode0_dc_ready_count < UINT16_MAX) app->mode0_dc_ready_count++;
    } else {
        app->mode0_dc_ready_count = 0U;
    }
    app->mode0_inverter_ready =
        (mode == MODE0_FULL) && (app->mode0_dc_ready_count >= app->cfg->rectifier.dc_ready_confirm_samples);
    app->rectifier_dc_ready =
        (mode == MODE0_FULL) ? app->mode0_inverter_ready : ((mode == MODE2_RECTIFIER) && dc_in_tolerance);

    inverter_enable = run && ((mode == MODE1_INVERTER) || ((mode == MODE0_FULL) && app->mode0_inverter_ready));
    if (app->inverter_run_mode == INVERTER_CLOSED_LOOP)
        inverter_ok = InverterControl_Step(&app->inverter, &inverter_measurement, inverter_enable, &inverter);
    else if (app->inverter_run_mode == INVERTER_OPEN_LOOP)
        inverter_ok = InverterControl_StepOpenLoop(&app->inverter, &inverter_measurement, inverter_enable, &inverter);
    else inverter_ok = false;

    if ((rectifier_enable && !rectifier_ok) || (inverter_enable && !inverter_ok)) {
        Protection_ForceFault(&app->protection, PROTECTION_FAULT_CONTROL);
        state_input.active_faults = app->protection.active_faults;
        state_input.latched_faults = app->protection.latched_faults;
        PowerState_Step(&app->state, &app->cfg->state, &state_input);
        rectifier_enable = false;
        inverter_enable = false;
        RectifierControl_Reset(&app->rectifier);
        InverterControl_Reset(&app->inverter);
        app->mode0_dc_ready_count = 0U;
        app->mode0_inverter_ready = false;
        app->rectifier_dc_ready = false;
    }

    out->measurement = measurement;
    out->state = app->state.state;
    out->faults = app->protection.latched_faults;
    out->board_config_valid = board_valid;
    out->pwm_allowed = app->state.pwm_allowed && (rectifier_enable || inverter_enable) &&
                       (!rectifier_enable || (rectifier_ok && rectifier_pwm_valid)) &&
                       (!inverter_enable || (inverter_ok && inverter.pwm.valid));

    if (inverter_enable && inverter_ok && inverter.pwm.valid) {
        out->inverter.a = inverter.pwm.duty_a;
        out->inverter.b = inverter.pwm.duty_b;
        out->inverter.c = inverter.pwm.duty_c;
        out->inverter.enabled = true;
        AppControl_RecordFaultDiagnostics(app, raw, &measurement, &inverter);
    }
    if (rectifier_enable && rectifier_ok && rectifier_pwm_valid) {
        out->rectifier.leg_a = rectifier_duty_a;
        out->rectifier.leg_b = rectifier_duty_b;
        /* 外环零电流段（软启动爬坡）门极保持关断，桥臂高阻。 */
        out->rectifier.enabled = rectifier_gates_enabled;
    }
    if (!out->inverter.enabled) app->fault_recorder.active = false;

    if (!out->pwm_allowed) AppControl_SetSafeCommands(out);

    if (measurement_valid) {
        app->previous_iin_sequence = measurement.sequence;
        app->previous_iin_raw = raw->iin;
        app->previous_iin_a = measurement.iin;
        app->previous_iin_valid = true;
        app->previous_vin_sequence = measurement.sequence;
        app->previous_vin_raw = raw->vin;
        app->previous_vin_v = measurement.vin;
        app->previous_vin_valid = true;
    }

    return true;
}

/* ══ ⑦ 单例发布 ═══════════════════════════════════════════════════════ */

/* 把内部状态搬进对外快照。后台任务读取时也会短暂关中断，避免读到半更新值。 */
static void AppRuntime_UpdatePublic(void) {
    /*
     * 快环结束后集中发布命令和状态。若计算与发布之间又有 ISR
     * 故障到达，TripFromISR 已撤销使能，这里再次强制安全命令，
     * 防止旧计算结果在下一拍重新打开任一级功率输出。
     */
    if (g_app.isr_faults != 0UL) AppControl_SetSafeCommands(&g_app.output);

    g_app.pwm.inverter_a = g_app.output.inverter.a;
    g_app.pwm.inverter_b = g_app.output.inverter.b;
    g_app.pwm.inverter_c = g_app.output.inverter.c;
    g_app.pwm.rectifier_a = g_app.output.rectifier.leg_a;
    g_app.pwm.rectifier_b = g_app.output.rectifier.leg_b;
    g_app.pwm.inverter_enable_request = g_app.output.pwm_allowed && g_app.output.inverter.enabled;
    g_app.pwm.rectifier_enable_request = g_app.output.pwm_allowed && g_app.output.rectifier.enabled;

    g_app.status.mode = g_app.control.mode;
    g_app.status.inverter_run_mode = g_app.control.inverter_run_mode;
    g_app.status.state = g_app.output.state;
    g_app.status.output_frequency_hz = g_app.control.inverter.cfg.output_frequency_hz;
    g_app.status.dc_voltage_v = g_app.output.measurement.vdc;
    g_app.status.output_line_voltage_reference_v = g_app.control.inverter.reference_target_rms * APP_SQRT3_F;
    g_app.status.dc_voltage_reference_v = g_app.command.dc_voltage_ref;
    g_app.status.iin_a = g_app.output.measurement.iin;
    g_app.status.ia_a = g_app.output.measurement.ia;
    g_app.status.ib_a = g_app.output.measurement.ib;
    g_app.status.vab_v = g_app.output.measurement.vab;
    g_app.status.vbc_v = g_app.output.measurement.vbc;
    g_app.status.iin_raw = g_app.raw.iin;
    /*
     * The extended diagnostic snapshot is immutable after capture. Copy it
     * only for a new event; clearing only needs to invalidate the old event.
     */
    if (g_app.status.fault_snapshot.valid && !g_app.control.fault_snapshot.valid) {
        g_app.status.fault_snapshot.valid = false;
    } else if (g_app.control.fault_snapshot.valid &&
               (!g_app.status.fault_snapshot.valid ||
                (g_app.status.fault_snapshot.sample_sequence != g_app.control.fault_snapshot.sample_sequence))) {
        g_app.status.fault_snapshot = g_app.control.fault_snapshot;
    }
    g_app.status.faults = g_app.output.faults;
    g_app.status.sample_sequence = g_app.output.measurement.sequence;
    g_app.status.board_config_valid = g_app.output.board_config_valid;
    g_app.status.pwm_allowed = g_app.output.pwm_allowed;
    g_app.status.rectifier_dc_ready = g_app.control.rectifier_dc_ready;
}

/* ══ ⑧ 单例接口（板级层调用） ═════════════════════════════════════════ */

void App_Init(void) {
    g_app = (AppRuntime){0};
    g_app.initialized = AppControl_Init(&g_app.control);
    if (g_app.initialized) {
        g_app.command.dc_voltage_ref = g_app.control.cfg->dc_voltage_ref;
        /* 输出参考固定初始化为赛题 32V 线电压对应的相电压，
         * 不随 62V 母线目标联动。 */
        g_app.command.inverter_vrms_ref = APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V * APP_1DIVSQRT3_F;
        InverterControl_SetReference(&g_app.control.inverter,
                                     AppControl_LimitPhaseReference(&g_app.control.inverter.cfg,
                                                                    g_app.command.inverter_vrms_ref,
                                                                    g_app.command.dc_voltage_ref));
        RectifierControl_SetDcReference(&g_app.control.rectifier, g_app.command.dc_voltage_ref);
        g_app.output.board_config_valid = g_app.control.board_config_valid;
    }
    g_app.output.state = OFF;
    AppControl_SetSafeCommands(&g_app.output);
    AppRuntime_UpdatePublic();
}

bool App_RequestMode(app_mode_t mode) {
    uint32_t primask;
    bool changed = false;
    bool state_allows_change;

    if (!g_app.initialized || g_app.command.enable) return false;

    primask = AppRuntime_EnterCritical();
    state_allows_change = ((g_app.status.state == OFF) || (g_app.status.state == FAULT_LATCH)) &&
                          !g_app.status.pwm_allowed && !g_app.command.enable;
    if (state_allows_change) changed = AppControl_SetMode(&g_app.control, mode);
    if (changed) {
        RectifierControl_SetDcReference(&g_app.control.rectifier, g_app.command.dc_voltage_ref);
        g_app.frame_pending = false;
        g_app.status.mode = mode;
        g_app.status.rectifier_dc_ready = false;
        g_app.output.board_config_valid = g_app.control.board_config_valid;
        g_app.status.board_config_valid = g_app.control.board_config_valid;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

bool App_SetInverterRunMode(inverter_run_mode_t mode) {
    uint32_t primask;
    bool changed = false;
    bool state_allows_change;

    if (!g_app.initialized || !AppControl_InverterRunModeValid(mode)) return false;

    primask = AppRuntime_EnterCritical();
    state_allows_change = ((g_app.status.state == OFF) || (g_app.status.state == FAULT_LATCH)) &&
                          !g_app.status.pwm_allowed && !g_app.command.enable;
    if (state_allows_change) {
        changed = AppControl_SetInverterRunMode(&g_app.control, mode);
        if (changed) g_app.status.inverter_run_mode = mode;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

void App_RequestStart(void) {
    if (g_app.initialized) {
        g_app.command.enable = true;
        g_app.command.reset_fault = false;
    }
}

void App_RequestStop(void) {
    g_app.command.enable = false;
}

bool App_ApplyAdcPairs(const uint32_t adc_cdr_pairs[4], uint32_t sequence) {
    if ((adc_cdr_pairs == NULL) || !g_app.initialized) return false;

    if (g_app.control.mode == MODE1_INVERTER) {
        if (!Measurement_UnpackInverterAdcPairs(adc_cdr_pairs, sequence, &g_app.raw)) {
            App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
            return false;
        }
    } else if (g_app.control.mode == MODE2_RECTIFIER) {
        if (!Measurement_UnpackRectifierAdcPairs(adc_cdr_pairs, sequence, &g_app.raw)) {
            App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
            return false;
        }
    } else if (g_app.control.mode == MODE0_FULL) {
        if (!Measurement_UnpackDualAdcPairs(adc_cdr_pairs, sequence, g_app.control.cfg->vin_diagnostic_tolerance_counts,
                                            &g_app.raw)) {
            App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
            return false;
        }
    } else {
        App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
        return false;
    }

    g_app.frame_pending = true;
    return true;
}

bool App_ApplyAdcSamples(const uint16_t adc_samples[8], uint32_t sequence) {
    uint32_t pairs[4];
    uint8_t i;

    if ((adc_samples == NULL) || !g_app.initialized) return false;

    for (i = 0U; i < 4U; i++) pairs[i] = (uint32_t)adc_samples[2U * i] | ((uint32_t)adc_samples[2U * i + 1U] << 16);
    return App_ApplyAdcPairs(pairs, sequence);
}

void App_FastStep20kHz(void) {
    uint32_t isr_faults;

    if (!g_app.initialized) return;

    /* 先处理异步硬件故障，再消费 ADC 帧，保证故障优先于本拍控制计算。 */
    isr_faults = g_app.isr_faults;
    if (isr_faults != 0UL) {
        Protection_ForceFault(&g_app.control.protection, isr_faults);
        g_app.command.external_fault = (isr_faults & PROTECTION_FAULT_EXTERNAL) != 0UL;
        g_app.command.loop_overrun = (isr_faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0UL;
        g_app.command.enable = false;
    }
    if (!g_app.frame_pending) g_app.raw.valid = false;

    (void)AppControl_Step(&g_app.control, &g_app.raw, &g_app.command, &g_app.output);
    g_app.frame_pending = false;
    g_app.command.reset_fault = false;
    AppRuntime_UpdatePublic();
}

void App_BackgroundTask(void) {
    /* 预留给低速命令解析/遥测；禁止在此直接打开PWM。 */
}

void App_TripFromISR(fault_flags_t faults) {
    uint32_t primask;

    if (faults == 0UL) faults = PROTECTION_FAULT_EXTERNAL;

    /*
     * 故障位可能由不同优先级中断同时置位；短临界区避免读改写丢位，
     * 同时保证复位操作不能覆盖刚到达的硬件故障。
     */
    primask = AppRuntime_EnterCritical();
    g_app.isr_faults |= faults;
    g_app.command.enable = false;
    g_app.pwm.inverter_enable_request = false;
    g_app.pwm.rectifier_enable_request = false;
    g_app.status.pwm_allowed = false;
    AppRuntime_ExitCritical(primask);
}

const AppPwmCommand *App_GetPwmCommand(void) {
    return &g_app.pwm;
}

const AppStatus *App_GetStatus(void) {
    return &g_app.status;
}

bool App_SetOutputFrequency(uint16_t frequency_hz) {
    uint32_t primask;
    bool changed = false;

    if (!g_app.initialized || ((frequency_hz != 30U) && (frequency_hz != 60U))) return false;

    primask = AppRuntime_EnterCritical();
    if ((g_app.status.state == OFF) && !g_app.command.enable && (g_app.control.mode != MODE2_RECTIFIER) &&
        InverterControl_SetFrequency(&g_app.control.inverter, (float)frequency_hz)) {
        g_app.status.output_frequency_hz = (float)frequency_hz;
        changed = true;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

bool App_SetOutputLineVoltageReference(float line_rms) {
    uint32_t primask;
    float phase_rms;
    bool state_allows_update;
    bool changed = false;

    if (!g_app.initialized || !isfinite(line_rms) || (line_rms < APP_INVERTER_OUTPUT_LINE_RMS_MIN_V) ||
        (line_rms > APP_INVERTER_OUTPUT_LINE_RMS_MAX_V)) {
        return false;
    }

    phase_rms = line_rms * APP_1DIVSQRT3_F;
    if (phase_rms > g_app.control.inverter.cfg.voltage_rms_max) return false;

    /*
     * 输出线电压参考与母线解耦后的独立设定入口（赛题 32V±0.1V
     * 现场微调通道）。仅停机/故障锁存且 PWM 关断时允许改写，
     * 与母线参考修改的门控一致。
     */
    primask = AppRuntime_EnterCritical();
    state_allows_update = ((g_app.status.state == OFF) && !g_app.command.enable) ||
                          ((g_app.status.state == FAULT_LATCH) && !g_app.status.pwm_allowed);
    if (state_allows_update) {
        g_app.command.inverter_vrms_ref = phase_rms;
        changed = true;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

bool App_SetDcVoltageReference(float volts) {
    uint32_t primask;
    bool state_allows_update;
    bool changed = false;

    if (!g_app.initialized || !AppControl_DcReferenceValid(volts)) return false;

    /*
     * 输出电压参考已与母线解耦：这里只改母线目标，
     * 不再联动修改 inverter_vrms_ref 与输出参考显示。
     */
    primask = AppRuntime_EnterCritical();
    state_allows_update = ((g_app.status.state == OFF) && !g_app.command.enable) ||
                          ((g_app.status.state == FAULT_LATCH) && !g_app.status.pwm_allowed);
    if (state_allows_update) {
        g_app.command.dc_voltage_ref = volts;
        if (AppControl_ModeHasRectifier(g_app.control.mode))
            RectifierControl_SetDcReference(&g_app.control.rectifier, volts);
        g_app.status.dc_voltage_reference_v = volts;
        changed = true;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

void App_ResetFault(void) {
    uint32_t primask = AppRuntime_EnterCritical();

    g_app.command.enable = false;
    g_app.command.external_fault = false;
    g_app.command.loop_overrun = false;
    g_app.command.reset_fault = true;
    g_app.isr_faults = 0UL;
    AppRuntime_ExitCritical(primask);
}

void App_SetPrechargeDone(bool done) {
    g_app.command.precharge_done = done;
}

void App_SetPhaseReady(bool ready) {
    g_app.command.phase_ready = ready;
}
