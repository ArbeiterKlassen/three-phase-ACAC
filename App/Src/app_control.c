#include "app_control.h"

#include <math.h>
#include <stddef.h>

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#endif

#define APP_INV_SQRT6_F (0.40824829046386301637f)

typedef struct
{
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
static uint32_t AppRuntime_EnterCritical(void)
{
#if defined(STM32F407xx)
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
#else
    return 0U;
#endif
}

static void AppRuntime_ExitCritical(uint32_t primask)
{
#if defined(STM32F407xx)
    __DMB();
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
#else
    (void)primask;
#endif
}

static bool AppControl_ModeValid(app_mode_t mode)
{
    return (mode == MODE0_FULL)
        || (mode == MODE1_RECTIFIER)
        || (mode == MODE2_INVERTER);
}

static bool AppControl_DcReferenceValid(float dc_voltage_v)
{
    return isfinite(dc_voltage_v)
        && (dc_voltage_v >= APP_INVERTER_DC_VOLTAGE_MIN_V)
        && (dc_voltage_v <= APP_INVERTER_DC_VOLTAGE_MAX_V);
}

static bool AppControl_DeratingDcValid(float dc_voltage_v)
{
    return isfinite(dc_voltage_v)
        && (dc_voltage_v
            >= (APP_INVERTER_DC_VOLTAGE_MIN_V
                - APP_INVERTER_DC_MATCH_TOLERANCE_V));
}

static float AppControl_LineReferenceFromDc(float dc_voltage_v)
{
    return dc_voltage_v * APP_INVERTER_OUTPUT_LINE_RMS_RATIO;
}

static float AppControl_PhaseReferenceFromDc(float dc_voltage_v)
{
    return AppControl_LineReferenceFromDc(dc_voltage_v)
         * APP_INV_SQRT3_F;
}

static float AppControl_LimitPhaseReference(
    const InverterControlConfig *cfg,
    float requested_phase_rms,
    float dc_voltage_v)
{
    float max_phase_rms;

    if ((cfg == NULL) || !isfinite(requested_phase_rms)
        || (requested_phase_rms <= 0.0f)
        || !AppControl_DeratingDcValid(dc_voltage_v))
    {
        return 0.0f;
    }

    /*
     * SVPWM允许的最大线电压峰值为(duty_max-duty_min)*Vdc，
     * 因而相电压RMS上限为该值/sqrt(6)。
     */
    max_phase_rms = (cfg->duty_max - cfg->duty_min)
                  * dc_voltage_v * APP_INV_SQRT6_F
                  * APP_INVERTER_VOLTAGE_MARGIN;
    return (requested_phase_rms < max_phase_rms)
        ? requested_phase_rms : max_phase_rms;
}

static void AppControl_SetSafeCommands(AppOutput *out)
{
    out->inverter.a = 0.5f;
    out->inverter.b = 0.5f;
    out->inverter.c = 0.5f;
    out->inverter.enabled = false;
    out->rectifier.leg_a = 0.5f;
    out->rectifier.leg_b = 0.5f;
    out->rectifier.enabled = false;
    out->pwm_allowed = false;
}

bool AppControl_Init(AppControl *app)
{
    const BoardPowerConfig *cfg;

    if (app == NULL)
    {
        return false;
    }

    *app = (AppControl){0};
    cfg = BoardPower_GetConfig();
    if (cfg == NULL)
    {
        return false;
    }

    app->cfg = cfg;
    app->mode = MODE0_FULL;
    Protection_Init(&app->protection);
    PowerState_Init(&app->state);
    if (!InverterControl_Init(&app->inverter, &cfg->inverter)
        || !RectifierControl_Init(&app->rectifier, &cfg->rectifier))
    {
        return false;
    }

    /* 整流级 DC 目标与键盘 Vdc 设定同源。 */
    RectifierControl_SetDcReference(&app->rectifier, cfg->dc_voltage_ref);
    app->initialized = true;
    return true;
}

void AppControl_Reset(AppControl *app)
{
    if ((app != NULL) && app->initialized)
    {
        InverterControl_Reset(&app->inverter);
        RectifierControl_Reset(&app->rectifier);
        Protection_Init(&app->protection);
        PowerState_Init(&app->state);
        app->mode0_dc_ready_count = 0U;
        app->mode0_inverter_ready = false;
        app->rectifier_dc_ready = false;
    }
}

bool AppControl_SetMode(AppControl *app, app_mode_t mode)
{
    if ((app == NULL) || !app->initialized
        || (app->state.state != OFF)
        || !AppControl_ModeValid(mode))
    {
        return false;
    }

    app->mode = mode;
    app->mode0_dc_ready_count = 0U;
    app->mode0_inverter_ready = false;
    app->rectifier_dc_ready = false;
    return true;
}

bool AppControl_Step(AppControl *app,
                     const AdcRawFrame *raw,
                     const AppCommand *command,
                     AppOutput *out)
{
    Measurements measurement = {0};
    PowerStateInput state_input = {0};
    InverterControlOutput inverter = {0};
    RectifierControlOutput rectifier = {0};
    app_mode_t mode;
    bool board_valid;
    bool measurement_valid;
    bool dc_reference_valid;
    bool precharge_done;
    bool phase_ready;
    bool run;
    bool rectifier_enable;
    bool inverter_enable;
    bool rectifier_ok;
    bool inverter_ok;
    bool dc_in_tolerance;
    float effective_reference_rms;
    float vdc_measured;

    if ((app == NULL) || (raw == NULL) || (command == NULL)
        || (out == NULL) || !app->initialized)
    {
        return false;
    }

    *out = (AppOutput){0};
    AppControl_SetSafeCommands(out);

    /* 单帧内使用同一模式快照，避免跨模式错帧解包/换算。 */
    mode = app->mode;
    board_valid = BoardPower_ValidateModeConfig(app->cfg, mode);
    switch (mode)
    {
    case MODE1_RECTIFIER:
        measurement_valid =
            Measurement_ConvertRectifier(&app->cfg->measurement,
                                         raw, &measurement);
        break;
    case MODE0_FULL:
        measurement_valid =
            Measurement_Convert(&app->cfg->measurement,
                                raw, &measurement);
        break;
    case MODE2_INVERTER:
    default:
        measurement_valid =
            Measurement_ConvertInverter(&app->cfg->measurement,
                                        raw, &measurement);
        break;
    }
    measurement.valid = measurement_valid;
    dc_reference_valid =
        AppControl_DcReferenceValid(command->dc_voltage_ref);

    if (board_valid)
    {
        /*
         * 所有模式都使用实测 Vdc：RUN 状态下启用母线欠压检查，
         * 过压检查始终有效。MODE1 的三相量由整流换算固定为 0，
         * 不会因未连接的三相输出通道误触发。
         */
        (void)Protection_Evaluate(&app->protection,
                                  &app->cfg->protection,
                                  &measurement,
                                  app->state.state == RUN,
                                  command->external_fault,
                                  command->loop_overrun);
        if (command->enable && !dc_reference_valid)
        {
            Protection_ForceFault(&app->protection,
                                  PROTECTION_FAULT_CONTROL);
        }
    }
    else
    {
        app->protection.active_faults = PROTECTION_FAULT_NONE;
        if (command->external_fault)
        {
            Protection_ForceFault(&app->protection,
                                  PROTECTION_FAULT_EXTERNAL);
        }
        if (command->loop_overrun)
        {
            Protection_ForceFault(&app->protection,
                                  PROTECTION_FAULT_LOOP_OVERRUN);
        }
    }

    if (command->reset_fault
        && (app->protection.active_faults == PROTECTION_FAULT_NONE))
    {
        (void)Protection_ClearLatched(&app->protection);
    }

    if (mode == MODE2_INVERTER)
    {
        /* 逆变模式使用外接直流源，不需要预充/相位联锁。 */
        precharge_done = true;
        phase_ready = true;
    }
    else
    {
        /*
         * MODE1/MODE0：板级接口（App_SetPrechargeDone/App_SetPhaseReady）
         * 优先；没有外部信号时按测量安全推导，绝不默认虚假置真——
         * precharge_done 要求实测 Vdc 达到运行欠压阈值（被动/外部预充
         * 已将母线充起），phase_ready 要求整流慢环测得的输入 RMS 超过
         * 最小输入电压（上一拍预热值，RMS 滤波时间常数远大于单拍）。
         */
        precharge_done = command->precharge_done
            || (measurement_valid
                && (measurement.vdc
                    >= app->cfg->protection.dc_voltage_min_run_v));
        phase_ready = command->phase_ready
            || (measurement_valid
                && (app->rectifier.vin_rms_v
                    >= app->cfg->rectifier.input_voltage_rms_min_v));
    }

    state_input.start_request = command->enable;
    state_input.stop_request = !command->enable;
    state_input.reset_fault_request = command->reset_fault;
    state_input.precharge_done = precharge_done;
    state_input.phase_ready = phase_ready;
    state_input.measurement_valid = measurement_valid;
    /*
     * 放电完成按真实测量判断：仅测量有效且实测母线低于安全阈值
     * （整流配置 dc_voltage_min_v，默认 10 V）才为 true。母线仍带电
     * 时状态机停在 DISCHARGE，不能回 OFF 切模式；MODE2 外接直流源
     * 同样适用，防止带电切模式。
     */
    state_input.discharge_done = measurement_valid
        && (measurement.vdc <= app->cfg->rectifier.dc_voltage_min_v);
    state_input.board_config_valid = board_valid;
    state_input.active_faults = app->protection.active_faults;
    state_input.latched_faults = app->protection.latched_faults;
    PowerState_Step(&app->state, &app->cfg->state, &state_input);
    run = app->state.pwm_allowed;

    /*
     * 键盘 Vdc 设定只是期望母线/输出参考来源；SVPWM 归一化和输出
     * 限幅一律使用实测 Vdc。实测无效时限幅为 0，逆变控制会因母线
     * 无效/欠压失败并按控制故障安全关断。
     */
    vdc_measured = measurement_valid ? measurement.vdc : 0.0f;
    effective_reference_rms =
        AppControl_LimitPhaseReference(&app->inverter.cfg,
                                       AppControl_PhaseReferenceFromDc(
                                           command->dc_voltage_ref),
                                       vdc_measured);
    InverterControl_SetReference(&app->inverter,
                                 effective_reference_rms);

    /*
     * 整流级在所有状态下都以 enabled=false 预热输入 RMS 和母线陷波；
     * 只有 RUN 且模式为 MODE1/MODE0 时才闭环使能。
     */
    rectifier_enable = run && (mode != MODE2_INVERTER);
    rectifier_ok = RectifierControl_Step(&app->rectifier,
                                         &measurement,
                                         rectifier_enable,
                                         &rectifier);

    /*
     * MODE0 联动：滤波 Vdc 连续落入目标容差后才开放逆变级；
     * MODE1 仅上报整流 ready 状态供显示。
     */
    dc_in_tolerance = rectifier_ok
        && (fabsf(rectifier.dc_voltage_filtered
                  - app->rectifier.dc_reference_target_v)
            <= app->cfg->rectifier.dc_ready_tolerance_v);
    if ((mode == MODE0_FULL) && rectifier_enable && dc_in_tolerance)
    {
        if (app->mode0_dc_ready_count < UINT16_MAX)
        {
            app->mode0_dc_ready_count++;
        }
    }
    else
    {
        app->mode0_dc_ready_count = 0U;
    }
    app->mode0_inverter_ready = (mode == MODE0_FULL)
        && (app->mode0_dc_ready_count
            >= app->cfg->rectifier.dc_ready_confirm_samples);
    app->rectifier_dc_ready = (mode == MODE0_FULL)
        ? app->mode0_inverter_ready
        : ((mode == MODE1_RECTIFIER) && dc_in_tolerance);

    /* MODE0 的逆变级同样使用实测 Vdc 做 SVPWM 归一化。 */
    inverter_enable = run
        && ((mode == MODE2_INVERTER)
         || ((mode == MODE0_FULL) && app->mode0_inverter_ready));
    inverter_ok = InverterControl_Step(&app->inverter,
                                       &measurement,
                                       inverter_enable,
                                       &inverter);

    /* 任一级控制失败都锁存 CONTROL 故障并关闭两级。 */
    if ((rectifier_enable && !rectifier_ok)
        || (inverter_enable && !inverter_ok))
    {
        Protection_ForceFault(&app->protection,
                              PROTECTION_FAULT_CONTROL);
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
    out->pwm_allowed = app->state.pwm_allowed
                    && (!rectifier_enable || rectifier_ok)
                    && (!inverter_enable || inverter_ok);

    if (inverter_enable && inverter_ok && inverter.pwm.valid)
    {
        out->inverter.a = inverter.pwm.duty_a;
        out->inverter.b = inverter.pwm.duty_b;
        out->inverter.c = inverter.pwm.duty_c;
        out->inverter.enabled = true;
    }
    if (rectifier_enable && rectifier_ok && rectifier.pwm.valid)
    {
        out->rectifier.leg_a = rectifier.pwm.duty_a;
        out->rectifier.leg_b = rectifier.pwm.duty_b;
        out->rectifier.enabled = true;
    }

    if (!out->pwm_allowed)
    {
        AppControl_SetSafeCommands(out);
    }

    return true;
}

static void AppRuntime_UpdatePublic(void)
{
    /*
     * 快环结束后集中发布命令和状态，使板级层读取同一控制周期的结果。
     * 若本拍与发布之间又有 ISR 故障到达（TripFromISR 已直接撤销使能），
     * 强制安全 PWM 并撤销两个 enable_request 和 pwm_allowed，消除
     * TripFromISR 与 FastStep 的一拍发布竞态，绝不用旧计算结果重新
     * 打开功率级。
     */
    if (g_app.isr_faults != 0UL)
    {
        AppControl_SetSafeCommands(&g_app.output);
    }

    g_app.pwm.inverter_a = g_app.output.inverter.a;
    g_app.pwm.inverter_b = g_app.output.inverter.b;
    g_app.pwm.inverter_c = g_app.output.inverter.c;
    g_app.pwm.rectifier_a = g_app.output.rectifier.leg_a;
    g_app.pwm.rectifier_b = g_app.output.rectifier.leg_b;
    g_app.pwm.inverter_enable_request =
        g_app.output.pwm_allowed && g_app.output.inverter.enabled;
    g_app.pwm.rectifier_enable_request =
        g_app.output.pwm_allowed && g_app.output.rectifier.enabled;

    g_app.status.mode = g_app.control.mode;
    g_app.status.state = g_app.output.state;
    g_app.status.output_frequency_hz =
        g_app.control.inverter.cfg.output_frequency_hz;
    g_app.status.dc_voltage_v = g_app.output.measurement.vdc;
    g_app.status.output_line_voltage_reference_v =
        g_app.control.inverter.reference_target_rms * APP_SQRT3_F;
    g_app.status.dc_voltage_reference_v =
        g_app.command.dc_voltage_ref;
    g_app.status.ia_a = g_app.output.measurement.ia;
    g_app.status.ib_a = g_app.output.measurement.ib;
    g_app.status.vab_v = g_app.output.measurement.vab;
    g_app.status.vbc_v = g_app.output.measurement.vbc;
    g_app.status.faults = g_app.output.faults;
    g_app.status.sample_sequence = g_app.output.measurement.sequence;
    g_app.status.board_config_valid =
        g_app.output.board_config_valid;
    g_app.status.pwm_allowed = g_app.output.pwm_allowed;
    g_app.status.rectifier_dc_ready = g_app.control.rectifier_dc_ready;
}

void App_Init(void)
{
    g_app = (AppRuntime){0};
    g_app.initialized = AppControl_Init(&g_app.control);
    if (g_app.initialized)
    {
        g_app.command.dc_voltage_ref =
            g_app.control.cfg->dc_voltage_ref;
        g_app.command.inverter_vrms_ref =
            AppControl_PhaseReferenceFromDc(
                g_app.command.dc_voltage_ref);
        InverterControl_SetReference(
            &g_app.control.inverter,
            AppControl_LimitPhaseReference(
                &g_app.control.inverter.cfg,
                g_app.command.inverter_vrms_ref,
                g_app.command.dc_voltage_ref));
        /*
         * 与运行时 AppControl_Step 同一口径：按当前（默认）模式校验，
         * 避免 OLED 初始 board_config_valid 与第一帧运行结果不一致。
         */
        g_app.output.board_config_valid = BoardPower_ValidateModeConfig(
            g_app.control.cfg, g_app.control.mode);
    }
    g_app.output.state = OFF;
    AppControl_SetSafeCommands(&g_app.output);
    AppRuntime_UpdatePublic();
}

bool App_RequestMode(app_mode_t mode)
{
    uint32_t primask;
    bool changed;

    if (!g_app.initialized || (g_app.status.state != OFF))
    {
        return false;
    }
    /*
     * 短临界区保证模式切换不会落在 20 kHz ISR 的解包与换算之间，
     * ADC 帧不会跨模式错帧；模式只在 OFF 切换，此时两级 PWM 已关闭。
     */
    primask = AppRuntime_EnterCritical();
    changed = AppControl_SetMode(&g_app.control, mode);
    if (changed)
    {
        g_app.status.mode = mode;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

void App_RequestStart(void)
{
    if (g_app.initialized)
    {
        g_app.command.enable = true;
        g_app.command.reset_fault = false;
    }
}

void App_RequestStop(void)
{
    g_app.command.enable = false;
}

bool App_ApplyAdcPairs(const uint32_t adc_cdr_pairs[4],
                       uint32_t sequence)
{
    bool unpacked;

    if ((adc_cdr_pairs == NULL) || !g_app.initialized)
    {
        return false;
    }

    /*
     * 按当前模式选择解包路径：逆变模式跳过 Vin 冗余诊断（该通道属于
     * 整流前端），整流/全模式执行含 Vin/Vin_diag 一致性的完整诊断。
     * 本函数与 App_FastStep20kHz 在同一 ISR 内顺序执行，且模式切换
     * 使用短临界区，因此解包与换算不会跨模式错帧。
     */
    if (g_app.control.mode == MODE2_INVERTER)
    {
        unpacked = Measurement_UnpackInverterAdcPairs(adc_cdr_pairs,
                                                      sequence,
                                                      &g_app.raw);
    }
    else
    {
        unpacked = Measurement_UnpackDualAdcPairs(
            adc_cdr_pairs,
            sequence,
            g_app.control.cfg->vin_diagnostic_tolerance_counts,
            &g_app.raw);
    }

    if (!unpacked)
    {
        App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
        return false;
    }

    g_app.frame_pending = true;
    return true;
}

bool App_ApplyAdcSamples(const uint16_t adc_samples[8],
                         uint32_t sequence)
{
    uint32_t pairs[4];
    uint8_t i;

    if ((adc_samples == NULL) || !g_app.initialized)
    {
        return false;
    }

    for (i = 0U; i < 4U; i++)
    {
        pairs[i] = (uint32_t)adc_samples[2U * i]
                 | ((uint32_t)adc_samples[2U * i + 1U] << 16);
    }
    return App_ApplyAdcPairs(pairs, sequence);
}

void App_FastStep20kHz(void)
{
    uint32_t isr_faults;

    if (!g_app.initialized)
    {
        return;
    }

    /* 先处理异步硬件故障，再消费 ADC 帧，保证故障优先于本拍控制计算。 */
    isr_faults = g_app.isr_faults;
    if (isr_faults != 0UL)
    {
        Protection_ForceFault(&g_app.control.protection, isr_faults);
        g_app.command.external_fault =
            (isr_faults & PROTECTION_FAULT_EXTERNAL) != 0UL;
        g_app.command.loop_overrun =
            (isr_faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0UL;
        g_app.command.enable = false;
    }
    if (!g_app.frame_pending)
    {
        g_app.raw.valid = false;
    }

    (void)AppControl_Step(&g_app.control,
                          &g_app.raw,
                          &g_app.command,
                          &g_app.output);
    g_app.frame_pending = false;
    g_app.command.reset_fault = false;
    AppRuntime_UpdatePublic();
}

void App_BackgroundTask(void)
{
    /* 预留给低速命令解析/遥测；禁止在此直接打开PWM。 */
}

void App_TripFromISR(fault_flags_t faults)
{
    uint32_t primask;

    if (faults == 0UL)
    {
        faults = PROTECTION_FAULT_EXTERNAL;
    }

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

const AppPwmCommand *App_GetPwmCommand(void)
{
    return &g_app.pwm;
}

const AppStatus *App_GetStatus(void)
{
    return &g_app.status;
}

bool App_SetOutputFrequency(uint16_t frequency_hz)
{
    uint32_t primask;
    bool changed = false;

    /* 整流模式没有三相输出，频率设置仅在 MODE0/MODE2 有意义。 */
    if (!g_app.initialized
        || ((frequency_hz != 30U) && (frequency_hz != 60U)))
    {
        return false;
    }

    /*
     * 与 App_RequestMode 相同的短临界区：状态/模式检查和控制器频率
     * 重建必须与 20 kHz ISR 的快环互斥，避免后台与 ISR 并发改写
     * QPR 系数和相位递推步长。
     */
    primask = AppRuntime_EnterCritical();
    if ((g_app.status.state == OFF)
        && (g_app.control.mode != MODE1_RECTIFIER)
        && InverterControl_SetFrequency(&g_app.control.inverter,
                                        (float)frequency_hz))
    {
        g_app.status.output_frequency_hz = (float)frequency_hz;
        changed = true;
    }
    AppRuntime_ExitCritical(primask);
    return changed;
}

bool App_SetOutputLineVoltageReference(float line_rms)
{
    float derived_line_rms;

    if (!g_app.initialized || !isfinite(line_rms))
    {
        return false;
    }

    /*
     * 比例跟随模式不允许独立改写输出电压；保留API仅用于兼容，
     * 传入值与当前Vdc推导值一致时才报告成功。
     */
    derived_line_rms =
        AppControl_LineReferenceFromDc(g_app.command.dc_voltage_ref);
    return fabsf(line_rms - derived_line_rms) < 1.0e-4f;
}

bool App_SetDcVoltageReference(float volts)
{
    uint32_t primask;
    float phase_rms;
    bool state_allows_update;
    bool updated = false;

    if (!g_app.initialized || !AppControl_DcReferenceValid(volts))
    {
        return false;
    }

    phase_rms = AppControl_PhaseReferenceFromDc(volts);
    if (phase_rms > g_app.control.inverter.cfg.voltage_rms_max)
    {
        return false;
    }

    /*
     * 与 App_RequestMode 相同的短临界区：OFF/FAULT_LATCH 门限判断与
     * 命令/参考字段的改写必须原子完成，避免 20 kHz ISR 在改写中途
     * 读到半成品参考；OFF 无启动请求或 PWM 已禁止的 FAULT_LATCH
     * 限制保持不变。
     */
    primask = AppRuntime_EnterCritical();
    state_allows_update =
        ((g_app.status.state == OFF) && !g_app.command.enable)
        || ((g_app.status.state == FAULT_LATCH)
            && !g_app.status.pwm_allowed);
    if (state_allows_update)
    {
        g_app.command.dc_voltage_ref = volts;
        g_app.command.inverter_vrms_ref = phase_rms;
        g_app.status.dc_voltage_reference_v = volts;
        g_app.status.output_line_voltage_reference_v =
            AppControl_LineReferenceFromDc(volts);
        /* MODE1/MODE0 中该设定同时是整流级直流母线目标。 */
        if (g_app.control.mode != MODE2_INVERTER)
        {
            RectifierControl_SetDcReference(&g_app.control.rectifier,
                                            volts);
        }
        updated = true;
    }
    AppRuntime_ExitCritical(primask);
    return updated;
}

void App_ResetFault(void)
{
    uint32_t primask = AppRuntime_EnterCritical();

    g_app.command.enable = false;
    g_app.command.external_fault = false;
    g_app.command.loop_overrun = false;
    g_app.command.reset_fault = true;
    g_app.isr_faults = 0UL;
    AppRuntime_ExitCritical(primask);
}

void App_SetPrechargeDone(bool done)
{
    g_app.command.precharge_done = done;
}

void App_SetPhaseReady(bool ready)
{
    g_app.command.phase_ready = ready;
}
