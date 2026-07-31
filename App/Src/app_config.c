#include "app_config.h"

#include <math.h>
#include <stddef.h>

#if defined(__GNUC__)
#define APP_WEAK __attribute__((weak))
#else
#define APP_WEAK
#endif

/*
 * 用户已确认 ADC 极性/标定、功率级、驱动关断、保护和 1 us 死区实测完成。
 * 配置校验仍会逐项检查标定、控制参数和保护阈值，不能只靠单一标志绕过。
 */
const BoardPowerConfig g_board_power_config =
{
    .config_valid = true,
    .adc_calibrated = true,
    .gate_driver_pins_valid = true,
    .power_stage_valid = true,
    .deadtime_valid = true,
    .protection_thresholds_valid = true,
    .output_line_voltage_rms =
        APP_INVERTER_DC_VOLTAGE_DEFAULT_V
        * APP_INVERTER_OUTPUT_LINE_RMS_RATIO,
    .dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V,
    .vin_diagnostic_tolerance_counts = 9U,
    .measurement =
    {
        /*
         * 同型传感器通道临时复用：Iin 暂用与 Ia/Ib 相同的已标定 TA 参数，
         * 换前端必须重标定（PA5 输入电流互感器的实际变比/ burden 未实测）。
         */
        .iin =
        {
            .offset_count = 2084.7273f,
            .gain = 0.002919044f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        /*
         * 同型传感器通道临时复用：Vin 暂用与 Vab/Vbc 相同的已标定 TV 参数，
         * 换前端必须重标定（PC5 输入电压互感器分压比未实测）。
         */
        .vin =
        {
            .offset_count = 2109.5455f,
            .gain = 0.051657744f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        /* TA 两个实测点按零点约束最小二乘折算为瞬时电流/ADC count。 */
        .ia =
        {
            .offset_count = 2084.7273f,
            .gain = 0.002919044f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        .ib =
        {
            .offset_count = 2084.7273f,
            .gain = 0.002919044f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        /* TV：10 Vpp/156 mVpp 与 20 Vpp/312 mVpp 给出相同比例。 */
        .vab =
        {
            .offset_count = 2109.5455f,
            .gain = 0.051657744f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        .vbc =
        {
            .offset_count = 2109.5455f,
            .gain = 0.051657744f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        /* 直流采样三点拟合：Vout=0.0487*Vdc+0.016 V。 */
        .vdc =
        {
            .offset_count = 19.8545f,
            .gain = 0.016547450f,
            .raw_min = 0U,
            .raw_max = 4031U
        }
    },
    .inverter =
    {
        .sample_time_s = 50.0e-6f,
        .output_frequency_hz = 60.0f,
        .voltage_rms_max =
            APP_INVERTER_DC_VOLTAGE_MAX_V
            * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        .reference_slew_v_per_s =
            APP_INVERTER_DC_VOLTAGE_DEFAULT_V
            * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        .feedforward_gain = 0.0f,
        .voltage_qpr_kp = 0.05f,
        .voltage_qpr_kr = 30.0f,
        .voltage_qpr_bandwidth_rad_s = 1.0f,
        .current_reference_limit_a = 3.0f,
        .current_qpr_kp = 2.5f,
        .current_qpr_kr = 30.0f,
        .current_qpr_bandwidth_rad_s = 5.0f,
        .command_limit_v = 200.0f,
        .dc_voltage_min_v = 10.0f,
        .duty_min = 0.02f,
        .duty_max = 0.98f
    },
    /*
     * 整流级保守可运行参数，按 32 Vrms/50 Hz 输入、60 V 母线目标填写。
     * PI/QPR 增益为保守小值，实机仍须按实测前端和负载重新整定、校核。
     */
    .rectifier =
    {
        .sample_time_s = 50.0e-6f,
        .line_frequency_hz = 50.0f,
        .outer_loop_divider = 20U,
        .rms_filter_time_constant_s = 0.05f,
        .input_voltage_rms_min_v = 20.0f,
        .dc_voltage_max_v = 65.0f,
        .dc_reference_slew_v_per_s = 20.0f,
        .dc_ready_tolerance_v = 1.5f,
        .dc_ready_confirm_samples = 1000U,
        .current_rms_max_a = 6.0f,
        .dc_pi_kp = 0.05f,
        .dc_pi_ki = 2.0f,
        .dc_pi_kaw = 5.0f,
        .current_qpr_kp = 2.0f,
        .current_qpr_kr = 20.0f,
        .current_qpr_bandwidth_rad_s = 5.0f,
        .inductor_voltage_limit_v = 60.0f,
        .dc_voltage_min_v = 10.0f,
        .current_reference_sign = 1.0f,
        .bridge_polarity = 1.0f,
        .notch_zero_damping = 0.02f,
        .notch_pole_damping = 0.5f,
        .duty_min = 0.02f,
        .duty_max = 0.98f
    },
    /* 保护阈值与 32 Vrms 输入 / 60 V 母线题目匹配；实机仍须校核。 */
    .protection =
    {
        /* 输入限流 6 Arms，峰值约 8.5 A，绝对保护取 10 A。 */
        .input_current_abs_max_a = 10.0f,
        .phase_current_abs_max_a = 3.5f,
        /* 32 Vrms 输入峰值约 45.3 V，绝对过压保护取 60 V。 */
        .input_voltage_abs_max_v = 60.0f,
        .dc_voltage_min_run_v =
            APP_INVERTER_DC_VOLTAGE_MIN_V
            - APP_INVERTER_DC_MATCH_TOLERANCE_V,
        .dc_voltage_max_v = APP_INVERTER_DC_OVERVOLTAGE_V,
        .output_voltage_abs_max_v = 32.0f,
        .stale_limit_samples = 2U
    },
    .state =
    {
        .self_test_confirm_samples = 4U,
        .ready_confirm_samples = 20U,
        .discharge_confirm_samples = 20U
    }
};

APP_WEAK const BoardPowerConfig *BoardPower_GetConfig(void)
{
    return &g_board_power_config;
}

/* 三种模式共享的安全标志、参考一致性、保护阈值和状态机参数检查。 */
static bool BoardPower_ValidateShared(const control_config_t *cfg)
{
    return (cfg != NULL)
        && cfg->config_valid
        && cfg->adc_calibrated
        && cfg->gate_driver_pins_valid
        && cfg->power_stage_valid
        && cfg->deadtime_valid
        && cfg->protection_thresholds_valid
        && isfinite(cfg->output_line_voltage_rms)
        && isfinite(cfg->dc_voltage_ref)
        && (cfg->output_line_voltage_rms >= 0.0f)
        && (cfg->output_line_voltage_rms
            <= cfg->inverter.voltage_rms_max * APP_SQRT3_F)
        && (fabsf(cfg->output_line_voltage_rms
                 - cfg->dc_voltage_ref
                   * APP_INVERTER_OUTPUT_LINE_RMS_RATIO) < 1.0e-4f)
        && (cfg->dc_voltage_ref >= APP_INVERTER_DC_VOLTAGE_MIN_V)
        && (cfg->dc_voltage_ref <= APP_INVERTER_DC_VOLTAGE_MAX_V)
        && isfinite(cfg->protection.dc_voltage_max_v)
        && isfinite(cfg->protection.input_current_abs_max_a)
        && isfinite(cfg->protection.phase_current_abs_max_a)
        && isfinite(cfg->protection.input_voltage_abs_max_v)
        && isfinite(cfg->protection.dc_voltage_min_run_v)
        && isfinite(cfg->protection.output_voltage_abs_max_v)
        && (cfg->protection.input_current_abs_max_a > 0.0f)
        && (cfg->protection.phase_current_abs_max_a > 0.0f)
        && (cfg->protection.input_voltage_abs_max_v > 0.0f)
        && (cfg->protection.output_voltage_abs_max_v > 0.0f)
        && (cfg->protection.dc_voltage_max_v
            > cfg->protection.dc_voltage_min_run_v)
        && (cfg->protection.stale_limit_samples > 0U)
        && (cfg->state.self_test_confirm_samples > 0U)
        && (cfg->state.ready_confirm_samples > 0U)
        && (cfg->state.discharge_confirm_samples > 0U);
}

/* MODE2/MODE0 需要的逆变五通道标定与控制器参数检查。 */
static bool BoardPower_ValidateInverter(const control_config_t *cfg)
{
    /*
     * 即使误把config_valid改成true，逆变所需通道的零增益仍会继续硬锁RUN。
     * 其余算法参数由各模块Init再次校验。
     */
    return Measurement_InverterCalibrationValid(&cfg->measurement)
        && (APP_INVERTER_DC_VOLTAGE_MIN_V
            > cfg->inverter.dc_voltage_min_v)
        && isfinite(cfg->inverter.sample_time_s)
        && isfinite(cfg->inverter.output_frequency_hz)
        && isfinite(cfg->inverter.voltage_rms_max)
        && isfinite(cfg->inverter.reference_slew_v_per_s)
        && isfinite(cfg->inverter.feedforward_gain)
        && (cfg->inverter.sample_time_s > 0.0f)
        && ((cfg->inverter.output_frequency_hz == 30.0f)
         || (cfg->inverter.output_frequency_hz == 60.0f))
        && (cfg->inverter.voltage_rms_max > 0.0f)
        && (cfg->inverter.reference_slew_v_per_s > 0.0f)
        && (cfg->inverter.voltage_qpr_kp >= 0.0f)
        && (cfg->inverter.voltage_qpr_kr >= 0.0f)
        && (cfg->inverter.voltage_qpr_bandwidth_rad_s > 0.0f)
        && ((cfg->inverter.voltage_qpr_kp
           + cfg->inverter.voltage_qpr_kr) > 0.0f)
        && (cfg->inverter.current_qpr_kp >= 0.0f)
        && (cfg->inverter.current_qpr_kr >= 0.0f)
        && (cfg->inverter.current_qpr_bandwidth_rad_s > 0.0f)
        && ((cfg->inverter.current_qpr_kp
           + cfg->inverter.current_qpr_kr) > 0.0f)
        && (cfg->inverter.current_reference_limit_a > 0.0f)
        && (cfg->inverter.command_limit_v > 0.0f)
        && (cfg->inverter.dc_voltage_min_v > 0.0f)
        && (cfg->inverter.duty_min >= 0.0f)
        && (cfg->inverter.duty_max <= 1.0f)
        && (cfg->inverter.duty_min < cfg->inverter.duty_max);
}

/*
 * MODE1/MODE0 需要的整流三通道标定与控制器参数检查。
 * 与 RectifierControl_Init 的参数约束保持一致，并额外要求 PI/QPR
 * 增益非零：零增益控制器会让母线环静默失效，必须视为无效配置。
 */
static bool BoardPower_ValidateRectifier(const control_config_t *cfg)
{
    return Measurement_RectifierCalibrationValid(&cfg->measurement)
        && isfinite(cfg->rectifier.sample_time_s)
        && isfinite(cfg->rectifier.line_frequency_hz)
        && isfinite(cfg->rectifier.rms_filter_time_constant_s)
        && isfinite(cfg->rectifier.input_voltage_rms_min_v)
        && isfinite(cfg->rectifier.dc_voltage_max_v)
        && isfinite(cfg->rectifier.dc_reference_slew_v_per_s)
        && isfinite(cfg->rectifier.dc_ready_tolerance_v)
        && isfinite(cfg->rectifier.current_rms_max_a)
        && isfinite(cfg->rectifier.inductor_voltage_limit_v)
        && isfinite(cfg->rectifier.dc_voltage_min_v)
        && isfinite(cfg->rectifier.current_reference_sign)
        && isfinite(cfg->rectifier.bridge_polarity)
        && isfinite(cfg->rectifier.notch_zero_damping)
        && isfinite(cfg->rectifier.notch_pole_damping)
        && (cfg->rectifier.sample_time_s > 0.0f)
        && (cfg->rectifier.line_frequency_hz > 0.0f)
        && (cfg->rectifier.outer_loop_divider > 0U)
        && (cfg->rectifier.rms_filter_time_constant_s > 0.0f)
        && (cfg->rectifier.input_voltage_rms_min_v > 0.0f)
        && (cfg->rectifier.dc_voltage_max_v
            > cfg->rectifier.dc_voltage_min_v)
        && (cfg->rectifier.dc_reference_slew_v_per_s > 0.0f)
        && (cfg->rectifier.dc_ready_tolerance_v > 0.0f)
        && (cfg->rectifier.dc_ready_confirm_samples > 0U)
        && (cfg->rectifier.current_rms_max_a > 0.0f)
        && (cfg->rectifier.inductor_voltage_limit_v > 0.0f)
        && (fabsf(cfg->rectifier.current_reference_sign) >= 0.5f)
        && (fabsf(cfg->rectifier.bridge_polarity) >= 0.5f)
        && (cfg->rectifier.notch_zero_damping >= 0.0f)
        && (cfg->rectifier.notch_pole_damping > 0.0f)
        && (cfg->rectifier.notch_zero_damping
            <= cfg->rectifier.notch_pole_damping)
        && isfinite(cfg->rectifier.dc_pi_kp)
        && isfinite(cfg->rectifier.dc_pi_ki)
        && isfinite(cfg->rectifier.dc_pi_kaw)
        && (cfg->rectifier.dc_pi_kp > 0.0f)
        && (cfg->rectifier.dc_pi_ki > 0.0f)
        && (cfg->rectifier.dc_pi_kaw > 0.0f)
        && isfinite(cfg->rectifier.current_qpr_kp)
        && isfinite(cfg->rectifier.current_qpr_kr)
        && (cfg->rectifier.current_qpr_kp > 0.0f)
        && (cfg->rectifier.current_qpr_kr >= 0.0f)
        && (cfg->rectifier.current_qpr_bandwidth_rad_s > 0.0f)
        && (cfg->rectifier.duty_min >= 0.0f)
        && (cfg->rectifier.duty_max <= 1.0f)
        && (cfg->rectifier.duty_min < cfg->rectifier.duty_max);
}

bool BoardPower_ValidateModeConfig(const control_config_t *cfg,
                                   app_mode_t mode)
{
    if (!BoardPower_ValidateShared(cfg))
    {
        return false;
    }

    switch (mode)
    {
    case MODE2_INVERTER:
        return BoardPower_ValidateInverter(cfg);
    case MODE1_RECTIFIER:
        return BoardPower_ValidateRectifier(cfg);
    case MODE0_FULL:
        return Measurement_CalibrationValid(&cfg->measurement)
            && BoardPower_ValidateInverter(cfg)
            && BoardPower_ValidateRectifier(cfg);
    default:
        return false;
    }
}

bool BoardPower_ValidateConfig(const control_config_t *cfg)
{
    /* 保持兼容：按最严格的 MODE0_FULL 校验。 */
    return BoardPower_ValidateModeConfig(cfg, MODE0_FULL);
}

bool BoardPower_ConfigValid(void)
{
    return BoardPower_ValidateConfig(BoardPower_GetConfig());
}
