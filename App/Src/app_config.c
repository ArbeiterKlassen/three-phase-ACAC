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
 * 用户确认 Iin 与 Ia、Vin 与 Vab 分别使用同型采样模块并复用标定；
 * 功率级、驱动关断、保护和 1 us 死区也已确认。模式校验仍会检查
 * 复用后的可测量程是否覆盖控制上限及软件过流阈值。
 */
const BoardPowerConfig g_board_power_config =
{
    .config_valid = true,
    .adc_calibrated = true,
    .rectifier_adc_calibrated = true,
    .gate_driver_pins_valid = true,
    .power_stage_valid = true,
    .deadtime_valid = true,
    .protection_thresholds_valid = true,
    .output_line_voltage_rms =
        APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V,
    .dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V,
    .vin_diagnostic_tolerance_counts = 9U,
    .measurement =
    {
        /*
         * 整流输入电流采样与已标定的 Ia 使用同型 TA 通道，复用其实测
         * 偏置和增益；后续若单通道复标，只需替换本项。
         */
        .iin =
        {
            .offset_count = 2084.7273f,
            .gain = 0.002919044f,
            .raw_min = 64U,
            .raw_max = 4031U
        },
        /*
         * 整流输入电压采样与已标定的 Vab 使用同型 TV 通道，复用其实测
         * 偏置和增益；后续若单通道复标，只需替换本项。
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
        /* 直流采样四点拟合：Vout=0.04779169*Vdc+0.00365764 V。 */
        .vdc =
        {
            .offset_count = 4.5388f,
            .gain = 0.016861944f,
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
    .rectifier =
    {
        .sample_time_s = 50.0e-6f,
        .line_frequency_hz = 50.0f,
        .outer_loop_divider = 20U,
        .rms_filter_time_constant_s = 0.05f,
        .input_voltage_rms_min_v = 10.0f,
        .dc_voltage_max_v = APP_INVERTER_DC_OVERVOLTAGE_V,
        .dc_reference_slew_v_per_s = 60.0f,
        .dc_ready_tolerance_v = 5.0f,
        /* 与参考工程一致：20 kHz 下连续 1000 拍（50 ms）确认母线就绪。 */
        .dc_ready_confirm_samples = 1000U,
        /* SLX 外环 PI 增益保持 Kp=1、Ki=1。 */
        /*
         * 赛题满载 Po=√3×32V×2.0A≈110.9W，η≥95% 时 Pin≈116.7W：
         * 36V 输入需 3.24A RMS，要求7的 31V 输入需 3.77A RMS。
         * 限幅取 3.8A RMS（5.37A 峰值）。TA 实测量程约 -5.90..+5.68A，
         * 峰值余量仅剩 ~6%，效率不达标时 31V 满载会贴量程顶。
         */
        .current_rms_max_a = 3.8f,
        .dc_pi_kp = 1.0f,
        .dc_pi_ki = 1.0f,
        .dc_pi_kaw = 0.0f,
        /* SLX 输入电流 PR/QPR 参数：Kp=5、Kr=60、Wc=5 rad/s。 */
        .current_qpr_kp = 5.0f,
        .current_qpr_kr = 60.0f,
        .current_qpr_bandwidth_rad_s = 5.0f,
        .inductor_voltage_limit_v = 1200.0f,
        .dc_voltage_min_v = 10.0f,
        /* 正电流幅值和 PI 的 0 A 下限共同禁止直流向交流回馈。 */
        .current_reference_sign = 1.0f,
        .bridge_polarity = 1.0f,
        .notch_zero_damping = 0.0f,
        .notch_pole_damping =
            50.0f / (4.0f * APP_PI_F * 100.0f),
        .duty_min = 0.02f,
        .duty_max = 0.98f
    },
    .protection =
    {
        /* 31V 满载输入电流峰值约 5.33A；阈值需高于它且不超过
         * TA 可测上限（+5.68A），取 5.6A——余量很薄，勿再调高。 */
        .input_current_abs_max_a = 5.6f,
        .phase_current_abs_max_a = 3.5f,
        .input_voltage_abs_max_v = 65.0f,
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

static bool BoardPower_CommonConfigValid(const control_config_t *cfg)
{
    if ((cfg == NULL)
        || !cfg->config_valid
        || !cfg->adc_calibrated
        || !cfg->gate_driver_pins_valid
        || !cfg->power_stage_valid
        || !cfg->deadtime_valid
        || !cfg->protection_thresholds_valid)
    {
        return false;
    }

    return isfinite(cfg->protection.dc_voltage_max_v)
        && isfinite(cfg->protection.input_current_abs_max_a)
        && isfinite(cfg->protection.phase_current_abs_max_a)
        && isfinite(cfg->protection.input_voltage_abs_max_v)
        && isfinite(cfg->protection.dc_voltage_min_run_v)
        && isfinite(cfg->protection.output_voltage_abs_max_v)
        && (cfg->protection.input_current_abs_max_a > 0.0f)
        && (cfg->protection.phase_current_abs_max_a > 0.0f)
        && (cfg->protection.input_voltage_abs_max_v > 0.0f)
        && (cfg->protection.dc_voltage_min_run_v >= 0.0f)
        && (cfg->protection.output_voltage_abs_max_v > 0.0f)
        && (cfg->protection.dc_voltage_max_v
            > cfg->protection.dc_voltage_min_run_v)
        && (cfg->protection.stale_limit_samples > 0U)
        && (cfg->state.self_test_confirm_samples > 0U)
        && (cfg->state.ready_confirm_samples > 0U)
        && (cfg->state.discharge_confirm_samples > 0U);
}

static bool BoardPower_InverterConfigValid(const control_config_t *cfg)
{
    return Measurement_InverterCalibrationValid(&cfg->measurement)
        && isfinite(cfg->output_line_voltage_rms)
        && isfinite(cfg->dc_voltage_ref)
        && (cfg->output_line_voltage_rms
            >= APP_INVERTER_OUTPUT_LINE_RMS_MIN_V)
        && (cfg->output_line_voltage_rms
            <= APP_INVERTER_OUTPUT_LINE_RMS_MAX_V)
        && (cfg->output_line_voltage_rms
            <= cfg->inverter.voltage_rms_max * APP_SQRT3_F)
        && (cfg->dc_voltage_ref >= APP_INVERTER_DC_VOLTAGE_MIN_V)
        && (cfg->dc_voltage_ref <= APP_INVERTER_DC_VOLTAGE_MAX_V)
        && (cfg->dc_voltage_ref < cfg->protection.dc_voltage_max_v)
        && (APP_INVERTER_DC_VOLTAGE_MIN_V
            > cfg->inverter.dc_voltage_min_v)
        && isfinite(cfg->inverter.sample_time_s)
        && isfinite(cfg->inverter.output_frequency_hz)
        && isfinite(cfg->inverter.voltage_rms_max)
        && isfinite(cfg->inverter.reference_slew_v_per_s)
        && isfinite(cfg->inverter.feedforward_gain)
        && isfinite(cfg->inverter.voltage_qpr_kp)
        && isfinite(cfg->inverter.voltage_qpr_kr)
        && isfinite(cfg->inverter.voltage_qpr_bandwidth_rad_s)
        && isfinite(cfg->inverter.current_reference_limit_a)
        && isfinite(cfg->inverter.current_qpr_kp)
        && isfinite(cfg->inverter.current_qpr_kr)
        && isfinite(cfg->inverter.current_qpr_bandwidth_rad_s)
        && isfinite(cfg->inverter.command_limit_v)
        && isfinite(cfg->inverter.dc_voltage_min_v)
        && isfinite(cfg->inverter.duty_min)
        && isfinite(cfg->inverter.duty_max)
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

static bool BoardPower_RectifierConfigValid(const control_config_t *cfg)
{
    float current_at_raw_min;
    float current_at_raw_max;
    float measurable_min;
    float measurable_max;
    float current_command_peak;
    float required_peak;

    current_at_raw_min =
        ((float)cfg->measurement.iin.raw_min
         - cfg->measurement.iin.offset_count)
        * cfg->measurement.iin.gain;
    current_at_raw_max =
        ((float)cfg->measurement.iin.raw_max
         - cfg->measurement.iin.offset_count)
        * cfg->measurement.iin.gain;
    measurable_min = fminf(current_at_raw_min, current_at_raw_max);
    measurable_max = fmaxf(current_at_raw_min, current_at_raw_max);
    current_command_peak =
        1.41421356237f * cfg->rectifier.current_rms_max_a;
    required_peak = fmaxf(
        current_command_peak,
        cfg->protection.input_current_abs_max_a);

    return cfg->rectifier_adc_calibrated
        && Measurement_RectifierCalibrationValid(&cfg->measurement)
        && isfinite(measurable_min)
        && isfinite(measurable_max)
        && isfinite(current_command_peak)
        && isfinite(required_peak)
        && (measurable_min <= -required_peak)
        && (measurable_max >= required_peak)
        && (cfg->vin_diagnostic_tolerance_counts > 0U)
        && isfinite(cfg->dc_voltage_ref)
        && (cfg->dc_voltage_ref >= cfg->rectifier.dc_voltage_min_v)
        && (cfg->dc_voltage_ref <= cfg->rectifier.dc_voltage_max_v)
        && (cfg->dc_voltage_ref < cfg->protection.dc_voltage_max_v)
        && isfinite(cfg->rectifier.sample_time_s)
        && isfinite(cfg->rectifier.line_frequency_hz)
        && isfinite(cfg->rectifier.rms_filter_time_constant_s)
        && isfinite(cfg->rectifier.input_voltage_rms_min_v)
        && isfinite(cfg->rectifier.dc_voltage_max_v)
        && isfinite(cfg->rectifier.dc_reference_slew_v_per_s)
        && isfinite(cfg->rectifier.dc_ready_tolerance_v)
        && isfinite(cfg->rectifier.current_rms_max_a)
        && isfinite(cfg->rectifier.dc_pi_kp)
        && isfinite(cfg->rectifier.dc_pi_ki)
        && isfinite(cfg->rectifier.dc_pi_kaw)
        && isfinite(cfg->rectifier.current_qpr_kp)
        && isfinite(cfg->rectifier.current_qpr_kr)
        && isfinite(cfg->rectifier.current_qpr_bandwidth_rad_s)
        && isfinite(cfg->rectifier.inductor_voltage_limit_v)
        && isfinite(cfg->rectifier.dc_voltage_min_v)
        && isfinite(cfg->rectifier.current_reference_sign)
        && isfinite(cfg->rectifier.bridge_polarity)
        && isfinite(cfg->rectifier.notch_zero_damping)
        && isfinite(cfg->rectifier.notch_pole_damping)
        && isfinite(cfg->rectifier.duty_min)
        && isfinite(cfg->rectifier.duty_max)
        && (cfg->rectifier.sample_time_s > 0.0f)
        && (cfg->rectifier.line_frequency_hz > 0.0f)
        && (cfg->rectifier.outer_loop_divider > 0U)
        && (cfg->rectifier.rms_filter_time_constant_s > 0.0f)
        && (cfg->rectifier.input_voltage_rms_min_v > 0.0f)
        && (cfg->rectifier.dc_voltage_min_v > 0.0f)
        && (cfg->rectifier.dc_voltage_max_v
            > cfg->rectifier.dc_voltage_min_v)
        && (cfg->rectifier.dc_voltage_max_v
            <= cfg->protection.dc_voltage_max_v)
        && (cfg->rectifier.dc_reference_slew_v_per_s > 0.0f)
        && (cfg->rectifier.dc_ready_tolerance_v > 0.0f)
        && (cfg->rectifier.dc_ready_confirm_samples > 0U)
        && (cfg->rectifier.current_rms_max_a > 0.0f)
        && (current_command_peak
            < cfg->protection.input_current_abs_max_a)
        && (cfg->rectifier.dc_pi_kp >= 0.0f)
        && (cfg->rectifier.dc_pi_ki >= 0.0f)
        && (cfg->rectifier.dc_pi_kaw >= 0.0f)
        && ((cfg->rectifier.dc_pi_kp
           + cfg->rectifier.dc_pi_ki) > 0.0f)
        && (cfg->rectifier.current_qpr_kp >= 0.0f)
        && (cfg->rectifier.current_qpr_kr >= 0.0f)
        && (cfg->rectifier.current_qpr_bandwidth_rad_s > 0.0f)
        && ((cfg->rectifier.current_qpr_kp
           + cfg->rectifier.current_qpr_kr) > 0.0f)
        && (cfg->rectifier.inductor_voltage_limit_v > 0.0f)
        /*
         * PI 输出下限在 RectifierControl_Init 中固定为 0 A；这里禁止
         * 负的电流参考极性，确保任何有效配置都只能 AC -> DC。
         */
        && (cfg->rectifier.current_reference_sign >= 0.5f)
        && (fabsf(cfg->rectifier.bridge_polarity) >= 0.5f)
        && (cfg->rectifier.notch_zero_damping >= 0.0f)
        && (cfg->rectifier.notch_pole_damping > 0.0f)
        && (cfg->rectifier.notch_zero_damping
            <= cfg->rectifier.notch_pole_damping)
        && (cfg->rectifier.duty_min >= 0.0f)
        && (cfg->rectifier.duty_max <= 1.0f)
        && (cfg->rectifier.duty_min < cfg->rectifier.duty_max);
}

bool BoardPower_ValidateModeConfig(const control_config_t *cfg,
                                   app_mode_t mode)
{
    if (!BoardPower_CommonConfigValid(cfg))
    {
        return false;
    }

    switch (mode)
    {
    case MODE1_INVERTER:
        return BoardPower_InverterConfigValid(cfg);

    case MODE2_RECTIFIER:
        return BoardPower_RectifierConfigValid(cfg);

    case MODE0_FULL:
        return Measurement_CalibrationValid(&cfg->measurement)
            && BoardPower_InverterConfigValid(cfg)
            && BoardPower_RectifierConfigValid(cfg)
            && (cfg->rectifier.sample_time_s
                == cfg->inverter.sample_time_s);

    default:
        return false;
    }
}

bool BoardPower_ValidateConfig(const control_config_t *cfg)
{
    return BoardPower_ValidateModeConfig(cfg, MODE0_FULL);
}

bool BoardPower_ConfigValid(void)
{
    return BoardPower_ValidateConfig(BoardPower_GetConfig());
}
