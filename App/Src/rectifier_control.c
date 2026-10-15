#include "rectifier_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float RectifierControl_Clamp(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static void RectifierControl_ClearOutput(RectifierControlOutput *out)
{
    *out = (RectifierControlOutput){0};
    RectifierPwm_SetNeutral(&out->pwm);
}

static void RectifierControl_InvalidateInputRms(RectifierControl *control)
{
    control->vin_square_sum = 0.0f;
    control->vin_rms_v = 0.0f;
    control->vin_square_index = 0U;
    control->vin_square_count = 0U;
    control->outer_count = 0U;
    control->vin_rms_valid = false;
}

static void RectifierControl_ResetInputRms(RectifierControl *control)
{
    memset(control->vin_square_window, 0,
           sizeof(control->vin_square_window));
    RectifierControl_InvalidateInputRms(control);
}

static void RectifierControl_UpdateInputRms(RectifierControl *control,
                                            float vin)
{
    float vin_square;
    float oldest_square = 0.0f;

    vin_square = vin * vin;
    if (!isfinite(vin_square))
    {
        RectifierControl_InvalidateInputRms(control);
        return;
    }

    if (control->vin_square_count
        >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES)
    {
        oldest_square =
            control->vin_square_window[control->vin_square_index];
    }
    else
    {
        control->vin_square_count++;
    }

    control->vin_square_window[control->vin_square_index] = vin_square;
    control->vin_square_sum += vin_square - oldest_square;
    control->vin_square_index++;
    if (control->vin_square_index
        >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES)
    {
        control->vin_square_index = 0U;
    }

    if (!isfinite(control->vin_square_sum))
    {
        RectifierControl_InvalidateInputRms(control);
        return;
    }

    if (control->vin_square_count
        >= RECTIFIER_VIN_RMS_WINDOW_SAMPLES)
    {
        control->vin_rms_v =
            sqrtf(fmaxf(control->vin_square_sum, 0.0f)
                  / (float)RECTIFIER_VIN_RMS_WINDOW_SAMPLES);
        control->vin_rms_valid = isfinite(control->vin_rms_v);
        if (!control->vin_rms_valid)
        {
            RectifierControl_InvalidateInputRms(control);
        }
    }
}

bool RectifierControl_Init(RectifierControl *control,
                           const RectifierControlConfig *cfg)
{
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
    w0 = 2.0f * APP_PI_F * cfg->line_frequency_hz;
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

void RectifierControl_Reset(RectifierControl *control)
{
    if (control != NULL)
    {
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

void RectifierControl_SetDcReference(RectifierControl *control,
                                     float dc_voltage)
{
    if ((control != NULL) && control->initialized && isfinite(dc_voltage))
    {
        control->dc_reference_target_v =
            RectifierControl_Clamp(dc_voltage, 0.0f,
                                   control->cfg.dc_voltage_max_v);
    }
}

static void RectifierControl_UpdateSlow(RectifierControl *control,
                                        const Measurements *measurement,
                                        bool enabled)
{
    float dc_error;

    /* Keep the 100 Hz notch and DC-voltage PI at 1 kHz. */
    if (control->outer_count + 1U < control->cfg.outer_loop_divider)
    {
        control->outer_count++;
        return;
    }

    control->outer_count = 0U;
    control->dc_voltage_filtered =
        NotchFilter_Step(&control->dc_notch, measurement->vdc);

    if (enabled)
    {
        dc_error = control->dc_reference_v - control->dc_voltage_filtered;
        control->current_rms_a =
            PiController_Step(&control->dc_pi, dc_error);
    }
    else
    {
        PiController_Reset(&control->dc_pi, 0.0f);
        control->current_rms_a = 0.0f;
    }
}

bool RectifierControl_Step(RectifierControl *control,
                           const Measurements *measurement,
                           bool enabled,
                           RectifierControlOutput *out)
{
    float dc_step;
    float error;
    float inductor_voltage;
    float inductor_voltage_applied;
    float modulation;
    bool control_enabled;

    if ((control == NULL) || (out == NULL) || !control->initialized)
    {
        return false;
    }
    RectifierControl_ClearOutput(out);
    out->input_voltage_rms = control->vin_rms_v;
    out->input_rms_valid = control->vin_rms_valid;

    if ((measurement == NULL) || !measurement->valid
        || !isfinite(measurement->vin)
        || !isfinite(measurement->iin)
        || !isfinite(measurement->vdc))
    {
        RectifierControl_InvalidateInputRms(control);
        out->input_voltage_rms = 0.0f;
        out->input_rms_valid = false;
        return !enabled;
    }

    /*
     * Keep a strict one-line-cycle moving RMS at the 20 kHz fast rate.
     * A disabled controller still fills this window so state-machine
     * preheating can complete without producing a control fault.
     */
    RectifierControl_UpdateInputRms(control, measurement->vin);
    control_enabled = enabled && control->vin_rms_valid;

    if (control_enabled && !control->soft_start_initialized)
    {
        /*
         * 从0 V参考完整起坡；若母线已被动充电，高于参考时PI下限为0，
         * 不会命令反向灌电，也不会把1 s软启动缩短为余量起坡。
         */
        control->dc_reference_v = 0.0f;
        control->soft_start_initialized = true;
    }
    if (control_enabled)
    {
        dc_step = control->cfg.dc_reference_slew_v_per_s
                * control->cfg.sample_time_s;
        control->dc_reference_v =
            RectifierControl_Clamp(control->dc_reference_v + dc_step,
                                   0.0f,
                                   control->dc_reference_target_v);
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

    if (!control_enabled)
    {
        QprController_Reset(&control->current_qpr);
        control->soft_start_initialized = false;
        return true;
    }
    if ((measurement->vdc < control->cfg.dc_voltage_min_v)
        || (control->vin_rms_v < control->cfg.input_voltage_rms_min_v))
    {
        return false;
    }

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
    if (!out->gates_enabled)
    {
        QprController_Reset(&control->current_qpr);
        out->current_reference = 0.0f;
    }
    error = out->current_reference - measurement->iin;
    inductor_voltage = QprController_Step(&control->current_qpr, error);
    out->bridge_voltage_command =
        control->cfg.bridge_polarity * (measurement->vin - inductor_voltage);
    modulation = out->bridge_voltage_command / measurement->vdc;
    out->valid = RectifierPwm_Generate(modulation,
                                       control->cfg.duty_min,
                                       control->cfg.duty_max,
                                       &out->pwm);
    if (out->valid && out->gates_enabled)
    {
        /* 门极关断时桥压由二极管决定，不按 PWM 公式反灌电流环状态。 */
        inductor_voltage_applied = measurement->vin
            - (out->pwm.modulation * measurement->vdc)
              / control->cfg.bridge_polarity;
        QprController_TrackOutput(&control->current_qpr,
                                  error,
                                  inductor_voltage_applied);
    }
    return out->valid;
}
