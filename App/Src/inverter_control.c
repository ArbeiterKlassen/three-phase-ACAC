#include "inverter_control.h"

#include <math.h>
#include <stddef.h>

#define INVERTER_SQRT2_F (1.41421356237309504880f)

static float InverterControl_Clamp(float value, float low, float high)
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

static void InverterControl_ClearOutput(InverterControlOutput *out)
{
    *out = (InverterControlOutput){0};
    Svpwm_SetNeutral(&out->pwm);
}

bool InverterControl_Init(InverterControl *control,
                          const InverterControlConfig *cfg)
{
    float w0;

    if ((control == NULL) || (cfg == NULL)
        || !isfinite(cfg->sample_time_s)
        || !isfinite(cfg->output_frequency_hz)
        || !isfinite(cfg->voltage_rms_max)
        || !isfinite(cfg->reference_slew_v_per_s)
        || !isfinite(cfg->feedforward_gain)
        || !isfinite(cfg->current_reference_limit_a)
        || !isfinite(cfg->command_limit_v)
        || !isfinite(cfg->dc_voltage_min_v)
        || (cfg->sample_time_s <= 0.0f)
        || (cfg->output_frequency_hz <= 0.0f)
        || (cfg->voltage_rms_max < 0.0f)
        || (cfg->reference_slew_v_per_s < 0.0f)
        || (cfg->current_reference_limit_a <= 0.0f)
        || (cfg->command_limit_v <= 0.0f)
        || (cfg->dc_voltage_min_v <= 0.0f)
        || (cfg->duty_min < 0.0f)
        || (cfg->duty_max > 1.0f)
        || (cfg->duty_min >= cfg->duty_max))
    {
        return false;
    }

    *control = (InverterControl){0};
    control->cfg = *cfg;
    w0 = 2.0f * APP_PI_F * cfg->output_frequency_hz;

    if (!PhaseRecurrence_Init(&control->phase,
                              cfg->output_frequency_hz,
                              cfg->sample_time_s,
                              0.0f)
        || !QprController_Init(&control->voltage_alpha_qpr,
                               cfg->voltage_qpr_kp,
                               cfg->voltage_qpr_kr, w0,
                               cfg->voltage_qpr_bandwidth_rad_s,
                               cfg->sample_time_s,
                               -cfg->current_reference_limit_a,
                               cfg->current_reference_limit_a)
        || !QprController_Init(&control->voltage_beta_qpr,
                               cfg->voltage_qpr_kp,
                               cfg->voltage_qpr_kr, w0,
                               cfg->voltage_qpr_bandwidth_rad_s,
                               cfg->sample_time_s,
                               -cfg->current_reference_limit_a,
                               cfg->current_reference_limit_a)
        || !QprController_Init(&control->current_alpha_qpr,
                               cfg->current_qpr_kp,
                               cfg->current_qpr_kr, w0,
                               cfg->current_qpr_bandwidth_rad_s,
                               cfg->sample_time_s,
                               -cfg->command_limit_v,
                               cfg->command_limit_v)
        || !QprController_Init(&control->current_beta_qpr,
                               cfg->current_qpr_kp,
                               cfg->current_qpr_kr, w0,
                               cfg->current_qpr_bandwidth_rad_s,
                               cfg->sample_time_s,
                               -cfg->command_limit_v,
                               cfg->command_limit_v))
    {
        *control = (InverterControl){0};
        return false;
    }

    control->initialized = true;
    return true;
}

void InverterControl_Reset(InverterControl *control)
{
    if (control != NULL)
    {
        QprController_Reset(&control->voltage_alpha_qpr);
        QprController_Reset(&control->voltage_beta_qpr);
        QprController_Reset(&control->current_alpha_qpr);
        QprController_Reset(&control->current_beta_qpr);
        control->reference_rms = 0.0f;
    }
}

void InverterControl_SetReference(InverterControl *control,
                                  float voltage_rms)
{
    if ((control != NULL) && control->initialized && isfinite(voltage_rms))
    {
        control->reference_target_rms =
            InverterControl_Clamp(voltage_rms, 0.0f,
                                  control->cfg.voltage_rms_max);
    }
}

bool InverterControl_SetFrequency(InverterControl *control,
                                  float frequency_hz)
{
    float w0;
    QprController voltage_alpha;
    QprController voltage_beta;
    QprController current_alpha;
    QprController current_beta;

    if ((control == NULL) || !control->initialized
        || !isfinite(frequency_hz) || (frequency_hz <= 0.0f))
    {
        return false;
    }

    w0 = 2.0f * APP_PI_F * frequency_hz;
    if (!QprController_Init(&voltage_alpha,
                            control->cfg.voltage_qpr_kp,
                            control->cfg.voltage_qpr_kr,
                            w0,
                            control->cfg.voltage_qpr_bandwidth_rad_s,
                            control->cfg.sample_time_s,
                            -control->cfg.current_reference_limit_a,
                            control->cfg.current_reference_limit_a)
        || !QprController_Init(&voltage_beta,
                               control->cfg.voltage_qpr_kp,
                               control->cfg.voltage_qpr_kr,
                               w0,
                               control->cfg.voltage_qpr_bandwidth_rad_s,
                               control->cfg.sample_time_s,
                               -control->cfg.current_reference_limit_a,
                               control->cfg.current_reference_limit_a)
        || !QprController_Init(&current_alpha,
                               control->cfg.current_qpr_kp,
                               control->cfg.current_qpr_kr,
                               w0,
                               control->cfg.current_qpr_bandwidth_rad_s,
                               control->cfg.sample_time_s,
                               -control->cfg.command_limit_v,
                               control->cfg.command_limit_v)
        || !QprController_Init(&current_beta,
                               control->cfg.current_qpr_kp,
                               control->cfg.current_qpr_kr,
                               w0,
                               control->cfg.current_qpr_bandwidth_rad_s,
                               control->cfg.sample_time_s,
                               -control->cfg.command_limit_v,
                               control->cfg.command_limit_v)
        || !PhaseRecurrence_SetFrequency(&control->phase, frequency_hz))
    {
        return false;
    }

    control->voltage_alpha_qpr = voltage_alpha;
    control->voltage_beta_qpr = voltage_beta;
    control->current_alpha_qpr = current_alpha;
    control->current_beta_qpr = current_beta;
    control->cfg.output_frequency_hz = frequency_hz;
    return true;
}

static bool InverterControl_StepInternal(InverterControl *control,
                                         const Measurements *measurement,
                                         bool enabled,
                                         bool closed_loop,
                                         InverterControlOutput *out)
{
    float max_step;
    float peak;
    float error_alpha;
    float error_beta;
    float current_error_alpha = 0.0f;
    float current_error_beta = 0.0f;

    if ((control == NULL) || (out == NULL) || !control->initialized)
    {
        return false;
    }
    InverterControl_ClearOutput(out);

    if (!enabled)
    {
        InverterControl_Reset(control);
        PhaseRecurrence_Step(&control->phase);
        return true;
    }
    if ((measurement == NULL) || !measurement->valid
        || !isfinite(measurement->vdc)
        || (measurement->vdc < control->cfg.dc_voltage_min_v))
    {
        return false;
    }

    max_step = control->cfg.reference_slew_v_per_s
             * control->cfg.sample_time_s;
    if (control->reference_rms < control->reference_target_rms)
    {
        control->reference_rms =
            InverterControl_Clamp(control->reference_rms + max_step,
                                  0.0f, control->reference_target_rms);
    }
    else
    {
        control->reference_rms =
            InverterControl_Clamp(control->reference_rms - max_step,
                                  control->reference_target_rms,
                                  control->cfg.voltage_rms_max);
    }

    peak = INVERTER_SQRT2_F * control->reference_rms;
    /* 正相序约定：A=sinθ，B=sin(θ-120°)，C=sin(θ+120°)。 */
    out->reference.alpha = peak * control->phase.sin_value;
    out->reference.beta = -peak * control->phase.cos_value;
    if (closed_loop)
    {
        out->measured = Measurement_VoltageAlphaBeta(measurement);
        error_alpha = out->reference.alpha - out->measured.alpha;
        error_beta = out->reference.beta - out->measured.beta;
        out->current_reference.alpha =
            QprController_Step(&control->voltage_alpha_qpr, error_alpha);
        out->current_reference.beta =
            QprController_Step(&control->voltage_beta_qpr, error_beta);
        out->current_measured = Measurement_CurrentAlphaBeta(measurement);
        current_error_alpha =
            out->current_reference.alpha - out->current_measured.alpha;
        current_error_beta =
            out->current_reference.beta - out->current_measured.beta;

    /* 电压外环给出i*，电流内环再给出逆变桥电压指令。 */
        out->voltage_command.alpha =
            control->cfg.feedforward_gain * out->reference.alpha
            + QprController_Step(&control->current_alpha_qpr,
                                 current_error_alpha);
        out->voltage_command.beta =
            control->cfg.feedforward_gain * out->reference.beta
            + QprController_Step(&control->current_beta_qpr,
                                 current_error_beta);
        out->voltage_command.alpha =
            InverterControl_Clamp(out->voltage_command.alpha,
                                  -control->cfg.command_limit_v,
                                  control->cfg.command_limit_v);
        out->voltage_command.beta =
            InverterControl_Clamp(out->voltage_command.beta,
                                  -control->cfg.command_limit_v,
                                  control->cfg.command_limit_v);
    }
    else
    {
        /*
         * Diagnostic open loop ignores voltage/current feedback. The
         * common soft-start, phase generator and SVPWM limits still apply.
         */
        out->voltage_command = out->reference;
    }

    out->valid = Svpwm_Generate(out->voltage_command.alpha,
                                out->voltage_command.beta,
                                measurement->vdc,
                                control->cfg.duty_min,
                                control->cfg.duty_max,
                                &out->pwm);
    if (out->valid && closed_loop)
    {
        QprController_TrackOutput(
            &control->current_alpha_qpr,
            current_error_alpha,
            out->pwm.applied_scale * out->voltage_command.alpha
                - control->cfg.feedforward_gain * out->reference.alpha);
        QprController_TrackOutput(
            &control->current_beta_qpr,
            current_error_beta,
            out->pwm.applied_scale * out->voltage_command.beta
                - control->cfg.feedforward_gain * out->reference.beta);
    }
    PhaseRecurrence_Step(&control->phase);
    return out->valid;
}

bool InverterControl_Step(InverterControl *control,
                          const Measurements *measurement,
                          bool enabled,
                          InverterControlOutput *out)
{
    return InverterControl_StepInternal(control, measurement, enabled,
                                        true, out);
}

bool InverterControl_StepOpenLoop(InverterControl *control,
                                  const Measurements *measurement,
                                  bool enabled,
                                  InverterControlOutput *out)
{
    return InverterControl_StepInternal(control, measurement, enabled,
                                        false, out);
}
