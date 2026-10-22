#include "acdc_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float AcdcControl_Clamp(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void AcdcPi_Reset(AcdcPi *pi, float output) {
    pi->integrator = AcdcControl_Clamp(output, pi->out_min, pi->out_max);
}

/*
 * 条件积分 PI。输出饱和且误差仍把输出往饱和方向推时，撤销本拍积分。
 * 离散积分与 host_tests/sim_input_current.c 的矩形积分一致，便于现场对照。
 */
static float AcdcPi_Step(AcdcPi *pi, float error) {
    const float old_integrator = pi->integrator;
    float unsaturated;
    float output;

    pi->integrator += pi->ki * pi->sample_time_s * error;
    unsaturated = pi->kp * error + pi->integrator;
    output = AcdcControl_Clamp(unsaturated, pi->out_min, pi->out_max);
    if (output != unsaturated) {
        if (((unsaturated > pi->out_max) && (error > 0.0f)) || ((unsaturated < pi->out_min) && (error < 0.0f)))
            pi->integrator = old_integrator;
    }
    pi->integrator = AcdcControl_Clamp(pi->integrator, pi->out_min, pi->out_max);
    return output;
}

static bool AcdcNotch_Init(AcdcNotch *notch, float center_rad_s, float zero_damping, float pole_damping,
                           float sample_time_s) {
    float k;
    float w2;
    float denominator;

    if ((notch == NULL) || !(center_rad_s > 0.0f) || !(pole_damping > 0.0f) ||
        !(zero_damping >= 0.0f && zero_damping <= pole_damping) || !(sample_time_s > 0.0f)) {
        return false;
    }

    k = 2.0f / sample_time_s;
    w2 = center_rad_s * center_rad_s;
    denominator = k * k + 2.0f * pole_damping * center_rad_s * k + w2;
    if (!(denominator > 0.0f) || !isfinite(denominator)) return false;

    notch->b0 = (k * k + 2.0f * zero_damping * center_rad_s * k + w2) / denominator;
    notch->b1 = 2.0f * (w2 - k * k) / denominator;
    notch->b2 = (k * k - 2.0f * zero_damping * center_rad_s * k + w2) / denominator;
    notch->a1 = notch->b1;
    notch->a2 = (k * k - 2.0f * pole_damping * center_rad_s * k + w2) / denominator;
    notch->s1 = 0.0f;
    notch->s2 = 0.0f;
    return isfinite(notch->b0) && isfinite(notch->b1) && isfinite(notch->b2) && isfinite(notch->a2);
}

static void AcdcNotch_Reset(AcdcNotch *notch, float input) {
    notch->s1 = (1.0f - notch->b0) * input;
    notch->s2 = (notch->b2 - notch->a2) * input;
}

static float AcdcNotch_Step(AcdcNotch *notch, float input) {
    float output = notch->b0 * input + notch->s1;
    float next_s1 = notch->b1 * input - notch->a1 * output + notch->s2;

    notch->s2 = notch->b2 * input - notch->a2 * output;
    notch->s1 = next_s1;
    if (!isfinite(output) || !isfinite(notch->s1) || !isfinite(notch->s2)) {
        AcdcNotch_Reset(notch, input);
        return input;
    }
    return output;
}

static void AcdcControl_InvalidateRms(AcdcControl *control) {
    control->vin_square_sum = 0.0f;
    control->vin_square_index = 0U;
    control->vin_square_count = 0U;
    control->vin_rms_v = 0.0f;
    control->rms_valid = false;
}

/*
 * 一个工频周期的滑动 RMS。窗口固定 400 点；如果改了 20 kHz 或 50 Hz，
 * Init 会因窗口长度不匹配而拒绝工作，不能只改一个频率。
 */
static void AcdcControl_UpdateRms(AcdcControl *control, float vin) {
    const float vin_square = vin * vin;
    float oldest = 0.0f;

    if (!isfinite(vin_square)) {
        AcdcControl_InvalidateRms(control);
        return;
    }
    if (control->vin_square_count >= ACDC_RMS_WINDOW_SAMPLES)
        oldest = control->vin_square_window[control->vin_square_index];
    else control->vin_square_count++;

    control->vin_square_window[control->vin_square_index] = vin_square;
    control->vin_square_sum += vin_square - oldest;
    control->vin_square_index++;
    if (control->vin_square_index >= ACDC_RMS_WINDOW_SAMPLES) control->vin_square_index = 0U;

    if (control->vin_square_count >= ACDC_RMS_WINDOW_SAMPLES) {
        control->vin_rms_v = sqrtf(fmaxf(control->vin_square_sum / (float)ACDC_RMS_WINDOW_SAMPLES, 0.0f));
        control->rms_valid = isfinite(control->vin_rms_v) && (control->vin_rms_v > 0.1f);
    }
}

static void AcdcControl_ResetDynamics(AcdcControl *control) {
    AcdcPi_Reset(&control->bus_pi, 0.0f);
    control->current_integral_v = 0.0f;
    control->outer_count = ACDC_OUTER_LOOP_DIVIDER - 1U;
    control->voltage_reference_v = 0.0f;
    control->current_rms_command_a = 0.0f;
    control->current_reference_a = 0.0f;
    control->enabled_previous = false;
}

bool AcdcControl_Init(AcdcControl *control, const AcdcControlConfig *cfg) {
    const float slow_time_s = ACDC_SAMPLE_TIME_S * (float)ACDC_OUTER_LOOP_DIVIDER;
    const float cycles = ACDC_SAMPLE_TIME_S * ACDC_LINE_FREQ_HZ * (float)ACDC_RMS_WINDOW_SAMPLES;

    if ((control == NULL) || (cfg == NULL) || !isfinite(cfg->vbus_target_v) || !isfinite(cfg->vbus_min_v) ||
        !isfinite(cfg->vbus_max_v) || !isfinite(cfg->vin_rms_min_v) || !isfinite(cfg->vin_abs_max_v) ||
        !isfinite(cfg->iin_abs_max_a) || !isfinite(cfg->iin_command_max_a) || !isfinite(cfg->vbus_slew_v_per_s) ||
        !isfinite(cfg->duty_min) || !isfinite(cfg->duty_max) || (cfg->vbus_max_v <= cfg->vbus_min_v) ||
        (cfg->vbus_target_v <= cfg->vbus_min_v) || (cfg->vbus_target_v >= cfg->vbus_max_v) ||
        !(cfg->vin_rms_min_v > 0.0f) || !(cfg->vin_abs_max_v > cfg->vbus_target_v) || !(cfg->iin_abs_max_a > 0.0f) ||
        !(cfg->iin_command_max_a > 0.0f) || !(cfg->vbus_slew_v_per_s > 0.0f) || (cfg->duty_min < 0.0f) ||
        (cfg->duty_max > 1.0f) || (cfg->duty_min >= cfg->duty_max) || (fabsf(cycles - 1.0f) > 1.0e-4f)) {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->cfg = *cfg;
    control->bus_pi = (AcdcPi){
        .kp = ACDC_BUS_PI_KP_A_PER_V,
        .ki = ACDC_BUS_PI_KI_A_PER_V_S,
        .sample_time_s = slow_time_s,
        .integrator = 0.0f,
        .out_min = 0.0f,
        .out_max = cfg->iin_command_max_a,
    };
    if (!AcdcNotch_Init(&control->bus_notch, 2.0f * 2.0f * 3.14159265358979323846f * ACDC_LINE_FREQ_HZ,
                        ACDC_NOTCH_ZERO_DAMPING, ACDC_NOTCH_POLE_DAMPING, slow_time_s)) {
        memset(control, 0, sizeof(*control));
        return false;
    }
    AcdcControl_Reset(control);
    return true;
}

void AcdcControl_Reset(AcdcControl *control) {
    if (control != NULL) {
        AcdcControl_InvalidateRms(control);
        AcdcControl_ResetDynamics(control);
        control->vbus_filtered_v = 0.0f;
        AcdcNotch_Reset(&control->bus_notch, 0.0f);
    }
}

bool AcdcControl_Step(AcdcControl *control, const AcdcMeasurement *measurement, bool enabled, AcdcControlOutput *out) {
    float requested_duty;
    float old_integral;
    float current_error;
    float inductor_voltage;
    float duty;

    if ((control == NULL) || (out == NULL)) return false;
    memset(out, 0, sizeof(*out));
    out->low_side_duty = 0.5f;
    if ((measurement == NULL) || !measurement->valid || !isfinite(measurement->iin_a) ||
        !isfinite(measurement->vin_v) || !isfinite(measurement->vcf_v) || !isfinite(measurement->vdc_v)) {
        AcdcControl_InvalidateRms(control);
        AcdcControl_ResetDynamics(control);
        return false;
    }

    /* 即使门极还没开，也持续填满 Vin RMS 窗口，缩短 READY 等待时间。 */
    AcdcControl_UpdateRms(control, measurement->vin_v);
    control->vbus_filtered_v = measurement->vdc_v;
    out->input_voltage_rms_v = control->vin_rms_v;

    if (!enabled) {
        AcdcControl_ResetDynamics(control);
        AcdcNotch_Reset(&control->bus_notch, measurement->vdc_v);
        out->vbus_filtered_v = measurement->vdc_v;
        out->valid = true;
        out->gates_enabled = false;
        return true;
    }

    if (!control->rms_valid || (control->vin_rms_v < control->cfg.vin_rms_min_v) ||
        (measurement->vdc_v < control->cfg.vbus_min_v) || (measurement->vdc_v > control->cfg.vbus_max_v) ||
        (fabsf(measurement->vin_v) > control->cfg.vin_abs_max_v) ||
        (fabsf(measurement->iin_a) > control->cfg.iin_abs_max_a)) {
        AcdcControl_ResetDynamics(control);
        return false;
    }

    if (!control->enabled_previous) {
        /*
         * 第一次开门的参考从当前母线下方 1 V 起坡，而不是从 0 V 起。
         * 这样不会先让母线自由放电，再等软启动追上来；积分器也从无扰状态
         * 开始，避免 ENTER RUN 瞬间冲电流。
         */
        AcdcPi_Reset(&control->bus_pi, 0.0f);
        control->current_integral_v = 0.0f;
        control->voltage_reference_v = AcdcControl_Clamp(measurement->vdc_v - 1.0f, 0.0f, control->cfg.vbus_target_v);
        AcdcNotch_Reset(&control->bus_notch, measurement->vdc_v);
        control->outer_count = ACDC_OUTER_LOOP_DIVIDER - 1U;
        control->enabled_previous = true;
    }

    control->voltage_reference_v += control->cfg.vbus_slew_v_per_s * ACDC_SAMPLE_TIME_S;
    control->voltage_reference_v = AcdcControl_Clamp(control->voltage_reference_v, 0.0f, control->cfg.vbus_target_v);

    control->outer_count++;
    if (control->outer_count >= ACDC_OUTER_LOOP_DIVIDER) {
        float bus_error;
        control->outer_count = 0U;
        control->vbus_filtered_v = AcdcNotch_Step(&control->bus_notch, measurement->vdc_v);
        bus_error = control->voltage_reference_v - control->vbus_filtered_v;
        control->current_rms_command_a = AcdcPi_Step(&control->bus_pi, bus_error);
    }

    control->current_reference_a =
        control->current_rms_command_a * fabsf(measurement->vin_v) / fmaxf(control->vin_rms_v, 1.0e-3f);
    current_error = control->current_reference_a - fabsf(measurement->iin_a);

    /*
     * 只用现有的交流输入电流传感器。控制器不假设 Lb 上还有第二路电流
     * 传感器：PI 输出电感电压 vL，再由 vCf/Vbus 前馈反算下管占空比。
     */
    old_integral = control->current_integral_v;
    control->current_integral_v += ACDC_CURRENT_PI_KI_V_PER_A_S * ACDC_SAMPLE_TIME_S * current_error;
    control->current_integral_v =
        AcdcControl_Clamp(control->current_integral_v, -ACDC_CURRENT_VOLTAGE_LIMIT_V, ACDC_CURRENT_VOLTAGE_LIMIT_V);
    inductor_voltage = ACDC_CURRENT_PI_KP_V_PER_A * current_error + control->current_integral_v;
    requested_duty = 1.0f - (measurement->vcf_v - inductor_voltage) / fmaxf(measurement->vdc_v, 1.0f);
    duty = AcdcControl_Clamp(requested_duty, control->cfg.duty_min, control->cfg.duty_max);

    if (fabsf(duty - requested_duty) > 1.0e-7f) {
        if (((requested_duty > control->cfg.duty_max) && (current_error > 0.0f)) ||
            ((requested_duty < control->cfg.duty_min) && (current_error < 0.0f))) {
            control->current_integral_v = old_integral;
        }
    }

    out->low_side_duty = duty;
    out->current_reference_a = control->current_reference_a;
    out->current_rms_cmd_a = control->current_rms_command_a;
    out->voltage_reference_v = control->voltage_reference_v;
    out->vbus_filtered_v = control->vbus_filtered_v;
    out->current_limited = control->current_rms_command_a >= (control->cfg.iin_command_max_a - 1.0e-5f);
    out->duty_limited = fabsf(duty - requested_duty) > 1.0e-7f;
    out->valid = isfinite(duty) && (control->current_rms_command_a >= 0.0f);
    out->gates_enabled = out->valid;
    return out->valid;
}
