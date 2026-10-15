#include "svpwm.h"

#include <math.h>
#include <stddef.h>

static float Svpwm_Max3(float a, float b, float c)
{
    float max = (a > b) ? a : b;
    return (max > c) ? max : c;
}

static float Svpwm_Min3(float a, float b, float c)
{
    float min = (a < b) ? a : b;
    return (min < c) ? min : c;
}

static float Svpwm_Clamp(float value, float low, float high)
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

static uint8_t Svpwm_Sector(float alpha, float beta)
{
    float root3_alpha;

    if ((fabsf(alpha) + fabsf(beta)) < 1.0e-12f)
    {
        return 0U;
    }

    /* 用 αβ 平面三条 60° 边界判定扇区，零矢量单独记为扇区 0。 */
    root3_alpha = APP_SQRT3_F * alpha;
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            return (beta <= root3_alpha) ? 1U : 2U;
        }
        return (beta >= -root3_alpha) ? 2U : 3U;
    }

    if (alpha < 0.0f)
    {
        return (-beta <= -root3_alpha) ? 4U : 5U;
    }
    return (-beta >= root3_alpha) ? 5U : 6U;
}

void Svpwm_SetNeutral(SvpwmOutput *out)
{
    if (out != NULL)
    {
        *out = (SvpwmOutput){0};
        out->duty_a = 0.5f;
        out->duty_b = 0.5f;
        out->duty_c = 0.5f;
        out->applied_scale = 0.0f;
    }
}

bool Svpwm_Generate(float voltage_alpha,
                    float voltage_beta,
                    float dc_voltage,
                    float duty_min,
                    float duty_max,
                    SvpwmOutput *out)
{
    float va;
    float vb;
    float vc;
    float vmax;
    float vmin;
    float span;
    float allowed_span;
    float scale = 1.0f;
    float common;
    float center;

    if (out == NULL)
    {
        return false;
    }
    Svpwm_SetNeutral(out);

    if (!isfinite(voltage_alpha) || !isfinite(voltage_beta)
        || !isfinite(dc_voltage)
        || !isfinite(duty_min) || !isfinite(duty_max)
        || (dc_voltage <= 0.0f)
        || (duty_min < 0.0f) || (duty_max > 1.0f)
        || (duty_min >= duty_max))
    {
        return false;
    }

    va = voltage_alpha;
    vb = -0.5f * voltage_alpha + 0.5f * APP_SQRT3_F * voltage_beta;
    vc = -0.5f * voltage_alpha - 0.5f * APP_SQRT3_F * voltage_beta;
    vmax = Svpwm_Max3(va, vb, vc);
    vmin = Svpwm_Min3(va, vb, vc);
    span = vmax - vmin;
    allowed_span = (duty_max - duty_min) * dc_voltage;

    /* 过调制前先等比例缩矢量，保持线电压方向和三相对称性。 */
    if (span > allowed_span)
    {
        scale = allowed_span / span;
        va *= scale;
        vb *= scale;
        vc *= scale;
        vmax *= scale;
        vmin *= scale;
        out->saturated = true;
    }

    common = -0.5f * (vmax + vmin);
    center = 0.5f * (duty_min + duty_max);
    out->duty_a = Svpwm_Clamp(center + (va + common) / dc_voltage,
                              duty_min, duty_max);
    out->duty_b = Svpwm_Clamp(center + (vb + common) / dc_voltage,
                              duty_min, duty_max);
    out->duty_c = Svpwm_Clamp(center + (vc + common) / dc_voltage,
                              duty_min, duty_max);
    out->applied_scale = scale;
    out->sector = Svpwm_Sector(voltage_alpha, voltage_beta);
    out->valid = true;
    return true;
}
