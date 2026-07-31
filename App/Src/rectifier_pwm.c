#include "rectifier_pwm.h"

#include <math.h>
#include <stddef.h>

static float RectifierPwm_Clamp(float value, float low, float high)
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

void RectifierPwm_SetNeutral(RectifierPwmOutput *out)
{
    if (out != NULL)
    {
        *out = (RectifierPwmOutput){0};
        out->duty_a = 0.5f;
        out->duty_b = 0.5f;
    }
}

bool RectifierPwm_Generate(float modulation,
                           float duty_min,
                           float duty_max,
                           RectifierPwmOutput *out)
{
    float requested_a;
    float requested_b;

    if (out == NULL)
    {
        return false;
    }
    RectifierPwm_SetNeutral(out);

    if (!isfinite(modulation)
        || !isfinite(duty_min) || !isfinite(duty_max)
        || (duty_min < 0.0f) || (duty_max > 1.0f)
        || (duty_min >= 0.5f) || (duty_max <= 0.5f))
    {
        return false;
    }

    /* 两桥臂反相载波比较，得到单相全桥单极性SPWM。 */
    requested_a = 0.5f + 0.5f * modulation;
    requested_b = 0.5f - 0.5f * modulation;
    out->duty_a = RectifierPwm_Clamp(requested_a, duty_min, duty_max);
    out->duty_b = RectifierPwm_Clamp(requested_b, duty_min, duty_max);
    out->modulation = out->duty_a - out->duty_b;
    out->saturated = fabsf(out->modulation - modulation) > 1.0e-7f;
    out->valid = true;
    return true;
}
