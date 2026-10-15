#include "notch_filter.h"

#include <math.h>
#include <stddef.h>

bool NotchFilter_Init(NotchFilter *notch,
                      float center_rad_s,
                      float zero_damping,
                      float pole_damping,
                      float sample_time_s)
{
    float k;
    float w2;
    float d0;

    if ((notch == NULL)
        || !isfinite(center_rad_s)
        || !isfinite(zero_damping)
        || !isfinite(pole_damping)
        || !isfinite(sample_time_s)
        || (center_rad_s <= 0.0f)
        || (zero_damping < 0.0f)
        || (pole_damping <= 0.0f)
        || (zero_damping > pole_damping)
        || (sample_time_s <= 0.0f))
    {
        return false;
    }

    /* 对模拟二阶陷波器使用双线性变换 s=2/T*(1-z^-1)/(1+z^-1)。 */
    k = 2.0f / sample_time_s;
    w2 = center_rad_s * center_rad_s;
    d0 = k * k + 2.0f * pole_damping * center_rad_s * k + w2;
    if (!isfinite(d0) || (d0 <= 0.0f))
    {
        return false;
    }

    *notch = (NotchFilter){0};
    notch->b0 = (k * k + 2.0f * zero_damping * center_rad_s * k + w2) / d0;
    notch->b1 = 2.0f * (w2 - k * k) / d0;
    notch->b2 = (k * k - 2.0f * zero_damping * center_rad_s * k + w2) / d0;
    notch->a1 = notch->b1;
    notch->a2 = (k * k - 2.0f * pole_damping * center_rad_s * k + w2) / d0;
    notch->enabled = true;
    return isfinite(notch->b0) && isfinite(notch->b1)
        && isfinite(notch->b2) && isfinite(notch->a2);
}

void NotchFilter_Reset(NotchFilter *notch, float input)
{
    if ((notch != NULL) && isfinite(input))
    {
        /* 以直流稳态初始化，避免使能滤波器时产生台阶。 */
        notch->s1 = (1.0f - notch->b0) * input;
        notch->s2 = (notch->b2 - notch->a2) * input;
    }
}

float NotchFilter_Step(NotchFilter *notch, float input)
{
    float output;
    float s1;

    if ((notch == NULL) || !isfinite(input) || !notch->enabled)
    {
        return input;
    }

    output = notch->b0 * input + notch->s1;
    s1 = notch->b1 * input - notch->a1 * output + notch->s2;
    notch->s2 = notch->b2 * input - notch->a2 * output;
    notch->s1 = s1;

    if (!isfinite(output) || !isfinite(notch->s1) || !isfinite(notch->s2))
    {
        NotchFilter_Reset(notch, input);
        return input;
    }
    return output;
}
