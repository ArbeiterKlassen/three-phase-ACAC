#include "phase_recurrence.h"

#include <math.h>

#define PHASE_DEFAULT_RENORM_PERIOD (256U)

static bool PhaseRecurrence_UpdateStep(PhaseRecurrence *phase,
                                       float frequency_hz)
{
    float angle_step;

    if ((phase == NULL)
        || !isfinite(frequency_hz)
        || !isfinite(phase->sample_time_s)
        || (phase->sample_time_s <= 0.0f)
        || (fabsf(frequency_hz) * phase->sample_time_s >= 0.5f))
    {
        return false;
    }

    angle_step = 2.0f * APP_PI_F * frequency_hz * phase->sample_time_s;
    phase->sin_step = sinf(angle_step);
    phase->cos_step = cosf(angle_step);
    return isfinite(phase->sin_step) && isfinite(phase->cos_step);
}

bool PhaseRecurrence_Init(PhaseRecurrence *phase,
                          float frequency_hz,
                          float sample_time_s,
                          float initial_angle_rad)
{
    if ((phase == NULL)
        || !isfinite(sample_time_s)
        || (sample_time_s <= 0.0f))
    {
        return false;
    }

    *phase = (PhaseRecurrence){0};
    phase->sample_time_s = sample_time_s;
    phase->renorm_period = PHASE_DEFAULT_RENORM_PERIOD;

    return PhaseRecurrence_UpdateStep(phase, frequency_hz)
        && PhaseRecurrence_SetAngle(phase, initial_angle_rad);
}

bool PhaseRecurrence_SetFrequency(PhaseRecurrence *phase, float frequency_hz)
{
    return PhaseRecurrence_UpdateStep(phase, frequency_hz);
}

bool PhaseRecurrence_SetAngle(PhaseRecurrence *phase, float angle_rad)
{
    if ((phase == NULL) || !isfinite(angle_rad))
    {
        return false;
    }

    phase->sin_value = sinf(angle_rad);
    phase->cos_value = cosf(angle_rad);
    phase->renorm_count = 0U;
    return isfinite(phase->sin_value) && isfinite(phase->cos_value);
}

void PhaseRecurrence_Reset(PhaseRecurrence *phase)
{
    if (phase != NULL)
    {
        phase->sin_value = 0.0f;
        phase->cos_value = 1.0f;
        phase->renorm_count = 0U;
    }
}

void PhaseRecurrence_Step(PhaseRecurrence *phase)
{
    float sin_next;
    float cos_next;
    float norm2;
    float gain;

    if (phase == NULL)
    {
        return;
    }

    /* 每拍仅乘加递推；不在快环内查正弦表或调用三角函数。 */
    sin_next = phase->sin_value * phase->cos_step
             + phase->cos_value * phase->sin_step;
    cos_next = phase->cos_value * phase->cos_step
             - phase->sin_value * phase->sin_step;
    phase->sin_value = sin_next;
    phase->cos_value = cos_next;

    phase->renorm_count++;
    if (phase->renorm_count < phase->renorm_period)
    {
        return;
    }

    phase->renorm_count = 0U;
    norm2 = sin_next * sin_next + cos_next * cos_next;
    if (isfinite(norm2) && (norm2 > 0.25f))
    {
        gain = 1.0f / sqrtf(norm2);
        phase->sin_value *= gain;
        phase->cos_value *= gain;
    }
    else
    {
        PhaseRecurrence_Reset(phase);
    }
}

ThreePhase PhaseRecurrence_GetThreePhase(const PhaseRecurrence *phase,
                                         float peak)
{
    ThreePhase out = {0};
    float half;
    float cross;

    if ((phase == NULL) || !isfinite(peak))
    {
        return out;
    }

    half = 0.5f * peak * phase->sin_value;
    cross = 0.5f * APP_SQRT3_F * peak * phase->cos_value;
    out.a = peak * phase->sin_value;
    out.b = -half - cross;
    out.c = -half + cross;
    return out;
}
