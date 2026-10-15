#include "pi_controller.h"

#include <math.h>
#include <stddef.h>

static float PiController_Clamp(float value, float low, float high)
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

bool PiController_Init(PiController *pi,
                       float kp,
                       float ki,
                       float kaw,
                       float sample_time_s,
                       float out_min,
                       float out_max)
{
    if ((pi == NULL)
        || !isfinite(kp) || !isfinite(ki) || !isfinite(kaw)
        || !isfinite(sample_time_s)
        || !isfinite(out_min) || !isfinite(out_max)
        || (ki < 0.0f) || (kaw < 0.0f)
        || (sample_time_s <= 0.0f)
        || (out_min >= out_max))
    {
        return false;
    }

    *pi = (PiController){0};
    pi->kp = kp;
    pi->ki = ki;
    pi->kaw = kaw;
    pi->sample_time_s = sample_time_s;
    pi->out_min = out_min;
    pi->out_max = out_max;
    return true;
}

void PiController_Reset(PiController *pi, float output)
{
    if (pi != NULL)
    {
        pi->integrator = PiController_Clamp(output,
                                            pi->out_min,
                                            pi->out_max);
        pi->error_prev = 0.0f;
    }
}

float PiController_Step(PiController *pi, float error)
{
    float integral_old;
    float unsaturated;
    float output;

    if ((pi == NULL) || !isfinite(error))
    {
        return 0.0f;
    }

    /* 积分项用梯形法离散，降低固定采样周期下的相位与幅值误差。 */
    integral_old = pi->integrator;
    pi->integrator += 0.5f * pi->ki * pi->sample_time_s
                    * (error + pi->error_prev);
    unsaturated = pi->kp * error + pi->integrator;
    output = PiController_Clamp(unsaturated, pi->out_min, pi->out_max);

    /* 优先使用反算抗饱和；kaw=0 时冻结会加重饱和的积分方向。 */
    if (output != unsaturated)
    {
        if (pi->kaw > 0.0f)
        {
            pi->integrator += pi->kaw * pi->sample_time_s
                            * (output - unsaturated);
        }
        else if (((unsaturated > pi->out_max) && (error > 0.0f))
              || ((unsaturated < pi->out_min) && (error < 0.0f)))
        {
            pi->integrator = integral_old;
        }
    }

    pi->integrator = PiController_Clamp(pi->integrator,
                                        pi->out_min,
                                        pi->out_max);
    pi->error_prev = error;
    return output;
}
