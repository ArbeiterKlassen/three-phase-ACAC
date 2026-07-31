#include "qpr_controller.h"

#include <math.h>
#include <stddef.h>

static float QprController_Clamp(float value, float low, float high)
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

bool QprController_Init(QprController *qpr,
                        float kp,
                        float kr,
                        float resonance_rad_s,
                        float bandwidth_rad_s,
                        float sample_time_s,
                        float out_min,
                        float out_max)
{
    float k;
    float d0;
    float n0;

    if ((qpr == NULL)
        || !isfinite(kp) || !isfinite(kr)
        || !isfinite(resonance_rad_s)
        || !isfinite(bandwidth_rad_s)
        || !isfinite(sample_time_s)
        || !isfinite(out_min) || !isfinite(out_max)
        || (kr < 0.0f)
        || (resonance_rad_s <= 0.0f)
        || (bandwidth_rad_s <= 0.0f)
        || (sample_time_s <= 0.0f)
        || (out_min >= out_max))
    {
        return false;
    }

    /* 对带宽有限的谐振支路使用 Tustin 变换，比例支路保持独立。 */
    k = 2.0f / sample_time_s;
    d0 = k * k + 2.0f * bandwidth_rad_s * k
       + resonance_rad_s * resonance_rad_s;
    n0 = 2.0f * kr * bandwidth_rad_s * k;
    if (!isfinite(d0) || (d0 <= 0.0f))
    {
        return false;
    }

    *qpr = (QprController){0};
    qpr->kp = kp;
    qpr->b0 = n0 / d0;
    qpr->b2 = -qpr->b0;
    qpr->a1 = 2.0f * (resonance_rad_s * resonance_rad_s - k * k) / d0;
    qpr->a2 = (k * k - 2.0f * bandwidth_rad_s * k
             + resonance_rad_s * resonance_rad_s) / d0;
    qpr->out_min = out_min;
    qpr->out_max = out_max;
    return isfinite(qpr->b0) && isfinite(qpr->a1) && isfinite(qpr->a2);
}

void QprController_Reset(QprController *qpr)
{
    if (qpr != NULL)
    {
        qpr->e1 = 0.0f;
        qpr->e2 = 0.0f;
        qpr->y1 = 0.0f;
        qpr->y2 = 0.0f;
    }
}

float QprController_Step(QprController *qpr, float error)
{
    float resonant;
    float output;

    if ((qpr == NULL) || !isfinite(error))
    {
        return 0.0f;
    }

    /* Tustin离散的准PR谐振支路，比例支路单独相加。 */
    resonant = qpr->b0 * error + qpr->b2 * qpr->e2
             - qpr->a1 * qpr->y1 - qpr->a2 * qpr->y2;
    output = QprController_Clamp(qpr->kp * error + resonant,
                                 qpr->out_min,
                                 qpr->out_max);
    if (!isfinite(output))
    {
        QprController_Reset(qpr);
        return 0.0f;
    }

    /* 饱和时让谐振状态跟踪实际输出，避免解除限幅后长期恢复。 */
    if (output != (qpr->kp * error + resonant))
    {
        resonant = output - qpr->kp * error;
    }

    qpr->e2 = qpr->e1;
    qpr->e1 = error;
    qpr->y2 = qpr->y1;
    qpr->y1 = resonant;
    return output;
}

void QprController_TrackOutput(QprController *qpr,
                               float error,
                               float applied_output)
{
    float resonant;

    if ((qpr == NULL) || !isfinite(error) || !isfinite(applied_output))
    {
        return;
    }

    /*
     * 最终PWM限幅发生在控制器之后；用实际施加量回写当前谐振状态，
     * 使下一拍从“applied-kp*error”继续，而不是积累不可实现的指令。
     */
    resonant = applied_output - qpr->kp * error;
    if (isfinite(resonant))
    {
        qpr->y1 = resonant;
    }
}
