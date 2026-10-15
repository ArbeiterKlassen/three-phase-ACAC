/*
 * 准比例谐振（QPR）控制器，Tustin 离散。
 *
 *      G(s) = Kp + Kr · 2ωc·s / (s² + 2ωc·s + ωr²)
 *                    └──────── 谐振支路 ────────┘
 *
 * 谐振支路在 ωr 处有极高的选频增益，所以能对**该频率的正弦量**做到近乎
 * 零稳态误差——这正是交流量控制需要的，也是它比 PI 强的地方。
 * 带宽 ωc 决定对频率漂移的宽容度：给宽一点，电网频率晃动时增益掉得少。
 *
 * 离散化只对谐振支路做，比例支路保持 Kp·e 直接相加，这样 Kp 的意义
 * 和推导都清楚。
 *
 * 本实现有一处关键设计：谐振状态 y1 存的是"谐振支路输出"而不是总输出。
 * 输出限幅时把 y1 拉回 (output − Kp·e)，这样解除限幅后不会因为
 * 内部积累了一个不可能实现的指令而长期恢复不过来。
 */

#include "qpr_controller.h"

#include <math.h>
#include <stddef.h>

/* 把 value 限制到 [low, high] */
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

/*
 * 用 Tustin 法离散谐振支路，算出四个系数。
 *
 * 要求 kr ≥ 0、ωr > 0、ωc > 0、Ts > 0、out_min < out_max。
 * b2 = −b0 是谐振支路的固有对称性（分子在 z=±1 处各有一个零点）。
 * 返回参数与算出的系数都有效。
 */
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

/*
 * 清零误差历史与谐振状态。用于使能瞬间或故障恢复后从干净状态起步。
 * NULL 安全。
 */
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

/*
 * 执行一次控制（差分方程，四次乘加）。
 *
 * 顺序：先算谐振支路输出 → 加比例项 → 限幅 → **若被限幅则把谐振状态
 * 拉回实际输出**。最后一步是抗饱和的关键，删掉它会在解除限幅后
 * 出现长时间过冲。
 *
 * 输出非有限时自动 Reset 并返回 0，避免 NaN 污染后级。
 * 必须按初始化时的 sample_time_s 固定周期调用。
 */
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

/*
 * 把谐振状态跟踪到下游最终实际施加的量。
 *
 * 与 Step 内部的抗饱和不同：这里处理的是**控制器之外**的限幅——
 * 典型例子是后级 SVPWM 的过调制缩放。控制器以为自己发出了 V，
 * 但实际只施加了 0.8V；不告知控制器的话它下一拍会继续按 V 积累。
 * 调用时机：本拍输出已经过全部后级处理、确定最终值之后。
 */
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
