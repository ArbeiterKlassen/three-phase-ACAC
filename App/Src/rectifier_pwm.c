/*
 * 单极性 SPWM：调制度 → 两桥臂占空比。
 *
 * 两个桥臂是"反相"的：一个加 0.5×m，另一个减 0.5×m，绕 50% 对称摆动。
 * 这样交流侧得到双极性电压，而每个桥臂自身始终在 0~1 内——两桥臂的
 * 上下管互锁仍由 TIM8 的互补输出与死区保证。
 */

#include "rectifier_pwm.h"

#include <math.h>
#include <stddef.h>

/* 把 value 限制到 [low, high] */
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

/* 两个桥臂置 50% 中性、清空其余字段。NULL 安全 */
void RectifierPwm_SetNeutral(RectifierPwmOutput *out)
{
    if (out != NULL)
    {
        *out = (RectifierPwmOutput){0};
        out->duty_a = 0.5f;
        out->duty_b = 0.5f;
    }
}

/*
 * 由调制度生成两桥臂占空比。
 *
 * duty_min / duty_max 必须夹在 0~0.5 与 0.5~1 两侧（即不允许骑跨中点），
 * 否则反相调制的对称性被破坏。限幅后回算 modulation 并置 saturated，
 * 供整流控制器做抗饱和回灌。
 *
 * 返回参数有效且生成成功；失败时输出保持中性值。
 */
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
