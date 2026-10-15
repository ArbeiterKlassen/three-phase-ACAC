/*
 * 连续 SVPWM：αβ 电压指令 → 三相归一化占空比。
 *
 * 三步：
 *   1. αβ → 三相瞬时值（幅值不变反 Clarke）
 *   2. 过调制时整体等比例缩小（保方向、保三相对称）
 *   3. 注入零序共模，把三相"抬"到占空比区间中央
 *
 * 第 3 步是关键：零序是三相共有的分量，加它不改变任何**线电压**，
 * 却能把可用范围从 Vdc/2 扩到 Vdc/√3（约 1.15 倍）——这就是 SVPWM
 * 比正弦调制多出 15% 电压利用率的来源。
 *
 * 本工程只在 20 kHz 快环里调用，全部是乘加与比较，无三角函数。
 */

#include "svpwm.h"

#include <math.h>
#include <stddef.h>

/* 三数取最大 */
static float Svpwm_Max3(float a, float b, float c)
{
    float max = (a > b) ? a : b;
    return (max > c) ? max : c;
}

/* 三数取最小 */
static float Svpwm_Min3(float a, float b, float c)
{
    float min = (a < b) ? a : b;
    return (min < c) ? min : c;
}

/* 把 value 限制到 [low, high] */
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

/*
 * 判定电压矢量落在哪个扇区（1~6），零矢量返回 0。
 *
 * 用 αβ 平面上三条相差 60° 的边界做比较即可，不需要查表也不需要
 * 算 atan2。**这个结果只用于诊断显示**（OLED/调试器），不参与占空比计算——
 * 所以即使判错也不会影响输出，别被"扇区看起来不对"误导。
 */
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

/*
 * 三相置 50% 中性、缩放比例置 0、清除有效与饱和标志。
 * 注意这只是"生成安全命令"，不代表硬件 PWM 已经关断——真正的关断
 * 由 board_runtime.c 清 CCER/MOE 完成。NULL 安全。
 */
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

/*
 * 由 αβ 电压指令生成三相占空比。
 *
 * 过调制处理：比较三相瞬时值的极差 (vmax−vmin) 与母线允许的极差
 * (duty_max−duty_min)×Vdc。超了就整体乘一个 <1 的比例——**等比例缩放
 * 而不是逐相限幅**，后者会破坏三相对称、在输出上产生低次谐波。
 * 实际缩放比例存进 applied_scale，供电流环做抗饱和回灌。
 *
 * 返回参数有效且生成成功；失败时输出保持中性值。
 */
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
