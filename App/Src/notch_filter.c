/*
 * 二阶数字陷波器：Tustin（双线性）离散，直接实现型结构。
 *
 * 用途是把母线电压上 100 Hz 的二倍频纹波从电压外环里摘掉。
 * 选择 Tustin 而不是后向差分，是因为它把虚轴映射到单位圆上，
 * 在目标频率处的衰减深度更准——陷波器最看重的就是这个。
 */

#include "notch_filter.h"

#include <math.h>
#include <stddef.h>

/*
 * 用 Tustin 法把模拟二阶陷波器离散化，算出五个系数。
 *
 * s = 2/T · (1−z⁻¹)/(1+z⁻¹)，代入后统一除以 d0 归一化。
 * zero_damping 决定陷波深度（越小坑越深），pole_damping 决定坑的宽度；
 * 要求 zero_damping ≤ pole_damping，否则零点跑到极点外面、滤波器不稳。
 *
 * 返回参数与算出的系数都有效。
 */
bool NotchFilter_Init(NotchFilter *notch, float center_rad_s, float zero_damping, float pole_damping,
                      float sample_time_s) {
    float k;
    float w2;
    float d0;

    if ((notch == NULL) || !isfinite(center_rad_s) || !isfinite(zero_damping) || !isfinite(pole_damping) ||
        !isfinite(sample_time_s) || (center_rad_s <= 0.0f) || (zero_damping < 0.0f) || (pole_damping <= 0.0f) ||
        (zero_damping > pole_damping) || (sample_time_s <= 0.0f)) {
        return false;
    }

    /* 对模拟二阶陷波器使用双线性变换 s=2/T*(1-z^-1)/(1+z^-1)。 */
    k = 2.0f / sample_time_s;
    w2 = center_rad_s * center_rad_s;
    d0 = k * k + 2.0f * pole_damping * center_rad_s * k + w2;
    if (!isfinite(d0) || (d0 <= 0.0f)) return false;

    *notch = (NotchFilter){0};
    notch->b0 = (k * k + 2.0f * zero_damping * center_rad_s * k + w2) / d0;
    notch->b1 = 2.0f * (w2 - k * k) / d0;
    notch->b2 = (k * k - 2.0f * zero_damping * center_rad_s * k + w2) / d0;
    notch->a1 = notch->b1;
    notch->a2 = (k * k - 2.0f * pole_damping * center_rad_s * k + w2) / d0;
    notch->enabled = true;
    return isfinite(notch->b0) && isfinite(notch->b1) && isfinite(notch->b2) && isfinite(notch->a2);
}

/*
 * 把内部状态设成"输入恒为 input 时的直流稳态"。
 * 目的是使能滤波器的瞬间不产生台阶——否则那个阶跃会被后级当成真实扰动。
 */
void NotchFilter_Reset(NotchFilter *notch, float input) {
    if ((notch != NULL) && isfinite(input)) {
        /* 以直流稳态初始化，避免使能滤波器时产生台阶。 */
        notch->s1 = (1.0f - notch->b0) * input;
        notch->s2 = (notch->b2 - notch->a2) * input;
    }
}

/*
 * 执行一次滤波（直接实现型，两次乘加）。
 *
 * 实例无效、未使能或输入非有限值时旁路返回 input；
 * 若状态发散（非有限），自动 Reset 后旁路——保证滤波器永远不会
 * 把 NaN 灌进控制环。
 */
float NotchFilter_Step(NotchFilter *notch, float input) {
    float output;
    float s1;

    if ((notch == NULL) || !isfinite(input) || !notch->enabled) return input;

    output = notch->b0 * input + notch->s1;
    s1 = notch->b1 * input - notch->a1 * output + notch->s2;
    notch->s2 = notch->b2 * input - notch->a2 * output;
    notch->s1 = s1;

    if (!isfinite(output) || !isfinite(notch->s1) || !isfinite(notch->s2)) {
        NotchFilter_Reset(notch, input);
        return input;
    }
    return output;
}
