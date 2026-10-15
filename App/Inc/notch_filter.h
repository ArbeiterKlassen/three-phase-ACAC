#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float s1;
    float s2;
    bool enabled;
} NotchFilter;

/**
 * @brief 初始化二阶数字陷波器并计算 Tustin 离散系数。
 * @param[out] notch 陷波器实例。
 * @param[in] center_rad_s 陷波中心角频率，单位 rad/s。
 * @param[in] zero_damping 分子零点阻尼系数，必须非负且不大于极点阻尼。
 * @param[in] pole_damping 分母极点阻尼系数，必须大于 0。
 * @param[in] sample_time_s 调用周期，单位 s。
 * @return 参数和离散系数有效时返回 true。
 */
bool NotchFilter_Init(NotchFilter *notch,
                      float center_rad_s,
                      float zero_damping,
                      float pole_damping,
                      float sample_time_s);

/**
 * @brief 按给定输入的直流稳态复位滤波器状态。
 * @param[in,out] notch 陷波器实例；NULL 将被忽略。
 * @param[in] input 当前输入值，用于无阶跃地初始化内部状态。
 */
void NotchFilter_Reset(NotchFilter *notch, float input);

/**
 * @brief 执行一次二阶陷波滤波。
 * @param[in,out] notch 已初始化的陷波器实例。
 * @param[in] input 当前采样值。
 * @return 滤波输出；实例无效、滤波未启用或数值异常时旁路返回 input。
 * @note 应按初始化时的 sample_time_s 固定周期调用。
 */
float NotchFilter_Step(NotchFilter *notch, float input);

#ifdef __cplusplus
}
#endif

#endif
