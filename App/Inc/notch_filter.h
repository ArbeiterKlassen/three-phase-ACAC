#pragma once

/*
 * 二阶数字陷波器（Tustin 离散）。
 *
 * 用途：滤掉母线电压上 100 Hz 的二倍频纹波，否则它会串进电压外环，
 * 让整流级在功率因数上出问题。中心频率设成两倍线频即可。
 */

#include <stdbool.h>

/* 二阶节系数与两个状态变量。直接实现（II 型）结构。 */
typedef struct
{
    float b0;         // 分子系数
    float b1;         // 分子系数
    float b2;         // 分子系数
    float a1;         // 分母系数
    float a2;         // 分母系数
    float s1;         // 状态变量 1（直接实现型）
    float s2;         // 状态变量 2
    bool enabled;     // false 时 Step 直接旁路输入
} NotchFilter;

// 用 Tustin 法离散并初始化。center_rad_s 为中心角频率 [rad/s]
// zero_damping 为分子零点阻尼（≥0 且 ≤ pole_damping）；pole_damping 为极点阻尼（>0）
// sample_time_s 为调用周期 [s]。参数或系数无效时返回 false
bool NotchFilter_Init(NotchFilter *notch,
                      float center_rad_s,
                      float zero_damping,
                      float pole_damping,
                      float sample_time_s);

// 按给定输入的直流稳态复位内部状态，避免复位瞬间产生阶跃。NULL 安全
void NotchFilter_Reset(NotchFilter *notch, float input);

// 执行一次滤波。必须按初始化时的 sample_time_s 固定周期调用
// 返回滤波输出；实例无效、未启用或数值异常时**旁路返回 input**
float NotchFilter_Step(NotchFilter *notch, float input);
