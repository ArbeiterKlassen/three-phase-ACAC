#pragma once

/*
 * 准比例谐振（QPR）控制器，Tustin 离散。
 *
 *      G(s) = Kp + Kr · 2ωc·s / (s² + 2ωc·s + ωr²)
 *
 * 与 PI 的区别：它在**某个特定频率**（ωr）上有极高增益，所以能对该频率的
 * 正弦量做到近乎零稳态误差——正是交流量控制需要的。带宽 ωc 决定它对
 * 频率漂移的容忍度：给宽一点，电网频率晃动时增益掉得少。
 *
 * 逆变级的电压外环与电流内环都用它。
 */

#include <stdbool.h>

/* QPR 实例：离散系数 + 误差/输出历史。 */
typedef struct {
    float kp;      // 比例增益
    float b0;      // 离散分子系数
    float b2;      // 离散分子系数
    float a1;      // 离散分母系数
    float a2;      // 离散分母系数
    float out_min; // 输出下限
    float out_max; // 输出上限
    float e1;      // 上一拍误差
    float e2;      // 上上拍误差
    float y1;      // 上一拍输出
    float y2;      // 上上拍输出
} QprController;

// 初始化。kr 必须非负；resonance_rad_s 为谐振角频率 [rad/s]，bandwidth_rad_s 为带宽 [rad/s]
// sample_time_s 为控制周期 [s]。参数与算出的离散系数都有效才返回 true
bool QprController_Init(QprController *qpr, float kp, float kr, float resonance_rad_s, float bandwidth_rad_s,
                        float sample_time_s, float out_min, float out_max);

// 清零误差历史与谐振支路状态。NULL 安全
void QprController_Reset(QprController *qpr);

// 执行一次控制。必须按初始化时的 sample_time_s 固定周期调用
// 返回限幅后的输出；实例或误差无效时返回 0
float QprController_Step(QprController *qpr, float error);

// 把谐振状态跟踪到下游**实际施加**的控制量
// 用于补偿 SVPWM 等后级限幅，否则谐振器会继续按"以为发出去了"的量积累
// applied_output 为经后级限幅或调制后真正生效的值
void QprController_TrackOutput(QprController *qpr, float error, float applied_output);
