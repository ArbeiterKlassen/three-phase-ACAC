#pragma once

/*
 * 无查表正交相位发生器。
 *
 * 用旋转矩阵递推 (sin, cos)：每拍做一次乘加即可推进一个采样周期。
 * 好处是 20 kHz 快环里完全没有 sinf/cosf 调用——三角函数只在上电和切频
 * 这些低频路径上出现。代价是浮点误差会累积，所以每若干拍按模长归一化一次。
 */

#include "app_types.h"

/* 相位递推状态。 */
typedef struct
{
    float sin_value;        // 当前 sin(θ)
    float cos_value;        // 当前 cos(θ)
    float sin_step;         // 每拍 sin 增量（由频率决定）
    float cos_step;         // 每拍 cos 增量
    float sample_time_s;    // 递推周期 [s]
    uint16_t renorm_count;  // 距上次归一化已过的拍数
    uint16_t renorm_period; // 归一化周期（默认 256 拍）
} PhaseRecurrence;

// 初始化。frequency_hz 为基波频率 [Hz]，可为负（相序反向）
// initial_angle_rad 为初始电角度 [rad]。满足 |f|·Ts < 0.5 的奈奎斯特约束才返回 true
bool PhaseRecurrence_Init(PhaseRecurrence *phase,
                          float frequency_hz,
                          float sample_time_s,
                          float initial_angle_rad);

// 只改频率，不动当前相角。用于运行中切频
bool PhaseRecurrence_SetFrequency(PhaseRecurrence *phase, float frequency_hz);

// 直接设置当前电角度 [rad]
bool PhaseRecurrence_SetAngle(PhaseRecurrence *phase, float angle_rad);

// 相位归零，保留频率与采样周期。NULL 安全
void PhaseRecurrence_Reset(PhaseRecurrence *phase);

// 推进一个采样周期，并按 renorm_period 周期性归一化幅值
// 注意：在 20 kHz 快环里，"关掉门极"的分支也要调用它，否则重新使能时相位会跳
void PhaseRecurrence_Step(PhaseRecurrence *phase);

// 按当前相位生成正相序三相正弦量，peak 为峰值
// 返回 A=sinθ、B=sin(θ−120°)、C=sin(θ+120°)；输入无效时返回全零
ThreePhase PhaseRecurrence_GetThreePhase(const PhaseRecurrence *phase,
                                         float peak);
