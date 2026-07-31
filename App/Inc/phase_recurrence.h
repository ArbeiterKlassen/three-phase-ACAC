#ifndef PHASE_RECURRENCE_H
#define PHASE_RECURRENCE_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float sin_value;
    float cos_value;
    float sin_step;
    float cos_step;
    float sample_time_s;
    uint16_t renorm_count;
    uint16_t renorm_period;
} PhaseRecurrence;

/**
 * @brief 初始化基于正余弦递推的相位发生器。
 * @param[out] phase 相位递推实例。
 * @param[in] frequency_hz 基波频率，单位 Hz，可为负值。
 * @param[in] sample_time_s 递推周期，单位 s。
 * @param[in] initial_angle_rad 初始电角度，单位 rad。
 * @return 参数有限、采样满足 |frequency_hz| * sample_time_s < 0.5 时返回 true。
 */
bool PhaseRecurrence_Init(PhaseRecurrence *phase,
                          float frequency_hz,
                          float sample_time_s,
                          float initial_angle_rad);

/**
 * @brief 更新相位递推频率而不改变当前相角。
 * @param[in,out] phase 已设置有效采样周期的相位实例。
 * @param[in] frequency_hz 新频率，单位 Hz。
 * @return 频率有限且满足奈奎斯特约束时返回 true。
 */
bool PhaseRecurrence_SetFrequency(PhaseRecurrence *phase, float frequency_hz);

/**
 * @brief 直接设置当前电角度。
 * @param[in,out] phase 相位递推实例。
 * @param[in] angle_rad 电角度，单位 rad。
 * @return 指针和角度有效时返回 true。
 */
bool PhaseRecurrence_SetAngle(PhaseRecurrence *phase, float angle_rad);

/**
 * @brief 将当前相位复位到 0 rad，同时保留频率和采样周期。
 * @param[in,out] phase 相位递推实例；NULL 将被忽略。
 */
void PhaseRecurrence_Reset(PhaseRecurrence *phase);

/**
 * @brief 按配置频率推进一个采样周期，并周期性归一化正余弦幅值。
 * @param[in,out] phase 已初始化的相位递推实例。
 * @note 快环内仅进行乘加，默认每 256 拍归一化一次以抑制累计误差。
 */
void PhaseRecurrence_Step(PhaseRecurrence *phase);

/**
 * @brief 根据当前相位生成正相序三相正弦量。
 * @param[in] phase 当前相位递推实例。
 * @param[in] peak 三相量的峰值。
 * @return A=sinθ、B=sin(θ-120°)、C=sin(θ+120°) 的三相结果；
 *         输入无效时返回零值。
 */
ThreePhase PhaseRecurrence_GetThreePhase(const PhaseRecurrence *phase,
                                         float peak);

#ifdef __cplusplus
}
#endif

#endif
