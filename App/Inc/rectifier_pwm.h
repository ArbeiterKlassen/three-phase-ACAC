#ifndef RECTIFIER_PWM_H
#define RECTIFIER_PWM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float duty_a;
    float duty_b;
    float modulation;
    bool saturated;
    bool valid;
} RectifierPwmOutput;

/**
 * @brief 将单相全桥两个桥臂设置为 0.5 中性占空比并标记为无效。
 * @param[out] out 整流 PWM 输出；NULL 将被忽略。
 * @note 本函数仅生成命令，不代表硬件驱动已经关闭。
 */
void RectifierPwm_SetNeutral(RectifierPwmOutput *out);

/**
 * @brief 将归一化桥压调制度转换为单极性全桥两桥臂占空比。
 * @param[in] modulation 期望调制度，理论范围为 -1～1，超出时按占空比限幅。
 * @param[in] duty_min 归一化最小占空比，必须位于 0～0.5 之间。
 * @param[in] duty_max 归一化最大占空比，必须位于 0.5～1 之间。
 * @param[out] out 两桥臂占空比、实际调制度和饱和标志。
 * @return 参数有效并成功生成命令时返回 true；失败时保持中性输出。
 */
bool RectifierPwm_Generate(float modulation,
                           float duty_min,
                           float duty_max,
                           RectifierPwmOutput *out);

#ifdef __cplusplus
}
#endif

#endif
