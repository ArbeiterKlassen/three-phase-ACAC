#ifndef SVPWM_H
#define SVPWM_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
    float applied_scale;
    uint8_t sector;
    bool saturated;
    bool valid;
} SvpwmOutput;

/**
 * @brief 将三相桥臂设置为 0.5 中性占空比并清除有效及饱和状态。
 * @param[out] out SVPWM 输出；NULL 将被忽略。
 * @note 本函数仅生成安全命令，不代表硬件 PWM 已经关断。
 */
void Svpwm_SetNeutral(SvpwmOutput *out);

/**
 * @brief 将 αβ 电压指令转换为三相归一化 SVPWM 占空比。
 * @param[in] voltage_alpha α 轴目标电压，单位 V。
 * @param[in] voltage_beta β 轴目标电压，单位 V。
 * @param[in] dc_voltage 直流母线电压，单位 V，必须大于 0。
 * @param[in] duty_min 归一化最小占空比，范围 0～1。
 * @param[in] duty_max 归一化最大占空比，范围 0～1 且大于 duty_min。
 * @param[out] out 三相占空比、扇区、实际缩放比例及饱和状态。
 * @return 参数有效并成功生成命令时返回 true；失败时保持中性输出。
 * @note 电压矢量超出可用母线范围时会等比例缩小，以保持方向和三相对称性。
 */
bool Svpwm_Generate(float voltage_alpha,
                    float voltage_beta,
                    float dc_voltage,
                    float duty_min,
                    float duty_max,
                    SvpwmOutput *out);

#ifdef __cplusplus
}
#endif

#endif
