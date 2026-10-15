#pragma once

/*
 * 连续 SVPWM：把 αβ 电压指令转成三相归一化占空比。
 *
 * 步骤：αβ → 三相 → 注入零序（共模）→ 归一化。零序不改变线电压，
 * 但把可用范围从 Vdc/2 扩到 Vdc/√3，这是 SVPWM 比正弦调制的优势所在。
 * 电压矢量超出母线能力时整体等比例缩小，保持方向与三相对称。
 */

#include "app_types.h"

/* 一次调制的结果。duty_a/b/c 直接写进 TIM1 的 CCR。 */
typedef struct {
    float duty_a;        // A 相归一化占空比 [0~1]
    float duty_b;        // B 相
    float duty_c;        // C 相
    float applied_scale; // 实际缩放比例；<1 表示发生了过调制缩放
    uint8_t sector;      // 电压矢量所在扇区 [1~6]，仅用于诊断
    bool saturated;      // 是否触发过调制缩放
    bool valid;          // 生成成功
} SvpwmOutput;

// 三相置 50% 中性占空比并清除有效/饱和标志。仅生成命令，不代表 PWM 已关断
void Svpwm_SetNeutral(SvpwmOutput *out);

// 由 αβ 指令生成三相占空比
// voltage_alpha / voltage_beta 为目标相电压 [V]；dc_voltage 为母线电压 [V]，必须 > 0
// duty_min / duty_max 为归一化边界（0~1，且 min < max）
// 返回参数有效且生成成功；失败时输出保持中性
bool Svpwm_Generate(float voltage_alpha, float voltage_beta, float dc_voltage, float duty_min, float duty_max,
                    SvpwmOutput *out);
