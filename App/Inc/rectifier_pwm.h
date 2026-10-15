#pragma once

/*
 * 单极性 SPWM：把归一化调制度变成单相全桥两个桥臂的占空比。
 *
 * 两桥臂反相调制，占空比绕 50% 对称：
 *     duty_a = 0.5 + 0.5 × m
 *     duty_b = 0.5 − 0.5 × m
 * 这样交流侧得到的是双极性电压，而每个桥臂各自只在 0~1 之间摆动。
 */

#include <stdbool.h>

/* 一次调制的结果。duty_a / duty_b 直接写进 TIM8 的 CCR。 */
typedef struct {
    float duty_a;     // 桥臂 A 归一化占空比 [0~1]
    float duty_b;     // 桥臂 B 归一化占空比 [0~1]
    float modulation; // 实际施加的调制度（限幅后的值，供抗饱和回灌）
    bool saturated;   // 是否被 duty_min/duty_max 限幅
    bool valid;       // 生成成功
} RectifierPwmOutput;

// 两个桥臂置 50% 中性占空比并标记无效。仅生成命令，不代表硬件驱动已关断
void RectifierPwm_SetNeutral(RectifierPwmOutput *out);

// 由调制度生成两桥臂占空比
// modulation 理论范围 −1~1，超出时按占空比限幅并置 saturated
// duty_min / duty_max 为归一化边界（0~0.5 / 0.5~1）
// 返回参数有效且生成成功；失败时输出保持中性
bool RectifierPwm_Generate(float modulation, float duty_min, float duty_max, RectifierPwmOutput *out);
