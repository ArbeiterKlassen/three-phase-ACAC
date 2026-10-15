#pragma once

/*
 * 带输出限幅与抗积分饱和的离散 PI 控制器（梯形积分）。
 *
 * 抗饱和两种模式，由 kaw 选择：
 *   kaw > 0 —— 反算法，用"实际输出 − 限幅前输出"回灌积分器
 *   kaw = 0 —— 条件积分，只在输出未限幅时累加积分
 * 整流级母线外环用这个，输出是输入电流幅值指令 [A]。
 */

#include <stdbool.h>

/* PI 实例状态。由 PiController_Init 初始化。 */
typedef struct
{
    float kp;             // 比例增益
    float ki;             // 积分增益
    float kaw;            // 抗饱和增益；0 表示改用条件积分
    float sample_time_s;  // 控制周期 [s]
    float out_min;        // 输出下限
    float out_max;        // 输出上限
    float integrator;     // 积分器状态
    float error_prev;     // 上一拍误差，用于梯形积分
} PiController;

// 初始化。ki 必须非负；kaw=0 时走条件积分
// 返回参数有限且范围合法
bool PiController_Init(PiController *pi,
                       float kp,
                       float ki,
                       float kaw,
                       float sample_time_s,
                       float out_min,
                       float out_max);

// 复位历史误差，并把积分状态无扰地跟踪到 output（内部按上下限裁剪）
void PiController_Reset(PiController *pi, float output);

// 执行一次 PI。必须按初始化时的 sample_time_s 固定周期调用
// 返回限幅后的输出；实例或误差无效时返回 0
float PiController_Step(PiController *pi, float error);
