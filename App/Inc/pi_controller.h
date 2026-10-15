#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kaw;
    float sample_time_s;
    float out_min;
    float out_max;
    float integrator;
    float error_prev;
} PiController;

/**
 * @brief 初始化带输出限幅和抗积分饱和的离散 PI 控制器。
 * @param[out] pi PI 控制器实例。
 * @param[in] kp 比例增益。
 * @param[in] ki 积分增益，必须非负。
 * @param[in] kaw 反算抗饱和增益，0 表示改用条件积分。
 * @param[in] sample_time_s 控制周期，单位 s。
 * @param[in] out_min 输出下限。
 * @param[in] out_max 输出上限。
 * @return 参数有限且范围合法时返回 true。
 */
bool PiController_Init(PiController *pi,
                       float kp,
                       float ki,
                       float kaw,
                       float sample_time_s,
                       float out_min,
                       float out_max);

/**
 * @brief 复位 PI 历史误差，并将积分状态跟踪到指定输出。
 * @param[in,out] pi PI 控制器实例。
 * @param[in] output 期望的无扰复位输出，内部按输出上下限裁剪。
 */
void PiController_Reset(PiController *pi, float output);

/**
 * @brief 使用梯形积分执行一次 PI 控制和抗饱和更新。
 * @param[in,out] pi 已初始化的 PI 控制器实例。
 * @param[in] error 当前控制误差。
 * @return 限幅后的控制输出；实例或误差无效时返回 0。
 * @note 应按初始化时的 sample_time_s 固定周期调用。
 */
float PiController_Step(PiController *pi, float error);

#ifdef __cplusplus
}
#endif

#endif
