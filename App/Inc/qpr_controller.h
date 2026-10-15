#ifndef QPR_CONTROLLER_H
#define QPR_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float b0;
    float b2;
    float a1;
    float a2;
    float out_min;
    float out_max;
    float e1;
    float e2;
    float y1;
    float y2;
} QprController;

/**
 * @brief 初始化 Tustin 离散的准比例谐振控制器。
 * @param[out] qpr QPR 控制器实例。
 * @param[in] kp 比例增益。
 * @param[in] kr 谐振增益，必须非负。
 * @param[in] resonance_rad_s 谐振中心角频率，单位 rad/s。
 * @param[in] bandwidth_rad_s 谐振带宽，单位 rad/s。
 * @param[in] sample_time_s 控制周期，单位 s。
 * @param[in] out_min 输出下限。
 * @param[in] out_max 输出上限。
 * @return 参数及计算得到的离散系数有效时返回 true。
 */
bool QprController_Init(QprController *qpr,
                        float kp,
                        float kr,
                        float resonance_rad_s,
                        float bandwidth_rad_s,
                        float sample_time_s,
                        float out_min,
                        float out_max);

/**
 * @brief 清零 QPR 的误差历史和谐振支路状态。
 * @param[in,out] qpr QPR 控制器实例；NULL 将被忽略。
 */
void QprController_Reset(QprController *qpr);

/**
 * @brief 执行一次比例加准谐振控制，并在自身输出限幅时跟踪实际输出。
 * @param[in,out] qpr 已初始化的 QPR 控制器实例。
 * @param[in] error 当前控制误差。
 * @return 限幅后的控制输出；实例或误差无效时返回 0。
 * @note 应按初始化时的 sample_time_s 固定周期调用。
 */
float QprController_Step(QprController *qpr, float error);

/**
 * @brief 将控制器谐振状态跟踪到下游最终实际施加的输出。
 * @param[in,out] qpr QPR 控制器实例。
 * @param[in] error 生成本周期输出时使用的误差。
 * @param[in] applied_output 经后级限幅或调制后实际施加的控制量。
 * @note 用于补偿 SVPWM 等控制器外部限幅，避免谐振状态继续积累。
 */
void QprController_TrackOutput(QprController *qpr,
                               float error,
                               float applied_output);

#ifdef __cplusplus
}
#endif

#endif
