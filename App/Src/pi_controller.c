/*
 * 离散 PI 控制器：梯形积分 + 双重抗饱和。
 *
 * 梯形积分（把本拍与上拍误差取平均）比矩形积分在固定采样周期下的
 * 相位滞后更小，代价是多存一个 error_prev。
 *
 * 抗饱和两种模式由 kaw 选择：
 *   kaw > 0 —— 反算法，把"限幅前 − 限幅后"的差值乘增益回灌积分器
 *   kaw = 0 —— 条件积分，只在积分方向会加重饱和时冻结
 * 之所以两种都留着：母线外环在不同工况下两种都不够好，需要可切换。
 */

#include "pi_controller.h"

#include <math.h>
#include <stddef.h>

/* 把 value 限制到 [low, high] */
static float PiController_Clamp(float value, float low, float high) {
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

/*
 * 初始化。ki 与 kaw 必须非负，sample_time_s 必须为正，out_min < out_max。
 * 内部状态（integrator / error_prev）清零，需配合 Reset 做无扰启动。
 * 返回参数合法。
 */
bool PiController_Init(PiController *pi, float kp, float ki, float kaw, float sample_time_s, float out_min,
                       float out_max) {
    if ((pi == NULL) || !isfinite(kp) || !isfinite(ki) || !isfinite(kaw) || !isfinite(sample_time_s) ||
        !isfinite(out_min) || !isfinite(out_max) || (ki < 0.0f) || (kaw < 0.0f) || (sample_time_s <= 0.0f) ||
        (out_min >= out_max)) {
        return false;
    }

    *pi = (PiController){0};
    pi->kp = kp;
    pi->ki = ki;
    pi->kaw = kaw;
    pi->sample_time_s = sample_time_s;
    pi->out_min = out_min;
    pi->out_max = out_max;
    return true;
}

/*
 * 无扰复位：把积分器设成 output（裁剪到输出范围），并清掉历史误差。
 *
 * 如果积分器从 0 起步而实际输出需要非零，会使能瞬间产生一个冲击；
 * 传入当前实际输出即可平稳接管。NULL 安全。
 */
void PiController_Reset(PiController *pi, float output) {
    if (pi != NULL) {
        pi->integrator = PiController_Clamp(output, pi->out_min, pi->out_max);
        pi->error_prev = 0.0f;
    }
}

/*
 * 执行一次 PI。必须按初始化时的 sample_time_s 固定周期调用。
 *
 * 输出限幅在抗饱和之前——先算出"本来想输出多少"，再和限幅后比较，
 * 差值才是真正的饱和量。顺序反了抗饱和就失效。
 *
 * 返回限幅后的输出；实例或误差无效时返回 0。
 */
float PiController_Step(PiController *pi, float error) {
    float integral_old;
    float unsaturated;
    float output;

    if ((pi == NULL) || !isfinite(error)) return 0.0f;

    /* 积分项用梯形法离散，降低固定采样周期下的相位与幅值误差。 */
    integral_old = pi->integrator;
    pi->integrator += 0.5f * pi->ki * pi->sample_time_s * (error + pi->error_prev);
    unsaturated = pi->kp * error + pi->integrator;
    output = PiController_Clamp(unsaturated, pi->out_min, pi->out_max);

    /* 优先使用反算抗饱和；kaw=0 时冻结会加重饱和的积分方向。 */
    if (output != unsaturated) {
        if (pi->kaw > 0.0f) pi->integrator += pi->kaw * pi->sample_time_s * (output - unsaturated);
        else if (((unsaturated > pi->out_max) && (error > 0.0f)) || ((unsaturated < pi->out_min) && (error < 0.0f)))
            pi->integrator = integral_old;
    }

    pi->integrator = PiController_Clamp(pi->integrator, pi->out_min, pi->out_max);
    pi->error_prev = error;
    return output;
}
