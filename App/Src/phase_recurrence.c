/*
 * 无查表正交相位发生器。
 *
 * 用旋转矩阵递推 (sinθ, cosθ)：
 *     sin' = sin·cosΔ + cos·sinΔ
 *     cos' = cos·cosΔ − sin·sinΔ
 * 每拍两次乘加，20 kHz 快环里完全没有 sinf/cosf 调用——三角函数只出现在
 * Init / SetFrequency / SetAngle 这些低频路径上。
 *
 * 代价是浮点误差累积：模长会缓慢漂移，所以每 256 拍按 1/√(sin²+cos²)
 * 归一化一次。归一化本身要用一次 sqrtf，摊到 256 拍上可以忽略。
 */

#include "phase_recurrence.h"

#include <math.h>

#define PHASE_DEFAULT_RENORM_PERIOD (256U)

/*
 * 由频率算出每拍的 sin/cos 增量。
 *
 * 频率可为负（相序反向）。必须满足奈奎斯特约束 |f|·Ts < 0.5，
 * 否则相位每拍转过半圈以上，递推出来的是混叠的假频率。
 *
 * 返回参数合法且算出的增量有限。
 */
static bool PhaseRecurrence_UpdateStep(PhaseRecurrence *phase, float frequency_hz) {
    float angle_step;

    if ((phase == NULL) || !isfinite(frequency_hz) || !isfinite(phase->sample_time_s) ||
        (phase->sample_time_s <= 0.0f) || (fabsf(frequency_hz) * phase->sample_time_s >= 0.5f)) {
        return false;
    }

    angle_step = 2.0f * APP_PI_F * frequency_hz * phase->sample_time_s;
    phase->sin_step = sinf(angle_step);
    phase->cos_step = cosf(angle_step);
    return isfinite(phase->sin_step) && isfinite(phase->cos_step);
}

/*
 * 初始化：设定采样周期、算出步长、把相位摆到初始角度。
 * 归一化周期用默认值 256 拍。返回全部成功。
 */
bool PhaseRecurrence_Init(PhaseRecurrence *phase, float frequency_hz, float sample_time_s, float initial_angle_rad) {
    if ((phase == NULL) || !isfinite(sample_time_s) || (sample_time_s <= 0.0f)) return false;

    *phase = (PhaseRecurrence){0};
    phase->sample_time_s = sample_time_s;
    phase->renorm_period = PHASE_DEFAULT_RENORM_PERIOD;

    return PhaseRecurrence_UpdateStep(phase, frequency_hz) && PhaseRecurrence_SetAngle(phase, initial_angle_rad);
}

/*
 * 只改频率，不动当前相角——这样运行中切频（30↔60 Hz）不会让输出跳相位。
 * 返回新步长是否可用。
 */
bool PhaseRecurrence_SetFrequency(PhaseRecurrence *phase, float frequency_hz) {
    return PhaseRecurrence_UpdateStep(phase, frequency_hz);
}

/*
 * 把相位直接摆到指定角度（用一次 sinf/cosf），并把归一化计数清零。
 * 用于重新同步或软启动对齐。
 */
bool PhaseRecurrence_SetAngle(PhaseRecurrence *phase, float angle_rad) {
    if ((phase == NULL) || !isfinite(angle_rad)) return false;

    phase->sin_value = sinf(angle_rad);
    phase->cos_value = cosf(angle_rad);
    phase->renorm_count = 0U;
    return isfinite(phase->sin_value) && isfinite(phase->cos_value);
}

/*
 * 相位归零（sin=0, cos=1），保留频率与采样周期。NULL 安全。
 */
void PhaseRecurrence_Reset(PhaseRecurrence *phase) {
    if (phase != NULL) {
        phase->sin_value = 0.0f;
        phase->cos_value = 1.0f;
        phase->renorm_count = 0U;
    }
}

/*
 * 推进一个采样周期。
 *
 * **重要**：即使本拍不输出（门极关断、状态机未放行），也必须调用它。
 * 否则相位会停在原地，重新使能的瞬间输出会从错误的相位开始，
 * 表现为波形上突然的一个台阶。
 */
void PhaseRecurrence_Step(PhaseRecurrence *phase) {
    float sin_next;
    float cos_next;
    float norm2;
    float gain;

    if (phase == NULL) return;

    /* 每拍仅乘加递推；不在快环内查正弦表或调用三角函数。 */
    sin_next = phase->sin_value * phase->cos_step + phase->cos_value * phase->sin_step;
    cos_next = phase->cos_value * phase->cos_step - phase->sin_value * phase->sin_step;
    phase->sin_value = sin_next;
    phase->cos_value = cos_next;

    phase->renorm_count++;
    if (phase->renorm_count < phase->renorm_period) return;

    phase->renorm_count = 0U;
    norm2 = sin_next * sin_next + cos_next * cos_next;
    if (isfinite(norm2) && (norm2 > 0.25f)) {
        gain = 1.0f / sqrtf(norm2);
        phase->sin_value *= gain;
        phase->cos_value *= gain;
    } else {
        PhaseRecurrence_Reset(phase);
    }
}

/*
 * 由当前相位算出正相序三相正弦量：
 *     A = peak·sinθ
 *     B = peak·sin(θ−120°) = −½A − (√3/2)·peak·cosθ
 *     C = peak·sin(θ+120°) = −½A + (√3/2)·peak·cosθ
 * 用和角公式直接由 (sinθ, cosθ) 展开，避免再算两次三角函数。
 * 输入无效时返回全零。
 */
ThreePhase PhaseRecurrence_GetThreePhase(const PhaseRecurrence *phase, float peak) {
    ThreePhase out = {0};
    float half;
    float cross;

    if ((phase == NULL) || !isfinite(peak)) return out;

    half = 0.5f * peak * phase->sin_value;
    cross = 0.5f * APP_SQRT3_F * peak * phase->cos_value;
    out.a = peak * phase->sin_value;
    out.b = -half - cross;
    out.c = -half + cross;
    return out;
}
