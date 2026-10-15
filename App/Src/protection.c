/*
 * 软件保护：判据、确认窗口与故障锁存。
 *
 * 与硬件 Break 的分工：BKIN 由定时器直接清 MOE，纳秒级、不经过本文件；
 * 本文件处理需要"多拍判断"的软件判据，动作在一个 ADC 周期（50 us）内。
 *
 * 三类判据的确认策略不同，别一概而论：
 *
 *   立刻判、无去抖  测量配置非法（配置错了就是错了，没什么可滤的）
 *   2 拍去抖        输入过流 / 输入过压 / 输出过压（滤单点噪声）
 *   3 拍去抖        母线过压（Vdc 分压通道在 PWM 启动瞬间有开关振铃）
 *   2000 拍（100 ms）测量无效（专门为躲开上电充电浪涌，见下方宏注释）
 *
 * 全局原则：**active_faults 每拍重建，latched_faults 只增不减**。
 * 清除锁存必须显式调用 Protection_ClearLatched，且要求当拍无活动故障。
 */

#include "protection.h"

#include <math.h>
#include <stddef.h>

#define PROTECTION_INPUT_OC_CONFIRM_SAMPLES (2U)
#define PROTECTION_INPUT_OV_CONFIRM_SAMPLES (2U)
#define PROTECTION_OUTPUT_OV_CONFIRM_SAMPLES (2U)
/*
 * 测量无效原来 2 拍（100 us）即锁存，上电时会被充电浪涌误触发：
 * 母线电容经整流二极管充电，浪涌电流远超 TA 量程（±5.9 A 对应原始码
 * 64..4031），采样被判"量程超限、测量无效"，两点之后系统在母线还没
 * 充起来时就已经 FAULT_LATCH —— 表现为"整流级上电毫无输出"。
 *
 * 该浪涌持续不到 1 ms（τ = R_loop·C_bus 量级），而真正的传感器断线、
 * 掉电或 ADC 故障会一直无效。因此把确认窗口放宽到 100 ms：暂态过后
 * 计数器自动清零，持续故障仍会被抓住。
 */
#define PROTECTION_MEASUREMENT_CONFIRM_SAMPLES (2000U)
/*
 * 直流过压原来单采样即锁存：PWM 启动瞬间 Vdc 分压通道上的开关
 * 噪声/振铃一个采样点就越阈误跳。改为连续3拍（20 kHz 下 150 us）
 * 确认；真实过压摆率为 V/ms 级，150 us 延迟仍可接受。
 */
#define PROTECTION_DC_OV_CONFIRM_SAMPLES (3U)

/*
 * 校验保护阈值本身是否可用：全部有限、正数、且上下限关系合理。
 *
 * 这是"配置体检"，不是"现场判断"。它不通过时会**立即**报
 * PROTECTION_FAULT_MEASUREMENT（无去抖）——因为阈值配错了，
 * 后面所有判据都不可信，没有继续跑的道理。
 */
static bool Protection_ConfigValid(const ProtectionConfig *cfg) {
    return (cfg != NULL) && isfinite(cfg->input_current_abs_max_a) && isfinite(cfg->phase_current_abs_max_a) &&
           isfinite(cfg->input_voltage_abs_max_v) && isfinite(cfg->dc_voltage_min_run_v) &&
           isfinite(cfg->dc_voltage_max_v) && isfinite(cfg->output_voltage_abs_max_v) &&
           (cfg->input_current_abs_max_a > 0.0f) && (cfg->phase_current_abs_max_a > 0.0f) &&
           (cfg->input_voltage_abs_max_v > 0.0f) && (cfg->dc_voltage_min_run_v >= 0.0f) &&
           (cfg->dc_voltage_max_v > cfg->dc_voltage_min_run_v) && (cfg->output_voltage_abs_max_v > 0.0f) &&
           (cfg->stale_limit_samples > 0U);
}

/*
 * 测量值是否可用于判据：valid 标志为真，且所有通道都是有限值。
 * 注意这里**不做量程检查**——量程检查在 measurement.c 的换算环节，
 * 越界会直接让 measurement->valid 变假，走到这里就是"无效"这条路径。
 */
static bool Protection_MeasurementFinite(const Measurements *m) {
    return (m != NULL) && m->valid && isfinite(m->iin) && isfinite(m->vin) && isfinite(m->ia) && isfinite(m->ib) &&
           isfinite(m->ic) && isfinite(m->va) && isfinite(m->vb) && isfinite(m->vc) && isfinite(m->vdc);
}

/* 全部状态清零。NULL 安全 */
void Protection_Init(Protection *protection) {
    if (protection != NULL) *protection = (Protection){0};
}

/*
 * 每拍评估全部软件判据。
 *
 * 顺序（前一条不通过就短路，后面的不再判）：
 *   ① 配置非法        → 立即 MEASUREMENT
 *   ② 测量无效        → 计数，达 2000 拍才 MEASUREMENT
 *   ③ 测量有效        → 查采样序号停滞、各路过流/过压
 *
 * 返回本周期活动故障掩码，并**同时累加进 latched_faults**。
 * latched_faults 不会被本函数清除；protection 为 NULL 时返回 CONTROL 故障。
 *
 * run_expected 只影响母线欠压判据：整流单级从低压被动母线起升时，
 * 母线本来就低，不该按逆变侧的运行下限去报欠压。
 */
uint32_t Protection_Evaluate(Protection *protection, const ProtectionConfig *cfg, const Measurements *measurement,
                             bool run_expected, bool external_fault, bool loop_overrun) {
    uint32_t faults = PROTECTION_FAULT_NONE;

    if (protection == NULL) return PROTECTION_FAULT_CONTROL;
    if (!Protection_ConfigValid(cfg)) {
        protection->measurement_invalid_count = 0U;
        protection->input_oc_count = 0U;
        protection->input_ov_count = 0U;
        protection->output_ov_count = 0U;
        faults |= PROTECTION_FAULT_MEASUREMENT;
    } else if (!Protection_MeasurementFinite(measurement)) {
        protection->input_oc_count = 0U;
        protection->input_ov_count = 0U;
        protection->output_ov_count = 0U;
        if (protection->measurement_invalid_count < PROTECTION_MEASUREMENT_CONFIRM_SAMPLES)
            protection->measurement_invalid_count++;
        if (protection->measurement_invalid_count >= PROTECTION_MEASUREMENT_CONFIRM_SAMPLES)
            faults |= PROTECTION_FAULT_MEASUREMENT;
    } else {
        protection->measurement_invalid_count = 0U;
        /* 序号不变表示控制循环重复使用旧帧，连续达到阈值后报采样陈旧。 */
        if (!protection->sequence_seen) {
            protection->sequence_seen = true;
            protection->last_sequence = measurement->sequence;
            protection->stale_count = 0U;
        } else if (measurement->sequence == protection->last_sequence) {
            if (protection->stale_count < UINT16_MAX) protection->stale_count++;
        } else {
            protection->last_sequence = measurement->sequence;
            protection->stale_count = 0U;
        }

        if (protection->stale_count >= cfg->stale_limit_samples) faults |= PROTECTION_FAULT_SAMPLE_STALE;
        if (fabsf(measurement->iin) > cfg->input_current_abs_max_a) {
            if (protection->input_oc_count < PROTECTION_INPUT_OC_CONFIRM_SAMPLES) protection->input_oc_count++;
            if (protection->input_oc_count >= PROTECTION_INPUT_OC_CONFIRM_SAMPLES)
                faults |= PROTECTION_FAULT_INPUT_OVERCURRENT;
        } else {
            protection->input_oc_count = 0U;
        }
        if ((fabsf(measurement->ia) > cfg->phase_current_abs_max_a) ||
            (fabsf(measurement->ib) > cfg->phase_current_abs_max_a) ||
            (fabsf(measurement->ic) > cfg->phase_current_abs_max_a)) {
            faults |= PROTECTION_FAULT_PHASE_OVERCURRENT;
        }
        if (fabsf(measurement->vin) > cfg->input_voltage_abs_max_v) {
            if (protection->input_ov_count < PROTECTION_INPUT_OV_CONFIRM_SAMPLES) protection->input_ov_count++;
            if (protection->input_ov_count >= PROTECTION_INPUT_OV_CONFIRM_SAMPLES)
                faults |= PROTECTION_FAULT_INPUT_OVERVOLTAGE;
        } else {
            protection->input_ov_count = 0U;
        }
        if (measurement->vdc > cfg->dc_voltage_max_v) {
            if (protection->dc_ov_count < PROTECTION_DC_OV_CONFIRM_SAMPLES) protection->dc_ov_count++;
            if (protection->dc_ov_count >= PROTECTION_DC_OV_CONFIRM_SAMPLES) faults |= PROTECTION_FAULT_DC_OVERVOLTAGE;
        } else {
            protection->dc_ov_count = 0U;
        }
        if (run_expected && (measurement->vdc < cfg->dc_voltage_min_run_v)) faults |= PROTECTION_FAULT_DC_UNDERVOLTAGE;
        if ((fabsf(measurement->va) > cfg->output_voltage_abs_max_v) ||
            (fabsf(measurement->vb) > cfg->output_voltage_abs_max_v) ||
            (fabsf(measurement->vc) > cfg->output_voltage_abs_max_v)) {
            if (protection->output_ov_count < PROTECTION_OUTPUT_OV_CONFIRM_SAMPLES) protection->output_ov_count++;
            if (protection->output_ov_count >= PROTECTION_OUTPUT_OV_CONFIRM_SAMPLES)
                faults |= PROTECTION_FAULT_OUTPUT_OVERVOLTAGE;
        } else {
            /* A normal frame breaks the consecutive overvoltage sequence. */
            protection->output_ov_count = 0U;
        }
    }

    if (external_fault) faults |= PROTECTION_FAULT_EXTERNAL;
    if (loop_overrun) faults |= PROTECTION_FAULT_LOOP_OVERRUN;

    /* active 反映本拍状态，latched 按位保留历史，直到显式且安全地复位。 */
    protection->active_faults = faults;
    protection->latched_faults |= faults;
    return faults;
}

/*
 * 外部强制置位故障（板级故障输入、启动阶段的自检失败等）。
 *
 * 本函数不带并发保护，**在 ISR 里请改用 App_TripFromISR**——
 * 那个版本用短临界区做原子合并，避免与主循环互相覆盖故障位。
 */
void Protection_ForceFault(Protection *protection, uint32_t faults) {
    if (protection != NULL) {
        protection->active_faults |= faults;
        protection->latched_faults |= faults;
    }
}

/*
 * 清除锁存故障。
 *
 * 有一个刻意设置的"防误解锁"：**只有当拍没有任何活动故障时才允许清除**。
 * 否则会出现"清掉 → 下一拍立刻重新锁存"的抖动，而真正的故障源还在。
 * 各去抖计数器也一并清零，让清除后的重新判定从干净状态开始。
 *
 * 返回清除成功。
 */
bool Protection_ClearLatched(Protection *protection) {
    if ((protection == NULL) || (protection->active_faults != PROTECTION_FAULT_NONE)) return false;

    protection->latched_faults = PROTECTION_FAULT_NONE;
    protection->measurement_invalid_count = 0U;
    protection->input_oc_count = 0U;
    protection->input_ov_count = 0U;
    protection->output_ov_count = 0U;
    protection->dc_ov_count = 0U;
    return true;
}
