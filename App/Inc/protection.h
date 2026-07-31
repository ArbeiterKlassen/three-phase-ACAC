#ifndef PROTECTION_H
#define PROTECTION_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    PROTECTION_FAULT_NONE             = 0UL,
    PROTECTION_FAULT_MEASUREMENT      = 1UL << 0,
    PROTECTION_FAULT_SAMPLE_STALE     = 1UL << 1,
    PROTECTION_FAULT_INPUT_OVERCURRENT = 1UL << 2,
    PROTECTION_FAULT_PHASE_OVERCURRENT = 1UL << 3,
    PROTECTION_FAULT_INPUT_OVERVOLTAGE = 1UL << 4,
    PROTECTION_FAULT_DC_OVERVOLTAGE    = 1UL << 5,
    PROTECTION_FAULT_DC_UNDERVOLTAGE   = 1UL << 6,
    PROTECTION_FAULT_OUTPUT_OVERVOLTAGE = 1UL << 7,
    PROTECTION_FAULT_EXTERNAL          = 1UL << 8,
    PROTECTION_FAULT_LOOP_OVERRUN      = 1UL << 9,
    PROTECTION_FAULT_CONTROL           = 1UL << 10,
    PROTECTION_FAULT_DC_MISMATCH       = 1UL << 11
};

typedef struct
{
    float input_current_abs_max_a;
    float phase_current_abs_max_a;
    float input_voltage_abs_max_v;
    float dc_voltage_min_run_v;
    float dc_voltage_max_v;
    float output_voltage_abs_max_v;
    uint16_t stale_limit_samples;
} ProtectionConfig;

typedef struct
{
    uint32_t active_faults;
    uint32_t latched_faults;
    uint32_t last_sequence;
    uint16_t stale_count;
    bool sequence_seen;
} Protection;

/**
 * @brief 清零活动故障、锁存故障和采样序号监视状态。
 * @param[out] protection 保护实例；NULL 将被忽略。
 */
void Protection_Init(Protection *protection);

/**
 * @brief 检查测量有效性、采样新鲜度、电流电压阈值及外部故障。
 * @param[in,out] protection 保护实例，本周期故障覆盖 active_faults 并累加到
 *                latched_faults。
 * @param[in] cfg 保护阈值和陈旧采样允许拍数。
 * @param[in] measurement 当前物理量和采样序号。
 * @param[in] run_expected true 时启用直流母线欠压检查。
 * @param[in] external_fault 外部硬件或板级故障输入。
 * @param[in] loop_overrun 快环超时输入。
 * @return 本周期活动故障掩码；protection 为 NULL 时返回控制故障。
 * @warning 本函数不会自动清除 latched_faults。
 */
uint32_t Protection_Evaluate(Protection *protection,
                             const ProtectionConfig *cfg,
                             const Measurements *measurement,
                             bool run_expected,
                             bool external_fault,
                             bool loop_overrun);

/**
 * @brief 强制设置指定活动故障并将其锁存。
 * @param[in,out] protection 保护实例；NULL 将被忽略。
 * @param[in] faults 要按位合并的故障掩码。
 * @note 本函数本身不提供并发保护；ISR 应通过 App_TripFromISR 提交故障。
 */
void Protection_ForceFault(Protection *protection, uint32_t faults);

/**
 * @brief 在所有活动故障消失后清除锁存故障。
 * @param[in,out] protection 保护实例。
 * @return active_faults 为 0 且成功清除时返回 true，否则返回 false。
 */
bool Protection_ClearLatched(Protection *protection);

#ifdef __cplusplus
}
#endif

#endif
