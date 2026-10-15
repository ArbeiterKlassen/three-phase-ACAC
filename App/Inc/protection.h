#pragma once

/*
 * 软件保护：判据、确认窗口与故障锁存。
 *
 * 硬件 Break 由定时器直接清 MOE，不经这里；本模块负责软件侧判据
 * （测量有效性、采样新鲜度、电流电压阈值）以及"故障后必须人工确认"
 * 的锁存语义。
 */

#include "app_types.h"

/* 故障位掩码，可按位或组合。OLED 显示的是最低有效位的序号。 */
enum {
    PROTECTION_FAULT_NONE = 0UL,
    PROTECTION_FAULT_MEASUREMENT = 1UL << 0,        // 解码失败/标定超限，持续 100 ms 才锁存
    PROTECTION_FAULT_SAMPLE_STALE = 1UL << 1,       // 采样序号停滞，控制环在吃旧帧
    PROTECTION_FAULT_INPUT_OVERCURRENT = 1UL << 2,  // 输入电流绝对值超阈值
    PROTECTION_FAULT_PHASE_OVERCURRENT = 1UL << 3,  // 任一相电流绝对值超阈值
    PROTECTION_FAULT_INPUT_OVERVOLTAGE = 1UL << 4,  // 输入电压绝对值超阈值
    PROTECTION_FAULT_DC_OVERVOLTAGE = 1UL << 5,     // 母线过压，连续 3 拍确认
    PROTECTION_FAULT_DC_UNDERVOLTAGE = 1UL << 6,    // 运行中母线欠压
    PROTECTION_FAULT_OUTPUT_OVERVOLTAGE = 1UL << 7, // 重构相电压超阈值
    PROTECTION_FAULT_EXTERNAL = 1UL << 8,           // BKIN / 驱动 / 板级故障
    PROTECTION_FAULT_LOOP_OVERRUN = 1UL << 9,       // 20 kHz 快环超过 50 us
    PROTECTION_FAULT_CONTROL = 1UL << 10,           // 控制器参数或数值计算失败
    PROTECTION_FAULT_DC_MISMATCH = 1UL << 11        // 母线设定与实测差异过大
};

/* 判据阈值。全部来自 app_config.c，改动前先读那里的来历注释。 */
typedef struct {
    float input_current_abs_max_a;  // 输入电流绝对值上限 [A]
    float phase_current_abs_max_a;  // 相电流绝对值上限 [A]
    float input_voltage_abs_max_v;  // 输入电压绝对值上限 [V]
    float dc_voltage_min_run_v;     // 运行中母线欠压阈值 [V]
    float dc_voltage_max_v;         // 母线过压阈值 [V]
    float output_voltage_abs_max_v; // 重构相电压绝对值上限 [V]
    uint16_t stale_limit_samples;   // 允许重复使用同一帧的拍数，超过判陈旧
} ProtectionConfig;

/* 保护实例状态。由 Protection_Init 清零，之后每个快环周期更新。 */
typedef struct {
    uint32_t active_faults;             // 本周期活动故障掩码
    uint32_t latched_faults;            // 累加的锁存故障，需 Protection_ClearLatched 清除
    uint32_t last_sequence;             // 上一帧采样序号，用于检测停滞
    uint16_t stale_count;               // 连续重复帧计数
    uint16_t measurement_invalid_count; // 连续无效帧计数，窗口 2000 拍（100 ms）
    uint8_t input_oc_count;             // 输入过流去抖计数，2 拍确认
    uint8_t input_ov_count;             // 输入过压去抖计数，2 拍确认
    uint8_t output_ov_count;            // 输出过压去抖计数，2 拍确认
    uint8_t dc_ov_count;                // 母线过压去抖计数，3 拍确认
    bool sequence_seen;                 // 是否已收到过第一帧
} Protection;

// 清零活动故障、锁存故障与采样序号监视状态。NULL 安全
void Protection_Init(Protection *protection);

// 每拍评估全部软件判据：测量有效性、采样新鲜度、电流电压阈值、外部故障
// 本周期故障写入 active_faults 并累加到 latched_faults；latched_faults 不会被自动清除
// 返回本周期活动故障掩码；protection 为 NULL 时返回 PROTECTION_FAULT_CONTROL
uint32_t Protection_Evaluate(Protection *protection, const ProtectionConfig *cfg, const Measurements *measurement,
                             bool run_expected,   // 为真才检查母线欠压
                             bool external_fault, // BKIN / 驱动故障输入
                             bool loop_overrun);  // 快环超时输入

// 强制设置活动故障并按位锁存。本身不带并发保护，ISR 里请用 App_TripFromISR
void Protection_ForceFault(Protection *protection, uint32_t faults);

// 所有活动故障都消失后清除锁存。active_faults 为 0 且清除成功才返回 true
bool Protection_ClearLatched(Protection *protection);
