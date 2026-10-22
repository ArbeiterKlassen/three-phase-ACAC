#pragma once

#include <stdint.h>

/*
 * 单半桥 ACDC 的唯一配置入口。
 *
 * 这张表故意保留两个“必须先确认”的开关：
 *   ACDC_ACTIVE_HALFBRIDGE_LEG
 *   ACDC_HARDWARE_COMMISSIONED
 * 前者选实际还存在的半桥，后者是上电门禁。未完成接线、标定和预充验证时，
 * 固件只采样，不会开任何门极。
 */

/* ── 采样 / PWM ────────────────────────────────────────────────────── */
#define ACDC_SAMPLE_TIME_S (50.0e-6f) // 20 kHz 快环周期
#define ACDC_SWITCH_FREQ_HZ (20000.0f)
#define ACDC_LINE_FREQ_HZ (50.0f)
#define ACDC_RMS_WINDOW_SAMPLES (400U) // 50 Hz 下正好一个输入周期
#define ACDC_OUTER_LOOP_DIVIDER (20U)  // 20 kHz / 20 = 1 kHz 母线环

/* ── 目标与安全窗口 ───────────────────────────────────────────────── */
#define ACDC_DC_VOLTAGE_TARGET_V (62.0f)
#define ACDC_DC_VOLTAGE_READY_TOLERANCE_V (3.0f)
#define ACDC_DC_PRECHARGE_MIN_V (25.0f)
#define ACDC_DC_RUN_MIN_V (30.0f)
#define ACDC_DC_MAX_V (66.0f)
#define ACDC_DC_START_MAX_V (64.0f)
#define ACDC_VIN_RMS_MIN_V (10.0f)
#define ACDC_VIN_INSTANT_MAX_V (65.0f)
#define ACDC_IIN_ABS_MAX_A (5.6f)
#define ACDC_IIN_COMMAND_MAX_A (5.5f)
#define ACDC_DC_REFERENCE_SLEW_V_PER_S (60.0f)

/*
 * 5% 低侧最小占空比 = 2.5 us @20 kHz。除了避开最小脉宽，也给 bootstrap
 * 高侧驱动留出刷新窗口。原全桥工程的 2%（1 us）在叠加 1 us 死区后过于
 * 接近极限，本项目不再沿用。
 */
#define ACDC_DUTY_MIN (0.05f)
#define ACDC_DUTY_MAX (0.95f)

/* ── 控制器增益（与 host_tests/sim_input_current.c 一致） ─────────── */
#define ACDC_BUS_PI_KP_A_PER_V (1.0f)
#define ACDC_BUS_PI_KI_A_PER_V_S (1.0f)
#define ACDC_CURRENT_PI_KP_V_PER_A (0.4f)
#define ACDC_CURRENT_PI_KI_V_PER_A_S (50.0f)
#define ACDC_CURRENT_VOLTAGE_LIMIT_V (150.0f)
#define ACDC_NOTCH_ZERO_DAMPING (0.0f)
#define ACDC_NOTCH_POLE_DAMPING (0.0397887357729738f) // 50 / (4π×100)

/* ── 状态确认时间 ─────────────────────────────────────────────────── */
#define ACDC_INVALID_CONFIRM_SAMPLES (2000U)     // 100 ms
#define ACDC_PRECHARGE_CONFIRM_SAMPLES (2000U)   // 100 ms
#define ACDC_UNDERVOLTAGE_CONFIRM_SAMPLES (800U) // 40 ms

/* ── 物理板选择 ───────────────────────────────────────────────────── */
#define ACDC_HALFBRIDGE_LEG_A (0)
#define ACDC_HALFBRIDGE_LEG_B (1)

#ifndef ACDC_ACTIVE_HALFBRIDGE_LEG
#define ACDC_ACTIVE_HALFBRIDGE_LEG ACDC_HALFBRIDGE_LEG_A
#endif

/*
 * 默认必须为 0。完成以下项目后才允许改 1：
 *   1) PC7/PB0（或 PA7/PC6）到实际半桥的 HI/LI 对应关系已用表确认；
 *   2) Iin/Vin/Vcf/Vdc 四路零点和比例已实测；
 *   3) 预充电阻/NTC 或限流电源已验证，零母线合闸不会超过器件量程；
 *   4) 1 us 死区和自举高侧供电已用示波器确认。
 */
#ifndef ACDC_HARDWARE_COMMISSIONED
#define ACDC_HARDWARE_COMMISSIONED (0)
#endif

/* 可选：实验台上由按键启动仍保留；若希望给电自动进入待机启动流程可改 1。 */
#ifndef ACDC_AUTO_START
#define ACDC_AUTO_START (0)
#endif

typedef struct {
    float offset_count; // 零点 [ADC 码]
    float gain;         // 物理量/码
    uint16_t raw_min;   // 合法窗口下限
    uint16_t raw_max;   // 合法窗口上限
} AcdcCalibration;

extern const AcdcCalibration g_acdc_cal_iin;
extern const AcdcCalibration g_acdc_cal_vin;
extern const AcdcCalibration g_acdc_cal_vcf;
extern const AcdcCalibration g_acdc_cal_vdc;
