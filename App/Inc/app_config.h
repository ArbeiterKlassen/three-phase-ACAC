#pragma once

/*
 * 集中式板级参数与"上电安全锁"（三相逆变工程）。
 *
 * 所有标定、限幅、保护阈值都集中在 app_config.c 的 g_board_power_config 里，
 * 不散落在各模块。改任何控制参数前先读那里的来历注释——每个数都写了理由。
 *
 * 上电安全锁：BoardPower_ValidateConfig 会逐项检查标定与参数。任一不通过，
 * 状态机就不允许进入 RUN。**config_valid 之类的总开关不能绕过这些检查**，
 * 这是刻意设计的，别改。
 */

#include "inverter_control.h"
#include "power_state.h"
#include "protection.h"

/* 直流母线目标与允许窗口（本工程的母线由**外部直流电源**提供）。 */
#define APP_INVERTER_DC_VOLTAGE_DEFAULT_V (62.0f)
#define APP_INVERTER_DC_VOLTAGE_MIN_V (30.0f)    // SET 允许下限 [V]
#define APP_INVERTER_DC_VOLTAGE_MAX_V (65.0f)    // SET 允许上限 [V]
#define APP_INVERTER_DC_MATCH_TOLERANCE_V (3.0f) // 设定与实测一致性窗口 [V]
#define APP_INVERTER_DC_OVERVOLTAGE_V (66.0f)    // 母线过压阈值 [V]
#define APP_INVERTER_VOLTAGE_MARGIN (0.95f)      // 参考限幅留 5% 余量

/* 输出线电压按母线设定值的 32/60 比例推导（旧"比例跟随"路径，保留兼容）。
 * 仅在 inverter_vrms_ref ≤ 0 时作为回退值使用，见 app_control.c。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_RATIO (32.0f / 60.0f)

/* 赛题要求 Uo = 32 V ± 0.1 V：输出线电压参考与母线参考**解耦**，独立设定。
 * 解耦后母线升到 62 V 不会抬高输出目标；28~34 V 为现场微调窗口。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V (32.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MIN_V (28.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MAX_V (34.0f)

/* 板级配置总表。安全标志是"这块板子确认过了"的集中声明，
 * 但它们只是入场券——每项具体参数仍会被逐条校验。 */
typedef struct control_config_t {
    bool config_valid;                  // 总标志
    bool adc_calibrated;                // 采样通道标定完成
    bool gate_driver_pins_valid;        // 驱动引脚确认
    bool power_stage_valid;             // 功率级确认
    bool deadtime_valid;                // 死区已实测确认
    bool protection_thresholds_valid;   // 保护阈值确认
    float output_line_voltage_rms;      // 输出线电压参考 [Vrms]
    float dc_voltage_ref;               // 母线目标 [V]
    MeasurementCalibration measurement; // 通道标定
    InverterControlConfig inverter;     // 逆变参数
    ProtectionConfig protection;        // 保护阈值
    PowerStateConfig state;             // 状态机确认拍数
} control_config_t;

typedef control_config_t BoardPowerConfig;

extern const BoardPowerConfig g_board_power_config;

// 取当前板级配置。GCC 下默认实现是弱符号，板级文件可提供强定义覆盖
// （PE/COFF 上退化为强定义，见 app_config.c 顶部说明）
const BoardPowerConfig *BoardPower_GetConfig(void);

// 逐项校验标定、逆变参数、保护阈值与状态机拍数
// 全部合法才返回 true；false 表示这份配置不允许进入 RUN
bool BoardPower_ValidateConfig(const control_config_t *cfg);

// 校验 BoardPower_GetConfig() 返回的当前配置
bool BoardPower_ConfigValid(void);
