#pragma once

/*
 * 集中式板级参数与"上电安全锁"。
 *
 * 所有标定、限幅、保护阈值都集中在 app_config.c 的 g_board_power_config 里，
 * 不散落在各模块。改任何控制参数前先读那里的来历注释——每个数都写了理由。
 *
 * 上电安全锁：BoardPower_ValidateModeConfig 会按当前模式逐项检查实际用到的
 * 标定与参数。任一不通过，状态机就不允许进入 RUN。**config_valid 之类的
 * 总开关不能绕过这些检查**，这是刻意设计的，别改。
 */

#include "inverter_control.h"
#include "power_state.h"
#include "protection.h"
#include "rectifier_control.h"

/* 母线目标与一致性窗口。
 * 默认 62 V：赛题输入上限 41 V（峰值 58 V）时升压调制比 m = 58/62 = 0.935，
 * 仍低于 0.96 占空比限幅；若用 60 V 母线，41 V 输入会撞上限幅失控工况。 */
#define APP_INVERTER_DC_VOLTAGE_DEFAULT_V   (62.0f)
#define APP_INVERTER_DC_VOLTAGE_MIN_V       (30.0f)   // SET 允许下限 [V]
#define APP_INVERTER_DC_VOLTAGE_MAX_V       (65.0f)   // SET 允许上限 [V]
#define APP_INVERTER_DC_MATCH_TOLERANCE_V   (3.0f)    // 设定与实测一致性窗口 [V]
#define APP_INVERTER_DC_OVERVOLTAGE_V       (66.0f)   // 母线过压阈值 [V]
#define APP_INVERTER_VOLTAGE_MARGIN         (0.95f)   // 参考限幅留 5% 余量

/* 输出线电压按母线设定值的 32/60 比例推导（旧的"比例跟随"模式，保留兼容）。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_RATIO  (32.0f / 60.0f)

/* 赛题要求 Uo = 32 V ± 0.1 V：输出线电压参考与母线参考**解耦**，独立设定。
 * 解耦后母线升到 62 V 不会抬高输出目标；28~34 V 为现场微调窗口。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V (32.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MIN_V     (28.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MAX_V     (34.0f)

/* 板级配置总表。七大安全标志是"这块板子校准过了"的集中声明，
 * 但它们只是入场券——每项具体参数仍会被逐条校验。 */
typedef struct control_config_t
{
    bool config_valid;                 // 总标志
    bool adc_calibrated;               // 三相通道标定完成
    bool rectifier_adc_calibrated;     // 整流通道标定完成
    bool gate_driver_pins_valid;       // 驱动引脚确认
    bool power_stage_valid;            // 功率级确认
    bool deadtime_valid;               // 死区已实测确认
    bool protection_thresholds_valid;  // 保护阈值确认
    float output_line_voltage_rms;     // 输出线电压参考 [Vrms]
    float dc_voltage_ref;              // 母线目标 [V]
    uint16_t vin_diagnostic_tolerance_counts;  // 两次 Vin 采样允许差 [ADC 码]
    MeasurementCalibration measurement;  // 七路通道标定
    InverterControlConfig inverter;      // 逆变级参数
    RectifierControlConfig rectifier;    // 整流级参数
    ProtectionConfig protection;         // 保护阈值
    PowerStateConfig state;              // 状态机确认拍数
} control_config_t;

typedef control_config_t BoardPowerConfig;

extern const BoardPowerConfig g_board_power_config;

// 取当前板级配置。GCC 下默认实现是弱符号，板级文件可提供强定义覆盖
// （PE/COFF 上退化为强定义，见 app_config.c 顶部说明）
const BoardPowerConfig *BoardPower_GetConfig(void);

// 按运行模式校验：MODE0 全级、MODE1 仅逆变、MODE2 仅整流
// 只检查该模式实际用到的标定与参数，未用到的通道不会误伤
// 对应模式所需项全部合法才返回 true
bool BoardPower_ValidateModeConfig(const control_config_t *cfg, app_mode_t mode);

// 按 MODE0_FULL 全量校验，保留旧调用接口
bool BoardPower_ValidateConfig(const control_config_t *cfg);

// 校验 BoardPower_GetConfig() 返回的当前配置
bool BoardPower_ConfigValid(void);
