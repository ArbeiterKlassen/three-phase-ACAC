#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "inverter_control.h"
#include "power_state.h"
#include "protection.h"
#include "rectifier_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 模块化三相逆变测试允许的用户母线设定及实测一致性窗口。
 * 默认母线 62V：赛题输入上限 41V（峰值 58V）时升压调制比 m=58/62=0.935
 * 仍低于 0.96 占空比限幅；若用 60V 母线，41V 输入会撞上 clip-pump
 * 失控工况（即 47.1V 锁死同一机理）。 */
#define APP_INVERTER_DC_VOLTAGE_DEFAULT_V   (62.0f)
#define APP_INVERTER_DC_VOLTAGE_MIN_V       (30.0f)
#define APP_INVERTER_DC_VOLTAGE_MAX_V       (65.0f)
#define APP_INVERTER_DC_MATCH_TOLERANCE_V   (3.0f)
#define APP_INVERTER_DC_OVERVOLTAGE_V       (66.0f)
#define APP_INVERTER_VOLTAGE_MARGIN         (0.95f)
/* 输出线电压RMS按Vdc设定值的32/60倍自动生成（比例跟随模式保留兼容）。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_RATIO  (32.0f / 60.0f)
/* 赛题 Uo=32V±0.1V：输出线电压参考与母线参考解耦，独立设定。
 * 解耦后母线升到 62V 不会抬高输出目标；28~34V 为允许的现场微调窗口。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V (32.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MIN_V     (28.0f)
#define APP_INVERTER_OUTPUT_LINE_RMS_MAX_V     (34.0f)

typedef struct control_config_t
{
    bool config_valid;
    bool adc_calibrated;
    bool rectifier_adc_calibrated;
    bool gate_driver_pins_valid;
    bool power_stage_valid;
    bool deadtime_valid;
    bool protection_thresholds_valid;
    float output_line_voltage_rms;
    float dc_voltage_ref;
    uint16_t vin_diagnostic_tolerance_counts;
    MeasurementCalibration measurement;
    InverterControlConfig inverter;
    RectifierControlConfig rectifier;
    ProtectionConfig protection;
    PowerStateConfig state;
} control_config_t;

typedef control_config_t BoardPowerConfig;

extern const BoardPowerConfig g_board_power_config;

/**
 * @brief 获取当前板级功率控制配置。
 * @return 指向静态有效期配置对象的指针，也允许板级覆盖实现返回自定义配置。
 * @note GCC 构建中的默认实现为弱符号，默认返回 g_board_power_config。
 */
const BoardPowerConfig *BoardPower_GetConfig(void);

/**
 * @brief 按运行模式校验板级安全标志、ADC 标定和对应控制参数。
 * @param[in] cfg 待检查的控制配置。
 * @param[in] mode MODE0 校验全级，MODE1 仅校验逆变，MODE2 仅校验整流。
 * @return 对应模式所需的安全标志、标定、保护阈值及控制参数合法时返回 true。
 * @warning config_valid 不能绕过其余检查。
 */
bool BoardPower_ValidateModeConfig(const control_config_t *cfg,
                                   app_mode_t mode);

/**
 * @brief 按 MODE0_FULL 完整校验板级配置，保留旧调用接口。
 * @param[in] cfg 待检查的控制配置。
 * @return 全级运行所需配置全部合法时返回 true。
 */
bool BoardPower_ValidateConfig(const control_config_t *cfg);

/**
 * @brief 校验 BoardPower_GetConfig() 返回的当前板级配置。
 * @return 当前配置可安全用于控制时返回 true，否则返回 false。
 */
bool BoardPower_ConfigValid(void);

#ifdef __cplusplus
}
#endif

#endif
