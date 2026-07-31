#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "inverter_control.h"
#include "power_state.h"
#include "protection.h"
#include "rectifier_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 模块化三相逆变测试允许的用户母线设定及实测一致性窗口。 */
#define APP_INVERTER_DC_VOLTAGE_DEFAULT_V   (60.0f)
#define APP_INVERTER_DC_VOLTAGE_MIN_V       (30.0f)
#define APP_INVERTER_DC_VOLTAGE_MAX_V       (65.0f)
#define APP_INVERTER_DC_MATCH_TOLERANCE_V   (3.0f)
#define APP_INVERTER_DC_OVERVOLTAGE_V       (66.0f)
#define APP_INVERTER_VOLTAGE_MARGIN         (0.95f)
/* 输出线电压RMS固定按Vdc设定值的0.60倍自动生成。 */
#define APP_INVERTER_OUTPUT_LINE_RMS_RATIO  (0.60f)

typedef struct control_config_t
{
    bool config_valid;
    bool adc_calibrated;
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
 * @brief 按运行模式校验板级安全标志、ADC 标定和控制参数。
 * @param[in] cfg 待检查的控制配置。
 * @param[in] mode 目标运行模式。
 * @return 该模式所需的安全就绪标志、标定系数、保护阈值及控制参数均
 *         合法时返回 true。
 * @note 共享部分检查六项安全标志、保护阈值、状态机参数和合法模式；
 *       MODE2 要求逆变控制器与 Ia/Ib/Vab/Vbc/Vdc 标定有效；
 *       MODE1 要求整流控制器与 Iin/Vin/Vdc 标定有效；
 *       MODE0 要求整流、逆变及全部七路 ADC 标定有效。
 * @warning 整流/逆变 PI、QPR 控制器的零增益视为无效配置。
 */
bool BoardPower_ValidateModeConfig(const control_config_t *cfg,
                                   app_mode_t mode);

/**
 * @brief 完整校验板级安全标志、ADC 标定和所有控制参数。
 * @param[in] cfg 待检查的控制配置。
 * @return 所有安全就绪标志、标定系数、保护阈值及控制参数均合法时返回 true。
 * @warning config_valid 不能绕过其他检查；本接口保持兼容，按最严格的
 *          MODE0_FULL 校验，七路 ADC 通道中任一零增益都会阻止进入 RUN。
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
