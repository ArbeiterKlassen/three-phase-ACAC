#ifndef INVERTER_CONTROL_H
#define INVERTER_CONTROL_H

#include "measurement.h"
#include "phase_recurrence.h"
#include "qpr_controller.h"
#include "svpwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float sample_time_s;
    float output_frequency_hz;
    float voltage_rms_max;
    float reference_slew_v_per_s;
    float feedforward_gain;
    float voltage_qpr_kp;
    float voltage_qpr_kr;
    float voltage_qpr_bandwidth_rad_s;
    float current_reference_limit_a;
    float current_qpr_kp;
    float current_qpr_kr;
    float current_qpr_bandwidth_rad_s;
    float command_limit_v;
    float dc_voltage_min_v;
    float duty_min;
    float duty_max;
} InverterControlConfig;

typedef struct
{
    PhaseRecurrence phase;
    QprController voltage_alpha_qpr;
    QprController voltage_beta_qpr;
    QprController current_alpha_qpr;
    QprController current_beta_qpr;
    InverterControlConfig cfg;
    float reference_rms;
    float reference_target_rms;
    bool initialized;
} InverterControl;

typedef struct
{
    SvpwmOutput pwm;
    AlphaBeta reference;
    AlphaBeta measured;
    AlphaBeta current_reference;
    AlphaBeta current_measured;
    AlphaBeta voltage_command;
    bool valid;
} InverterControlOutput;

/**
 * @brief 初始化三相逆变电压外环、电流内环、相位递推器和 SVPWM 配置。
 * @param[out] control 逆变控制实例。
 * @param[in] cfg 采样周期、输出频率、环路参数、限幅和占空比配置。
 * @return 参数全部有效且各子控制器初始化成功时返回 true。
 */
bool InverterControl_Init(InverterControl *control,
                          const InverterControlConfig *cfg);

/**
 * @brief 清零逆变 QPR 动态状态并将软启动电压参考复位到 0 V RMS。
 * @param[in,out] control 逆变控制实例；NULL 将被忽略。
 * @note 目标参考、频率和静态配置保持不变。
 */
void InverterControl_Reset(InverterControl *control);

/**
 * @brief 设置逆变相电压有效值目标，并按配置范围自动限幅。
 * @param[in,out] control 已初始化的逆变控制实例。
 * @param[in] voltage_rms 相电压 RMS 目标，单位 V。
 * @note 实际 reference_rms 在 Step 中按 reference_slew_v_per_s 斜坡跟随。
 */
void InverterControl_SetReference(InverterControl *control,
                                  float voltage_rms);

/**
 * @brief 重配置基波频率、相位步长及四个 QPR 控制器。
 * @param[in,out] control 已初始化的逆变控制实例。
 * @param[in] frequency_hz 新基波频率，单位 Hz。
 * @return 参数有效且全部临时控制器初始化成功时返回 true。
 * @note 失败时保留原有控制器和频率配置。
 */
bool InverterControl_SetFrequency(InverterControl *control,
                                  float frequency_hz);

/**
 * @brief 执行一次三相逆变电压外环、电流内环和 SVPWM 计算。
 * @param[in,out] control 已初始化的逆变控制实例。
 * @param[in] measurement 当前物理量测量值，电压单位 V、电流单位 A。
 * @param[in] enabled true 时闭环运行，false 时复位控制状态并输出中性占空比。
 * @param[out] out 参考量、反馈量、电压指令和归一化三相占空比。
 * @return 实例有效，且启用时测量和 SVPWM 计算有效则返回 true。
 * @note 应按 cfg.sample_time_s 固定周期调用；直流母线低于下限时启用计算失败。
 */
bool InverterControl_Step(InverterControl *control,
                          const Measurements *measurement,
                          bool enabled,
                          InverterControlOutput *out);

/**
 * @brief Execute diagnostic open-loop voltage generation and SVPWM.
 * @param[in,out] control Initialized inverter control instance.
 * @param[in] measurement Used only for the validated DC-link SET value.
 * @param[in] enabled true to run the soft-started open-loop command.
 * @param[out] out Reference, direct voltage command and SVPWM result.
 * @return true when the instance, DC-link value and SVPWM result are valid.
 * @note Voltage/current feedback and all four QPR controllers are bypassed.
 */
bool InverterControl_StepOpenLoop(InverterControl *control,
                                  const Measurements *measurement,
                                  bool enabled,
                                  InverterControlOutput *out);

#ifdef __cplusplus
}
#endif

#endif
