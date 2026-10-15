#ifndef RECTIFIER_CONTROL_H
#define RECTIFIER_CONTROL_H

#include "measurement.h"
#include "notch_filter.h"
#include "pi_controller.h"
#include "qpr_controller.h"
#include "rectifier_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RECTIFIER_VIN_RMS_WINDOW_SAMPLES (400U)

typedef struct
{
    float sample_time_s;
    float line_frequency_hz;
    uint16_t outer_loop_divider;
    /* Retained for configuration compatibility; moving RMS does not use it. */
    float rms_filter_time_constant_s;
    float input_voltage_rms_min_v;
    float dc_voltage_max_v;
    float dc_reference_slew_v_per_s;
    float dc_ready_tolerance_v;
    uint16_t dc_ready_confirm_samples;
    float current_rms_max_a;
    float dc_pi_kp;
    float dc_pi_ki;
    float dc_pi_kaw;
    float current_qpr_kp;
    float current_qpr_kr;
    float current_qpr_bandwidth_rad_s;
    float inductor_voltage_limit_v;
    float dc_voltage_min_v;
    float current_reference_sign;
    float bridge_polarity;
    float notch_zero_damping;
    float notch_pole_damping;
    float duty_min;
    float duty_max;
} RectifierControlConfig;

typedef struct
{
    PiController dc_pi;
    QprController current_qpr;
    NotchFilter dc_notch;
    RectifierControlConfig cfg;
    float dc_reference_v;
    float dc_reference_target_v;
    float current_rms_a;
    float vin_square_window[RECTIFIER_VIN_RMS_WINDOW_SAMPLES];
    float vin_square_sum;
    float vin_rms_v;
    float dc_voltage_filtered;
    uint16_t vin_square_index;
    uint16_t vin_square_count;
    uint16_t outer_count;
    bool vin_rms_valid;
    bool soft_start_initialized;
    bool initialized;
} RectifierControl;

typedef struct
{
    RectifierPwmOutput pwm;
    float dc_voltage_filtered;
    float dc_voltage_reference;
    float input_voltage_rms;
    float current_reference;
    float bridge_voltage_command;
    bool input_rms_valid;
    /*
     * 外环未请求电流时为 false：桥臂门极应保持关断（高阻，仅二极
     * 管通路），避免 vin 跟随调制被 0.96 占空比限幅后把母线泵升到
     * Vin_peak/0.96。不能用 m=0 代替——单极性 SPWM 下 m=0 会把
     * 交流侧经桥臂短接到升压电感上。
     */
    bool gates_enabled;
    bool valid;
} RectifierControlOutput;

/**
 * @brief 初始化单相整流母线电压外环、电流内环、陷波器和 PWM 参数。
 * @param[out] control 整流控制实例。
 * @param[in] cfg 快慢环周期、工频、控制器增益、保护限幅和极性配置。
 * @return 参数有效且 PI、QPR、二倍频陷波器均初始化成功时返回 true。
 */
bool RectifierControl_Init(RectifierControl *control,
                           const RectifierControlConfig *cfg);

/**
 * @brief 复位整流控制动态状态和软启动参考。
 * @param[in,out] control 整流控制实例；NULL 将被忽略。
 * @note 目标直流参考和静态配置保持不变；400 点 RMS 窗口及有效标志清零。
 */
void RectifierControl_Reset(RectifierControl *control);

/**
 * @brief 设置整流级直流母线电压目标并按配置上限裁剪。
 * @param[in,out] control 已初始化的整流控制实例。
 * @param[in] dc_voltage 直流母线目标，单位 V。
 * @note 实际 dc_reference_v 在 Step 中按 dc_reference_slew_v_per_s 起坡。
 */
void RectifierControl_SetDcReference(RectifierControl *control,
                                     float dc_voltage);

/**
 * @brief 执行一次整流电流快环，并按分频周期更新母线电压外环。
 * @param[in,out] control 已初始化的整流控制实例。
 * @param[in] measurement 当前输入电压、电流及直流母线测量值。
 * @param[in] enabled true 时闭环运行，false 时复位调节器并输出中性占空比。
 * @param[out] out 滤波母线电压、参考、电流指令、桥压指令和 PWM 结果。
 * @return 实例有效，且启用后测量、母线条件和 PWM 计算有效则返回 true；
 *         RMS 窗口预热期间也返回 true，但 out->valid 保持 false。
 * @note 电流快环按 cfg.sample_time_s 调用；外环周期为该周期乘
 *       outer_loop_divider。禁用时仍填充 400 点输入真 RMS 窗口，窗口填满前
 *       不进入 PWM 控制。电压单位 V，电流单位 A，占空比归一化为 0～1。
 */
bool RectifierControl_Step(RectifierControl *control,
                           const Measurements *measurement,
                           bool enabled,
                           RectifierControlOutput *out);

#ifdef __cplusplus
}
#endif

#endif
