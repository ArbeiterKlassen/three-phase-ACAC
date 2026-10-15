#pragma once

/*
 * 单相全桥升压整流级控制（ACDC）。
 *
 * 控制结构（快慢双环）：
 *
 *   母线电压环 PI（1 kHz）──► 输入电流幅值指令 Iamp ──┐
 *                                                     ▼
 *   输入电流 QPR（20 kHz）◄── Iref = Iamp × vin / Vin_rms
 *                                    │
 *                                    ▼
 *                            桥压指令 = vin − vL ──► 单极性 SPWM
 *
 * 要点：
 *   - Iref 与 vin 同相，所以只从交流侧向直流侧送能（PI 下限固定 0）。
 *   - 母线 100 Hz 陷波器滤二倍频纹波，否则它会串进电压环。
 *   - 400 点窗口算输入电压真有效值，用于建立启动条件。
 *   - 软启动：dc_reference_v 从 0 按斜率爬到目标。
 *   - **门极门控**：外环没请求电流时桥臂保持高阻（仅二极管通路）。
 *     不能改用 m=0 代替——单极性 SPWM 下 m=0 等于把交流侧经桥臂
 *     短接到电感上，会产生几十安培的工频短路电流。
 */

#include "measurement.h"
#include "notch_filter.h"
#include "pi_controller.h"
#include "qpr_controller.h"
#include "rectifier_pwm.h"

/* 输入电压真有效值的滑动窗口长度：20 kHz 下一个工频周期（50 Hz） */
#define RECTIFIER_VIN_RMS_WINDOW_SAMPLES (400U)

/* 整流级配置。全部来自 app_config.c，改前先读那里的来历注释。 */
typedef struct {
    float sample_time_s;               // 快环周期 [s]
    float line_frequency_hz;           // 输入工频 [Hz]
    uint16_t outer_loop_divider;       // 外环分频：每 N 拍跑一次 → 1 kHz
    float rms_filter_time_constant_s;  // 保留字段；移动 RMS 不用它
    float input_voltage_rms_min_v;     // 输入就绪所需的 RMS 下限 [V]
    float dc_voltage_max_v;            // 母线目标允许上限 [V]
    float dc_reference_slew_v_per_s;   // 母线参考软启动斜率 [V/s]
    float dc_ready_tolerance_v;        // 母线"已就绪"的容差 [V]
    uint16_t dc_ready_confirm_samples; // 就绪所需的连续确认拍数
    float current_rms_max_a;           // 输入电流 RMS 限幅 [A]
    float dc_pi_kp;                    // 母线环比例增益
    float dc_pi_ki;                    // 母线环积分增益
    float dc_pi_kaw;                   // 母线环抗饱和增益，0 = 条件积分
    float current_qpr_kp;              // 电流内环比例增益
    float current_qpr_kr;              // 电流内环谐振增益
    float current_qpr_bandwidth_rad_s; // 电流内环谐振带宽 [rad/s]
    float inductor_voltage_limit_v;    // 电感电压指令限幅 [V]
    float dc_voltage_min_v;            // 母线欠压下限 [V]，低于则拒绝运行
    float current_reference_sign;      // 电流参考极性（+1 = AC→DC）
    float bridge_polarity;             // 桥臂极性（+1 或 −1）
    float notch_zero_damping;          // 陷波器零点阻尼
    float notch_pole_damping;          // 陷波器极点阻尼
    float duty_min;                    // 占空比下限（归一化）
    float duty_max;                    // 占空比上限（归一化）
} RectifierControlConfig;

/* 整流控制实例。一个 PI（母线环）+ 一个 QPR（电流环）+ 一个陷波器。 */
typedef struct {
    PiController dc_pi;                                        // 母线电压外环
    QprController current_qpr;                                 // 输入电流内环
    NotchFilter dc_notch;                                      // 母线 100 Hz 陷波
    RectifierControlConfig cfg;                                // 生效配置
    float dc_reference_v;                                      // 当前软启动参考 [V]
    float dc_reference_target_v;                               // 母线目标 [V]
    float current_rms_a;                                       // 外环给出的电流幅值指令 [A]
    float vin_square_window[RECTIFIER_VIN_RMS_WINDOW_SAMPLES]; // vin² 环形窗口
    float vin_square_sum;                                      // 窗口内 vin² 之和
    float vin_rms_v;                                           // 输入电压真有效值 [V]
    float dc_voltage_filtered;                                 // 陷波后的母线电压 [V]
    uint16_t vin_square_index;                                 // 环形窗口写指针
    uint16_t vin_square_count;                                 // 窗口内已填样本数
    uint16_t outer_count;                                      // 外环分频计数
    bool vin_rms_valid;                                        // 400 点窗口是否已填满
    bool soft_start_initialized;                               // 软启动是否已起坡
    bool initialized;                                          // 是否已成功初始化
} RectifierControl;

/* 一次控制周期的输出。 */
typedef struct {
    RectifierPwmOutput pwm;       // 两桥臂占空比（写 TIM8 的 CCR）
    float dc_voltage_filtered;    // 陷波后的母线电压 [V]
    float dc_voltage_reference;   // 当前软启动参考 [V]
    float input_voltage_rms;      // 输入电压真有效值 [V]
    float current_reference;      // 瞬时电流参考 [A]
    float bridge_voltage_command; // 桥压指令 [V]
    bool input_rms_valid;         // 输入 RMS 窗口是否有效
    /*
     * 外环未请求电流（软启动爬坡段、或母线已高于参考）时为 false：
     * 桥臂应保持关断，只留二极管通路。不能用 m=0 代替——单极性 SPWM 下
     * m=0 会把交流侧经桥臂短接到升压电感上，产生几十安培工频短路电流。
     */
    bool gates_enabled;
    bool valid; // 本周期 PWM 指令有效
} RectifierControlOutput;

// 初始化母线外环、电流内环、100 Hz 陷波器与 PWM 配置
// PI、QPR、陷波器任一初始化失败则返回 false
bool RectifierControl_Init(RectifierControl *control, const RectifierControlConfig *cfg);

// 复位调节器动态状态与软启动参考，并清空 400 点 RMS 窗口
// 母线目标与静态配置保持不变。NULL 安全
void RectifierControl_Reset(RectifierControl *control);

// 设置母线电压目标（按配置上限裁剪）
// 实际 dc_reference_v 会在 Step 里按 dc_reference_slew_v_per_s 起坡
void RectifierControl_SetDcReference(RectifierControl *control, float dc_voltage);

// 执行一次整流控制。电流快环每拍跑，母线外环每 outer_loop_divider 拍跑一次
// enabled=false 时复位调节器并输出中性占空比，但**仍继续填 RMS 窗口**——
// 这样状态机预热阶段不会误报控制故障
// 返回实例有效且（启用时）测量/母线条件/PWM 计算均有效
// 注意：RMS 窗口预热期间返回 true 但 out->valid 为 false，这是正常的
bool RectifierControl_Step(RectifierControl *control, const Measurements *measurement, bool enabled,
                           RectifierControlOutput *out);
