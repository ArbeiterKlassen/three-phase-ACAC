#pragma once

/*
 * 三相逆变级控制（DCAC）。
 *
 * 控制结构：
 *
 *     参考 U*  ──► 电压外环 QPR ──► 电流参考 i*  ──► 电流内环 QPR ──► 桥压 V ──► SVPWM
 *              ▲                              ▲
 *        Clarke(实测相电压)            Clarke(实测相电流)
 *
 * 两个外环都在 αβ 静止坐标系里做，不需要 Park 变换与角度同步——
 * 对固定的 50/60 Hz 输出，QPR 在谐振频率上的高增益已经能保证稳态精度。
 *
 * 软启动：reference_rms 按 reference_slew_v_per_s 从 0 爬到目标，
 * 避免上电瞬间给满调制。
 */

#include "measurement.h"
#include "phase_recurrence.h"
#include "qpr_controller.h"
#include "svpwm.h"

/* 逆变级配置。全部来自 app_config.c，改前先读那里的来历注释。 */
typedef struct {
    float sample_time_s;               // 控制周期 [s]，20 kHz → 50 us
    float output_frequency_hz;         // 输出基波频率 [Hz]，30 或 60
    float voltage_rms_max;             // 相电压 RMS 上限 [V]，参考限幅用
    float reference_slew_v_per_s;      // 参考软启动斜率 [V/s]
    float feedforward_gain;            // 电压前馈系数，0 表示纯反馈
    float voltage_qpr_kp;              // 电压外环比例增益
    float voltage_qpr_kr;              // 电压外环谐振增益
    float voltage_qpr_bandwidth_rad_s; // 电压外环谐振带宽 [rad/s]
    float current_reference_limit_a;   // 外环输出的电流参考上限 [A]
    float current_qpr_kp;              // 电流内环比例增益
    float current_qpr_kr;              // 电流内环谐振增益
    float current_qpr_bandwidth_rad_s; // 电流内环谐振带宽 [rad/s]
    float command_limit_v;             // 桥压指令限幅 [V]
    float dc_voltage_min_v;            // 母线欠压保护下限 [V]，低于则拒绝运行
    float duty_min;                    // 占空比下限（归一化）
    float duty_max;                    // 占空比上限（归一化）
} InverterControlConfig;

/* 逆变控制实例。四个 QPR：电压/电流 × α/β 轴。 */
typedef struct {
    PhaseRecurrence phase;           // 无查表相位发生器
    QprController voltage_alpha_qpr; // α 轴电压外环
    QprController voltage_beta_qpr;  // β 轴电压外环
    QprController current_alpha_qpr; // α 轴电流内环
    QprController current_beta_qpr;  // β 轴电流内环
    InverterControlConfig cfg;       // 生效配置
    float reference_rms;             // 当前实际相电压参考 [Vrms]（软启动中）
    float reference_target_rms;      // 目标相电压参考 [Vrms]
    bool initialized;                // 是否已成功初始化
} InverterControl;

/* 一次控制周期的输入输出，全部为 αβ 或占空比量。 */
typedef struct {
    SvpwmOutput pwm;             // 三相占空比（写 TIM1 的 CCR）
    AlphaBeta reference;         // 参考相电压 αβ [V]
    AlphaBeta measured;          // 实测相电压 αβ [V]
    AlphaBeta current_reference; // 电流参考 αβ [A]
    AlphaBeta current_measured;  // 实测电流 αβ [A]
    AlphaBeta voltage_command;   // 桥压指令 αβ [V]
    bool valid;                  // 本周期输出有效
} InverterControlOutput;

// 初始化相位递推、四个 QPR 与 SVPWM 配置。参数或子控制器任一失败则返回 false
bool InverterControl_Init(InverterControl *control, const InverterControlConfig *cfg);

// 清零四个 QPR 的动态状态，并把软启动参考拉回 0 Vrms
// 目标参考、频率与静态配置保持不变。NULL 安全
void InverterControl_Reset(InverterControl *control);

// 设置相电压 RMS 目标（内部按配置范围限幅）
// 实际 reference_rms 会在 Step 里按 reference_slew_v_per_s 斜坡跟上
void InverterControl_SetReference(InverterControl *control, float voltage_rms);

// 重配基波频率：同时更新相位步长与四个 QPR 的谐振系数
// 失败时保留原配置（先算好再整体替换，不会改一半）
bool InverterControl_SetFrequency(InverterControl *control, float frequency_hz);

// 执行一次双闭环 + SVPWM。必须按 cfg.sample_time_s 固定周期调用
// enabled=false 时复位控制状态并输出中性占空比（但相位仍推进）
// 返回实例有效且（启用时）测量与 SVPWM 计算均有效
// 注意：母线低于 cfg.dc_voltage_min_v 时启用路径会失败——这是保护，不是 bug
bool InverterControl_Step(InverterControl *control, const Measurements *measurement, bool enabled,
                          InverterControlOutput *out);

// 开环诊断通道：绕过四个 QPR，把软启动后的电压参考直接送 SVPWM
// measurement 只用于取母线电压；返回实例、母线值与 SVPWM 结果均有效
bool InverterControl_StepOpenLoop(InverterControl *control, const Measurements *measurement, bool enabled,
                                  InverterControlOutput *out);
