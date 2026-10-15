#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    AppFaultRawSample fast_history[APP_FAULT_FAST_HISTORY_DEPTH];
    InverterControlOutput previous_control;
    float cycle_peak_v[APP_FAULT_CYCLE_HISTORY_DEPTH];
    uint32_t previous_control_sequence;
    float partial_cycle_peak_v;
    uint16_t partial_cycle_samples;
    uint16_t cycle_target_samples;
    uint8_t fast_write_index;
    uint8_t fast_history_count;
    uint8_t cycle_write_index;
    uint8_t cycle_peak_count;
    bool active;
} AppFaultRecorder;

typedef struct
{
    InverterControl inverter;
    RectifierControl rectifier;
    Protection protection;
    PowerStateMachine state;
    const BoardPowerConfig *cfg;
    const BoardPowerConfig *validated_cfg;
    app_mode_t mode;
    app_mode_t validated_mode;
    inverter_run_mode_t inverter_run_mode;
    AppFaultSnapshot fault_snapshot;
    AppFaultRecorder fault_recorder;
    uint32_t previous_iin_sequence;
    uint32_t previous_vin_sequence;
    uint16_t mode0_dc_ready_count;
    uint16_t previous_iin_raw;
    uint16_t previous_vin_raw;
    uint8_t rectifier_start_count;
    float previous_iin_a;
    float previous_vin_v;
    bool mode0_inverter_ready;
    bool rectifier_dc_ready;
    bool rectifier_start_ready;
    bool previous_iin_valid;
    bool previous_vin_valid;
    bool board_config_valid;
    bool initialized;
} AppControl;

/**
 * @brief 初始化三模式 AC-AC 控制实例。
 * @param[out] app 控制实例，初始化逆变、保护、状态机，并尝试初始化整流。
 * @return 配置指针及默认 MODE1 逆变控制有效时返回 true，否则返回 false。
 * @note 整流参数无效不会连带破坏 MODE1；MODE0/MODE2 仍会被按模式配置
 *       校验及整流控制 initialized 状态锁止。
 * @note 默认模式为 MODE1_INVERTER，初始状态为 OFF；本函数不直接使能 PWM。
 */
bool AppControl_Init(AppControl *app);

/**
 * @brief 复位控制器动态状态、保护记录及电源状态机。
 * @param[in,out] app 已成功初始化的控制实例；NULL 或未初始化实例将被忽略。
 * @note 运行模式和板级配置保持不变，逆变控制状态回到安全初值。
 */
void AppControl_Reset(AppControl *app);

/**
 * @brief 在停机或 PWM 已关闭的故障锁存状态下选择运行模式。
 * @param[in,out] app 已成功初始化的控制实例。
 * @param[in] mode MODE0_FULL、MODE1_INVERTER 或 MODE2_RECTIFIER。
 * @return 实例有效、mode 合法，且处于 OFF 或 PWM 已关闭的
 *         FAULT_LATCH 时返回 true。
 * @note 在 FAULT_LATCH 中切换模式不会清除锁存故障或启动 PWM。
 */
bool AppControl_SetMode(AppControl *app, app_mode_t mode);

/**
 * @brief Select closed-loop or diagnostic open-loop inverter control.
 * @param[in,out] app Initialized inverter application instance.
 * @param[in] mode INVERTER_CLOSED_LOOP or INVERTER_OPEN_LOOP.
 * @return true only while PWM is disabled in OFF or FAULT_LATCH.
 * @note A successful change resets the voltage ramp and all QPR states.
 */
bool AppControl_SetInverterRunMode(AppControl *app,
                                   inverter_run_mode_t mode);

/**
 * @brief 执行一次完整控制计算并生成归一化 PWM 命令。
 * @param[in,out] app 已成功初始化的控制实例。
 * @param[in] raw 当前 ADC 原始采样帧。
 * @param[in] command 本周期启停、联锁、故障及参考值命令。
 * @param[out] out 测量结果、状态、故障以及两级 PWM 命令。
 * @return 参数及实例有效时返回 true；控制环失败通过故障位和安全输出上报。
 * @note 典型调用周期为 20 kHz。流程为测量换算、保护判断、状态机更新、
 *       模式控制及 PWM 命令生成；未获准运行时输出 0.5 中性占空比并撤销使能。
 */
bool AppControl_Step(AppControl *app,
                     const AdcRawFrame *raw,
                     const AppCommand *command,
                     AppOutput *out);

/**
 * @brief 初始化面向 Core 适配层的全局 App 运行实例。
 * @note 使用板级默认参考值，状态保持 OFF，所有 PWM 输出保持安全中性值。
 */
void App_Init(void);

/**
 * @brief 选择全局 App 运行模式。
 * @param[in] mode MODE0_FULL、MODE1_INVERTER 或 MODE2_RECTIFIER。
 * @return App 已初始化、无启动请求、mode 合法，且处于 OFF 或
 *         PWM 已关闭的 FAULT_LATCH 时返回 true。
 * @warning 运行或放电过程中不能切换模式；故障模式切换不会清故障。
 */
bool App_RequestMode(app_mode_t mode);

/**
 * @brief Select the inverter control path while PWM is safely disabled.
 * @param[in] mode Closed-loop or diagnostic open-loop operation.
 * @return true in OFF or FAULT_LATCH with no pending start request.
 * @warning Selection is rejected as soon as start has been requested.
 */
bool App_SetInverterRunMode(inverter_run_mode_t mode);

/**
 * @brief 提交启动请求，由后续 20 kHz 快环和电源状态机决定何时放行 PWM。
 * @note 本函数不会绕过自检、预充、锁相、测量有效性或故障联锁。
 */
void App_RequestStart(void);

/**
 * @brief 撤销启动请求，使状态机进入安全停机和放电流程。
 * @note 本函数只更新命令；板级适配层仍应按 PWM 使能请求立即控制驱动器。
 */
void App_RequestStop(void);

/**
 * @brief 提交一帧 ADC 双重规则同步模式的 CDR 打包采样。
 * @param[in] adc_cdr_pairs 四个转换序列结果；每项低 16 位为 ADC1，
 *            高 16 位为 ADC2。
 * @param[in] sequence DMA 完整帧单调序号，用于检测陈旧或重复采样。
 * @return 打包格式和 App 状态均有效时返回 true。
 * @note 固定 rank 映射依次为 Iin/Vin、Ib/Ia、Vab/Vbc、Vdc/Vin诊断。
 * @note MODE1 只使用 Ib、Ia、Vab、Vbc。MODE2 只使用 Iin、Vin
 *       Rank1 和 Vdc，不以 Rank4 的重复 Vin 诊断阻断单级测试。
 *       MODE0 仍执行完整映射和两次 Vin 一致性检查。
 */
bool App_ApplyAdcPairs(const uint32_t adc_cdr_pairs[4],
                       uint32_t sequence);

/**
 * @brief 以八个独立 16 位采样值提交一帧 ADC 数据。
 * @param[in] adc_samples 按 ADC1 rank0、ADC2 rank0、…、ADC1 rank3、
 *            ADC2 rank3 交错排列的八个采样值。
 * @param[in] sequence DMA 完整帧单调序号。
 * @return 转换为 CDR 数据并成功提交时返回 true。
 */
bool App_ApplyAdcSamples(const uint16_t adc_samples[8],
                         uint32_t sequence);

/**
 * @brief 执行全局 App 的一次 20 kHz 快速控制循环。
 * @note 消费最近提交的一帧 ADC 数据，依次完成测量、保护、状态机和控制计算，
 *       然后发布新的 PWM 及状态快照；无新帧时按无效测量处理。
 */
void App_FastStep20kHz(void);

/**
 * @brief 执行低速后台任务。
 * @note 当前仅为命令解析和遥测预留，不在此函数中直接使能 PWM。
 */
void App_BackgroundTask(void);

/**
 * @brief 从中断上下文提交并锁存紧急故障，同时立即撤销 PWM 使能请求。
 * @param[in] faults 要置位的故障掩码；传入 0 时按外部故障处理。
 * @warning 本函数可在 ISR 中调用，并使用短临界区合并并发故障位。
 */
void App_TripFromISR(fault_flags_t faults);

/**
 * @brief 获取最近一次快环发布的归一化 PWM 命令。
 * @return 指向全局只读命令快照的稳定指针；初始化前也不会返回 NULL。
 * @note 占空比范围为 0.0～1.0，实际开关使能还必须检查对应 enable_request。
 */
const AppPwmCommand *App_GetPwmCommand(void);

/**
 * @brief 获取最近一次快环发布的 App 状态快照。
 * @return 指向全局只读状态的稳定指针；调用方不得修改其内容。
 */
const AppStatus *App_GetStatus(void);

/**
 * @brief 设置三相逆变输出基波频率。
 * @param[in] frequency_hz 输出频率，单位 Hz；当前只接受 30 或 60。
 * @return App 已初始化、处于 OFF 且控制器重配置成功时返回 true。
 */
bool App_SetOutputFrequency(uint16_t frequency_hz);

/**
 * @brief 比例跟随模式下校验输出线电压有效值参考。
 * @param[in] line_rms 线电压 RMS，单位 V。
 * @return 仅当数值等于当前 Vdc 设定值乘固定比例时返回 true；
 *         本接口不允许独立改写输出电压。
 */
bool App_SetOutputLineVoltageReference(float line_rms);

/**
 * @brief 设置逆变控制使用的直流母线电压，并按固定比例同步输出参考。
 * @param[in] volts 允许范围为 30～65 V。
 * @return App 已初始化、设定值在范围内，并且处于无启动请求的 OFF，
 *         或 PWM 已禁止的 FAULT_LATCH 时返回 true。
 * @note MODE1 逆变单测继续用该设定值代替未接入的 Vdc；MODE0/MODE2
 *       使用实测 Vdc。输出线电压 RMS 自动设为固定比例乘 volts。
 */
bool App_SetDcVoltageReference(float volts);

/**
 * @brief 请求清除全局 App 的锁存故障并保持停机。
 * @note 清除请求在后续快环处理；只有活动故障已经消失时才能解除锁存。
 */
void App_ResetFault(void);

/**
 * @brief 更新板级预充完成联锁信号。
 * @param[in] done true 表示预充条件已满足。
 * @note MODE1 内部视为已完成；MODE0/MODE2 会真实使用该联锁。
 */
void App_SetPrechargeDone(bool done);

/**
 * @brief 更新板级输入相位或同步就绪联锁信号。
 * @param[in] ready true 表示相位条件已满足。
 * @note MODE1 内部视为已就绪；MODE0/MODE2 会真实使用该联锁。
 */
void App_SetPhaseReady(bool ready);

#ifdef __cplusplus
}
#endif

#endif
