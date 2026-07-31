#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    InverterControl inverter;
    RectifierControl rectifier;
    Protection protection;
    PowerStateMachine state;
    const BoardPowerConfig *cfg;
    app_mode_t mode;
    uint16_t mode0_dc_ready_count;
    bool mode0_inverter_ready;
    bool rectifier_dc_ready;
    bool initialized;
} AppControl;

/**
 * @brief 初始化三模式 AC-AC 控制实例。
 * @param[out] app 控制实例，成功时初始化逆变、整流、保护和状态机。
 * @return 配置指针有效且两级控制模块均初始化成功时返回 true。
 * @note 默认模式为 MODE0_FULL，初始状态为 OFF；整流 DC 目标取配置的
 *       dc_voltage_ref；本函数不直接使能 PWM。
 */
bool AppControl_Init(AppControl *app);

/**
 * @brief 复位控制器动态状态、保护记录及电源状态机。
 * @param[in,out] app 已成功初始化的控制实例；NULL 或未初始化实例将被忽略。
 * @note 运行模式和板级配置保持不变，两级控制状态回到安全初值。
 */
void AppControl_Reset(AppControl *app);

/**
 * @brief 在停机状态下切换运行模式。
 * @param[in,out] app 已成功初始化的控制实例。
 * @param[in] mode MODE0_FULL、MODE1_RECTIFIER 或 MODE2_INVERTER。
 * @return 仅当实例有效、状态为 OFF 且 mode 合法时返回 true。
 */
bool AppControl_SetMode(AppControl *app, app_mode_t mode);

/**
 * @brief 执行一次完整控制计算并生成归一化 PWM 命令。
 * @param[in,out] app 已成功初始化的控制实例。
 * @param[in] raw 当前 ADC 原始采样帧。
 * @param[in] command 本周期启停、联锁、故障及参考值命令。
 * @param[out] out 测量结果、状态、故障和整流/逆变 PWM 命令。
 * @return 参数及实例有效时返回 true；控制环失败通过故障位和安全输出上报。
 * @note 典型调用周期为 20 kHz。流程为按模式测量换算、保护判断、状态机
 *       更新、两级控制及 PWM 命令生成。整流级在所有状态下以
 *       enabled=false 预热 RMS/陷波；SVPWM 归一化和限幅使用实测 Vdc，
 *       键盘 Vdc 设定只作为期望母线/输出参考来源。未获准运行时输出
 *       0.5 中性占空比并撤销使能。
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
 * @brief 请求切换全局 App 运行模式。
 * @param[in] mode MODE0_FULL、MODE1_RECTIFIER 或 MODE2_INVERTER。
 * @return 仅当 App 已初始化、当前处于 OFF 且 mode 合法时返回 true。
 * @warning 运行、故障锁存或放电过程中不能切换模式；模式写入在短临界
 *          区内完成，避免 20 kHz ISR 跨模式错帧解包/换算。
 */
bool App_RequestMode(app_mode_t mode);

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
 * @note 固定 rank 映射依次为 Iin/Vin、Ia/Ib、Vab/Vbc、Vdc/Vin诊断。
 * @note MODE2 跳过 Vin 冗余诊断；MODE0/MODE1 执行 Vin/Vin_diag
 *       一致性检查，诊断失败会锁存测量故障。
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
 * @return App 已初始化、处于 OFF 且控制器重配置成功时返回 true；
 *         MODE1_RECTIFIER 没有三相输出，直接返回 false。
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
 * @brief 设置直流母线电压目标，并按固定比例同步逆变输出参考。
 * @param[in] volts 允许范围为 30～65 V。
 * @return App 已初始化、设定值在范围内，并且处于无启动请求的 OFF，
 *         或 PWM 已禁止的 FAULT_LATCH 时返回 true。
 * @note 该设定只是期望母线/输出参考来源：MODE2 用它推导输出线电压
 *      RMS（APP_INVERTER_OUTPUT_LINE_RMS_RATIO * volts），MODE1/MODE0
 *       同时把它作为整流级直流母线目标；SVPWM 归一化始终使用实测 Vdc。
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
 * @note MODE1/MODE0 真实生效；无外部信号时由测量推导（实测 Vdc 达到
 *       运行欠压阈值）。MODE2 使用外接直流源，内部固定视为已完成。
 */
void App_SetPrechargeDone(bool done);

/**
 * @brief 更新板级输入相位或同步就绪联锁信号。
 * @param[in] ready true 表示相位条件已满足。
 * @note MODE1/MODE0 真实生效；无外部信号时由测量推导（输入 RMS 超过
 *       最小输入电压）。MODE2 内部固定视为已就绪。
 */
void App_SetPhaseReady(bool ready);

#ifdef __cplusplus
}
#endif

#endif
