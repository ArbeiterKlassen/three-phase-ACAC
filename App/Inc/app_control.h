#pragma once

/*
 * 应用层门面：三模式调度、20 kHz 快环，以及对 Core 板级层公开的单例接口。
 *
 * 两种用法：
 *   多实例（AppControl_*）—— 纯 C、不依赖 HAL，host_tests 用这套
 *   单例（App_*）        —— 板级用这套，内部持有一个 AppControl
 *
 * 一次快环（App_FastStep20kHz）的固定顺序：
 *     提交采样 → 换算物理量 → 保护判断 → 状态机
 *       → 按模式跑逆变/整流控制 → 归一化占空比
 *     → 延迟一拍写 CCR/CCER/MOE（避开比较值更新瞬间的毛刺）
 *     → 发布状态快照
 *
 * 安全语义：**没有任何软件路径能绕过状态机的 pwm_allowed**。
 * 硬件 Break 更快——它由定时器直接清 MOE，不经这里。
 */

#include "app_config.h"

/* 故障记录器：跳闸前滚动保存原始码、控制量与工频周期峰值，供事后定位。 */
typedef struct {
    AppFaultRawSample fast_history[APP_FAULT_FAST_HISTORY_DEPTH]; // 触发前逐拍原始码
    InverterControlOutput previous_control;                       // 触发前最后一拍控制量
    float cycle_peak_v[APP_FAULT_CYCLE_HISTORY_DEPTH];            // 历次工频周期峰值
    uint32_t previous_control_sequence;                           // 上一拍控制量的帧号
    float partial_cycle_peak_v;                                   // 未完成周期的峰值
    uint16_t partial_cycle_samples;                               // 未完成周期已累计拍数
    uint16_t cycle_target_samples;                                // 一个工频周期多少拍（按频率算）
    uint8_t fast_write_index;                                     // fast_history 写指针
    uint8_t fast_history_count;                                   // fast_history 有效条数
    uint8_t cycle_write_index;                                    // cycle_peak_v 写指针
    uint8_t cycle_peak_count;                                     // cycle_peak_v 有效条数
    bool active;                                                  // 是否正在记录
} AppFaultRecorder;

/* 控制实例。持有逆变、整流、保护、状态机四块状态与各自的配置绑定。 */
typedef struct {
    InverterControl inverter;              // 逆变控制
    RectifierControl rectifier;            // 整流控制
    Protection protection;                 // 保护
    PowerStateMachine state;               // 电源状态机
    const BoardPowerConfig *cfg;           // 当前配置
    const BoardPowerConfig *validated_cfg; // 上一次校验通过的配置指针
    app_mode_t mode;                       // 当前模式
    app_mode_t validated_mode;             // 上一次校验通过的模式
    inverter_run_mode_t inverter_run_mode; // 闭环/开环
    AppFaultSnapshot fault_snapshot;       // 故障快照
    AppFaultRecorder fault_recorder;       // 故障记录器
    uint32_t previous_iin_sequence;        // 上一次输入电流有效的帧号
    uint32_t previous_vin_sequence;        // 上一次输入电压有效的帧号
    uint16_t mode0_dc_ready_count;         // MODE0 母线就绪连续计数
    uint16_t previous_iin_raw;             // 上一次输入电流原始码
    uint16_t previous_vin_raw;             // 上一次输入电压原始码
    uint8_t rectifier_start_count;         // 整流启动过零确认计数
    float previous_iin_a;                  // 上一次输入电流 [A]
    float previous_vin_v;                  // 上一次输入电压 [V]
    bool mode0_inverter_ready;             // MODE0 母线是否已就绪
    bool rectifier_dc_ready;               // 整流级母线是否就绪
    bool rectifier_start_ready;            // 整流级是否满足启动条件
    bool previous_iin_valid;               // 上一次输入电流是否有效
    bool previous_vin_valid;               // 上一次输入电压是否有效
    bool board_config_valid;               // 板级参数是否通过校验
    bool initialized;                      // 是否已成功初始化
} AppControl;

// 初始化控制实例：逆变 + 保护 + 状态机，并尝试初始化整流
// 默认 MODE1_INVERTER、状态 OFF；**本函数不使能 PWM**
// 整流参数无效不会连带破坏 MODE1；MODE0/MODE2 仍会被模式校验锁止
bool AppControl_Init(AppControl *app);

// 复位控制器动态状态、保护记录与状态机；运行模式和板级配置保持不变
// NULL 或未初始化实例安全
void AppControl_Reset(AppControl *app);

// 选模式。只在 OFF（或 PWM 已关闭的 FAULT_LATCH）允许
// 注意：FAULT_LATCH 下切模式**不会**清除锁存故障，也不会启动 PWM
bool AppControl_SetMode(AppControl *app, app_mode_t mode);

// 选闭环或诊断开环。只在 PWM 已关闭的 OFF / FAULT_LATCH 允许
// 切换成功会复位电压斜坡与四个 QPR 状态
bool AppControl_SetInverterRunMode(AppControl *app, inverter_run_mode_t mode);

// 执行一次完整控制计算。典型周期 20 kHz
// 流程：换算 → 保护 → 状态机 → 按模式控制 → 生成 PWM 命令
// 未获准运行时输出 0.5 中性占空比并撤销使能请求
// 返回参数与实例有效；控制环失败通过故障位与安全输出上报，不靠返回值
bool AppControl_Step(AppControl *app, const AdcRawFrame *raw, const AppCommand *command, AppOutput *out);

/* ── 以下是板级使用的单例接口，内部持有一个 AppControl ───────────────── */

// 初始化全局单例。默认参考值、状态 OFF、所有 PWM 输出保持安全中性值
void App_Init(void);

// 选全局模式。要求：已初始化、无启动请求、mode 合法、处于 OFF 或 PWM 已关的 FAULT_LATCH
// 运行或放电过程中不能切模式；故障下切换不会清故障
bool App_RequestMode(app_mode_t mode);

// 选全局逆变环路方式，只在 PWM 安全关闭时允许
// 一旦已提交启动请求，选择会被拒绝
bool App_SetInverterRunMode(inverter_run_mode_t mode);

// 提交启动请求。何时真正放行 PWM 由后续快环与状态机决定
// **不会绕过**自检、预充、锁相、测量有效性或故障联锁
void App_RequestStart(void);

// 撤销启动请求，使状态机转入安全停机与放电
// 只更新命令；板级层仍应按 PWM 使能请求立即控制驱动器
void App_RequestStop(void);

// 提交一帧双 ADC 同步模式的 CDR 打包采样
// adc_cdr_pairs 每项低 16 位为 ADC1、高 16 位为 ADC2，
// 固定 rank 映射依次为：Iin/Vin、Ib/Ia、Vab/Vbc、Vdc/Vin诊断
// sequence 为 DMA 完整帧单调序号，用于检测陈旧或重复帧
// 三种模式用到的通道不同，见 measurement.h 的三个解包函数
bool App_ApplyAdcPairs(const uint32_t adc_cdr_pairs[4], uint32_t sequence);

// 以八个独立 16 位采样值提交一帧（按 ADC1 r0、ADC2 r0 … ADC1 r3、ADC2 r3 交错）
// 供主机测试使用；内部转成 CDR 打包格式后走同一条路径
bool App_ApplyAdcSamples(const uint16_t adc_samples[8], uint32_t sequence);

// 执行一次 20 kHz 快环。消费最近提交的一帧，完成换算、保护、状态机与控制，
// 然后发布新的 PWM 与状态快照；**没有新帧时按无效测量处理**
void App_FastStep20kHz(void);

// 低速后台任务。当前只做命令解析与遥测，不在这里直接使能 PWM
void App_BackgroundTask(void);

// 从中断上下文提交并锁存紧急故障，同时立即撤销 PWM 使能请求
// faults 传 0 时按外部故障处理；内部用短临界区合并并发故障位
void App_TripFromISR(fault_flags_t faults);

// 取最近一次快环发布的 PWM 命令。返回指向全局只读快照的稳定指针，初始化前也不为 NULL
// 注意：占空比 0~1 只是数值，是否真的输出还必须看对应的 enable_request
const AppPwmCommand *App_GetPwmCommand(void);

// 取最近一次快环发布的状态快照。返回全局只读指针，调用方不得修改
const AppStatus *App_GetStatus(void);

// 设置输出基波频率。当前只接受 30 或 60 Hz
// 要求已初始化、处于 OFF，且控制器重配置成功
bool App_SetOutputFrequency(uint16_t frequency_hz);

// 设置输出线电压参考 [Vrms]（现场 ±0.1 V 微调通道）
// 允许窗口 28~34 V；且换算后的相电压不得超过逆变上限
// 只在 OFF 且无启动请求、或 FAULT_LATCH 且 PWM 已关时接受
bool App_SetOutputLineVoltageReference(float line_rms);

// 设置母线目标 [V]，允许 30~65 V
// 要求已初始化、处于无启动请求的 OFF，或 PWM 已禁止的 FAULT_LATCH
// 注意：输出参考**已与母线解耦**，本函数只改母线目标，不再联动输出参考
// MODE1 逆变单测继续用该设定值代替未接入的 Vdc；MODE0/MODE2 用实测 Vdc
bool App_SetDcVoltageReference(float volts);

// 请求清除锁存故障并保持停机
// 清除动作在后续快环处理；只有活动故障已消失时才真正解锁
void App_ResetFault(void);

// 更新板级预充完成联锁。MODE1 内部视为已完成；MODE0/MODE2 真实使用
void App_SetPrechargeDone(bool done);

// 更新板级相位/同步就绪联锁。MODE1 内部视为已就绪；MODE0/MODE2 真实使用
void App_SetPhaseReady(bool ready);
