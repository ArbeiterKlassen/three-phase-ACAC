#pragma once

/*
 * 跨模块共享的类型与常量。
 *
 * 这里是整个 App 层的"公共词汇表"：原始采样帧 → 物理量测量值 → 控制输出，
 * 各模块通过这几个结构体交换数据，不直接互相调用。
 *
 * 单位约定（改名前先看这条）：
 *   电压 _v [V]、电流 _a [A]、时间 _s [s]、频率 _hz [Hz]
 *   数值字段一律带单位后缀；不带后缀的 float 只允许是归一化量（占空比 0~1）
 */

#include <stdbool.h>
#include <stdint.h>

#define APP_PI_F (3.14159265358979323846f)
#define APP_SQRT3_F (1.73205080756887729353f)     // √3
#define APP_INV_SQRT3_F (0.57735026918962576451f) // 1/√3，线电压↔相电压换算用
#define APP_FAULT_FAST_HISTORY_DEPTH (16U)        // 故障快照保留的拍数
#define APP_FAULT_CYCLE_HISTORY_DEPTH (32U)       // 故障快照保留的工频周期数

/* 一帧未换算的 ADC 原始码。由 DMA/解包产生，尚未乘标定系数。 */
typedef struct {
    uint16_t iin;      // 整流输入电流原始码
    uint16_t vin;      // 整流输入电压原始码
    uint16_t ia;       // A 相电流原始码
    uint16_t ib;       // B 相电流原始码
    uint16_t vab;      // 线电压 Vab 原始码
    uint16_t vbc;      // 线电压 Vbc 原始码
    uint16_t vdc;      // 母线电压原始码
    uint16_t vin_diag; // Vin 的诊断重复采样（MODE0 用它做冗余校验）
    uint32_t sequence; // DMA 完整帧序号，用于检测采样停滞
    bool valid;        // 本帧是否通过解包校验
} AdcRawFrame;

/* 换算后的物理量。第三相由前两相重构（三线制）。 */
typedef struct measurements_t {
    float iin;         // 整流输入电流 [A]
    float vin;         // 整流输入电压 [V]
    float ia;          // A 相电流 [A]
    float ib;          // B 相电流 [A]
    float ic;          // C 相电流 [A]，ic = −(ia+ib)
    float vab;         // 线电压 Vab [V]
    float vbc;         // 线电压 Vbc [V]
    float vca;         // 线电压 Vca [V]，vca = −(vab+vbc)
    float va;          // A 相相电压 [V]
    float vb;          // B 相相电压 [V]
    float vc;          // C 相相电压 [V]
    float vdc;         // 直流母线电压 [V]
    uint32_t sequence; // 对应原始帧序号
    bool valid;        // 换算是否全部成功
} measurements_t;

typedef measurements_t Measurements;
typedef uint32_t fault_flags_t; // 故障位掩码，见 protection.h

/* αβ 静止坐标系分量。Clarke 变换的输出、控制器内部流通的量。 */
typedef struct {
    float alpha; // α 轴
    float beta;  // β 轴
} AlphaBeta;

/* 三相量，按 A/B/C 存放。 */
typedef struct {
    float a;
    float b;
    float c;
} ThreePhase;

/* 逆变级 PWM 指令：三路归一化占空比 + 是否允许输出。 */
typedef struct {
    float a;      // A 相占空比 [0~1]
    float b;      // B 相占空比 [0~1]
    float c;      // C 相占空比 [0~1]
    bool enabled; // 是否请求开放门极
} InverterPwmCommand;

/* 整流级 PWM 指令：两桥臂 + 是否允许输出。 */
typedef struct {
    float leg_a;  // 桥臂 A 占空比 [0~1]
    float leg_b;  // 桥臂 B 占空比 [0~1]
    bool enabled; // 是否请求开放门极
} RectifierPwmCommand;

/* 三种运行模式。只能在 OFF 状态切换。 */
typedef enum app_mode_t {
    MODE0_FULL = 0,     // 整流 + 逆变联动
    MODE1_INVERTER = 1, // 只测逆变（上电默认）
    MODE2_RECTIFIER = 2 // 只测整流
} app_mode_t;

/* 逆变环路的运行方式，仅调试用，键盘不暴露。 */
typedef enum inverter_run_mode_t {
    INVERTER_CLOSED_LOOP = 0, // 正常双闭环
    INVERTER_OPEN_LOOP = 1    // 诊断开环，绕过四个 QPR
} inverter_run_mode_t;

/* 电源状态机的状态。转移关系见 power_state.h。 */
typedef enum app_state_t {
    OFF = 0,     // 停机
    SELF_TEST,   // 自检：等待连续有效采样
    PRECHARGE,   // 预充：等待母线建立
    READY,       // 就绪：等待相位确认
    RUN,         // 运行
    FAULT_LATCH, // 故障锁存，需人工确认复位
    DISCHARGE    // 放电：等待母线降到安全阈值
} app_state_t;

/* 故障现场的一组原始采样值，用于事后定位。 */
typedef struct {
    uint32_t sample_sequence; // 该组样本对应的帧序号
    uint16_t pc0_raw;         // PC0 原始码（Ib）
    uint16_t pc1_raw;         // PC1 原始码（Ia）
    uint16_t pc2_raw;         // PC2 原始码（Vab）
    uint16_t pc3_raw;         // PC3 原始码（Vbc）
} AppFaultRawSample;

/* 故障现场的一组控制量，用于判断是"算错了"还是"量错了"。 */
typedef struct {
    AlphaBeta voltage_reference; // 电压参考 αβ [V]
    AlphaBeta voltage_measured;  // 实测电压 αβ [V]
    AlphaBeta current_reference; // 电流参考 αβ [A]
    AlphaBeta current_measured;  // 实测电流 αβ [A]
    AlphaBeta voltage_command;   // 桥压指令 αβ [V]
    uint32_t sample_sequence;    // 帧序号
    float duty_a;                // A 相占空比
    float duty_b;                // B 相占空比
    float duty_c;                // C 相占空比
    float applied_scale;         // 过调制缩放比例
    bool saturated;              // 是否触发过调制缩放
    bool valid;                  // 该组样本是否有效
} AppFaultControlSample;

/* 故障现场总快照。跳闸瞬间冻结，供 OLED / 调试器事后查看。 */
typedef struct {
    inverter_run_mode_t inverter_run_mode;                        // 跳闸时的环路方式
    fault_flags_t faults;                                         // 跳闸时的故障掩码
    uint32_t sample_sequence;                                     // 跳闸帧序号
    float dc_voltage_reference_v;                                 // 母线参考 [V]
    float vab_v, vbc_v;                                           // 线电压 [V]
    float va_v, vb_v, vc_v;                                       // 相电压 [V]
    float ia_a, ib_a;                                             // 相电流 [A]
    float iin_a, previous_iin_a;                                  // 输入电流及其上一有效值 [A]
    float vin_v, previous_vin_v;                                  // 输入电压及其上一有效值 [V]
    float vdc_v;                                                  // 母线电压 [V]
    uint32_t previous_iin_sequence;                               // 上一有效 iin 的帧号
    uint32_t previous_vin_sequence;                               // 上一有效 vin 的帧号
    uint16_t iin_raw, previous_iin_raw;                           // 输入电流原始码及上一有效值
    uint16_t vin_raw, previous_vin_raw;                           // 输入电压原始码及上一有效值
    bool previous_iin_valid;                                      // 上一 iin 是否有效
    bool previous_vin_valid;                                      // 上一 vin 是否有效
    AppFaultRawSample trigger_raw;                                // 触发那一帧的原始码
    AppFaultRawSample fast_history[APP_FAULT_FAST_HISTORY_DEPTH]; // 触发前若干拍
    AppFaultControlSample previous_control;                       // 触发前最后一拍控制量
    float cycle_peak_v[APP_FAULT_CYCLE_HISTORY_DEPTH];            // 历次工频周期峰值，[0] 最新
    float partial_cycle_peak_v;                                   // 当拍未完成周期的峰值（不含触发帧）
    uint16_t partial_cycle_samples;                               // 未完成周期已累计拍数
    uint16_t pc2_raw, pc3_raw;                                    // 触发帧的 Vab/Vbc 原始码
    uint8_t fast_history_count;                                   // fast_history 中有效条数
    uint8_t cycle_peak_count;                                     // cycle_peak_v 中有效条数
    bool valid;                                                   // 整份快照是否有效
} AppFaultSnapshot;

/* 交给板级层的 PWM 指令：五路占空比 + 两级使能请求。 */
typedef struct {
    float inverter_a;              // TIM1 CCR1 [0~1]
    float inverter_b;              // TIM1 CCR2
    float inverter_c;              // TIM1 CCR3
    float rectifier_a;             // TIM8 CCR2
    float rectifier_b;             // TIM8 CCR1
    bool inverter_enable_request;  // 请求开放 TIM1 门极
    bool rectifier_enable_request; // 请求开放 TIM8 门极
} AppPwmCommand;

/* 对外状态快照。后台任务读取时应短暂关中断复制，避免读到半更新值。 */
typedef struct {
    app_mode_t mode;                       // 当前模式
    inverter_run_mode_t inverter_run_mode; // 闭环/开环
    app_state_t state;                     // 状态机状态
    float output_frequency_hz;             // 输出基波频率 [Hz]
    float dc_voltage_v;                    // 实测母线电压 [V]
    float output_line_voltage_reference_v; // 输出线电压参考 [Vrms]
    float dc_voltage_reference_v;          // 母线参考 [V]
    float iin_a;                           // 输入电流 [A]
    float ia_a, ib_a;                      // 相电流 [A]
    float vab_v, vbc_v;                    // 线电压 [V]
    uint16_t iin_raw;                      // 输入电流原始码（诊断用）
    AppFaultSnapshot fault_snapshot;       // 故障现场
    fault_flags_t faults;                  // 当前故障掩码
    uint32_t sample_sequence;              // 最近一帧序号
    bool board_config_valid;               // 板级参数是否通过校验
    bool pwm_allowed;                      // 状态机是否放行 PWM
    bool rectifier_dc_ready;               // 母线是否达到"就绪"
} AppStatus;

/* 上层喂给应用层的命令。 */
typedef struct {
    bool enable;             // 启动请求
    bool reset_fault;        // 复位故障请求
    bool precharge_done;     // 板级预充完成
    bool phase_ready;        // 板级相位就绪
    bool external_fault;     // 板级故障输入
    bool loop_overrun;       // 快环超时输入
    float inverter_vrms_ref; // 输出相电压参考 [Vrms]；≤0 表示按母线比例推导
    float dc_voltage_ref;    // 母线目标 [V]
} AppCommand;

/* 应用层每拍的输出。 */
typedef struct {
    Measurements measurement;      // 本拍测量值
    InverterPwmCommand inverter;   // 逆变 PWM 指令
    RectifierPwmCommand rectifier; // 整流 PWM 指令
    app_state_t state;             // 状态机状态
    fault_flags_t faults;          // 本拍故障
    bool board_config_valid;       // 板级参数是否有效
    bool pwm_allowed;              // 是否放行 PWM
} AppOutput;
