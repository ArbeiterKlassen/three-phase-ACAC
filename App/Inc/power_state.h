#pragma once

/*
 * 电源状态机。
 *
 *     OFF → SELF_TEST → PRECHARGE → READY → RUN
 *      ↑                                        │
 *      └──── DISCHARGE ←── 停机/条件丢失 ────────┘
 *
 *     任意状态出现有效或锁存故障 → FAULT_LATCH（需人工确认才复位）
 *
 * pwm_allowed 是唯一的放行开关：只有同时满足全部安全条件才为真。
 * 硬件 Break 不走这里，由定时器直接清 MOE。
 */

#include "app_types.h"

/* 各阶段所需的连续确认拍数，用来滤掉瞬时抖动。 */
typedef struct {
    uint16_t self_test_confirm_samples; // 自检连续有效拍数
    uint16_t ready_confirm_samples;     // 就绪连续确认拍数
    uint16_t discharge_confirm_samples; // 放电完成连续确认拍数
} PowerStateConfig;

/* 每拍喂给状态机的输入。 */
typedef struct {
    bool start_request;       // 启动请求（键盘长按 #）
    bool stop_request;        // 停机请求
    bool reset_fault_request; // 复位故障请求（键盘长按 *）
    bool precharge_done;      // 预充完成（MODE0/MODE2 由实测 Vdc 判定）
    bool phase_ready;         // 输入相位/同步就绪
    bool measurement_valid;   // 本拍测量有效
    bool discharge_done;      // 母线已降到安全阈值
    bool board_config_valid;  // 板级参数校验通过
    uint32_t active_faults;   // 本拍活动故障掩码
    uint32_t latched_faults;  // 已锁存故障掩码
} PowerStateInput;

/* 状态机实例。由 PowerState_Init 初始化，之后每拍调用 PowerState_Step。 */
typedef struct {
    app_state_t state;      // 当前状态
    uint16_t confirm_count; // 当前阶段的连续确认计数
    bool pwm_allowed;       // 是否允许开放 PWM（唯一放行开关）
} PowerStateMachine;

// 初始化到 OFF 且禁止 PWM。NULL 安全
void PowerState_Init(PowerStateMachine *machine);

// 执行一次状态跳转。任一活动或锁存故障都会立即进入 FAULT_LATCH
// 只有 RUN 且全部安全条件同时满足时 pwm_allowed 才为真
void PowerState_Step(PowerStateMachine *machine, const PowerStateConfig *cfg, const PowerStateInput *input);
