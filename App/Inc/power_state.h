#ifndef POWER_STATE_H
#define POWER_STATE_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t self_test_confirm_samples;
    uint16_t ready_confirm_samples;
    uint16_t discharge_confirm_samples;
} PowerStateConfig;

typedef struct
{
    bool start_request;
    bool stop_request;
    bool reset_fault_request;
    bool precharge_done;
    bool phase_ready;
    bool measurement_valid;
    bool discharge_done;
    bool board_config_valid;
    uint32_t active_faults;
    uint32_t latched_faults;
} PowerStateInput;

typedef struct
{
    app_state_t state;
    uint16_t confirm_count;
    bool pwm_allowed;
} PowerStateMachine;

/**
 * @brief 初始化电源状态机到 OFF，并禁止 PWM。
 * @param[out] machine 电源状态机实例；NULL 将被忽略。
 */
void PowerState_Init(PowerStateMachine *machine);

/**
 * @brief 根据启停请求、联锁、测量和故障执行一次电源状态跳转。
 * @param[in,out] machine 电源状态机实例。
 * @param[in] cfg 自检、就绪和放电条件的连续确认拍数。
 * @param[in] input 当前请求、联锁、测量有效性和故障输入。
 * @note 仅在 RUN 且所有安全条件同时满足时置 pwm_allowed=true；
 *       任一活动或锁存故障都会立即进入 FAULT_LATCH。
 */
void PowerState_Step(PowerStateMachine *machine,
                     const PowerStateConfig *cfg,
                     const PowerStateInput *input);

#ifdef __cplusplus
}
#endif

#endif
