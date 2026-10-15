/*
 * 电源状态机。
 *
 *     OFF → SELF_TEST → PRECHARGE → READY → RUN
 *      ↑                                        │
 *      └──── DISCHARGE ←── 停机/条件丢失 ────────┘
 *
 *     任意状态出现活动或锁存故障 → FAULT_LATCH（只能靠"复位请求 + 停机请求"退出）
 *
 * 三条贯穿全文件的规则：
 *
 *   1. **每拍先撤销 pwm_allowed，最后再按全部条件重新授予**。
 *      万一中间某条 return 提前退出，PWM 也一定是关的——这是刻意的"默认安全"。
 *
 *   2. **故障优先级最高**，在任何正常跳转之前处理，且立即撤销 PWM。
 *
 *   3. **所有需要"持续成立"的条件都走连续确认计数**（见 PowerState_Confirmed），
 *      用来滤掉联锁信号抖动。拍数由 cfg 给出，现场可调。
 */

#include "power_state.h"

#include <stddef.h>

/*
 * 把配置的"0 拍"规整为 1 拍。
 * 0 会让确认条件恒不成立（计数永远 >= 0 且条件为真却从没计数过），
 * 所以这里兜一个底，避免配置成 0 之后状态机卡死在原地。
 */
static uint16_t PowerState_Required(uint16_t configured) {
    return (configured == 0U) ? 1U : configured;
}

/*
 * 连续确认计数器。condition 为假时清零重计，为真时累加。
 * 返回计数是否已达到 required。
 */
static bool PowerState_Confirmed(PowerStateMachine *machine, bool condition, uint16_t required) {
    /* 条件必须连续成立；任一拍失效都重新计数，抑制联锁信号抖动。 */
    if (!condition) {
        machine->confirm_count = 0U;
        return false;
    }
    if (machine->confirm_count < UINT16_MAX) machine->confirm_count++;
    return machine->confirm_count >= PowerState_Required(required);
}

/*
 * 进入新状态。**必须走这个函数**而不是直接赋值 state——
 * 它同时把确认计数清零，否则上个状态的计数会漏到新状态，
 * 让本该等 N 拍的条件在第 1 拍就成立。
 */
static void PowerState_Enter(PowerStateMachine *machine, app_state_t state) {
    machine->state = state;
    machine->confirm_count = 0U;
}

/* 初始化到 OFF 且禁止 PWM。NULL 安全 */
void PowerState_Init(PowerStateMachine *machine) {
    if (machine != NULL) {
        *machine = (PowerStateMachine){0};
        machine->state = OFF;
    }
}

/*
 * 执行一次状态跳转。每拍调用一次（与快环同频）。
 *
 * 处理顺序：
 *   ① 先关 PWM，再判故障（默认安全）
 *   ② 板级配置无效 → 退回 OFF
 *   ③ FAULT_LATCH 特殊处理：只有"复位请求 + 停机请求"同时具备才放行到 DISCHARGE
 *   ④ 停机请求 → DISCHARGE
 *   ⑤ 按当前状态做跳转
 *   ⑥ 最后按全部条件重新授予 pwm_allowed
 *
 * DISCHARGE 里那段逻辑容易看错，单独说明：交流源接在桥上时**会通过二极管
 * 持续给母线充电**，所以"'等母线降下来'才允许重启"这条在带电现场等于
 * 永远不让你重启。因此只要条件齐备（启动请求 + 测量有效 + 预充完成 +
 * 相位就绪），就直接从 DISCHARGE 重新走完整的启动链（回到 SELF_TEST），
 * 不必等放电完成。
 */
void PowerState_Step(PowerStateMachine *machine, const PowerStateConfig *cfg, const PowerStateInput *input) {
    bool faulted;

    if ((machine == NULL) || (cfg == NULL) || (input == NULL)) return;
    machine->pwm_allowed = false;
    faulted = (input->active_faults != 0UL) || (input->latched_faults != 0UL);

    /* 故障优先级高于所有正常状态跳转，并立即撤销 PWM 许可。 */
    if (faulted) {
        PowerState_Enter(machine, FAULT_LATCH);
        return;
    }

    if (!input->board_config_valid) {
        PowerState_Enter(machine, OFF);
        return;
    }

    if (machine->state == FAULT_LATCH) {
        if (input->reset_fault_request && input->stop_request) PowerState_Enter(machine, DISCHARGE);
        return;
    }

    if (input->stop_request && (machine->state != OFF) && (machine->state != DISCHARGE))
        PowerState_Enter(machine, DISCHARGE);

    switch (machine->state) {
    case OFF:
        if (input->start_request) PowerState_Enter(machine, SELF_TEST);
        break;

    case SELF_TEST:
        if (PowerState_Confirmed(machine, input->measurement_valid, cfg->self_test_confirm_samples))
            PowerState_Enter(machine, PRECHARGE);
        break;

    case PRECHARGE:
        if (input->precharge_done && input->measurement_valid) PowerState_Enter(machine, READY);
        break;

    case READY:
        if (!input->precharge_done) {
            PowerState_Enter(machine, PRECHARGE);
        } else if (PowerState_Confirmed(machine, input->phase_ready && input->measurement_valid,
                                        cfg->ready_confirm_samples)) {
            PowerState_Enter(machine, RUN);
        }
        break;

    case RUN:
        if (!input->measurement_valid || !input->phase_ready || !input->precharge_done || input->stop_request)
            PowerState_Enter(machine, DISCHARGE);
        break;

    case DISCHARGE:
        /*
         * A connected AC source can passively charge the DC link through
         * the bridge, so waiting for a low bus would block every restart.
         * Re-enter the complete startup chain only on an explicit request
         * with valid measurement, passive precharge and phase conditions.
         */
        if (input->start_request && !input->stop_request && input->measurement_valid && input->precharge_done &&
            input->phase_ready) {
            PowerState_Enter(machine, SELF_TEST);
        } else if (PowerState_Confirmed(machine, input->discharge_done, cfg->discharge_confirm_samples)) {
            PowerState_Enter(machine, OFF);
        }
        break;

    case FAULT_LATCH:
    default:
        PowerState_Enter(machine, FAULT_LATCH);
        break;
    }

    machine->pwm_allowed = (machine->state == RUN) && input->board_config_valid && input->measurement_valid &&
                           input->precharge_done && input->phase_ready && !faulted && !input->stop_request;
}
