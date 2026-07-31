#include "power_state.h"

#include <stddef.h>

static uint16_t PowerState_Required(uint16_t configured)
{
    return (configured == 0U) ? 1U : configured;
}

static bool PowerState_Confirmed(PowerStateMachine *machine,
                                 bool condition,
                                 uint16_t required)
{
    /* 条件必须连续成立；任一拍失效都重新计数，抑制联锁信号抖动。 */
    if (!condition)
    {
        machine->confirm_count = 0U;
        return false;
    }
    if (machine->confirm_count < UINT16_MAX)
    {
        machine->confirm_count++;
    }
    return machine->confirm_count >= PowerState_Required(required);
}

static void PowerState_Enter(PowerStateMachine *machine,
                             app_state_t state)
{
    machine->state = state;
    machine->confirm_count = 0U;
}

void PowerState_Init(PowerStateMachine *machine)
{
    if (machine != NULL)
    {
        *machine = (PowerStateMachine){0};
        machine->state = OFF;
    }
}

void PowerState_Step(PowerStateMachine *machine,
                     const PowerStateConfig *cfg,
                     const PowerStateInput *input)
{
    bool faulted;

    if ((machine == NULL) || (cfg == NULL) || (input == NULL))
    {
        return;
    }
    machine->pwm_allowed = false;
    faulted = (input->active_faults != 0UL)
           || (input->latched_faults != 0UL);

    /* 故障优先级高于所有正常状态跳转，并立即撤销 PWM 许可。 */
    if (faulted)
    {
        PowerState_Enter(machine, FAULT_LATCH);
        return;
    }

    if (!input->board_config_valid)
    {
        PowerState_Enter(machine, OFF);
        return;
    }

    if (machine->state == FAULT_LATCH)
    {
        if (input->reset_fault_request && input->stop_request)
        {
            PowerState_Enter(machine, DISCHARGE);
        }
        return;
    }

    if (input->stop_request && (machine->state != OFF)
        && (machine->state != DISCHARGE))
    {
        PowerState_Enter(machine, DISCHARGE);
    }

    switch (machine->state)
    {
    case OFF:
        if (input->start_request)
        {
            PowerState_Enter(machine, SELF_TEST);
        }
        break;

    case SELF_TEST:
        if (PowerState_Confirmed(machine,
                                 input->measurement_valid,
                                 cfg->self_test_confirm_samples))
        {
            PowerState_Enter(machine, PRECHARGE);
        }
        break;

    case PRECHARGE:
        if (input->precharge_done && input->measurement_valid)
        {
            PowerState_Enter(machine, READY);
        }
        break;

    case READY:
        if (!input->precharge_done)
        {
            PowerState_Enter(machine, PRECHARGE);
        }
        else if (PowerState_Confirmed(machine,
                                      input->phase_ready
                                          && input->measurement_valid,
                                      cfg->ready_confirm_samples))
        {
            PowerState_Enter(machine, RUN);
        }
        break;

    case RUN:
        if (!input->measurement_valid || !input->phase_ready
            || !input->precharge_done || input->stop_request)
        {
            PowerState_Enter(machine, DISCHARGE);
        }
        break;

    case DISCHARGE:
        if (PowerState_Confirmed(machine,
                                 input->discharge_done,
                                 cfg->discharge_confirm_samples))
        {
            PowerState_Enter(machine, OFF);
        }
        break;

    case FAULT_LATCH:
    default:
        PowerState_Enter(machine, FAULT_LATCH);
        break;
    }

    machine->pwm_allowed = (machine->state == RUN)
                        && input->board_config_valid
                        && input->measurement_valid
                        && input->precharge_done
                        && input->phase_ready
                        && !faulted
                        && !input->stop_request;
}
