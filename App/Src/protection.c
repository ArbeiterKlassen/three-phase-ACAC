#include "protection.h"

#include <math.h>
#include <stddef.h>

static bool Protection_ConfigValid(const ProtectionConfig *cfg)
{
    return (cfg != NULL)
        && isfinite(cfg->input_current_abs_max_a)
        && isfinite(cfg->phase_current_abs_max_a)
        && isfinite(cfg->input_voltage_abs_max_v)
        && isfinite(cfg->dc_voltage_min_run_v)
        && isfinite(cfg->dc_voltage_max_v)
        && isfinite(cfg->output_voltage_abs_max_v)
        && (cfg->input_current_abs_max_a > 0.0f)
        && (cfg->phase_current_abs_max_a > 0.0f)
        && (cfg->input_voltage_abs_max_v > 0.0f)
        && (cfg->dc_voltage_min_run_v >= 0.0f)
        && (cfg->dc_voltage_max_v > cfg->dc_voltage_min_run_v)
        && (cfg->output_voltage_abs_max_v > 0.0f)
        && (cfg->stale_limit_samples > 0U);
}

static bool Protection_MeasurementFinite(const Measurements *m)
{
    return (m != NULL) && m->valid
        && isfinite(m->iin) && isfinite(m->vin)
        && isfinite(m->ia) && isfinite(m->ib) && isfinite(m->ic)
        && isfinite(m->va) && isfinite(m->vb) && isfinite(m->vc)
        && isfinite(m->vdc);
}

void Protection_Init(Protection *protection)
{
    if (protection != NULL)
    {
        *protection = (Protection){0};
    }
}

uint32_t Protection_Evaluate(Protection *protection,
                             const ProtectionConfig *cfg,
                             const Measurements *measurement,
                             bool run_expected,
                             bool external_fault,
                             bool loop_overrun)
{
    uint32_t faults = PROTECTION_FAULT_NONE;

    if (protection == NULL)
    {
        return PROTECTION_FAULT_CONTROL;
    }
    if (!Protection_ConfigValid(cfg)
        || !Protection_MeasurementFinite(measurement))
    {
        faults |= PROTECTION_FAULT_MEASUREMENT;
    }
    else
    {
        /* 序号不变表示控制循环重复使用旧帧，连续达到阈值后报采样陈旧。 */
        if (!protection->sequence_seen)
        {
            protection->sequence_seen = true;
            protection->last_sequence = measurement->sequence;
            protection->stale_count = 0U;
        }
        else if (measurement->sequence == protection->last_sequence)
        {
            if (protection->stale_count < UINT16_MAX)
            {
                protection->stale_count++;
            }
        }
        else
        {
            protection->last_sequence = measurement->sequence;
            protection->stale_count = 0U;
        }

        if (protection->stale_count >= cfg->stale_limit_samples)
        {
            faults |= PROTECTION_FAULT_SAMPLE_STALE;
        }
        if (fabsf(measurement->iin) > cfg->input_current_abs_max_a)
        {
            faults |= PROTECTION_FAULT_INPUT_OVERCURRENT;
        }
        if ((fabsf(measurement->ia) > cfg->phase_current_abs_max_a)
         || (fabsf(measurement->ib) > cfg->phase_current_abs_max_a)
         || (fabsf(measurement->ic) > cfg->phase_current_abs_max_a))
        {
            faults |= PROTECTION_FAULT_PHASE_OVERCURRENT;
        }
        if (fabsf(measurement->vin) > cfg->input_voltage_abs_max_v)
        {
            faults |= PROTECTION_FAULT_INPUT_OVERVOLTAGE;
        }
        if (measurement->vdc > cfg->dc_voltage_max_v)
        {
            faults |= PROTECTION_FAULT_DC_OVERVOLTAGE;
        }
        if (run_expected
            && (measurement->vdc < cfg->dc_voltage_min_run_v))
        {
            faults |= PROTECTION_FAULT_DC_UNDERVOLTAGE;
        }
        if ((fabsf(measurement->va) > cfg->output_voltage_abs_max_v)
         || (fabsf(measurement->vb) > cfg->output_voltage_abs_max_v)
         || (fabsf(measurement->vc) > cfg->output_voltage_abs_max_v))
        {
            faults |= PROTECTION_FAULT_OUTPUT_OVERVOLTAGE;
        }
    }

    if (external_fault)
    {
        faults |= PROTECTION_FAULT_EXTERNAL;
    }
    if (loop_overrun)
    {
        faults |= PROTECTION_FAULT_LOOP_OVERRUN;
    }

    /* active 反映本拍状态，latched 按位保留历史，直到显式且安全地复位。 */
    protection->active_faults = faults;
    protection->latched_faults |= faults;
    return faults;
}

void Protection_ForceFault(Protection *protection, uint32_t faults)
{
    if (protection != NULL)
    {
        protection->active_faults |= faults;
        protection->latched_faults |= faults;
    }
}

bool Protection_ClearLatched(Protection *protection)
{
    if ((protection == NULL)
        || (protection->active_faults != PROTECTION_FAULT_NONE))
    {
        return false;
    }

    protection->latched_faults = PROTECTION_FAULT_NONE;
    return true;
}
