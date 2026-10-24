#include "acdc_application.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void AcdcApp_LatchFault(AcdcApplication *app, uint32_t fault);

static void AcdcApp_SafeOutput(AcdcControlOutput *out) {
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->low_side_duty = 0.5f;
    }
}

/*
 * 母线/输入过压先立即关门，但必须连续确认 3 拍才锁存，避免把开关沿
 * 单拍尖峰当成真实过压。输入过流等其余保护仍在 RUN 中立即锁存。
 */
static void AcdcApp_HandleProtection(AcdcApplication *app, uint32_t fault, AcdcControlOutput *out) {
    const bool voltage_overvoltage = (fault == ACDC_FAULT_BUS_OVERVOLTAGE) || (fault == ACDC_FAULT_INPUT_OVERVOLTAGE);

    /*
     * 只关本拍门极，不能调用 disabled 控制步：它会清零母线参考，
     * 让一次开关尖峰把整个软启动重新开始。
     */
    AcdcApp_SafeOutput(out);

    if (app->invalid_count < UINT32_MAX) app->invalid_count++;
    if (fault == ACDC_FAULT_BUS_OVERVOLTAGE) {
        if (app->bus_overvoltage_count < UINT32_MAX) app->bus_overvoltage_count++;
    } else {
        app->bus_overvoltage_count = 0U;
    }
    if (fault == ACDC_FAULT_INPUT_OVERVOLTAGE) {
        if (app->input_overvoltage_count < UINT32_MAX) app->input_overvoltage_count++;
    } else {
        app->input_overvoltage_count = 0U;
    }

    if (app->state == ACDC_STATE_RUN) {
        if (!voltage_overvoltage || (app->bus_overvoltage_count >= ACDC_OVERVOLTAGE_CONFIRM_SAMPLES) ||
            (app->input_overvoltage_count >= ACDC_OVERVOLTAGE_CONFIRM_SAMPLES)) {
            AcdcApp_LatchFault(app, fault);
        }
    } else if (app->invalid_count >= ACDC_INVALID_CONFIRM_SAMPLES) {
        AcdcApp_LatchFault(app, fault);
    }
}

static void AcdcApp_LatchFault(AcdcApplication *app, uint32_t fault) {
    app->faults |= fault;
    app->start_requested = false;
    /*
     * Commissioning 锁优先级最高。未标定硬件即使遇到故障也不能通过
     * “清故障”路径进入 OFF，再被按键启动。
     */
    app->state = ACDC_HARDWARE_COMMISSIONED ? ACDC_STATE_FAULT : ACDC_STATE_LOCKED;
}

static uint32_t AcdcApp_MeasurementFault(const AcdcMeasurement *measurement) {
    if (fabsf(measurement->iin_a) > ACDC_IIN_ABS_MAX_A) return ACDC_FAULT_INPUT_OVERCURRENT;
    if (measurement->vcf_v > ACDC_VCF_INSTANT_MAX_V) return ACDC_FAULT_INPUT_OVERVOLTAGE;
    if (measurement->vdc_v > ACDC_DC_MAX_V) return ACDC_FAULT_BUS_OVERVOLTAGE;
    return 0U;
}

static uint32_t AcdcApp_InvalidMeasurementFault(const AcdcMeasurement *measurement) {
    if (measurement->iin_overrange) return ACDC_FAULT_INPUT_OVERCURRENT;
    if (measurement->vdc_overrange) return ACDC_FAULT_BUS_OVERVOLTAGE;
    if (measurement->vcf_overrange) return ACDC_FAULT_INPUT_OVERVOLTAGE;
    return ACDC_FAULT_MEASUREMENT;
}

static void AcdcApp_RunControl(AcdcApplication *app, AcdcControlOutput *out) {
    bool ok;

    /*
     * 母线跌到 RUN 下限后先立即关门，再等 40 ms 判断是否为真实欠压。
     * 不能等 40 ms 才关，也不能让控制器的“母线下限失败”抢先报成
     * ACDC_FAULT_CONTROL，否则现场看不到真正的故障原因。
     */
    if (app->measurement.vdc_v < ACDC_DC_RUN_MIN_V) {
        if (app->undervoltage_count < UINT32_MAX) app->undervoltage_count++;
        /* 单拍欠压只关门，不重置已经爬升的软启动参考。 */
        AcdcApp_SafeOutput(out);
        if (app->undervoltage_count >= ACDC_UNDERVOLTAGE_CONFIRM_SAMPLES)
            AcdcApp_LatchFault(app, ACDC_FAULT_BUS_UNDERVOLTAGE);
        return;
    }
    app->undervoltage_count = 0U;

    ok = AcdcControl_Step(&app->control, &app->measurement, true, out);
    if (!ok || !out->valid) {
        AcdcApp_LatchFault(app, ACDC_FAULT_CONTROL);
        AcdcApp_SafeOutput(out);
    }
}

void AcdcApp_Init(AcdcApplication *app) {
    const AcdcControlConfig cfg = {
        .vbus_target_v = ACDC_DC_VOLTAGE_TARGET_V,
        .vbus_min_v = ACDC_DC_RUN_MIN_V,
        .vbus_max_v = ACDC_DC_MAX_V,
        .vcf_rms_min_v = ACDC_VCF_RMS_MIN_V,
        .vcf_abs_max_v = ACDC_VCF_INSTANT_MAX_V,
        .iin_abs_max_a = ACDC_IIN_ABS_MAX_A,
        .iin_command_max_a = ACDC_IIN_COMMAND_MAX_A,
        .vbus_slew_v_per_s = ACDC_DC_REFERENCE_SLEW_V_PER_S,
        .duty_min = ACDC_DUTY_MIN,
        .duty_max = ACDC_DUTY_MAX,
    };

    if (app == NULL) return;
    memset(app, 0, sizeof(*app));
    app->state = ACDC_HARDWARE_COMMISSIONED ? ACDC_STATE_OFF : ACDC_STATE_LOCKED;
    app->start_requested = ACDC_AUTO_START != 0;
    if (!AcdcControl_Init(&app->control, &cfg)) {
        app->state = ACDC_HARDWARE_COMMISSIONED ? ACDC_STATE_FAULT : ACDC_STATE_LOCKED;
        app->faults = ACDC_FAULT_CONTROL;
    }
}

void AcdcApp_FastStep(AcdcApplication *app, const uint32_t frame[ACDC_ADC_RANK_COUNT], AcdcControlOutput *out) {
    AcdcMeasurement measurement;
    AcdcControlOutput local;
    bool measured;
    uint32_t protection_fault;

    if ((app == NULL) || (out == NULL)) return;
    AcdcApp_SafeOutput(&local);
    measured = AcdcMeasurement_Convert(frame, &measurement);
    app->measurement = measurement;
    app->control_output = local;

    if (!measured) {
        protection_fault = AcdcApp_InvalidMeasurementFault(&measurement);
        /*
         * RUN 时绝不在无效测量下继续保留门极；先立即关门。母线过压
         * 连续 3 拍锁存，其余运行故障立即锁存；启动阶段统一等 100 ms。
         */
        AcdcApp_HandleProtection(app, protection_fault, out);
        return;
    }

    protection_fault = AcdcApp_MeasurementFault(&measurement);
    if (protection_fault != 0U) {
        AcdcApp_HandleProtection(app, protection_fault, out);
        return;
    }
    app->invalid_count = 0U;
    app->bus_overvoltage_count = 0U;
    app->input_overvoltage_count = 0U;

    if (app->state == ACDC_STATE_LOCKED) {
        (void)AcdcControl_Step(&app->control, &measurement, false, &local);
        AcdcApp_SafeOutput(out);
        return;
    }

    if ((app->state == ACDC_STATE_FAULT) || !app->start_requested) {
        app->state = (app->state == ACDC_STATE_FAULT) ? ACDC_STATE_FAULT : ACDC_STATE_OFF;
        app->precharge_count = 0U;
        app->undervoltage_count = 0U;
        (void)AcdcControl_Step(&app->control, &measurement, false, &local);
        AcdcApp_SafeOutput(out);
        return;
    }

    if ((app->state == ACDC_STATE_OFF) || (app->state == ACDC_STATE_WAIT)) {
        bool ready;

        app->state = ACDC_STATE_WAIT;
        (void)AcdcControl_Step(&app->control, &measurement, false, &local);
        ready = (local.vcf_rms_v >= ACDC_VCF_RMS_MIN_V) &&
                (measurement.vdc_v >= fmaxf(ACDC_DC_PRECHARGE_MIN_V, ACDC_DC_RUN_MIN_V)) &&
                (measurement.vdc_v <= ACDC_DC_START_MAX_V);
        if (ready) {
            if (app->precharge_count < UINT32_MAX) app->precharge_count++;
        } else {
            app->precharge_count = 0U;
        }
        if (app->precharge_count >= ACDC_PRECHARGE_CONFIRM_SAMPLES) {
            app->state = ACDC_STATE_RUN;
            app->undervoltage_count = 0U;
        } else {
            AcdcApp_SafeOutput(out);
            return;
        }
    }

    if (app->state == ACDC_STATE_RUN) {
        AcdcApp_RunControl(app, out);
        app->control_output = *out;
    }
}

void AcdcApp_ToggleRequest(AcdcApplication *app) {
    if (app == NULL) return;
    if (!ACDC_HARDWARE_COMMISSIONED) {
        app->state = ACDC_STATE_LOCKED;
        app->start_requested = false;
        return;
    }
    if (app->state == ACDC_STATE_LOCKED) return;
    if (app->state == ACDC_STATE_FAULT) {
        app->faults = 0U;
        app->state = ACDC_STATE_OFF;
        app->start_requested = false;
        app->invalid_count = 0U;
        app->precharge_count = 0U;
        app->undervoltage_count = 0U;
        app->bus_overvoltage_count = 0U;
        app->input_overvoltage_count = 0U;
        AcdcControl_Reset(&app->control);
        return;
    }
    app->start_requested = !app->start_requested;
    if (!app->start_requested) {
        app->state = ACDC_STATE_OFF;
        app->precharge_count = 0U;
        app->undervoltage_count = 0U;
        app->bus_overvoltage_count = 0U;
        app->input_overvoltage_count = 0U;
        AcdcControl_Reset(&app->control);
    }
}

void AcdcApp_LatchExternalFault(AcdcApplication *app) {
    if (app != NULL) AcdcApp_LatchFault(app, ACDC_FAULT_EXTERNAL_BREAK);
}

void AcdcApp_LatchMeasurementFault(AcdcApplication *app) {
    if (app != NULL) AcdcApp_LatchFault(app, ACDC_FAULT_MEASUREMENT);
}

void AcdcApp_LatchControlFault(AcdcApplication *app) {
    if (app != NULL) AcdcApp_LatchFault(app, ACDC_FAULT_CONTROL);
}

void AcdcApp_GetSnapshot(const AcdcApplication *app, AcdcSnapshot *snapshot) {
    if ((app == NULL) || (snapshot == NULL)) return;
    snapshot->state = app->state;
    snapshot->faults = app->faults;
    snapshot->iin_a = app->measurement.iin_a;
    snapshot->vcf_v = app->measurement.vcf_v;
    snapshot->vdc_v = app->measurement.vdc_v;
    snapshot->duty_low_side = app->control_output.low_side_duty;
    snapshot->current_reference_a = app->control_output.current_reference_a;
    snapshot->vcf_rms_v = app->control_output.vcf_rms_v;
    snapshot->gates_enabled = app->control_output.gates_enabled;
}
