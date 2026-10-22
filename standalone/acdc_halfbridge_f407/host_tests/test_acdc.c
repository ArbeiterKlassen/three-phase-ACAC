#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "acdc_application.h"
#include "acdc_config.h"
#include "acdc_control.h"
#include "acdc_measurement.h"

static int g_failures;

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #condition);                                              \
            g_failures++;                                                                                              \
        }                                                                                                              \
    } while (0)

static uint16_t RawFromPhysical(const AcdcCalibration *cal, float physical) {
    float raw = cal->offset_count + physical / cal->gain;
    if (raw < 0.0f) raw = 0.0f;
    if (raw > 4095.0f) raw = 4095.0f;
    return (uint16_t)(raw + 0.5f);
}

static uint32_t Pair(uint16_t adc1, uint16_t adc2) {
    return ((uint32_t)adc1 & 0xFFFFU) | (((uint32_t)adc2 & 0xFFFFU) << 16U);
}

static void MakeFrame(uint32_t frame[ACDC_ADC_RANK_COUNT], float iin, float vin, float vcf, float vdc) {
    frame[0] = Pair(RawFromPhysical(&g_acdc_cal_iin, iin), RawFromPhysical(&g_acdc_cal_vin, vin));
    frame[1] = 0U;
    frame[2] = Pair(RawFromPhysical(&g_acdc_cal_vcf, vcf), 0U);
    frame[3] = Pair(RawFromPhysical(&g_acdc_cal_vdc, vdc), RawFromPhysical(&g_acdc_cal_vin, vin));
}

static AcdcControlConfig TestConfig(void) {
    return (AcdcControlConfig){
        .vbus_target_v = ACDC_DC_VOLTAGE_TARGET_V,
        .vbus_min_v = ACDC_DC_RUN_MIN_V,
        .vbus_max_v = ACDC_DC_MAX_V,
        .vin_rms_min_v = ACDC_VIN_RMS_MIN_V,
        .vin_abs_max_v = ACDC_VIN_INSTANT_MAX_V,
        .iin_abs_max_a = ACDC_IIN_ABS_MAX_A,
        .iin_command_max_a = ACDC_IIN_COMMAND_MAX_A,
        .vbus_slew_v_per_s = ACDC_DC_REFERENCE_SLEW_V_PER_S,
        .duty_min = ACDC_DUTY_MIN,
        .duty_max = ACDC_DUTY_MAX,
    };
}

static void TestMeasurement(void) {
    uint32_t frame[ACDC_ADC_RANK_COUNT];
    AcdcMeasurement m;

    MakeFrame(frame, 2.5f, 36.0f, 24.0f, 55.0f);
    CHECK(AcdcMeasurement_Convert(frame, &m));
    CHECK(m.valid);
    CHECK(fabsf(m.iin_a - 2.5f) < 0.01f);
    CHECK(fabsf(m.vin_v - 36.0f) < 0.12f);
    CHECK(fabsf(m.vcf_v - 24.0f) < 0.12f);
    CHECK(fabsf(m.vdc_v - 55.0f) < 0.04f);

    frame[0] = Pair(1U, 4095U);
    CHECK(!AcdcMeasurement_Convert(frame, &m));
    CHECK(!m.valid);
    CHECK(m.iin_overrange);
}

static void TestControlWarmupAndRun(void) {
    AcdcControl control;
    AcdcControlConfig cfg = TestConfig();
    AcdcControlOutput out;
    AcdcMeasurement m = {
        .iin_a = 2.0f,
        .vin_v = 36.0f,
        .vcf_v = 20.0f,
        .vdc_v = 50.0f,
        .valid = true,
    };
    uint32_t i;

    CHECK(AcdcControl_Init(&control, &cfg));
    for (i = 0U; i < (ACDC_RMS_WINDOW_SAMPLES - 1U); i++) {
        CHECK(AcdcControl_Step(&control, &m, false, &out));
        CHECK(!out.gates_enabled);
    }
    CHECK(AcdcControl_Step(&control, &m, false, &out));
    CHECK(fabsf(out.input_voltage_rms_v - 36.0f) < 0.25f);

    CHECK(AcdcControl_Step(&control, &m, true, &out));
    CHECK(out.valid);
    CHECK(out.gates_enabled);
    CHECK(out.low_side_duty >= ACDC_DUTY_MIN);
    CHECK(out.low_side_duty <= ACDC_DUTY_MAX);

    m.valid = false;
    CHECK(!AcdcControl_Step(&control, &m, true, &out));
    CHECK(!out.gates_enabled);
}

static void TestApplicationStartAndProtection(void) {
    AcdcApplication app;
    AcdcControlOutput out;
    uint32_t frame[ACDC_ADC_RANK_COUNT];
    AcdcSnapshot snapshot;
    uint32_t i;

    AcdcApp_Init(&app);
    CHECK(app.state == ACDC_STATE_OFF);
    AcdcApp_ToggleRequest(&app);
    CHECK(app.start_requested);
    CHECK(app.state == ACDC_STATE_OFF);

    MakeFrame(frame, 1.0f, 36.0f, 25.0f, 50.0f);
    for (i = 0U; i < (ACDC_RMS_WINDOW_SAMPLES + ACDC_PRECHARGE_CONFIRM_SAMPLES + 10U); i++)
        AcdcApp_FastStep(&app, frame, &out);
    CHECK(app.state == ACDC_STATE_RUN);
    CHECK(!app.faults);
    CHECK(out.gates_enabled);
    CHECK(out.low_side_duty >= ACDC_DUTY_MIN);
    CHECK(out.low_side_duty <= ACDC_DUTY_MAX);

    /* 5.63 A 仍在 TA 正向可测量程内，因此应先判过流而不是量程无效。 */
    MakeFrame(frame, ACDC_IIN_ABS_MAX_A + 0.03f, 36.0f, 20.0f, 55.0f);
    AcdcApp_FastStep(&app, frame, &out);
    CHECK(app.state == ACDC_STATE_FAULT);
    CHECK((app.faults & ACDC_FAULT_INPUT_OVERCURRENT) != 0U);
    CHECK(!out.gates_enabled);

    /* 原始码越上限也必须报过流，而不是报“测量故障”。 */
    frame[0] = Pair(4095U, RawFromPhysical(&g_acdc_cal_vin, 36.0f));
    AcdcApp_FastStep(&app, frame, &out);
    CHECK((app.faults & ACDC_FAULT_INPUT_OVERCURRENT) != 0U);

    AcdcApp_ToggleRequest(&app);
    CHECK(app.state == ACDC_STATE_OFF);
    CHECK(app.faults == 0U);
    AcdcApp_GetSnapshot(&app, &snapshot);
    CHECK(snapshot.state == ACDC_STATE_OFF);
}

static void TestUndervoltageTripsWithoutControlFault(void) {
    AcdcApplication app;
    AcdcControlOutput out;
    uint32_t frame[ACDC_ADC_RANK_COUNT];
    uint32_t i;

    AcdcApp_Init(&app);
    AcdcApp_ToggleRequest(&app);
    MakeFrame(frame, 1.0f, 36.0f, 25.0f, 50.0f);
    for (i = 0U; i < (ACDC_RMS_WINDOW_SAMPLES + ACDC_PRECHARGE_CONFIRM_SAMPLES + 10U); i++)
        AcdcApp_FastStep(&app, frame, &out);
    CHECK(app.state == ACDC_STATE_RUN);

    MakeFrame(frame, 3.0f, 36.0f, 20.0f, 29.0f);
    for (i = 0U; i < (ACDC_UNDERVOLTAGE_CONFIRM_SAMPLES - 1U); i++) {
        AcdcApp_FastStep(&app, frame, &out);
        CHECK(!out.gates_enabled);
    }
    CHECK(app.state == ACDC_STATE_RUN);
    AcdcApp_FastStep(&app, frame, &out);
    CHECK(app.state == ACDC_STATE_FAULT);
    CHECK((app.faults & ACDC_FAULT_BUS_UNDERVOLTAGE) != 0U);
    CHECK((app.faults & ACDC_FAULT_CONTROL) == 0U);
}

static void TestVoltageProtection(void) {
    AcdcApplication app;
    AcdcControlOutput out;
    uint32_t frame[ACDC_ADC_RANK_COUNT];

    AcdcApp_Init(&app);
    MakeFrame(frame, 1.0f, 66.0f, 20.0f, 50.0f);
    AcdcApp_FastStep(&app, frame, &out);
    CHECK((app.faults & ACDC_FAULT_INPUT_OVERVOLTAGE) != 0U);
    CHECK(!out.gates_enabled);

    AcdcApp_Init(&app);
    MakeFrame(frame, 1.0f, 36.0f, 20.0f, 67.0f);
    AcdcApp_FastStep(&app, frame, &out);
    CHECK((app.faults & ACDC_FAULT_BUS_OVERVOLTAGE) != 0U);
    CHECK(!out.gates_enabled);
}

int main(void) {
    TestMeasurement();
    TestControlWarmupAndRun();
    TestApplicationStartAndProtection();
    TestUndervoltageTripsWithoutControlFault();
    TestVoltageProtection();

    if (g_failures != 0) {
        printf("%d host test(s) failed.\n", g_failures);
        return 1;
    }
    printf("All ACDC host tests passed.\n");
    return 0;
}
