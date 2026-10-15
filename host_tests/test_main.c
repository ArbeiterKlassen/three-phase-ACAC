#include "app_config.h"
#include "app_control.h"
#include "inverter_control.h"
#include "measurement.h"
#include "phase_recurrence.h"
#include "power_state.h"
#include "protection.h"
#include "qpr_controller.h"
#include "svpwm.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static int g_failed;

#define TEST_CHECK(expr)                                                                                               \
    do {                                                                                                               \
        if (!(expr)) {                                                                                                 \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);                                                   \
            return false;                                                                                              \
        }                                                                                                              \
    } while (0)

static ChannelCalibration Test_Cal(float offset, float gain) {
    ChannelCalibration cal;
    cal.offset_count = offset;
    cal.gain = gain;
    cal.raw_min = 0U;
    cal.raw_max = 4095U;
    return cal;
}

static bool Test_Measurement(void) {
    MeasurementCalibration cal;
    AdcRawFrame raw = {0};
    Measurements m;
    AlphaBeta voltage;

    cal.ia = Test_Cal(100.0f, 1.0f);
    cal.ib = Test_Cal(100.0f, 1.0f);
    cal.vab = Test_Cal(100.0f, 1.0f);
    cal.vbc = Test_Cal(100.0f, 1.0f);
    cal.vdc = Test_Cal(0.0f, 0.1f);

    raw.ia = 110U;
    raw.ib = 120U;
    raw.vab = 130U;
    raw.vbc = 80U;
    raw.sequence = 7U;
    raw.valid = true;

    TEST_CHECK(Measurement_CalibrationValid(&cal));
    TEST_CHECK(Measurement_ConvertInverter(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.ic, -30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vca, -10.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.va, 13.333333f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(m.vb, -16.666667f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(m.vc, 3.333333f, 1.0e-4f));
    /* 母线通道不参与换算，恒为 0（调制分母由 App 层用 SET 值提供） */
    TEST_CHECK(TEST_NEAR(m.vdc, 0.0f, 1.0e-6f));
    voltage = Measurement_VoltageAlphaBeta(&m);
    TEST_CHECK(TEST_NEAR(voltage.alpha, m.va, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(voltage.beta, APP_1DIVSQRT3_F * (m.vb - m.vc), 1.0e-4f));

    raw.valid = false;
    TEST_CHECK(!Measurement_ConvertInverter(&cal, &raw, &m));
    TEST_CHECK(!m.valid);
    return true;
}

static bool Test_AdcRankMapping(void) {
    uint32_t pair[4];
    AdcRawFrame raw;

    pair[0] = ((uint32_t)202U << 16) | 101U;
    pair[1] = ((uint32_t)404U << 16) | 303U;
    pair[2] = ((uint32_t)606U << 16) | 505U;
    pair[3] = ((uint32_t)203U << 16) | 707U;
    TEST_CHECK(Measurement_UnpackInverterAdcPairs(pair, 99U, &raw));
    /* rank2：低 16 位 = ADC1/PC0 = Ib，高 16 位 = ADC2/PC1 = Ia */
    TEST_CHECK(raw.ib == 303U);
    TEST_CHECK(raw.ia == 404U);
    /* rank3：低 16 位 = ADC1/PC2 = Vab，高 16 位 = ADC2/PC3 = Vbc */
    TEST_CHECK(raw.vab == 505U);
    TEST_CHECK(raw.vbc == 606U);
    TEST_CHECK(raw.sequence == 99U);
    return true;
}


static bool Test_InverterMeasurementPath(void) {
    MeasurementCalibration cal = {0};
    uint32_t pair[4];
    AdcRawFrame raw;
    Measurements m;

    cal.ia = Test_Cal(100.0f, 1.0f);
    cal.ib = Test_Cal(100.0f, 1.0f);
    cal.vab = Test_Cal(100.0f, 1.0f);
    cal.vbc = Test_Cal(100.0f, 1.0f);

    TEST_CHECK(!Measurement_CalibrationValid(&cal));
    TEST_CHECK(Measurement_InverterCalibrationValid(&cal));

    pair[0] = ((uint32_t)4000U << 16) | 1U;
    pair[1] = ((uint32_t)120U << 16) | 110U;
    pair[2] = ((uint32_t)80U << 16) | 130U;
    pair[3] = UINT32_MAX;
    TEST_CHECK(Measurement_UnpackInverterAdcPairs(pair, 101U, &raw));
    TEST_CHECK(raw.valid);
    /* rank1 / rank4 不参与解包：整流相关字段本来就不存在 */
    TEST_CHECK(Measurement_ConvertInverter(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.ia, 20.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ib, 10.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ic, -30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vab, 30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vbc, -20.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vdc, 0.0f, 1.0e-6f));
    TEST_CHECK(m.sequence == 101U);

    cal.vbc.gain = 0.0f;
    TEST_CHECK(!Measurement_InverterCalibrationValid(&cal));
    TEST_CHECK(!Measurement_ConvertInverter(&cal, &raw, &m));
    return true;
}


static bool Test_PhaseAtFrequency(float frequency_hz, unsigned int sample_count) {
    PhaseRecurrence phase;
    ThreePhase abc;
    unsigned int i;
    float norm;

    TEST_CHECK(PhaseRecurrence_Init(&phase, frequency_hz, 50.0e-6f, 0.0f));
    for (i = 0U; i < sample_count; i++) PhaseRecurrence_Step(&phase);

    norm = phase.sin_value * phase.sin_value + phase.cos_value * phase.cos_value;
    TEST_CHECK(TEST_NEAR(norm, 1.0f, 3.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.sin_value, 0.0f, 3.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.cos_value, 1.0f, 3.0e-5f));
    abc = PhaseRecurrence_GetThreePhase(&phase, 10.0f);
    TEST_CHECK(TEST_NEAR(abc.a, 0.0f, 3.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.b, -8.660254f, 3.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.c, 8.660254f, 3.0e-4f));
    return true;
}

static bool Test_PhaseRecurrence(void) {
    PhaseRecurrence phase;
    ThreePhase abc;
    unsigned int i;
    float norm;

    TEST_CHECK(PhaseRecurrence_Init(&phase, 50.0f, 50.0e-6f, 0.0f));
    for (i = 0U; i < 400U; i++) PhaseRecurrence_Step(&phase);
    TEST_CHECK(TEST_NEAR(phase.sin_value, 0.0f, 2.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.cos_value, 1.0f, 2.0e-5f));
    norm = phase.sin_value * phase.sin_value + phase.cos_value * phase.cos_value;
    TEST_CHECK(TEST_NEAR(norm, 1.0f, 2.0e-5f));

    abc = PhaseRecurrence_GetThreePhase(&phase, 10.0f);
    TEST_CHECK(TEST_NEAR(abc.a + abc.b + abc.c, 0.0f, 1.0e-5f));
    TEST_CHECK(TEST_NEAR(abc.a, 0.0f, 2.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.b, -8.660254f, 2.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.c, 8.660254f, 2.0e-4f));
    TEST_CHECK(Test_PhaseAtFrequency(60.0f, 1000U));
    TEST_CHECK(Test_PhaseAtFrequency(30.0f, 2000U));
    TEST_CHECK(!PhaseRecurrence_SetFrequency(&phase, 10000.0f));
    return true;
}


static float Test_QprRms(float frequency_hz) {
    QprController qpr;
    float sum = 0.0f;
    float input;
    float output;
    unsigned int i;
    unsigned int count = 0U;

    if (!QprController_Init(&qpr, 0.0f, 10.0f, 2.0f * APP_PI_F * 50.0f, 5.0f, 50.0e-6f, -100.0f, 100.0f)) return -1.0f;

    for (i = 0U; i < 40000U; i++) {
        input = sinf(2.0f * APP_PI_F * frequency_hz * 50.0e-6f * (float)i);
        output = QprController_Step(&qpr, input);
        if (i >= 20000U) {
            sum += output * output;
            count++;
        }
    }
    return sqrtf(sum / (float)count);
}

static bool Test_Qpr(void) {
    float at_resonance = Test_QprRms(50.0f);
    float away = Test_QprRms(250.0f);
    QprController qpr;
    float output = 0.0f;
    unsigned int i;

    TEST_CHECK(at_resonance > 5.0f);
    TEST_CHECK(at_resonance > 8.0f * away);
    TEST_CHECK(QprController_Init(&qpr, 1.0f, 20.0f, 2.0f * APP_PI_F * 50.0f, 5.0f, 50.0e-6f, -100.0f, 100.0f));
    for (i = 0U; i < 2000U; i++) {
        (void)QprController_Step(&qpr, 5.0f);
        QprController_TrackOutput(&qpr, 5.0f, 0.0f);
    }
    for (i = 0U; i < 40000U; i++) {
        output = QprController_Step(&qpr, 0.0f);
        QprController_TrackOutput(&qpr, 0.0f, output);
    }
    TEST_CHECK(fabsf(output) < 0.1f);
    return true;
}

static bool Test_Modulation(void) {
    SvpwmOutput sv;

    TEST_CHECK(Svpwm_Generate(10.0f, 0.0f, 100.0f, 0.0f, 1.0f, &sv));
    TEST_CHECK(sv.sector == 1U);
    TEST_CHECK(TEST_NEAR((sv.duty_a - sv.duty_b) * 100.0f, 15.0f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(sv.duty_b, sv.duty_c, 1.0e-6f));

    TEST_CHECK(Svpwm_Generate(1000.0f, 500.0f, 100.0f, 0.02f, 0.98f, &sv));
    TEST_CHECK(sv.saturated);
    TEST_CHECK((sv.duty_a >= 0.02f) && (sv.duty_a <= 0.98f));
    TEST_CHECK((sv.duty_b >= 0.02f) && (sv.duty_b <= 0.98f));
    TEST_CHECK((sv.duty_c >= 0.02f) && (sv.duty_c <= 0.98f));
    return true;
}

static InverterControlConfig Test_InverterConfig(void) {
    InverterControlConfig cfg = {0};
    cfg.sample_time_s = 50.0e-6f;
    cfg.output_frequency_hz = 60.0f;
    cfg.voltage_rms_max = 100.0f;
    cfg.reference_slew_v_per_s = 1.0e6f;
    cfg.feedforward_gain = 0.0f;
    cfg.voltage_qpr_kp = 0.5f;
    cfg.voltage_qpr_kr = 0.0f;
    cfg.voltage_qpr_bandwidth_rad_s = 5.0f;
    cfg.current_reference_limit_a = 20.0f;
    cfg.current_qpr_kp = 2.0f;
    cfg.current_qpr_kr = 0.0f;
    cfg.current_qpr_bandwidth_rad_s = 20.0f;
    cfg.command_limit_v = 40.0f;
    cfg.dc_voltage_min_v = 10.0f;
    cfg.duty_min = 0.02f;
    cfg.duty_max = 0.98f;
    return cfg;
}

static bool Test_InverterCascade(void) {
    InverterControl control;
    InverterControlConfig cfg = Test_InverterConfig();
    InverterControlOutput out;
    Measurements m = {0};

    m.valid = true;
    m.vdc = 100.0f;
    TEST_CHECK(InverterControl_Init(&control, &cfg));
    InverterControl_SetReference(&control, 10.0f);
    TEST_CHECK(InverterControl_Step(&control, &m, true, &out));
    TEST_CHECK(out.valid && out.pwm.valid);
    TEST_CHECK(out.reference.beta < -10.0f);
    TEST_CHECK(out.current_reference.beta < -1.0f);
    TEST_CHECK(fabsf(out.voltage_command.beta) > fabsf(out.current_reference.beta));
    TEST_CHECK(fabsf(out.current_reference.beta) <= cfg.current_reference_limit_a);
    TEST_CHECK(InverterControl_SetFrequency(&control, 30.0f));
    TEST_CHECK(TEST_NEAR(control.cfg.output_frequency_hz, 30.0f, 1.0e-6f));
    return true;
}

static bool Test_InverterOpenLoop(void) {
    InverterControl first;
    InverterControl second;
    InverterControlOutput out_first;
    InverterControlOutput out_second;
    InverterControlConfig cfg = Test_InverterConfig();
    Measurements m_first = {0};
    Measurements m_second = {0};
    float saved_qpr_state;
    unsigned int i;

    cfg.reference_slew_v_per_s = 100.0f;
    m_first.valid = true;
    m_first.vdc = 60.0f;
    m_first.ia = 1.0f;
    m_first.ib = -0.5f;
    m_first.vab = 25.0f;
    m_first.vbc = -10.0f;
    m_second = m_first;
    m_second.ia = -3.0f;
    m_second.ib = 2.0f;
    m_second.vab = -40.0f;
    m_second.vbc = 35.0f;

    TEST_CHECK(InverterControl_Init(&first, &cfg));
    TEST_CHECK(InverterControl_Init(&second, &cfg));
    InverterControl_SetReference(&first, 10.0f);
    InverterControl_SetReference(&second, 10.0f);

    first.voltage_alpha_qpr.y1 = 7.0f;
    first.current_beta_qpr.e1 = -4.0f;
    second.voltage_alpha_qpr.y1 = 7.0f;
    second.current_beta_qpr.e1 = -4.0f;
    saved_qpr_state = first.voltage_alpha_qpr.y1;

    TEST_CHECK(InverterControl_StepOpenLoop(&first, &m_first, true, &out_first));
    TEST_CHECK(InverterControl_StepOpenLoop(&second, &m_second, true, &out_second));
    TEST_CHECK(out_first.valid && out_first.pwm.valid);
    TEST_CHECK(TEST_NEAR(first.reference_rms, cfg.reference_slew_v_per_s * cfg.sample_time_s, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.voltage_command.alpha, out_first.reference.alpha, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.voltage_command.beta, out_first.reference.beta, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.current_reference.alpha, 0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.current_reference.beta, 0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_a, out_second.pwm.duty_a, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_b, out_second.pwm.duty_b, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_c, out_second.pwm.duty_c, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(first.voltage_alpha_qpr.y1, saved_qpr_state, 1.0e-7f));

    cfg.reference_slew_v_per_s = 1.0e6f;
    for (i = 0U; i < 2U; i++) {
        InverterControl sweep;
        InverterControlOutput sweep_out;
        Measurements sweep_m = {0};
        float vdc = (i == 0U) ? 30.0f : 65.0f;
        unsigned int sample;

        sweep_m.valid = true;
        sweep_m.vdc = vdc;
        TEST_CHECK(InverterControl_Init(&sweep, &cfg));
        InverterControl_SetReference(&sweep, vdc * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_1DIVSQRT3_F);
        for (sample = 0U; sample < 400U; sample++) {
            TEST_CHECK(InverterControl_StepOpenLoop(&sweep, &sweep_m, true, &sweep_out));
            TEST_CHECK(!sweep_out.pwm.saturated);
            TEST_CHECK((sweep_out.pwm.duty_a >= cfg.duty_min) && (sweep_out.pwm.duty_a <= cfg.duty_max));
            TEST_CHECK((sweep_out.pwm.duty_b >= cfg.duty_min) && (sweep_out.pwm.duty_b <= cfg.duty_max));
            TEST_CHECK((sweep_out.pwm.duty_c >= cfg.duty_min) && (sweep_out.pwm.duty_c <= cfg.duty_max));
        }
    }

    TEST_CHECK(InverterControl_StepOpenLoop(&first, &m_first, false, &out_first));
    TEST_CHECK(TEST_NEAR(first.reference_rms, 0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_a, 0.5f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_b, 0.5f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_c, 0.5f, 1.0e-7f));
    return true;
}

static ProtectionConfig Test_ProtectionConfig(void) {
    ProtectionConfig cfg;
    cfg.phase_current_abs_max_a = 10.0f;
    cfg.dc_voltage_min_run_v = 20.0f;
    cfg.dc_voltage_max_v = 200.0f;
    cfg.output_voltage_abs_max_v = 100.0f;
    cfg.stale_limit_samples = 2U;
    return cfg;
}

static bool Test_Protection(void) {
    Protection protection;
    ProtectionConfig cfg = Test_ProtectionConfig();
    Measurements m = {0};

    m.valid = true;
    m.vdc = 50.0f;
    m.sequence = 1U;
    Protection_Init(&protection);

    /*
     * One invalid conversion stops PWM through measurement_valid. Latching
     * ADC INVALID takes a sustained 100 ms window (2000 fast samples) so the
     * power-on capacitor inrush, which briefly drives the current channel past
     * its raw range, cannot brick the board before the bus is even up.
     */
    m.valid = false;
    {
        uint32_t k;

        for (k = 0U; k < 1999U; k++) {
            TEST_CHECK(
                (Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_MEASUREMENT) == 0UL);
        }
        TEST_CHECK(protection.measurement_invalid_count == (uint16_t)1999U);
        TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_MEASUREMENT) !=
                   0UL);
    }
    m.valid = true;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m, true, false, false) == 0UL);
    TEST_CHECK(protection.measurement_invalid_count == 0U);
    TEST_CHECK(Protection_ClearLatched(&protection));

    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m, true, false, false) == 0UL);
    /* 相过流是"单拍即判"：出现就锁存，序列恢复后可以清除 */
    m.ia = 11.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_PHASE_OVERCURRENT) !=
               0UL);
    TEST_CHECK(!Protection_ClearLatched(&protection));
    m.ia = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m, true, false, false) == 0UL);
    TEST_CHECK(Protection_ClearLatched(&protection));
    TEST_CHECK(protection.latched_faults == 0UL);

    /* One isolated output-voltage excursion must not latch a fault. */
    m.va = 101.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) ==
               0UL);
    TEST_CHECK(protection.output_ov_count == 1U);
    m.va = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m, true, false, false) == 0UL);
    TEST_CHECK(protection.output_ov_count == 0U);

    /* Two consecutive excursions must retain the original protection. */
    m.va = 101.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) ==
               0UL);
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) !=
               0UL);
    m.va = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m, true, false, false) == 0UL);
    TEST_CHECK(Protection_ClearLatched(&protection));

    (void)Protection_Evaluate(&protection, &cfg, &m, true, false, false);
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m, true, false, false) & PROTECTION_FAULT_SAMPLE_STALE) != 0UL);
    return true;
}

static bool Test_PowerState(void) {
    PowerStateMachine machine;
    PowerStateConfig cfg = {2U, 2U, 2U};
    PowerStateInput in = {0};

    PowerState_Init(&machine);
    in.board_config_valid = true;
    in.measurement_valid = true;
    in.start_request = true;
    TEST_CHECK(machine.state == OFF);
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == SELF_TEST);
    PowerState_Step(&machine, &cfg, &in);
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == PRECHARGE);
    in.precharge_done = true;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == READY);
    in.phase_ready = true;
    PowerState_Step(&machine, &cfg, &in);
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == RUN);
    TEST_CHECK(machine.pwm_allowed);

    in.active_faults = PROTECTION_FAULT_EXTERNAL;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == FAULT_LATCH);
    TEST_CHECK(!machine.pwm_allowed);
    in.active_faults = 0UL;
    in.start_request = false;
    in.stop_request = true;
    in.reset_fault_request = true;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == DISCHARGE);

    /* A passively charged bus must not block an explicit safe restart. */
    in.reset_fault_request = false;
    in.stop_request = false;
    in.start_request = true;
    in.discharge_done = false;
    in.precharge_done = false;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == DISCHARGE);
    in.precharge_done = true;
    in.phase_ready = false;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == DISCHARGE);
    in.phase_ready = true;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == SELF_TEST);
    TEST_CHECK(!machine.pwm_allowed);

    /* With no restart request, the original confirmed discharge remains. */
    in.start_request = false;
    in.stop_request = true;
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == DISCHARGE);
    in.discharge_done = true;
    PowerState_Step(&machine, &cfg, &in);
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == OFF);
    return true;
}

static control_config_t Test_ValidInverterBoardConfig(void) {
    control_config_t cfg = g_board_power_config;

    cfg.config_valid = true;
    cfg.adc_calibrated = true;
    cfg.gate_driver_pins_valid = true;
    cfg.power_stage_valid = true;
    cfg.deadtime_valid = true;
    cfg.protection_thresholds_valid = true;
    cfg.measurement.ia.gain = 1.0f;
    cfg.measurement.ib.gain = 1.0f;
    cfg.measurement.vab.gain = 1.0f;
    cfg.measurement.vbc.gain = 1.0f;
    cfg.measurement.vdc.gain = 0.0f;
    cfg.inverter.voltage_qpr_kp = 1.0f;
    cfg.inverter.current_qpr_kp = 1.0f;
    return cfg;
}

static control_config_t Test_ValidFullBoardConfig(void) {
    control_config_t cfg = g_board_power_config;

    cfg.config_valid = true;
    cfg.adc_calibrated = true;
    cfg.gate_driver_pins_valid = true;
    cfg.power_stage_valid = true;
    cfg.deadtime_valid = true;
    cfg.protection_thresholds_valid = true;
    cfg.measurement.ia = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vdc = Test_Cal(0.0f, 0.1f);
    return cfg;
}

static bool Test_InverterOnlyAppFlow(void) {
    control_config_t cfg = Test_ValidInverterBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    unsigned int i;

    cfg.measurement.ia = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2000.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2000.0f, 0.1f);
    cfg.protection.dc_voltage_min_run_v = 10.0f;
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(app.inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(AppControl_SetInverterRunMode(&app, INVERTER_OPEN_LOOP));
    TEST_CHECK(app.inverter_run_mode == INVERTER_OPEN_LOOP);
    TEST_CHECK(!AppControl_SetInverterRunMode(&app, (inverter_run_mode_t)2));
    TEST_CHECK(AppControl_SetInverterRunMode(&app, INVERTER_CLOSED_LOOP));

    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.vdc = UINT16_MAX;
    raw.valid = true;
    command.enable = true;
    /* 独立输出参考未设置（<=0）时回退为按 Vdc 32/60 比例推导。 */
    command.inverter_vrms_ref = 0.0f;
    command.dc_voltage_ref = 30.0f;

    for (i = 1U; i <= 4U; i++) {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }

    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.inverter.enabled);
    TEST_CHECK(!AppControl_SetInverterRunMode(&app, INVERTER_OPEN_LOOP));
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(app.inverter.reference_target_rms,
                         30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_1DIVSQRT3_F, 1.0e-5f));

    command.dc_voltage_ref = 65.0f;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(TEST_NEAR(app.inverter.reference_target_rms,
                         65.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_1DIVSQRT3_F, 1.0e-5f));

    command.dc_voltage_ref = 30.0f;
    raw.vdc = 0U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.state == RUN);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_OVERVOLTAGE) == 0U);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_UNDERVOLTAGE) == 0U);
    TEST_CHECK(TEST_NEAR(app.inverter.reference_target_rms,
                         30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO * APP_1DIVSQRT3_F, 1.0e-5f));

    command.enable = false;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == OFF);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(AppControl_SetInverterRunMode(&app, INVERTER_OPEN_LOOP));

    return true;
}


static bool Test_OpenLoopProtection(void) {
    control_config_t cfg = Test_ValidInverterBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    unsigned int i;

    cfg.measurement.ia = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2000.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2000.0f, 0.1f);
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(AppControl_SetInverterRunMode(&app, INVERTER_OPEN_LOOP));

    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.vdc = UINT16_MAX;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 30.0f;

    for (i = 1U; i <= 4U; i++) {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.inverter.enabled);
    TEST_CHECK(app.inverter_run_mode == INVERTER_OPEN_LOOP);

    raw.ia = 2500U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == FAULT_LATCH);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK((out.faults & PROTECTION_FAULT_PHASE_OVERCURRENT) != 0U);
    TEST_CHECK(!app.fault_snapshot.valid);
    TEST_CHECK(AppControl_SetInverterRunMode(&app, INVERTER_CLOSED_LOOP));
    TEST_CHECK(app.inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(app.state.state == FAULT_LATCH);
    TEST_CHECK((app.protection.latched_faults & PROTECTION_FAULT_PHASE_OVERCURRENT) != 0U);
    return true;
}


/* 配置校验：合法配置通过；逐项破坏后必须被拒绝 */
static bool Test_BoardConfigGates(void) {
    control_config_t cfg = Test_ValidFullBoardConfig();

    TEST_CHECK(BoardPower_ValidateConfig(&cfg));

    cfg = Test_ValidFullBoardConfig();
    cfg.dc_voltage_ref = 29.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 65.0f;
    /* 输出参考与母线解耦：母线 65V 时输出仍可独立设 32V。 */
    cfg.output_line_voltage_rms = APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V;
    TEST_CHECK(BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 66.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V;
    /* 输出参考超出 28~34V 微调窗口判非法。 */
    cfg.output_line_voltage_rms = 40.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.output_line_voltage_rms = APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V;

    cfg = Test_ValidFullBoardConfig();
    cfg.measurement.ia.gain = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));

    cfg = Test_ValidFullBoardConfig();
    cfg.adc_calibrated = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.adc_calibrated = true;
    cfg.gate_driver_pins_valid = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.gate_driver_pins_valid = true;
    cfg.power_stage_valid = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.power_stage_valid = true;
    cfg.deadtime_valid = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.deadtime_valid = true;
    cfg.protection_thresholds_valid = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.protection_thresholds_valid = true;
    /* QPR 增益不能同时为 0，否则控制器等于停摆 */
    cfg.inverter.current_qpr_kp = 0.0f;
    cfg.inverter.current_qpr_kr = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));

    /* 保护阈值之间的大小关系也要成立 */
    cfg = Test_ValidFullBoardConfig();
    cfg.protection.dc_voltage_max_v = 10.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));

    /* 基波频率白名单：只放行 30 / 60 Hz */
    cfg = Test_ValidFullBoardConfig();
    cfg.inverter.output_frequency_hz = 50.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));

    return true;
}

static bool Test_PublicApiAndVdcSetting(void) {
    const AppStatus *status;
    const AppPwmCommand *pwm;
    uint32_t pair[4];
    unsigned int i;

    App_Init();
    status = App_GetStatus();
    TEST_CHECK(status != NULL);
    TEST_CHECK(BoardPower_ConfigValid());
    TEST_CHECK(status->state == OFF);
    TEST_CHECK(status->board_config_valid);
    TEST_CHECK(status->inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(TEST_NEAR(status->output_line_voltage_reference_v, 32.0f, 1.0e-5f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v, 62.0f, 1.0e-5f));
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    TEST_CHECK(App_GetStatus()->inverter_run_mode == INVERTER_OPEN_LOOP);
    TEST_CHECK(App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(!App_SetInverterRunMode((inverter_run_mode_t)2));
    TEST_CHECK(App_SetOutputFrequency(30U));
    /* 解耦后：28~34V 窗口内可独立设定输出线电压参考。 */
    TEST_CHECK(App_SetOutputLineVoltageReference(32.0f));
    TEST_CHECK(App_SetOutputLineVoltageReference(30.0f));
    TEST_CHECK(App_SetOutputLineVoltageReference(32.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(16.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(40.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(300.0f));
    TEST_CHECK(!App_SetDcVoltageReference(29.0f));
    TEST_CHECK(App_SetDcVoltageReference(65.0f));
    /* 母线参考修改不再联动输出参考。 */
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_line_voltage_reference_v, 32.0f, 1.0e-5f));
    TEST_CHECK(!App_SetDcVoltageReference(66.0f));
    TEST_CHECK(App_SetDcVoltageReference(30.0f));
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_line_voltage_reference_v, 32.0f, 1.0e-5f));

    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    pair[1] = ((uint32_t)2085U << 16) | 2085U;
    pair[2] = ((uint32_t)2110U << 16) | 2110U;
    pair[3] = ((uint32_t)2048U << 16) | 1833U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 1U));
    pair[3] = ((uint32_t)3000U << 16) | 1833U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    App_RequestStart();
    TEST_CHECK(!App_SetOutputFrequency(60U));
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_frequency_hz, 30.0f, 1.0e-6f));
    TEST_CHECK(!App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(!App_SetDcVoltageReference(31.0f));
    for (i = 3U; i <= 50U; i++) {
        TEST_CHECK(App_ApplyAdcPairs(pair, i));
        App_FastStep20kHz();
    }

    status = App_GetStatus();
    pwm = App_GetPwmCommand();
    TEST_CHECK(status->state == RUN);
    TEST_CHECK(status->pwm_allowed);
    TEST_CHECK(status->inverter_run_mode == INVERTER_OPEN_LOOP);
    TEST_CHECK(!App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(pwm->inverter_enable_request);
    TEST_CHECK(TEST_NEAR(status->dc_voltage_v, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v, 30.0f, 1.0e-6f));
    /* 独立输出参考 32V 线受 30V 母线过调制限幅：
     * (duty_max-duty_min)*Vdc/sqrt(6)*0.95 相 RMS，再折算线电压。 */
    TEST_CHECK(TEST_NEAR(status->output_line_voltage_reference_v,
                         (0.98f - 0.02f) * 30.0f * 0.40824829046386301637f * APP_INVERTER_VOLTAGE_MARGIN * APP_SQRT3_F,
                         1.0e-4f));
    TEST_CHECK(!App_SetDcVoltageReference(60.0f));

    App_RequestStop();
    for (i = 51U; i <= 75U; i++) {
        TEST_CHECK(App_ApplyAdcPairs(pair, i));
        App_FastStep20kHz();
    }
    TEST_CHECK(App_GetStatus()->state == OFF);
    TEST_CHECK(App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(App_SetDcVoltageReference(60.0f));
    /* 解耦后输出参考由控制步根据当前母线参考重算，补一拍刷新。 */
    TEST_CHECK(App_ApplyAdcPairs(pair, 76U));
    App_FastStep20kHz();
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_line_voltage_reference_v, 32.0f, 1.0e-5f));

    App_TripFromISR(PROTECTION_FAULT_EXTERNAL);
    TEST_CHECK(!App_GetPwmCommand()->inverter_enable_request);
    return true;
}

static bool Test_OutputOvSnapshot(void) {
    const AppStatus *status;
    uint32_t pair[4];

    App_Init();
    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    /* PC0=Ib and PC1=Ia remain near their calibrated offsets. */
    pair[1] = ((uint32_t)2085U << 16) | 2085U;
    /*
     * PC2=Uab is deliberately high while PC3=Ubc stays near zero.
     * The reconstructed Va exceeds the 32 V instantaneous limit.
     */
    pair[2] = ((uint32_t)2110U << 16) | 3500U;
    pair[3] = 0UL;
    TEST_CHECK(App_ApplyAdcPairs(pair, 7001U));
    App_FastStep20kHz();

    status = App_GetStatus();
    TEST_CHECK(status->state != FAULT_LATCH);
    TEST_CHECK((status->faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
    TEST_CHECK(!status->fault_snapshot.valid);

    TEST_CHECK(App_ApplyAdcPairs(pair, 7002U));
    App_FastStep20kHz();

    status = App_GetStatus();
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK((status->faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(status->fault_snapshot.valid);
    TEST_CHECK(status->fault_snapshot.inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK((status->fault_snapshot.faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(status->fault_snapshot.sample_sequence == 7002U);
    TEST_CHECK(TEST_NEAR(status->fault_snapshot.dc_voltage_reference_v, 62.0f, 1.0e-6f));
    TEST_CHECK(status->fault_snapshot.pc2_raw == 3500U);
    TEST_CHECK(status->fault_snapshot.pc3_raw == 2110U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc0_raw == 2085U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc1_raw == 2085U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc2_raw == 3500U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc3_raw == 2110U);
    TEST_CHECK(status->fault_snapshot.fast_history_count == 0U);
    TEST_CHECK(!status->fault_snapshot.previous_control.valid);
    TEST_CHECK(status->fault_snapshot.va_v > 32.0f);
    TEST_CHECK(TEST_NEAR(status->fault_snapshot.vab_v,
                         ((float)3500U - g_board_power_config.measurement.vab.offset_count) *
                             g_board_power_config.measurement.vab.gain,
                         1.0e-4f));

    /* A later fault frame must not overwrite the first captured frame. */
    pair[2] = ((uint32_t)2110U << 16) | 3900U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 7003U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK(status->fault_snapshot.sample_sequence == 7002U);
    TEST_CHECK(status->fault_snapshot.pc2_raw == 3500U);

    /* A successful fault reset clears the snapshot for the next event. */
    pair[2] = ((uint32_t)2110U << 16) | 2110U;
    App_ResetFault();
    TEST_CHECK(App_ApplyAdcPairs(pair, 7004U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK(status->faults == PROTECTION_FAULT_NONE);
    TEST_CHECK(!status->fault_snapshot.valid);
    return true;
}


static bool Test_OutputOvDiagnosticHistory(void) {
    control_config_t cfg = Test_ValidInverterBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    unsigned int i;

    cfg.measurement.ia = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2000.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2000.0f, 0.1f);
    cfg.protection.output_voltage_abs_max_v = 32.0f;
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 45.0f;

    for (i = 1U; i <= 4U; i++) {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(out.state == RUN);

    /*
     * Fill and wrap the 16-frame fast history. One safe 20 V line
     * excursion also leaves a recognizable completed-cycle peak.
     */
    for (i = 0U; i < 360U; i++) {
        raw.ia = (uint16_t)(2000U + (i % 5U));
        raw.ib = (uint16_t)(2000U + (i % 7U));
        raw.vab = (uint16_t)(2000U + (i % 20U));
        raw.vbc = (i == 100U) ? 2200U : 2000U;
        raw.sequence = 5U + i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK((out.faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
    }

    raw.ia = 2011U;
    raw.ib = 2022U;
    raw.vab = 2500U;
    raw.vbc = 2000U;
    raw.sequence = 365U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state != FAULT_LATCH);
    TEST_CHECK(!app.fault_snapshot.valid);

    raw.sequence = 366U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == FAULT_LATCH);
    TEST_CHECK(app.fault_snapshot.valid);
    TEST_CHECK(app.fault_snapshot.sample_sequence == 366U);
    TEST_CHECK(app.fault_snapshot.fast_history_count == APP_FAULT_FAST_HISTORY_DEPTH);
    TEST_CHECK(app.fault_snapshot.fast_history[0].sample_sequence == 350U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].sample_sequence == 365U);
    TEST_CHECK(app.fault_snapshot.fast_history[0].pc2_raw == 2005U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc2_raw == 2500U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc0_raw == 2022U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc1_raw == 2011U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc0_raw == 2022U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc1_raw == 2011U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc2_raw == 2500U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc3_raw == 2000U);
    TEST_CHECK(app.fault_snapshot.previous_control.valid);
    TEST_CHECK(app.fault_snapshot.previous_control.sample_sequence == 365U);
    TEST_CHECK(app.fault_snapshot.previous_control.applied_scale > 0.0f);
    TEST_CHECK(app.fault_snapshot.cycle_peak_count >= 1U);
    TEST_CHECK(app.fault_snapshot.cycle_peak_v[0] > 10.0f);
    TEST_CHECK(app.fault_snapshot.partial_cycle_samples > 0U);
    TEST_CHECK(app.fault_snapshot.partial_cycle_samples < app.fault_recorder.cycle_target_samples);

    /* A safe reset clears both the published event and all old history. */
    command.enable = false;
    command.reset_fault = true;
    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.sequence = 367U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!app.fault_snapshot.valid);
    TEST_CHECK(app.fault_recorder.fast_history_count == 0U);
    TEST_CHECK(app.fault_recorder.cycle_peak_count == 0U);

    command.reset_fault = false;
    raw.vab = 2500U;
    raw.sequence = 368U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!app.fault_snapshot.valid);
    raw.sequence = 369U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(app.fault_snapshot.valid);
    TEST_CHECK(app.fault_snapshot.fast_history_count == 0U);
    TEST_CHECK(!app.fault_snapshot.previous_control.valid);
    return true;
}

static bool Test_OutputOvNoStaleHistoryAfterStop(void) {
    control_config_t cfg = Test_ValidInverterBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    unsigned int i;

    cfg.measurement.ia = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2000.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2000.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2000.0f, 0.1f);
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 45.0f;

    for (i = 1U; i <= 24U; i++) {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(app.fault_recorder.active);
    TEST_CHECK(app.fault_recorder.fast_history_count == APP_FAULT_FAST_HISTORY_DEPTH);

    command.enable = false;
    raw.sequence = 25U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == OFF);
    TEST_CHECK(!app.fault_recorder.active);

    raw.vab = 2500U;
    raw.sequence = 26U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!app.fault_snapshot.valid);
    raw.sequence = 27U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(app.fault_snapshot.valid);
    TEST_CHECK(app.fault_snapshot.fast_history_count == 0U);
    TEST_CHECK(app.fault_snapshot.cycle_peak_count == 0U);
    TEST_CHECK(!app.fault_snapshot.previous_control.valid);
    return true;
}

static bool Test_IsrFaultResetSemantics(void) {
    const AppStatus *status;
    uint32_t pair[4];

    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    pair[1] = ((uint32_t)2048U << 16) | 2048U;
    pair[2] = ((uint32_t)2048U << 16) | 2048U;
    pair[3] = ((uint32_t)2048U << 16) | 2048U;

    App_Init();
    App_TripFromISR(PROTECTION_FAULT_EXTERNAL | PROTECTION_FAULT_LOOP_OVERRUN);
    TEST_CHECK(App_ApplyAdcPairs(pair, 1U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK(!status->pwm_allowed);
    TEST_CHECK((status->faults & PROTECTION_FAULT_EXTERNAL) != 0U);
    TEST_CHECK((status->faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0U);
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    TEST_CHECK(App_GetStatus()->inverter_run_mode == INVERTER_OPEN_LOOP);
    TEST_CHECK(App_SetDcVoltageReference(59.0f));
    TEST_CHECK(TEST_NEAR(App_GetStatus()->dc_voltage_reference_v, 59.0f, 1.0e-6f));
    /* 解耦后母线参考改为 59V 不影响输出参考（仍为独立设定的 32V）。 */
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_line_voltage_reference_v, 32.0f, 1.0e-5f));
    status = App_GetStatus();
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK(!status->pwm_allowed);
    TEST_CHECK((status->faults & PROTECTION_FAULT_EXTERNAL) != 0U);
    TEST_CHECK((status->faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0U);

    App_ResetFault();
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    App_FastStep20kHz();
    TEST_CHECK(App_GetStatus()->faults == PROTECTION_FAULT_NONE);
    TEST_CHECK(App_GetStatus()->inverter_run_mode == INVERTER_OPEN_LOOP);

    App_TripFromISR(PROTECTION_FAULT_LOOP_OVERRUN);
    TEST_CHECK(App_ApplyAdcPairs(pair, 3U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK((status->faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0U);
    TEST_CHECK((status->faults & PROTECTION_FAULT_EXTERNAL) == 0U);

    App_ResetFault();
    TEST_CHECK(App_ApplyAdcPairs(pair, 4U));
    App_FastStep20kHz();
    TEST_CHECK(App_GetStatus()->faults == PROTECTION_FAULT_NONE);

    /* 复位后的新故障必须能重新锁存，不能被旧的清零操作覆盖。 */
    App_TripFromISR(PROTECTION_FAULT_MEASUREMENT);
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults & PROTECTION_FAULT_MEASUREMENT) != 0U);
    return true;
}

typedef bool (*TestFunction)(void);

typedef struct {
    const char *name;
    TestFunction run;
} TestCase;

int main(void) {
    static const TestCase tests[] = {{"measurement", Test_Measurement},
                                     {"dual ADC rank mapping", Test_AdcRankMapping},
                                     {"inverter ADC path", Test_InverterMeasurementPath},
                                     {"phase recurrence", Test_PhaseRecurrence},
                                     {"Tustin QPR", Test_Qpr},
                                     {"SVPWM", Test_Modulation},
                                     {"inverter cascaded loops", Test_InverterCascade},
                                     {"inverter open loop", Test_InverterOpenLoop},
                                     {"protection latch", Test_Protection},
                                     {"power state machine", Test_PowerState},
                                     {"board configuration gates", Test_BoardConfigGates},
                                     {"inverter app flow", Test_InverterOnlyAppFlow},
                                     {"open-loop protection", Test_OpenLoopProtection},
                                     {"public API and Vdc setting", Test_PublicApiAndVdcSetting},
                                     {"output OV fault snapshot", Test_OutputOvSnapshot},
                                     {"output OV diagnostic history", Test_OutputOvDiagnosticHistory},
                                     {"output OV stale history guard", Test_OutputOvNoStaleHistoryAfterStop},
                                     {"ISR fault reset semantics", Test_IsrFaultResetSemantics}};
    size_t i;

    for (i = 0U; i < sizeof(tests) / sizeof(tests[0]); i++) {
        bool passed = tests[i].run();
        printf("[%s] %s\n", passed ? "PASS" : "FAIL", tests[i].name);
        if (!passed) g_failed++;
    }

    if (g_failed != 0) {
        printf("%d test group(s) failed.\n", g_failed);
        return 1;
    }
    printf("All host tests passed.\n");
    return 0;
}
