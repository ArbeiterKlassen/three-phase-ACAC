#include "app_config.h"
#include "app_control.h"
#include "inverter_control.h"
#include "measurement.h"
#include "notch_filter.h"
#include "phase_recurrence.h"
#include "pi_controller.h"
#include "power_state.h"
#include "protection.h"
#include "qpr_controller.h"
#include "rectifier_control.h"
#include "rectifier_pwm.h"
#include "svpwm.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

static int g_failed;

#define TEST_CHECK(expr)                                                     \
    do                                                                       \
    {                                                                        \
        if (!(expr))                                                         \
        {                                                                    \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);         \
            return false;                                                    \
        }                                                                    \
    } while (0)

static ChannelCalibration Test_Cal(float offset, float gain)
{
    ChannelCalibration cal;
    cal.offset_count = offset;
    cal.gain = gain;
    cal.raw_min = 0U;
    cal.raw_max = 4095U;
    return cal;
}

static bool Test_Measurement(void)
{
    MeasurementCalibration cal;
    AdcRawFrame raw = {0};
    Measurements m;
    AlphaBeta voltage;

    cal.iin = Test_Cal(100.0f, 0.1f);
    cal.vin = Test_Cal(100.0f, 1.0f);
    cal.ia = Test_Cal(100.0f, 1.0f);
    cal.ib = Test_Cal(100.0f, 1.0f);
    cal.vab = Test_Cal(100.0f, 1.0f);
    cal.vbc = Test_Cal(100.0f, 1.0f);
    cal.vdc = Test_Cal(0.0f, 0.1f);

    raw.iin = 110U;
    raw.vin = 150U;
    raw.ia = 110U;
    raw.ib = 120U;
    raw.vab = 130U;
    raw.vbc = 80U;
    raw.vdc = 1000U;
    raw.sequence = 7U;
    raw.valid = true;

    TEST_CHECK(Measurement_CalibrationValid(&cal));
    TEST_CHECK(Measurement_Convert(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.iin, 1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ic, -30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vca, -10.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.va, 13.333333f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(m.vb, -16.666667f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(m.vc, 3.333333f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(m.vdc, 100.0f, 1.0e-6f));
    voltage = Measurement_VoltageAlphaBeta(&m);
    TEST_CHECK(TEST_NEAR(voltage.alpha, m.va, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(voltage.beta,
                         APP_INV_SQRT3_F * (m.vb - m.vc),
                         1.0e-4f));

    raw.valid = false;
    TEST_CHECK(!Measurement_Convert(&cal, &raw, &m));
    TEST_CHECK(!m.valid);
    return true;
}

static bool Test_AdcRankMapping(void)
{
    uint32_t pair[4];
    AdcRawFrame raw;

    pair[0] = ((uint32_t)202U << 16) | 101U;
    pair[1] = ((uint32_t)404U << 16) | 303U;
    pair[2] = ((uint32_t)606U << 16) | 505U;
    pair[3] = ((uint32_t)203U << 16) | 707U;
    TEST_CHECK(Measurement_UnpackDualAdcPairs(pair, 99U, 9U, &raw));
    TEST_CHECK(raw.iin == 101U);
    TEST_CHECK(raw.vin == 202U);
    TEST_CHECK(raw.ib == 303U);
    TEST_CHECK(raw.ia == 404U);
    TEST_CHECK(raw.vab == 505U);
    TEST_CHECK(raw.vbc == 606U);
    TEST_CHECK(raw.vdc == 707U);
    TEST_CHECK(raw.vin_diag == 203U);
    TEST_CHECK(raw.sequence == 99U);

    pair[3] = ((uint32_t)212U << 16) | 707U;
    TEST_CHECK(!Measurement_UnpackDualAdcPairs(pair, 100U, 9U, &raw));
    TEST_CHECK(!raw.valid);

    pair[1] = UINT32_MAX;
    pair[2] = UINT32_MAX;
    pair[3] = ((uint32_t)4095U << 16) | 707U;
    TEST_CHECK(Measurement_UnpackRectifierAdcPairs(pair, 101U, &raw));
    TEST_CHECK(raw.valid);
    TEST_CHECK(raw.iin == 101U);
    TEST_CHECK(raw.vin == 202U);
    TEST_CHECK(raw.vdc == 707U);
    TEST_CHECK(raw.vin_diag == 4095U);
    TEST_CHECK(raw.sequence == 101U);
    return true;
}

static bool Test_ModeSpecificAdcDiagnostics(void)
{
    const AppStatus *status;
    uint32_t pair[4];

    pair[0] = ((uint32_t)2100U << 16) | 2085U;
    pair[1] = UINT32_MAX;
    pair[2] = UINT32_MAX;
    pair[3] = ((uint32_t)4095U << 16) | 1833U;

    App_Init();
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    TEST_CHECK(App_ApplyAdcPairs(pair, 201U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK(status->mode == MODE2_RECTIFIER);
    TEST_CHECK(status->iin_raw == 2085U);
    TEST_CHECK(TEST_NEAR(status->iin_a,
                         ((float)2085U
                          - g_board_power_config.measurement.iin.offset_count)
                         * g_board_power_config.measurement.iin.gain,
                         1.0e-6f));
    TEST_CHECK((status->faults
                & PROTECTION_FAULT_MEASUREMENT) == 0U);

    App_Init();
    TEST_CHECK(App_RequestMode(MODE0_FULL));
    TEST_CHECK(!App_ApplyAdcPairs(pair, 202U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) != 0U);
    TEST_CHECK(!App_GetStatus()->pwm_allowed);

    App_Init();
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    pair[0] = ((uint32_t)2100U << 16) | 0U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 203U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) == 0U);
    TEST_CHECK(App_ApplyAdcPairs(pair, 204U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) != 0U);
    TEST_CHECK(!App_GetStatus()->pwm_allowed);

    App_Init();
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    pair[0] = ((uint32_t)0U << 16) | 2085U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 205U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) == 0U);
    TEST_CHECK(App_ApplyAdcPairs(pair, 206U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) != 0U);
    TEST_CHECK(!App_GetStatus()->pwm_allowed);

    App_Init();
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    pair[0] = ((uint32_t)2100U << 16) | 2085U;
    pair[3] = ((uint32_t)4095U << 16) | 4095U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 207U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) == 0U);
    TEST_CHECK(App_ApplyAdcPairs(pair, 208U));
    App_FastStep20kHz();
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) != 0U);
    TEST_CHECK(!App_GetStatus()->pwm_allowed);
    return true;
}

static bool Test_InverterMeasurementPath(void)
{
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
    TEST_CHECK(raw.iin == 0U);
    TEST_CHECK(raw.vin == 0U);
    TEST_CHECK(raw.vdc == 0U);
    TEST_CHECK(raw.vin_diag == 0U);
    TEST_CHECK(Measurement_ConvertInverter(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.iin, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vin, 0.0f, 1.0e-6f));
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

static bool Test_RectifierMeasurementPath(void)
{
    MeasurementCalibration cal = {0};
    AdcRawFrame raw = {0};
    Measurements m;

    cal.iin = Test_Cal(2000.0f, 0.01f);
    cal.vin = Test_Cal(2000.0f, 0.1f);
    cal.vdc = Test_Cal(0.0f, 0.1f);
    raw.iin = 2100U;
    raw.vin = 2150U;
    raw.vdc = 600U;
    raw.ia = UINT16_MAX;
    raw.ib = UINT16_MAX;
    raw.vab = UINT16_MAX;
    raw.vbc = UINT16_MAX;
    raw.sequence = 102U;
    raw.valid = true;

    TEST_CHECK(!Measurement_CalibrationValid(&cal));
    TEST_CHECK(Measurement_RectifierCalibrationValid(&cal));
    TEST_CHECK(Measurement_ConvertRectifier(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.iin, 1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vin, 15.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vdc, 60.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ia, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vab, 0.0f, 1.0e-6f));
    TEST_CHECK(m.sequence == 102U);
    TEST_CHECK(m.valid);

    cal.vdc.gain = 0.0f;
    TEST_CHECK(!Measurement_RectifierCalibrationValid(&cal));
    TEST_CHECK(!Measurement_ConvertRectifier(&cal, &raw, &m));
    return true;
}

static bool Test_PhaseAtFrequency(float frequency_hz,
                                  unsigned int sample_count)
{
    PhaseRecurrence phase;
    ThreePhase abc;
    unsigned int i;
    float norm;

    TEST_CHECK(PhaseRecurrence_Init(&phase, frequency_hz,
                                    50.0e-6f, 0.0f));
    for (i = 0U; i < sample_count; i++)
    {
        PhaseRecurrence_Step(&phase);
    }

    norm = phase.sin_value * phase.sin_value
         + phase.cos_value * phase.cos_value;
    TEST_CHECK(TEST_NEAR(norm, 1.0f, 3.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.sin_value, 0.0f, 3.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.cos_value, 1.0f, 3.0e-5f));
    abc = PhaseRecurrence_GetThreePhase(&phase, 10.0f);
    TEST_CHECK(TEST_NEAR(abc.a, 0.0f, 3.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.b, -8.660254f, 3.0e-4f));
    TEST_CHECK(TEST_NEAR(abc.c, 8.660254f, 3.0e-4f));
    return true;
}

static bool Test_PhaseRecurrence(void)
{
    PhaseRecurrence phase;
    ThreePhase abc;
    unsigned int i;
    float norm;

    TEST_CHECK(PhaseRecurrence_Init(&phase, 50.0f, 50.0e-6f, 0.0f));
    for (i = 0U; i < 400U; i++)
    {
        PhaseRecurrence_Step(&phase);
    }
    TEST_CHECK(TEST_NEAR(phase.sin_value, 0.0f, 2.0e-5f));
    TEST_CHECK(TEST_NEAR(phase.cos_value, 1.0f, 2.0e-5f));
    norm = phase.sin_value * phase.sin_value
         + phase.cos_value * phase.cos_value;
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

static bool Test_PiAntiWindup(void)
{
    PiController pi;
    float out = 0.0f;
    unsigned int i;

    TEST_CHECK(PiController_Init(&pi, 0.5f, 20.0f, 50.0f,
                                 1.0e-3f, -1.0f, 1.0f));
    for (i = 0U; i < 500U; i++)
    {
        out = PiController_Step(&pi, 10.0f);
        TEST_CHECK((out >= -1.0f) && (out <= 1.0f));
    }
    TEST_CHECK(TEST_NEAR(out, 1.0f, 1.0e-6f));
    for (i = 0U; i < 200U; i++)
    {
        out = PiController_Step(&pi, -1.0f);
    }
    TEST_CHECK(out < 0.0f);
    TEST_CHECK(fabsf(pi.integrator) <= 1.0f);
    return true;
}

static float Test_QprRms(float frequency_hz)
{
    QprController qpr;
    float sum = 0.0f;
    float input;
    float output;
    unsigned int i;
    unsigned int count = 0U;

    if (!QprController_Init(&qpr, 0.0f, 10.0f,
                            2.0f * APP_PI_F * 50.0f, 5.0f,
                            50.0e-6f, -100.0f, 100.0f))
    {
        return -1.0f;
    }

    for (i = 0U; i < 40000U; i++)
    {
        input = sinf(2.0f * APP_PI_F * frequency_hz
                   * 50.0e-6f * (float)i);
        output = QprController_Step(&qpr, input);
        if (i >= 20000U)
        {
            sum += output * output;
            count++;
        }
    }
    return sqrtf(sum / (float)count);
}

static bool Test_Qpr(void)
{
    float at_resonance = Test_QprRms(50.0f);
    float away = Test_QprRms(250.0f);
    QprController qpr;
    float output = 0.0f;
    unsigned int i;

    TEST_CHECK(at_resonance > 5.0f);
    TEST_CHECK(at_resonance > 8.0f * away);
    TEST_CHECK(QprController_Init(&qpr, 1.0f, 20.0f,
                                  2.0f * APP_PI_F * 50.0f, 5.0f,
                                  50.0e-6f, -100.0f, 100.0f));
    for (i = 0U; i < 2000U; i++)
    {
        (void)QprController_Step(&qpr, 5.0f);
        QprController_TrackOutput(&qpr, 5.0f, 0.0f);
    }
    for (i = 0U; i < 40000U; i++)
    {
        output = QprController_Step(&qpr, 0.0f);
        QprController_TrackOutput(&qpr, 0.0f, output);
    }
    TEST_CHECK(fabsf(output) < 0.1f);
    return true;
}

static float Test_NotchRms(float frequency_hz)
{
    NotchFilter notch;
    float sum = 0.0f;
    float input;
    float output;
    unsigned int i;
    unsigned int count = 0U;

    if (!NotchFilter_Init(&notch, 2.0f * APP_PI_F * 100.0f,
                          0.01f, 0.5f, 1.0e-3f))
    {
        return -1.0f;
    }
    for (i = 0U; i < 5000U; i++)
    {
        input = sinf(2.0f * APP_PI_F * frequency_hz
                   * 1.0e-3f * (float)i);
        output = NotchFilter_Step(&notch, input);
        if (i >= 3000U)
        {
            sum += output * output;
            count++;
        }
    }
    return sqrtf(sum / (float)count);
}

static bool Test_Notch(void)
{
    float center = Test_NotchRms(100.0f);
    float passband = Test_NotchRms(20.0f);

    TEST_CHECK(center < 0.05f);
    TEST_CHECK(passband > 0.5f);
    TEST_CHECK(center < 0.1f * passband);
    return true;
}

static bool Test_Modulation(void)
{
    SvpwmOutput sv;
    RectifierPwmOutput rect;

    TEST_CHECK(Svpwm_Generate(10.0f, 0.0f, 100.0f,
                              0.0f, 1.0f, &sv));
    TEST_CHECK(sv.sector == 1U);
    TEST_CHECK(TEST_NEAR((sv.duty_a - sv.duty_b) * 100.0f,
                         15.0f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(sv.duty_b, sv.duty_c, 1.0e-6f));

    TEST_CHECK(Svpwm_Generate(1000.0f, 500.0f, 100.0f,
                              0.02f, 0.98f, &sv));
    TEST_CHECK(sv.saturated);
    TEST_CHECK((sv.duty_a >= 0.02f) && (sv.duty_a <= 0.98f));
    TEST_CHECK((sv.duty_b >= 0.02f) && (sv.duty_b <= 0.98f));
    TEST_CHECK((sv.duty_c >= 0.02f) && (sv.duty_c <= 0.98f));

    TEST_CHECK(RectifierPwm_Generate(0.0f, 0.02f, 0.98f, &rect));
    TEST_CHECK(TEST_NEAR(rect.duty_a, 0.5f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(rect.duty_b, 0.5f, 1.0e-6f));
    TEST_CHECK(!rect.saturated);
    TEST_CHECK(RectifierPwm_Generate(0.5f, 0.02f, 0.98f, &rect));
    TEST_CHECK(TEST_NEAR(rect.duty_a, 0.75f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(rect.duty_b, 0.25f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(rect.modulation, 0.5f, 1.0e-6f));
    TEST_CHECK(!rect.saturated);
    TEST_CHECK(TEST_NEAR(rect.duty_a + rect.duty_b, 1.0f, 1.0e-6f));
    TEST_CHECK(RectifierPwm_Generate(1.0f, 0.02f, 0.98f, &rect));
    TEST_CHECK(rect.saturated);
    TEST_CHECK(TEST_NEAR(rect.modulation, 0.96f, 1.0e-6f));
    TEST_CHECK(RectifierPwm_Generate(-1.0f, 0.02f, 0.98f, &rect));
    TEST_CHECK(rect.saturated);
    TEST_CHECK(TEST_NEAR(rect.modulation, -0.96f, 1.0e-6f));
    TEST_CHECK(RectifierPwm_Generate(2.0f, 0.02f, 0.98f, &rect));
    TEST_CHECK(rect.saturated);
    TEST_CHECK(TEST_NEAR(rect.duty_a, 0.98f, 1.0e-6f));
    return true;
}

static InverterControlConfig Test_InverterConfig(void)
{
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

static bool Test_InverterCascade(void)
{
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
    TEST_CHECK(fabsf(out.voltage_command.beta)
               > fabsf(out.current_reference.beta));
    TEST_CHECK(fabsf(out.current_reference.beta)
               <= cfg.current_reference_limit_a);
    TEST_CHECK(InverterControl_SetFrequency(&control, 30.0f));
    TEST_CHECK(TEST_NEAR(control.cfg.output_frequency_hz,
                         30.0f, 1.0e-6f));
    return true;
}

static bool Test_InverterOpenLoop(void)
{
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

    TEST_CHECK(InverterControl_StepOpenLoop(
        &first, &m_first, true, &out_first));
    TEST_CHECK(InverterControl_StepOpenLoop(
        &second, &m_second, true, &out_second));
    TEST_CHECK(out_first.valid && out_first.pwm.valid);
    TEST_CHECK(TEST_NEAR(first.reference_rms,
                         cfg.reference_slew_v_per_s
                             * cfg.sample_time_s,
                         1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.voltage_command.alpha,
                         out_first.reference.alpha, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.voltage_command.beta,
                         out_first.reference.beta, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.current_reference.alpha,
                         0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.current_reference.beta,
                         0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_a,
                         out_second.pwm.duty_a, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_b,
                         out_second.pwm.duty_b, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_c,
                         out_second.pwm.duty_c, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(first.voltage_alpha_qpr.y1,
                         saved_qpr_state, 1.0e-7f));

    cfg.reference_slew_v_per_s = 1.0e6f;
    for (i = 0U; i < 2U; i++)
    {
        InverterControl sweep;
        InverterControlOutput sweep_out;
        Measurements sweep_m = {0};
        float vdc = (i == 0U) ? 30.0f : 65.0f;
        unsigned int sample;

        sweep_m.valid = true;
        sweep_m.vdc = vdc;
        TEST_CHECK(InverterControl_Init(&sweep, &cfg));
        InverterControl_SetReference(
            &sweep,
            vdc * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
                * APP_INV_SQRT3_F);
        for (sample = 0U; sample < 400U; sample++)
        {
            TEST_CHECK(InverterControl_StepOpenLoop(
                &sweep, &sweep_m, true, &sweep_out));
            TEST_CHECK(!sweep_out.pwm.saturated);
            TEST_CHECK((sweep_out.pwm.duty_a >= cfg.duty_min)
                    && (sweep_out.pwm.duty_a <= cfg.duty_max));
            TEST_CHECK((sweep_out.pwm.duty_b >= cfg.duty_min)
                    && (sweep_out.pwm.duty_b <= cfg.duty_max));
            TEST_CHECK((sweep_out.pwm.duty_c >= cfg.duty_min)
                    && (sweep_out.pwm.duty_c <= cfg.duty_max));
        }
    }

    TEST_CHECK(InverterControl_StepOpenLoop(
        &first, &m_first, false, &out_first));
    TEST_CHECK(TEST_NEAR(first.reference_rms, 0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_a, 0.5f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_b, 0.5f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(out_first.pwm.duty_c, 0.5f, 1.0e-7f));
    return true;
}

static RectifierControlConfig Test_RectifierConfig(void)
{
    RectifierControlConfig cfg = {0};
    cfg.sample_time_s = 50.0e-6f;
    cfg.line_frequency_hz = 50.0f;
    cfg.outer_loop_divider = 20U;
    cfg.rms_filter_time_constant_s = 0.02f;
    cfg.input_voltage_rms_min_v = 1.0f;
    cfg.dc_voltage_max_v = 200.0f;
    cfg.dc_reference_slew_v_per_s = 2000.0f;
    cfg.dc_ready_tolerance_v = 2.0f;
    cfg.dc_ready_confirm_samples = 5U;
    cfg.current_rms_max_a = 10.0f;
    cfg.dc_pi_kp = 0.1f;
    cfg.dc_pi_ki = 1.0f;
    cfg.dc_pi_kaw = 20.0f;
    cfg.current_qpr_kp = 1.0f;
    cfg.current_qpr_kr = 0.5f;
    cfg.current_qpr_bandwidth_rad_s = 10.0f;
    cfg.inductor_voltage_limit_v = 200.0f;
    cfg.dc_voltage_min_v = 10.0f;
    cfg.current_reference_sign = 1.0f;
    cfg.bridge_polarity = 1.0f;
    cfg.notch_zero_damping = 0.01f;
    cfg.notch_pole_damping = 0.5f;
    cfg.duty_min = 0.02f;
    cfg.duty_max = 0.98f;
    return cfg;
}

static bool Test_RectifierControl(void)
{
    RectifierControl control;
    RectifierControlConfig cfg = Test_RectifierConfig();
    RectifierControlOutput out;
    Measurements m = {0};
    unsigned int i;
    float time;
    bool saw_positive_power = false;

    m.valid = true;
    m.vdc = 50.0f;
    TEST_CHECK(RectifierControl_Init(&control, &cfg));
    RectifierControl_SetDcReference(&control, 150.0f);

    /* 先在PWM关闭状态建立输入RMS估计。 */
    for (i = 0U; i < RECTIFIER_VIN_RMS_WINDOW_SAMPLES - 1U; i++)
    {
        time = cfg.sample_time_s * (float)i;
        m.vin = 100.0f * 1.41421356237f
              * cosf(2.0f * APP_PI_F * 50.0f * time);
        TEST_CHECK(RectifierControl_Step(&control, &m, false, &out));
        TEST_CHECK(!control.vin_rms_valid);
        TEST_CHECK(!out.input_rms_valid);
    }
    i = RECTIFIER_VIN_RMS_WINDOW_SAMPLES - 1U;
    time = cfg.sample_time_s * (float)i;
    m.vin = 100.0f * 1.41421356237f
          * cosf(2.0f * APP_PI_F * 50.0f * time);
    TEST_CHECK(RectifierControl_Step(&control, &m, false, &out));
    TEST_CHECK(control.vin_rms_valid);
    TEST_CHECK(out.input_rms_valid);
    TEST_CHECK(TEST_NEAR(control.vin_rms_v, 100.0f, 0.05f));

    for (i = 0U; i < 1000U; i++)
    {
        time = cfg.sample_time_s
             * (float)(RECTIFIER_VIN_RMS_WINDOW_SAMPLES + i);
        m.vin = 100.0f * 1.41421356237f
              * cosf(2.0f * APP_PI_F * 50.0f * time);
        TEST_CHECK(RectifierControl_Step(&control, &m, true, &out));
        TEST_CHECK((out.current_reference * m.vin) >= -1.0e-4f);
        if ((out.current_reference * m.vin) > 1.0e-3f)
        {
            saw_positive_power = true;
        }
    }

    /* Accelerated test slew: 2000 V/s reaches 100 V after 50 ms. */
    TEST_CHECK(TEST_NEAR(out.dc_voltage_reference, 100.0f, 0.2f));
    TEST_CHECK(TEST_NEAR(out.input_voltage_rms, 100.0f, 0.05f));
    TEST_CHECK(out.input_rms_valid);
    TEST_CHECK(saw_positive_power);
    TEST_CHECK(out.pwm.valid);
    m.valid = false;
    TEST_CHECK(!RectifierControl_Step(&control, &m, true, &out));
    TEST_CHECK(!control.vin_rms_valid);
    return true;
}

static bool Test_ProductionRectifierParameters(void)
{
    const RectifierControlConfig *cfg = &g_board_power_config.rectifier;

    TEST_CHECK(TEST_NEAR(cfg->sample_time_s, 50.0e-6f, 1.0e-9f));
    TEST_CHECK(TEST_NEAR(cfg->line_frequency_hz, 50.0f, 1.0e-6f));
    TEST_CHECK(cfg->outer_loop_divider == 20U);
    TEST_CHECK(TEST_NEAR(
        cfg->sample_time_s * cfg->line_frequency_hz
            * (float)RECTIFIER_VIN_RMS_WINDOW_SAMPLES,
        1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->dc_pi_kp, 1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->dc_pi_ki, 1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->current_rms_max_a, 3.8f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        g_board_power_config.protection.input_current_abs_max_a,
        5.6f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        g_board_power_config.protection.input_voltage_abs_max_v,
        65.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        g_board_power_config.protection.dc_voltage_max_v,
        66.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        cfg->dc_reference_slew_v_per_s, 60.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->current_qpr_kp, 5.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->current_qpr_kr, 60.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        cfg->current_qpr_bandwidth_rad_s, 5.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        cfg->inductor_voltage_limit_v, 1200.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(cfg->notch_zero_damping, 0.0f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(
        cfg->notch_pole_damping,
        50.0f / (4.0f * APP_PI_F * 100.0f), 1.0e-7f));
    TEST_CHECK(cfg->current_reference_sign > 0.0f);
    TEST_CHECK(TEST_NEAR(cfg->bridge_polarity, 1.0f, 1.0e-7f));
    TEST_CHECK(cfg->dc_ready_confirm_samples == 1000U);
    TEST_CHECK(TEST_NEAR(cfg->duty_min, 0.02f, 1.0e-7f));
    TEST_CHECK(TEST_NEAR(cfg->duty_max, 0.98f, 1.0e-7f));
    return true;
}

static ProtectionConfig Test_ProtectionConfig(void)
{
    ProtectionConfig cfg;
    cfg.input_current_abs_max_a = 10.0f;
    cfg.phase_current_abs_max_a = 10.0f;
    cfg.input_voltage_abs_max_v = 300.0f;
    cfg.dc_voltage_min_run_v = 20.0f;
    cfg.dc_voltage_max_v = 200.0f;
    cfg.output_voltage_abs_max_v = 100.0f;
    cfg.stale_limit_samples = 2U;
    return cfg;
}

static bool Test_Protection(void)
{
    Protection protection;
    ProtectionConfig cfg = Test_ProtectionConfig();
    Measurements m = {0};

    m.valid = true;
    m.vdc = 50.0f;
    m.sequence = 1U;
    Protection_Init(&protection);

    /* One invalid conversion stops PWM through measurement_valid, but only
     * two consecutive invalid frames latch ADC INVALID. */
    m.valid = false;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_MEASUREMENT) == 0UL);
    TEST_CHECK(protection.measurement_invalid_count == 1U);
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_MEASUREMENT) != 0UL);
    m.valid = true;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(protection.measurement_invalid_count == 0U);
    TEST_CHECK(Protection_ClearLatched(&protection));

    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    /* One isolated input-current spike must not latch a fault. */
    m.iin = 11.0f;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERCURRENT) == 0UL);
    TEST_CHECK(protection.input_oc_count == 1U);
    m.iin = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(protection.input_oc_count == 0U);

    /* Two consecutive input-current excursions must latch the fault. */
    m.iin = 11.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERCURRENT) == 0UL);
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERCURRENT) != 0UL);
    TEST_CHECK(!Protection_ClearLatched(&protection));
    m.iin = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(Protection_ClearLatched(&protection));
    TEST_CHECK(protection.latched_faults == 0UL);

    /* One isolated input-voltage spike must not latch a fault. */
    m.vin = 301.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) == 0UL);
    TEST_CHECK(protection.input_ov_count == 1U);
    m.vin = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(protection.input_ov_count == 0U);

    /* Two consecutive input-voltage excursions must latch the fault. */
    m.vin = 301.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) == 0UL);
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) != 0UL);
    m.vin = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(Protection_ClearLatched(&protection));

    /* One isolated output-voltage excursion must not latch a fault. */
    m.va = 101.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
    TEST_CHECK(protection.output_ov_count == 1U);
    m.va = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(protection.output_ov_count == 0U);

    /* Two consecutive excursions must retain the original protection. */
    m.va = 101.0f;
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
    m.sequence++;
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    m.va = 0.0f;
    m.sequence++;
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    TEST_CHECK(Protection_ClearLatched(&protection));

    (void)Protection_Evaluate(&protection, &cfg, &m,
                              true, false, false);
    TEST_CHECK((Protection_Evaluate(&protection, &cfg, &m,
                                    true, false, false)
                & PROTECTION_FAULT_SAMPLE_STALE) != 0UL);
    return true;
}

static bool Test_PowerState(void)
{
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

static control_config_t Test_ValidInverterBoardConfig(void)
{
    control_config_t cfg = g_board_power_config;

    cfg.config_valid = true;
    cfg.adc_calibrated = true;
    cfg.rectifier_adc_calibrated = false;
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

static control_config_t Test_ValidFullBoardConfig(void)
{
    control_config_t cfg = g_board_power_config;

    cfg.config_valid = true;
    cfg.adc_calibrated = true;
    cfg.rectifier_adc_calibrated = true;
    cfg.gate_driver_pins_valid = true;
    cfg.power_stage_valid = true;
    cfg.deadtime_valid = true;
    cfg.protection_thresholds_valid = true;
    cfg.measurement.iin = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vin = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.ia = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vdc = Test_Cal(0.0f, 0.1f);
    return cfg;
}

static uint16_t Test_RawFromPhysical(const ChannelCalibration *cal,
                                     float value)
{
    float counts = cal->offset_count + value / cal->gain;

    if (counts <= 0.0f)
    {
        return 0U;
    }
    if (counts >= 65535.0f)
    {
        return UINT16_MAX;
    }
    return (uint16_t)(counts + 0.5f);
}

static void Test_SetFullRawFrame(const control_config_t *cfg,
                                 AdcRawFrame *raw,
                                 uint32_t sequence,
                                 float vin_rms,
                                 float vdc)
{
    float time = cfg->rectifier.sample_time_s * (float)(sequence - 1U);
    float vin = 1.41421356237f * vin_rms
              * cosf(2.0f * APP_PI_F
                     * cfg->rectifier.line_frequency_hz * time);

    *raw = (AdcRawFrame){0};
    raw->iin = Test_RawFromPhysical(&cfg->measurement.iin, 0.0f);
    raw->vin = Test_RawFromPhysical(&cfg->measurement.vin, vin);
    raw->ia = Test_RawFromPhysical(&cfg->measurement.ia, 0.0f);
    raw->ib = Test_RawFromPhysical(&cfg->measurement.ib, 0.0f);
    raw->vab = Test_RawFromPhysical(&cfg->measurement.vab, 0.0f);
    raw->vbc = Test_RawFromPhysical(&cfg->measurement.vbc, 0.0f);
    raw->vdc = Test_RawFromPhysical(&cfg->measurement.vdc, vdc);
    raw->vin_diag = raw->vin;
    raw->sequence = sequence;
    raw->valid = true;
}

static bool Test_InverterOnlyAppFlow(void)
{
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
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    TEST_CHECK(app.mode == MODE1_INVERTER);
    TEST_CHECK(app.inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(app.rectifier.initialized);
    TEST_CHECK(!AppControl_SetMode(&app, MODE0_FULL));
    TEST_CHECK(!AppControl_SetMode(&app, MODE2_RECTIFIER));
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    TEST_CHECK(!AppControl_SetMode(&app, (app_mode_t)3));
    TEST_CHECK(AppControl_SetInverterRunMode(
        &app, INVERTER_OPEN_LOOP));
    TEST_CHECK(app.inverter_run_mode == INVERTER_OPEN_LOOP);
    TEST_CHECK(!AppControl_SetInverterRunMode(
        &app, (inverter_run_mode_t)2));
    TEST_CHECK(AppControl_SetInverterRunMode(
        &app, INVERTER_CLOSED_LOOP));

    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.vdc = UINT16_MAX;
    raw.valid = true;
    command.enable = true;
    command.precharge_done = false;
    command.phase_ready = false;
    /* 独立输出参考未设置（<=0）时回退为按 Vdc 32/60 比例推导。 */
    command.inverter_vrms_ref = 0.0f;
    command.dc_voltage_ref = 30.0f;

    for (i = 1U; i <= 4U; i++)
    {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }

    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.inverter.enabled);
    TEST_CHECK(!AppControl_SetInverterRunMode(
        &app, INVERTER_OPEN_LOOP));
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(TEST_NEAR(out.rectifier.leg_a, 0.5f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        app.inverter.reference_target_rms,
        30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        1.0e-5f));

    command.dc_voltage_ref = 65.0f;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(TEST_NEAR(
        app.inverter.reference_target_rms,
        65.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        1.0e-5f));

    command.dc_voltage_ref = 30.0f;
    raw.vdc = 0U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.state == RUN);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_OVERVOLTAGE) == 0U);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_UNDERVOLTAGE) == 0U);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_MISMATCH) == 0U);
    TEST_CHECK(TEST_NEAR(
        app.inverter.reference_target_rms,
        30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        1.0e-5f));

    command.enable = false;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == OFF);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(AppControl_SetInverterRunMode(
        &app, INVERTER_OPEN_LOOP));

    return true;
}

static bool Test_RectifierOnlyAppFlow(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw;
    uint32_t i;

    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;
    cfg.rectifier.dc_reference_slew_v_per_s = 1200000.0f;

    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));
    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(RectifierControl_Init(&app.rectifier, &cfg.rectifier));
    TEST_CHECK(AppControl_SetMode(&app, MODE2_RECTIFIER));

    command.enable = true;
    command.dc_voltage_ref = 60.0f;
    for (i = 1U; i <= 520U; i++)
    {
        Test_SetFullRawFrame(&cfg, &raw, i, 36.0f, 19.3f);
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK(!out.inverter.enabled);
        TEST_CHECK(app.rectifier.current_rms_a >= -1.0e-6f);
    }

    TEST_CHECK(app.rectifier.vin_rms_valid);
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.rectifier.enabled);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 19.3f, 0.11f));
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_UNDERVOLTAGE) == 0U);

    command.external_fault = true;
    Test_SetFullRawFrame(&cfg, &raw, ++i, 36.0f, 19.3f);
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == FAULT_LATCH);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(!out.inverter.enabled);

    command.external_fault = false;
    command.reset_fault = true;
    command.enable = false;
    Test_SetFullRawFrame(&cfg, &raw, ++i, 36.0f, 19.3f);
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == DISCHARGE);

    command.reset_fault = false;
    command.enable = true;
    for (uint32_t restart = 0U; restart < 250U; restart++)
    {
        Test_SetFullRawFrame(&cfg, &raw, ++i, 36.0f, 19.3f);
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        if (out.rectifier.enabled)
        {
            break;
        }
    }
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.rectifier.enabled);
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_UNDERVOLTAGE) == 0U);
    return true;
}

static bool Test_RectifierZeroCrossStart(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw;
    uint32_t i;

    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;
    /* 加速斜坡，使参考在数百拍内越过 19.3 V 母线以覆盖门极使能。 */
    cfg.rectifier.dc_reference_slew_v_per_s = 1200.0f;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(RectifierControl_Init(&app.rectifier, &cfg.rectifier));
    TEST_CHECK(AppControl_SetMode(&app, MODE2_RECTIFIER));

    command.dc_voltage_ref = 60.0f;
    for (i = 1U; i <= RECTIFIER_VIN_RMS_WINDOW_SAMPLES; i++)
    {
        Test_SetFullRawFrame(&cfg, &raw, i, 36.0f, 19.3f);
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(app.rectifier.vin_rms_valid);

    command.enable = true;
    for (i++; i <= RECTIFIER_VIN_RMS_WINDOW_SAMPLES + 6U; i++)
    {
        Test_SetFullRawFrame(&cfg, &raw, i, 36.0f, 19.3f);
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK(!out.rectifier.enabled);
    }
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(!app.rectifier_start_ready);

    raw.vin = Test_RawFromPhysical(&cfg.measurement.vin, 0.0f);
    raw.iin = Test_RawFromPhysical(&cfg.measurement.iin, 0.0f);
    raw.sequence = i++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(app.rectifier_start_count == 1U);

    raw.sequence = i;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(app.rectifier_start_ready);
    /*
     * 过零确认后，若外环尚未请求电流（参考斜坡仍低于 19.3 V 母线），
     * 门极继续保持关断——避免 vin 跟随调制被 0.96 限幅后削顶泵升。
     */
    TEST_CHECK(!out.rectifier.enabled);

    /* 参考斜坡越过母线后 PI 请求电流，门极使能、PWM 放行。 */
    {
        bool saw_gates_on = false;
        uint32_t k;

        for (k = 0U; k < 600U; k++)
        {
            Test_SetFullRawFrame(&cfg, &raw, ++i, 36.0f, 19.3f);
            TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
            if (out.rectifier.enabled)
            {
                saw_gates_on = true;
                break;
            }
        }
        TEST_CHECK(saw_gates_on);
        TEST_CHECK(out.pwm_allowed);
    }

    command.enable = false;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!app.rectifier_start_ready);
    TEST_CHECK(app.rectifier_start_count == 0U);
    TEST_CHECK(!out.rectifier.enabled);
    return true;
}

static bool Test_FullStageAppFlow(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw;
    uint32_t i;
    uint32_t rectifier_only_samples = 0U;
    bool saw_rectifier = false;
    bool saw_both_stages = false;
    bool forced_counter_reset = false;

    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;
    cfg.rectifier.dc_reference_slew_v_per_s = 1200000.0f;
    cfg.rectifier.dc_ready_confirm_samples = 1000U;

    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));
    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(RectifierControl_Init(&app.rectifier, &cfg.rectifier));
    TEST_CHECK(AppControl_SetMode(&app, MODE0_FULL));

    command.enable = true;
    command.dc_voltage_ref = 60.0f;
    for (i = 1U; i <= 6000U; i++)
    {
        bool force_tolerance_drop =
            !forced_counter_reset
            && (app.mode0_dc_ready_count
                == cfg.rectifier.dc_ready_confirm_samples - 1U);

        if (force_tolerance_drop)
        {
            RectifierControl_SetDcReference(&app.rectifier, 50.0f);
        }
        /*
         * 母线馈 58 V（低于 60 V 目标但在 5 V 容差内）：
         * 新门控行为下 PI 必须实际请求电流才会使能整流门极，
         * 馈 60 V 时 PI 输出为 0，门极按设计保持关断。
         */
        Test_SetFullRawFrame(&cfg, &raw, i, 36.0f, 58.0f);
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        if (force_tolerance_drop)
        {
            TEST_CHECK(app.mode0_dc_ready_count == 0U);
            TEST_CHECK(!out.inverter.enabled);
            RectifierControl_SetDcReference(&app.rectifier, 60.0f);
            forced_counter_reset = true;
        }
        if (out.rectifier.enabled)
        {
            saw_rectifier = true;
        }
        if (out.inverter.enabled)
        {
            TEST_CHECK(out.rectifier.enabled);
            saw_both_stages = true;
            break;
        }
        if (out.rectifier.enabled)
        {
            rectifier_only_samples++;
        }
    }

    TEST_CHECK(saw_rectifier);
    TEST_CHECK(saw_both_stages);
    TEST_CHECK(forced_counter_reset);
    TEST_CHECK(rectifier_only_samples >= cfg.rectifier.dc_ready_confirm_samples
                                      - 1U);
    TEST_CHECK(app.mode0_inverter_ready);
    TEST_CHECK(app.rectifier_dc_ready);
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);

    command.external_fault = true;
    Test_SetFullRawFrame(&cfg, &raw, ++i, 36.0f, 60.0f);
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == FAULT_LATCH);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(!out.inverter.enabled);
    return true;
}

static bool Test_OpenLoopProtection(void)
{
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
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    TEST_CHECK(AppControl_SetInverterRunMode(
        &app, INVERTER_OPEN_LOOP));

    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.vdc = UINT16_MAX;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 30.0f;

    for (i = 1U; i <= 4U; i++)
    {
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
    TEST_CHECK(AppControl_SetInverterRunMode(
        &app, INVERTER_CLOSED_LOOP));
    TEST_CHECK(app.inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(app.state.state == FAULT_LATCH);
    TEST_CHECK((app.protection.latched_faults
                & PROTECTION_FAULT_PHASE_OVERCURRENT) != 0U);
    return true;
}

static bool Test_BoardConfigGates(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();

    TEST_CHECK(BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, (app_mode_t)3));

    cfg.rectifier_adc_calibrated = false;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.measurement.iin = Test_Cal(2048.0f, 0.001f);
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.rectifier.current_rms_max_a = 4.0f;
    cfg.protection.input_current_abs_max_a = 4.5f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.dc_voltage_ref = 29.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    cfg.dc_voltage_ref = 65.0f;
    /* 输出参考与母线解耦：母线 65V 时输出仍可独立设 32V。 */
    cfg.output_line_voltage_rms =
        APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V;
    TEST_CHECK(BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 66.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V;
    /* 解耦后输出参考超出 28~34V 微调窗口判非法。 */
    cfg.output_line_voltage_rms = 40.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.output_line_voltage_rms =
        APP_INVERTER_OUTPUT_LINE_RMS_DEFAULT_V;

    cfg.rectifier.sample_time_s = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.measurement.ia.gain = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.measurement.iin.gain = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

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
    cfg.inverter.current_qpr_kp = 0.0f;
    cfg.inverter.current_qpr_kr = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));

    cfg = Test_ValidFullBoardConfig();
    cfg.rectifier.current_qpr_kp = 0.0f;
    cfg.rectifier.current_qpr_kr = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_RECTIFIER));
    return true;
}

static bool Test_ModeValidationCache(void)
{
    control_config_t cfg = Test_ValidInverterBoardConfig();
    control_config_t invalid_cfg;
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    TEST_CHECK(app.board_config_valid);
    TEST_CHECK(app.validated_cfg == &cfg);
    TEST_CHECK(app.validated_mode == MODE1_INVERTER);

    invalid_cfg = cfg;
    invalid_cfg.measurement.ia.gain = 0.0f;
    app.cfg = &invalid_cfg;
    raw.ia = 2048U;
    raw.ib = 2048U;
    raw.vab = 2048U;
    raw.vbc = 2048U;
    raw.sequence = 1U;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 30.0f;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!out.board_config_valid);
    TEST_CHECK(out.state == OFF);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);

    app.cfg = &cfg;
    TEST_CHECK(!AppControl_SetMode(&app, MODE0_FULL));
    TEST_CHECK(app.mode == MODE1_INVERTER);
    TEST_CHECK(app.board_config_valid);
    TEST_CHECK(app.validated_cfg == &cfg);
    TEST_CHECK(app.validated_mode == MODE1_INVERTER);

    app.mode = (app_mode_t)3;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(!out.board_config_valid);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);
    return true;
}

static bool Test_PublicApiAndVdcSetting(void)
{
    const AppStatus *status;
    const AppPwmCommand *pwm;
    uint32_t pair[4];
    unsigned int i;

    App_Init();
    status = App_GetStatus();
    TEST_CHECK(status != NULL);
    TEST_CHECK(BoardPower_ConfigValid());
    TEST_CHECK(BoardPower_ValidateModeConfig(
        &g_board_power_config, MODE1_INVERTER));
    TEST_CHECK(BoardPower_ValidateModeConfig(
        &g_board_power_config, MODE2_RECTIFIER));
    TEST_CHECK(status->state == OFF);
    TEST_CHECK(status->board_config_valid);
    TEST_CHECK(status->mode == MODE1_INVERTER);
    TEST_CHECK(status->inverter_run_mode == INVERTER_CLOSED_LOOP);
    TEST_CHECK(TEST_NEAR(status->output_line_voltage_reference_v,
                         32.0f, 1.0e-5f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v,
                         62.0f, 1.0e-5f));
    TEST_CHECK(App_RequestMode(MODE0_FULL));
    TEST_CHECK(App_GetStatus()->mode == MODE0_FULL);
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    TEST_CHECK(App_GetStatus()->mode == MODE2_RECTIFIER);
    TEST_CHECK(!App_SetOutputFrequency(30U));
    TEST_CHECK(App_RequestMode(MODE1_INVERTER));
    TEST_CHECK(App_GetStatus()->mode == MODE1_INVERTER);
    TEST_CHECK(!App_RequestMode((app_mode_t)3));
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    TEST_CHECK(App_GetStatus()->inverter_run_mode
               == INVERTER_OPEN_LOOP);
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
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        32.0f, 1.0e-5f));
    TEST_CHECK(!App_SetDcVoltageReference(66.0f));
    TEST_CHECK(App_SetDcVoltageReference(30.0f));
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        32.0f, 1.0e-5f));

    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    pair[1] = ((uint32_t)2085U << 16) | 2085U;
    pair[2] = ((uint32_t)2110U << 16) | 2110U;
    pair[3] = ((uint32_t)2048U << 16) | 1833U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 1U));
    pair[3] = ((uint32_t)3000U << 16) | 1833U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    App_RequestStart();
    TEST_CHECK(!App_RequestMode(MODE2_RECTIFIER));
    TEST_CHECK(!App_SetOutputFrequency(60U));
    TEST_CHECK(App_GetStatus()->mode == MODE1_INVERTER);
    TEST_CHECK(TEST_NEAR(App_GetStatus()->output_frequency_hz,
                         30.0f, 1.0e-6f));
    TEST_CHECK(!App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(!App_SetDcVoltageReference(31.0f));
    for (i = 3U; i <= 50U; i++)
    {
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
    TEST_CHECK(!pwm->rectifier_enable_request);
    TEST_CHECK(TEST_NEAR(pwm->rectifier_a, 0.5f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_v, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v, 30.0f, 1.0e-6f));
    /* 独立输出参考 32V 线受 30V 母线过调制限幅：
     * (duty_max-duty_min)*Vdc/sqrt(6)*0.95 相 RMS，再折算线电压。 */
    TEST_CHECK(TEST_NEAR(
        status->output_line_voltage_reference_v,
        (0.98f - 0.02f) * 30.0f * 0.40824829046386301637f
            * APP_INVERTER_VOLTAGE_MARGIN * APP_SQRT3_F,
        1.0e-4f));
    TEST_CHECK(!App_SetDcVoltageReference(60.0f));

    App_RequestStop();
    for (i = 51U; i <= 75U; i++)
    {
        TEST_CHECK(App_ApplyAdcPairs(pair, i));
        App_FastStep20kHz();
    }
    TEST_CHECK(App_GetStatus()->state == OFF);
    TEST_CHECK(App_SetInverterRunMode(INVERTER_CLOSED_LOOP));
    TEST_CHECK(App_SetDcVoltageReference(60.0f));
    /* 解耦后输出参考由控制步根据当前母线参考重算，补一拍刷新。 */
    TEST_CHECK(App_ApplyAdcPairs(pair, 76U));
    App_FastStep20kHz();
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        32.0f, 1.0e-5f));

    App_TripFromISR(PROTECTION_FAULT_EXTERNAL);
    TEST_CHECK(!App_GetPwmCommand()->inverter_enable_request);
    TEST_CHECK(!App_GetPwmCommand()->rectifier_enable_request);
    return true;
}

static bool Test_OutputOvSnapshot(void)
{
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
    TEST_CHECK((status->faults
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
    TEST_CHECK(!status->fault_snapshot.valid);

    TEST_CHECK(App_ApplyAdcPairs(pair, 7002U));
    App_FastStep20kHz();

    status = App_GetStatus();
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK((status->faults
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(status->fault_snapshot.valid);
    TEST_CHECK(status->fault_snapshot.inverter_run_mode
               == INVERTER_CLOSED_LOOP);
    TEST_CHECK((status->fault_snapshot.faults
                & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(status->fault_snapshot.sample_sequence == 7002U);
    TEST_CHECK(TEST_NEAR(
        status->fault_snapshot.dc_voltage_reference_v,
        62.0f, 1.0e-6f));
    TEST_CHECK(status->fault_snapshot.pc2_raw == 3500U);
    TEST_CHECK(status->fault_snapshot.pc3_raw == 2110U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc0_raw == 2085U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc1_raw == 2085U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc2_raw == 3500U);
    TEST_CHECK(status->fault_snapshot.trigger_raw.pc3_raw == 2110U);
    TEST_CHECK(status->fault_snapshot.fast_history_count == 0U);
    TEST_CHECK(!status->fault_snapshot.previous_control.valid);
    TEST_CHECK(status->fault_snapshot.va_v > 32.0f);
    TEST_CHECK(TEST_NEAR(
        status->fault_snapshot.vab_v,
        ((float)3500U - g_board_power_config.measurement.vab.offset_count)
            * g_board_power_config.measurement.vab.gain,
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

static bool Test_InputOvSnapshot(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    uint16_t high_raw;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(RectifierControl_Init(&app.rectifier, &cfg.rectifier));
    TEST_CHECK(AppControl_SetMode(&app, MODE2_RECTIFIER));

    command.dc_voltage_ref = 60.0f;
    raw.iin = Test_RawFromPhysical(&cfg.measurement.iin, 0.0f);
    high_raw = Test_RawFromPhysical(&cfg.measurement.vin, 65.5f);
    raw.vin = high_raw;
    raw.vdc = Test_RawFromPhysical(&cfg.measurement.vdc, 19.3f);
    raw.valid = true;
    raw.sequence = 9001U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK((out.faults
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) == 0UL);
    TEST_CHECK(!app.fault_snapshot.valid);

    raw.sequence = 9002U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK((out.faults
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(app.fault_snapshot.valid);
    TEST_CHECK((app.fault_snapshot.faults
                & PROTECTION_FAULT_INPUT_OVERVOLTAGE) != 0UL);
    TEST_CHECK(app.fault_snapshot.sample_sequence == 9002U);
    TEST_CHECK(app.fault_snapshot.vin_raw == high_raw);
    TEST_CHECK(app.fault_snapshot.previous_vin_valid);
    TEST_CHECK(app.fault_snapshot.previous_vin_sequence == 9001U);
    TEST_CHECK(app.fault_snapshot.previous_vin_raw == high_raw);
    TEST_CHECK(TEST_NEAR(app.fault_snapshot.vin_v, 65.5f, 0.11f));
    TEST_CHECK(TEST_NEAR(
        app.fault_snapshot.previous_vin_v, 65.5f, 0.11f));
    TEST_CHECK(TEST_NEAR(app.fault_snapshot.vdc_v, 19.3f, 0.11f));

    raw.vin = Test_RawFromPhysical(&cfg.measurement.vin, 0.0f);
    raw.sequence = 9003U;
    command.reset_fault = true;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.faults == PROTECTION_FAULT_NONE);
    TEST_CHECK(!app.fault_snapshot.valid);
    return true;
}

static bool Test_InputOcSnapshot(void)
{
    control_config_t cfg = Test_ValidFullBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    uint16_t high_raw;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(RectifierControl_Init(&app.rectifier, &cfg.rectifier));
    TEST_CHECK(AppControl_SetMode(&app, MODE2_RECTIFIER));

    command.dc_voltage_ref = 60.0f;
    /* 输入过流阈值现为 5.6A：用 5.65A 触发（仍在 TA 可测上限 5.68A 内）。 */
    high_raw = Test_RawFromPhysical(&cfg.measurement.iin, 5.65f);
    raw.iin = high_raw;
    raw.vin = Test_RawFromPhysical(&cfg.measurement.vin, 0.0f);
    raw.vdc = Test_RawFromPhysical(&cfg.measurement.vdc, 19.3f);
    raw.valid = true;
    raw.sequence = 9101U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK((out.faults
                & PROTECTION_FAULT_INPUT_OVERCURRENT) == 0UL);
    TEST_CHECK(!app.fault_snapshot.valid);

    raw.sequence = 9102U;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK((out.faults
                & PROTECTION_FAULT_INPUT_OVERCURRENT) != 0UL);
    TEST_CHECK(app.fault_snapshot.valid);
    TEST_CHECK((app.fault_snapshot.faults
                & PROTECTION_FAULT_INPUT_OVERCURRENT) != 0UL);
    TEST_CHECK(app.fault_snapshot.sample_sequence == 9102U);
    TEST_CHECK(app.fault_snapshot.iin_raw == high_raw);
    TEST_CHECK(app.fault_snapshot.previous_iin_valid);
    TEST_CHECK(app.fault_snapshot.previous_iin_sequence == 9101U);
    TEST_CHECK(app.fault_snapshot.previous_iin_raw == high_raw);
    TEST_CHECK(TEST_NEAR(app.fault_snapshot.iin_a, 5.65f, 0.01f));
    TEST_CHECK(TEST_NEAR(
        app.fault_snapshot.previous_iin_a, 5.65f, 0.01f));
    TEST_CHECK(TEST_NEAR(app.fault_snapshot.vdc_v, 19.3f, 0.11f));

    raw.iin = Test_RawFromPhysical(&cfg.measurement.iin, 0.0f);
    raw.sequence = 9103U;
    command.reset_fault = true;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.faults == PROTECTION_FAULT_NONE);
    TEST_CHECK(!app.fault_snapshot.valid);
    return true;
}

static bool Test_OutputOvDiagnosticHistory(void)
{
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
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 45.0f;

    for (i = 1U; i <= 4U; i++)
    {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(out.state == RUN);

    /*
     * Fill and wrap the 16-frame fast history. One safe 20 V line
     * excursion also leaves a recognizable completed-cycle peak.
     */
    for (i = 0U; i < 360U; i++)
    {
        raw.ia = (uint16_t)(2000U + (i % 5U));
        raw.ib = (uint16_t)(2000U + (i % 7U));
        raw.vab = (uint16_t)(2000U + (i % 20U));
        raw.vbc = (i == 100U) ? 2200U : 2000U;
        raw.sequence = 5U + i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK((out.faults
                    & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) == 0UL);
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
    TEST_CHECK(app.fault_snapshot.fast_history_count
               == APP_FAULT_FAST_HISTORY_DEPTH);
    TEST_CHECK(app.fault_snapshot.fast_history[0].sample_sequence
               == 350U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].sample_sequence
               == 365U);
    TEST_CHECK(app.fault_snapshot.fast_history[0].pc2_raw == 2005U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc2_raw == 2500U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc0_raw == 2022U);
    TEST_CHECK(app.fault_snapshot.fast_history[15].pc1_raw == 2011U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc0_raw == 2022U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc1_raw == 2011U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc2_raw == 2500U);
    TEST_CHECK(app.fault_snapshot.trigger_raw.pc3_raw == 2000U);
    TEST_CHECK(app.fault_snapshot.previous_control.valid);
    TEST_CHECK(app.fault_snapshot.previous_control.sample_sequence
               == 365U);
    TEST_CHECK(app.fault_snapshot.previous_control.applied_scale > 0.0f);
    TEST_CHECK(app.fault_snapshot.cycle_peak_count >= 1U);
    TEST_CHECK(app.fault_snapshot.cycle_peak_v[0] > 10.0f);
    TEST_CHECK(app.fault_snapshot.partial_cycle_samples > 0U);
    TEST_CHECK(app.fault_snapshot.partial_cycle_samples
               < app.fault_recorder.cycle_target_samples);

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

static bool Test_OutputOvNoStaleHistoryAfterStop(void)
{
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
    TEST_CHECK(AppControl_SetMode(&app, MODE1_INVERTER));
    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.valid = true;
    command.enable = true;
    command.dc_voltage_ref = 45.0f;

    for (i = 1U; i <= 24U; i++)
    {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(app.fault_recorder.active);
    TEST_CHECK(app.fault_recorder.fast_history_count
               == APP_FAULT_FAST_HISTORY_DEPTH);

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

static bool Test_IsrFaultResetSemantics(void)
{
    const AppStatus *status;
    uint32_t pair[4];

    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    pair[1] = ((uint32_t)2048U << 16) | 2048U;
    pair[2] = ((uint32_t)2048U << 16) | 2048U;
    pair[3] = ((uint32_t)2048U << 16) | 2048U;

    App_Init();
    App_TripFromISR(PROTECTION_FAULT_EXTERNAL
                  | PROTECTION_FAULT_LOOP_OVERRUN);
    TEST_CHECK(App_ApplyAdcPairs(pair, 1U));
    App_FastStep20kHz();
    status = App_GetStatus();
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK(!status->pwm_allowed);
    TEST_CHECK((status->faults & PROTECTION_FAULT_EXTERNAL) != 0U);
    TEST_CHECK((status->faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0U);
    TEST_CHECK(App_SetInverterRunMode(INVERTER_OPEN_LOOP));
    TEST_CHECK(App_GetStatus()->inverter_run_mode
               == INVERTER_OPEN_LOOP);
    TEST_CHECK(App_SetDcVoltageReference(59.0f));
    TEST_CHECK(TEST_NEAR(App_GetStatus()->dc_voltage_reference_v,
                         59.0f, 1.0e-6f));
    /* 解耦后母线参考改为 59V 不影响输出参考（仍为独立设定的 32V）。 */
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        32.0f,
        1.0e-5f));
    TEST_CHECK(App_RequestMode(MODE2_RECTIFIER));
    status = App_GetStatus();
    TEST_CHECK(status->mode == MODE2_RECTIFIER);
    TEST_CHECK(status->state == FAULT_LATCH);
    TEST_CHECK(!status->pwm_allowed);
    TEST_CHECK((status->faults & PROTECTION_FAULT_EXTERNAL) != 0U);
    TEST_CHECK((status->faults & PROTECTION_FAULT_LOOP_OVERRUN) != 0U);

    App_ResetFault();
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    App_FastStep20kHz();
    TEST_CHECK(App_GetStatus()->faults == PROTECTION_FAULT_NONE);
    TEST_CHECK(App_GetStatus()->mode == MODE2_RECTIFIER);
    TEST_CHECK(App_GetStatus()->inverter_run_mode
               == INVERTER_OPEN_LOOP);

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
    TEST_CHECK((App_GetStatus()->faults
                & PROTECTION_FAULT_MEASUREMENT) != 0U);
    return true;
}

typedef bool (*TestFunction)(void);

typedef struct
{
    const char *name;
    TestFunction run;
} TestCase;

int main(void)
{
    static const TestCase tests[] =
    {
        {"measurement", Test_Measurement},
        {"dual ADC rank mapping", Test_AdcRankMapping},
        {"mode-specific ADC diagnostics",
         Test_ModeSpecificAdcDiagnostics},
        {"inverter-only ADC path", Test_InverterMeasurementPath},
        {"rectifier-only ADC path", Test_RectifierMeasurementPath},
        {"phase recurrence", Test_PhaseRecurrence},
        {"PI anti-windup", Test_PiAntiWindup},
        {"Tustin QPR", Test_Qpr},
        {"Tustin notch", Test_Notch},
        {"SVPWM and rectifier PWM", Test_Modulation},
        {"inverter cascaded loops", Test_InverterCascade},
        {"inverter open loop", Test_InverterOpenLoop},
        {"rectifier synchronized loop", Test_RectifierControl},
        {"production rectifier parameters",
         Test_ProductionRectifierParameters},
        {"protection latch", Test_Protection},
        {"power state machine", Test_PowerState},
        {"board configuration gates", Test_BoardConfigGates},
        {"mode validation cache", Test_ModeValidationCache},
        {"inverter-only app flow", Test_InverterOnlyAppFlow},
        {"rectifier-only app flow", Test_RectifierOnlyAppFlow},
        {"rectifier zero-cross start", Test_RectifierZeroCrossStart},
        {"full-stage app flow", Test_FullStageAppFlow},
        {"open-loop protection", Test_OpenLoopProtection},
        {"public API and Vdc setting", Test_PublicApiAndVdcSetting},
        {"input OC fault snapshot", Test_InputOcSnapshot},
        {"input OV fault snapshot", Test_InputOvSnapshot},
        {"output OV fault snapshot", Test_OutputOvSnapshot},
        {"output OV diagnostic history", Test_OutputOvDiagnosticHistory},
        {"output OV stale history guard",
         Test_OutputOvNoStaleHistoryAfterStop},
        {"ISR fault reset semantics", Test_IsrFaultResetSemantics}
    };
    size_t i;

    for (i = 0U; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        bool passed = tests[i].run();
        printf("[%s] %s\n", passed ? "PASS" : "FAIL", tests[i].name);
        if (!passed)
        {
            g_failed++;
        }
    }

    if (g_failed != 0)
    {
        printf("%d test group(s) failed.\n", g_failed);
        return 1;
    }
    printf("All host tests passed.\n");
    return 0;
}
