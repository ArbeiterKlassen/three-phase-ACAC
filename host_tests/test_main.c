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
    TEST_CHECK(raw.ia == 303U);
    TEST_CHECK(raw.ib == 404U);
    /* 硬件契约：低 16 位 ADC1 rank3/PC2 是 Vab，高 16 位 ADC2 rank3/PC3 是 Vbc。 */
    TEST_CHECK(raw.vab == 505U);
    TEST_CHECK(raw.vbc == 606U);
    TEST_CHECK(raw.vdc == 707U);
    TEST_CHECK(raw.vin_diag == 203U);
    TEST_CHECK(raw.sequence == 99U);

    pair[3] = ((uint32_t)212U << 16) | 707U;
    TEST_CHECK(!Measurement_UnpackDualAdcPairs(pair, 100U, 9U, &raw));
    TEST_CHECK(!raw.valid);
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
    cal.vdc = Test_Cal(0.0f, 0.1f);

    TEST_CHECK(!Measurement_CalibrationValid(&cal));
    TEST_CHECK(Measurement_InverterCalibrationValid(&cal));
    TEST_CHECK(!Measurement_RectifierCalibrationValid(&cal));

    pair[0] = ((uint32_t)4000U << 16) | 1U;
    pair[1] = ((uint32_t)120U << 16) | 110U;
    /* 低 16 位 ADC1/PC2 = Vab(80)，高 16 位 ADC2/PC3 = Vbc(130)。 */
    pair[2] = ((uint32_t)130U << 16) | 80U;
    pair[3] = ((uint32_t)203U << 16) | 707U;
    TEST_CHECK(Measurement_UnpackInverterAdcPairs(pair, 101U, &raw));
    TEST_CHECK(raw.valid);
    /* 逆变解包保留全部 rank（含实测 Vdc），只是跳过 Vin 冗余诊断。 */
    TEST_CHECK(raw.iin == 1U);
    TEST_CHECK(raw.vin == 4000U);
    TEST_CHECK(raw.vdc == 707U);
    TEST_CHECK(raw.vin_diag == 203U);
    TEST_CHECK(Measurement_ConvertInverter(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.iin, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vin, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ia, 10.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ib, 20.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.ic, -30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vab, -20.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vbc, 30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vdc, 70.7f, 1.0e-5f));
    TEST_CHECK(m.sequence == 101U);

    cal.vbc.gain = 0.0f;
    TEST_CHECK(!Measurement_InverterCalibrationValid(&cal));
    TEST_CHECK(!Measurement_ConvertInverter(&cal, &raw, &m));
    cal.vbc.gain = 1.0f;
    cal.vdc.gain = 0.0f;
    TEST_CHECK(!Measurement_InverterCalibrationValid(&cal));
    TEST_CHECK(!Measurement_ConvertInverter(&cal, &raw, &m));
    return true;
}

static bool Test_RectifierMeasurementPath(void)
{
    MeasurementCalibration cal = {0};
    AdcRawFrame raw = {0};
    Measurements m;

    cal.iin = Test_Cal(100.0f, 0.1f);
    cal.vin = Test_Cal(100.0f, 1.0f);
    cal.vdc = Test_Cal(0.0f, 0.1f);
    TEST_CHECK(Measurement_RectifierCalibrationValid(&cal));
    TEST_CHECK(!Measurement_InverterCalibrationValid(&cal));

    raw.iin = 110U;
    raw.vin = 150U;
    raw.vdc = 450U;
    raw.sequence = 5U;
    raw.valid = true;
    TEST_CHECK(Measurement_ConvertRectifier(&cal, &raw, &m));
    TEST_CHECK(TEST_NEAR(m.iin, 1.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vin, 50.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vdc, 45.0f, 1.0e-6f));
    /* 未连接的三相通道固定为 0，避免悬空采样误触发保护。 */
    TEST_CHECK(TEST_NEAR(m.ia, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.vab, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(m.va, 0.0f, 1.0e-6f));
    TEST_CHECK(m.sequence == 5U);

    cal.vin.gain = 0.0f;
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

static RectifierControlConfig Test_RectifierConfig(void)
{
    RectifierControlConfig cfg = {0};
    cfg.sample_time_s = 50.0e-6f;
    cfg.line_frequency_hz = 50.0f;
    cfg.outer_loop_divider = 20U;
    cfg.rms_filter_time_constant_s = 0.02f;
    cfg.input_voltage_rms_min_v = 1.0f;
    cfg.dc_voltage_max_v = 200.0f;
    cfg.dc_reference_slew_v_per_s = 200.0f;
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

    m.valid = true;
    m.vdc = 50.0f;
    TEST_CHECK(RectifierControl_Init(&control, &cfg));
    RectifierControl_SetDcReference(&control, 150.0f);

    /* 先在PWM关闭状态建立输入RMS估计。 */
    for (i = 0U; i < 4000U; i++)
    {
        time = cfg.sample_time_s * (float)i;
        m.vin = 100.0f * 1.41421356237f
              * cosf(2.0f * APP_PI_F * 50.0f * time);
        TEST_CHECK(RectifierControl_Step(&control, &m, false, &out));
    }
    TEST_CHECK(TEST_NEAR(control.vin_rms_v, 100.0f, 3.0f));

    for (i = 0U; i < 1000U; i++)
    {
        time = cfg.sample_time_s * (float)(4000U + i);
        m.vin = 100.0f * 1.41421356237f
              * cosf(2.0f * APP_PI_F * 50.0f * time);
        TEST_CHECK(RectifierControl_Step(&control, &m, true, &out));
        TEST_CHECK((out.current_reference * m.vin) >= -1.0e-4f);
    }

    /* 200 V/s软启动从0 V参考起步，50 ms后约为10 V。 */
    TEST_CHECK(TEST_NEAR(out.dc_voltage_reference, 10.0f, 0.2f));
    TEST_CHECK(out.input_voltage_rms > 90.0f);
    TEST_CHECK(out.pwm.valid);
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
    TEST_CHECK(Protection_Evaluate(&protection, &cfg, &m,
                                   true, false, false) == 0UL);
    m.iin = 11.0f;
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
    in.discharge_done = true;
    PowerState_Step(&machine, &cfg, &in);
    PowerState_Step(&machine, &cfg, &in);
    TEST_CHECK(machine.state == OFF);
    return true;
}

static control_config_t Test_ValidBoardConfig(void)
{
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
    cfg.inverter.voltage_qpr_kp = 1.0f;
    cfg.inverter.current_qpr_kp = 1.0f;
    return cfg;
}

static void Test_FillInverterCals(control_config_t *cfg)
{
    cfg->measurement.ia = Test_Cal(2000.0f, 0.01f);
    cfg->measurement.ib = Test_Cal(2000.0f, 0.01f);
    cfg->measurement.vab = Test_Cal(2000.0f, 0.1f);
    cfg->measurement.vbc = Test_Cal(2000.0f, 0.1f);
    cfg->measurement.vdc = Test_Cal(0.0f, 0.1f);
    cfg->protection.dc_voltage_min_run_v = 10.0f;
    cfg->state.self_test_confirm_samples = 1U;
    cfg->state.ready_confirm_samples = 1U;
    cfg->state.discharge_confirm_samples = 1U;
}

static bool Test_InverterOnlyAppFlow(void)
{
    control_config_t cfg = Test_ValidBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out;
    AdcRawFrame raw = {0};
    unsigned int i;

    Test_FillInverterCals(&cfg);

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    /* 默认模式为 MODE0_FULL；OFF 状态下三种模式均可切换。 */
    TEST_CHECK(app.mode == MODE0_FULL);
    TEST_CHECK(app.rectifier.initialized);
    TEST_CHECK(AppControl_SetMode(&app, MODE1_RECTIFIER));
    TEST_CHECK(AppControl_SetMode(&app, MODE0_FULL));
    TEST_CHECK(AppControl_SetMode(&app, MODE2_INVERTER));

    raw.ia = 2000U;
    raw.ib = 2000U;
    raw.vab = 2000U;
    raw.vbc = 2000U;
    raw.vdc = 300U; /* 实测母线 30.0 V */
    raw.valid = true;
    command.enable = true;
    command.precharge_done = false;
    command.phase_ready = false;
    /* 比例模式必须忽略独立输出参考，只按 Vdc 推导。 */
    command.inverter_vrms_ref = 1.0f;
    command.dc_voltage_ref = 30.0f;

    for (i = 1U; i <= 4U; i++)
    {
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    }

    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);
    TEST_CHECK(TEST_NEAR(out.rectifier.leg_a, 0.5f, 1.0e-6f));
    /* 实测 Vdc 来自 PC4 换算，不再被键盘设定值替代。 */
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 30.0f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(
        app.inverter.reference_target_rms,
        30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        1.0e-5f));

    /* 键盘设定升到 65 V 且实测母线也为 65 V 时，参考同步升高。 */
    command.dc_voltage_ref = 65.0f;
    raw.vdc = 650U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 65.0f, 1.0e-4f));
    TEST_CHECK(TEST_NEAR(
        app.inverter.reference_target_rms,
        65.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
            * APP_INV_SQRT3_F,
        1.0e-5f));

    /* 键盘设定保持 65 V 但实测母线只有 30 V：限幅按实测 Vdc 收紧。 */
    raw.vdc = 300U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(app.inverter.reference_target_rms
               < 65.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO
                     * APP_INV_SQRT3_F);

    /* 实测母线跌落到 0：欠压保护锁存，状态机安全关断。 */
    raw.vdc = 0U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK((out.faults & PROTECTION_FAULT_DC_UNDERVOLTAGE) != 0U);
    TEST_CHECK(out.state == FAULT_LATCH);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);

    command.enable = false;
    command.reset_fault = true;
    raw.vdc = 300U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    command.reset_fault = false;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    /* 母线仍有 30 V：保持 DISCHARGE，不允许带电回 OFF 切模式。 */
    TEST_CHECK(out.state == DISCHARGE);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);

    /* 母线放电到安全阈值（10 V）以下并连续确认后才回到 OFF。 */
    raw.vdc = 50U;
    raw.sequence++;
    TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
    TEST_CHECK(out.state == OFF);
    TEST_CHECK(!out.pwm_allowed);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK(!out.rectifier.enabled);

    return true;
}

static bool Test_BoardModeValidation(void)
{
    control_config_t cfg = Test_ValidBoardConfig();

    /* 整流参数为零增益/未标定时：MODE2 仍有效，MODE1/MODE0 无效。 */
    cfg.measurement.iin.gain = 0.0f;
    cfg.measurement.vin.gain = 0.0f;
    cfg.rectifier.dc_pi_kp = 0.0f;
    cfg.rectifier.dc_pi_ki = 0.0f;
    cfg.rectifier.current_qpr_kp = 0.0f;
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));

    /* 恢复整流参数后三模式均有效；原接口按 MODE0 最严格校验。 */
    cfg = Test_ValidBoardConfig();
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE2_INVERTER));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));
    TEST_CHECK(BoardPower_ValidateConfig(&cfg));

    /* 逆变 Vdc 标定缺失：MODE2/MODE0 无效，MODE1 仍有效。 */
    cfg.measurement.vdc.gain = 0.0f;
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));

    /* 逆变三相标定缺失不影响 MODE1。 */
    cfg = Test_ValidBoardConfig();
    cfg.measurement.ia.gain = 0.0f;
    cfg.measurement.vab.gain = 0.0f;
    TEST_CHECK(BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE2_INVERTER));
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE0_FULL));

    /* 整流 QPR 零增益必须视为无效。 */
    cfg = Test_ValidBoardConfig();
    cfg.rectifier.current_qpr_kp = 0.0f;
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));
    cfg.rectifier.current_qpr_kp = 2.0f;
    cfg.rectifier.dc_pi_kaw = 0.0f;
    TEST_CHECK(!BoardPower_ValidateModeConfig(&cfg, MODE1_RECTIFIER));
    return true;
}

static bool Test_BoardConfigGates(void)
{
    control_config_t cfg = Test_ValidBoardConfig();

    TEST_CHECK(BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 29.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 65.0f;
    cfg.output_line_voltage_rms =
        65.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO;
    TEST_CHECK(BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = 66.0f;
    cfg.output_line_voltage_rms =
        66.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.dc_voltage_ref = APP_INVERTER_DC_VOLTAGE_DEFAULT_V;
    cfg.output_line_voltage_rms =
        APP_INVERTER_DC_VOLTAGE_DEFAULT_V
        * APP_INVERTER_OUTPUT_LINE_RMS_RATIO;
    cfg.measurement.ia.gain = 0.0f;
    TEST_CHECK(!BoardPower_ValidateConfig(&cfg));
    cfg.measurement.ia.gain = 1.0f;
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
    TEST_CHECK(status->state == OFF);
    TEST_CHECK(status->board_config_valid);
    /* 上电默认 MODE0_FULL；OFF 下可自由切换三种模式。 */
    TEST_CHECK(status->mode == MODE0_FULL);
    TEST_CHECK(TEST_NEAR(status->output_line_voltage_reference_v,
                         36.0f, 1.0e-5f));
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v,
                         60.0f, 1.0e-5f));
    TEST_CHECK(App_RequestMode(MODE1_RECTIFIER));
    TEST_CHECK(App_GetStatus()->mode == MODE1_RECTIFIER);
    /* 整流模式没有三相输出，频率设置必须被拒绝。 */
    TEST_CHECK(!App_SetOutputFrequency(30U));
    TEST_CHECK(App_RequestMode(MODE2_INVERTER));
    TEST_CHECK(App_GetStatus()->mode == MODE2_INVERTER);
    TEST_CHECK(App_SetOutputFrequency(30U));
    TEST_CHECK(App_SetOutputLineVoltageReference(36.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(30.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(40.0f));
    TEST_CHECK(!App_SetOutputLineVoltageReference(300.0f));
    TEST_CHECK(!App_SetDcVoltageReference(29.0f));
    TEST_CHECK(App_SetDcVoltageReference(65.0f));
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        39.0f, 1.0e-5f));
    TEST_CHECK(!App_SetDcVoltageReference(66.0f));
    TEST_CHECK(App_SetDcVoltageReference(30.0f));
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        18.0f, 1.0e-5f));
    TEST_CHECK(App_SetOutputLineVoltageReference(18.0f));

    pair[0] = ((uint32_t)2048U << 16) | 2048U;
    pair[1] = ((uint32_t)2085U << 16) | 2085U;
    pair[2] = ((uint32_t)2110U << 16) | 2110U;
    pair[3] = ((uint32_t)2048U << 16) | 1833U;
    TEST_CHECK(App_ApplyAdcPairs(pair, 1U));
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    App_RequestStart();
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
    TEST_CHECK(pwm->inverter_enable_request);
    TEST_CHECK(!pwm->rectifier_enable_request);
    TEST_CHECK(TEST_NEAR(pwm->rectifier_a, 0.5f, 1.0e-6f));
    /* MODE2 状态里的 Vdc 也是 PC4 实测换算值（约 30 V）。 */
    TEST_CHECK(TEST_NEAR(status->dc_voltage_v, 30.0f, 0.1f));
    TEST_CHECK(!status->rectifier_dc_ready);
    TEST_CHECK(TEST_NEAR(status->dc_voltage_reference_v, 30.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        status->output_line_voltage_reference_v,
        30.0f * APP_INVERTER_OUTPUT_LINE_RMS_RATIO,
        1.0e-4f));
    TEST_CHECK(!App_SetDcVoltageReference(60.0f));

    App_RequestStop();
    for (i = 51U; i <= 75U; i++)
    {
        TEST_CHECK(App_ApplyAdcPairs(pair, i));
        App_FastStep20kHz();
    }
    /* 母线仍约 30 V：停机后保持 DISCHARGE，不能带电回 OFF 改设定。 */
    TEST_CHECK(App_GetStatus()->state == DISCHARGE);
    TEST_CHECK(!App_SetDcVoltageReference(60.0f));

    /* 母线放电到安全阈值（10 V）以下并连续确认后才允许回 OFF。 */
    pair[3] = ((uint32_t)2048U << 16) | 300U; /* Vdc 约 4.6 V */
    for (i = 76U; i <= 125U; i++)
    {
        TEST_CHECK(App_ApplyAdcPairs(pair, i));
        App_FastStep20kHz();
    }
    TEST_CHECK(App_GetStatus()->state == OFF);
    TEST_CHECK(App_SetDcVoltageReference(60.0f));
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        36.0f, 1.0e-5f));

    App_TripFromISR(PROTECTION_FAULT_EXTERNAL);
    TEST_CHECK(!App_GetPwmCommand()->inverter_enable_request);
    TEST_CHECK(!App_GetPwmCommand()->rectifier_enable_request);
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
    TEST_CHECK(App_SetDcVoltageReference(59.0f));
    TEST_CHECK(TEST_NEAR(App_GetStatus()->dc_voltage_reference_v,
                         59.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(
        App_GetStatus()->output_line_voltage_reference_v,
        35.4f, 1.0e-5f));

    App_ResetFault();
    TEST_CHECK(App_ApplyAdcPairs(pair, 2U));
    App_FastStep20kHz();
    TEST_CHECK(App_GetStatus()->faults == PROTECTION_FAULT_NONE);

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

static uint16_t Test_FloatToRaw(float value, float offset, float gain)
{
    float counts = value / gain + offset;

    if (counts < 0.0f)
    {
        counts = 0.0f;
    }
    if (counts > 4095.0f)
    {
        counts = 4095.0f;
    }
    return (uint16_t)(counts + 0.5f);
}

static bool Test_SvpwmVdcNormalization(void)
{
    SvpwmOutput sv;
    unsigned int period;
    unsigned int i;
    float peak_prev = -1.0f;

    /* 固定 60 V 母线：多个基波周期内重建相电压幅值包络必须恒定。 */
    for (period = 0U; period < 6U; period++)
    {
        float period_peak = 0.0f;

        for (i = 0U; i < 333U; i++)
        {
            float t = (float)(period * 333U + i) * 50.0e-6f;
            float theta = 2.0f * APP_PI_F * 60.0f * t;
            float va_cmd = 20.0f * cosf(theta);
            float vb_cmd = 20.0f * sinf(theta);
            float common;
            float va_applied;
            float vab_applied;
            float vab_expected;

            TEST_CHECK(Svpwm_Generate(va_cmd, vb_cmd, 60.0f,
                                      0.02f, 0.98f, &sv));
            TEST_CHECK(!sv.saturated);
            common = (sv.duty_a + sv.duty_b + sv.duty_c) / 3.0f;
            va_applied = (sv.duty_a - common) * 60.0f;
            vab_applied = (sv.duty_a - sv.duty_b) * 60.0f;
            vab_expected = va_cmd
                - (-0.5f * va_cmd + 0.5f * APP_SQRT3_F * vb_cmd);
            TEST_CHECK(TEST_NEAR(va_applied, va_cmd, 2.0e-3f));
            TEST_CHECK(TEST_NEAR(vab_applied, vab_expected, 2.0e-3f));
            if (fabsf(va_applied) > period_peak)
            {
                period_peak = fabsf(va_applied);
            }
        }
        if (peak_prev >= 0.0f)
        {
            TEST_CHECK(TEST_NEAR(period_peak, peak_prev, 1.0e-3f));
        }
        peak_prev = period_peak;
    }

    /* 母线叠加 100 Hz 纹波：1/vdc 归一化必须抵消 AM，不能产生包络。 */
    for (i = 0U; i < 2000U; i++)
    {
        float t = (float)i * 50.0e-6f;
        float theta = 2.0f * APP_PI_F * 60.0f * t;
        float va_cmd = 20.0f * cosf(theta);
        float vb_cmd = 20.0f * sinf(theta);
        float vdc = 55.0f + 5.0f * sinf(2.0f * APP_PI_F * 100.0f * t);
        float common;
        float va_applied;
        float vab_applied;
        float vab_expected;

        TEST_CHECK(Svpwm_Generate(va_cmd, vb_cmd, vdc,
                                  0.02f, 0.98f, &sv));
        common = (sv.duty_a + sv.duty_b + sv.duty_c) / 3.0f;
        va_applied = (sv.duty_a - common) * vdc;
        vab_applied = (sv.duty_a - sv.duty_b) * vdc;
        vab_expected = va_cmd
            - (-0.5f * va_cmd + 0.5f * APP_SQRT3_F * vb_cmd);
        TEST_CHECK(TEST_NEAR(va_applied, va_cmd, 2.0e-3f));
        TEST_CHECK(TEST_NEAR(vab_applied, vab_expected, 2.0e-3f));
    }
    return true;
}

static bool Test_Mode2FixedVdcNoEnvelope(void)
{
    control_config_t cfg = Test_ValidBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out = {0};
    AdcRawFrame raw = {0};
    float ia = 0.0f;
    float ib = 0.0f;
    float cycle_peak = 0.0f;
    float peak_min = 1.0e9f;
    float peak_max = 0.0f;
    unsigned int i;
    const float vdc = 48.0f;
    const float r_load = 20.0f;
    const float l_load = 20.0e-3f;
    const unsigned int settle_start = 18000U - 6U * 333U;

    cfg.measurement.ia = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vdc = Test_Cal(0.0f, 0.1f);
    cfg.protection.dc_voltage_min_run_v = 10.0f;
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 1U;
    cfg.state.discharge_confirm_samples = 1U;
    cfg.inverter.reference_slew_v_per_s = 400.0f;

    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(AppControl_SetMode(&app, MODE2_INVERTER));

    out.inverter.a = 0.5f;
    out.inverter.b = 0.5f;
    out.inverter.c = 0.5f;
    command.enable = true;
    command.dc_voltage_ref = 48.0f;
    raw.valid = true;

    for (i = 1U; i <= 18000U; i++)
    {
        float duty_mid = (out.inverter.a + out.inverter.b
                          + out.inverter.c) / 3.0f;
        /* 与硬件一致：上一拍占空比延迟一周期施加到 RL 负载。 */
        float va_app = (out.inverter.a - duty_mid) * vdc;
        float vb_app = (out.inverter.b - duty_mid) * vdc;
        float vc_app = (out.inverter.c - duty_mid) * vdc;
        float vab_app = va_app - vb_app;

        ia += (va_app - r_load * ia) * 50.0e-6f / l_load;
        ib += (vb_app - r_load * ib) * 50.0e-6f / l_load;

        raw.ia = Test_FloatToRaw(ia, 2048.0f, 0.01f);
        raw.ib = Test_FloatToRaw(ib, 2048.0f, 0.01f);
        raw.vab = Test_FloatToRaw(vab_app, 2048.0f, 0.1f);
        raw.vbc = Test_FloatToRaw(vb_app - vc_app, 2048.0f, 0.1f);
        raw.vdc = Test_FloatToRaw(vdc, 0.0f, 0.1f);
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK(out.faults == PROTECTION_FAULT_NONE);
        /* SVPWM 归一化必须使用实测母线而不是键盘设定值。 */
        TEST_CHECK(TEST_NEAR(out.measurement.vdc, vdc, 0.2f));

        if (i > settle_start)
        {
            unsigned int phase_idx = (i - settle_start - 1U) % 333U;

            if (fabsf(vab_app) > cycle_peak)
            {
                cycle_peak = fabsf(vab_app);
            }
            if (phase_idx == 332U)
            {
                if (cycle_peak < peak_min)
                {
                    peak_min = cycle_peak;
                }
                if (cycle_peak > peak_max)
                {
                    peak_max = cycle_peak;
                }
                cycle_peak = 0.0f;
            }
        }
    }

    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.inverter.enabled);
    /*
     * 稳态 6 个 60 Hz 周期的线电压幅值包络变化必须小于 5%，
     * 且幅值接近 48*0.6=28.8 Vrms（峰值约 40.7 V）。
     */
    TEST_CHECK(peak_min > 0.80f * 40.7f);
    TEST_CHECK(peak_max < 1.20f * 40.7f);
    TEST_CHECK((peak_max - peak_min) < 0.05f * peak_max);
    return true;
}

static bool Test_RectifierAndFullModeFlow(void)
{
    control_config_t cfg = Test_ValidBoardConfig();
    AppControl app;
    AppCommand command = {0};
    AppOutput out = {0};
    AdcRawFrame raw = {0};
    unsigned int i;
    unsigned int run_seen = 0U;
    bool saw_rect_only = false;
    bool saw_full = false;

    cfg.measurement.iin = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vin = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.ia = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.ib = Test_Cal(2048.0f, 0.01f);
    cfg.measurement.vab = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vbc = Test_Cal(2048.0f, 0.1f);
    cfg.measurement.vdc = Test_Cal(0.0f, 0.1f);
    cfg.rectifier.dc_ready_confirm_samples = 10U;
    cfg.state.self_test_confirm_samples = 1U;
    cfg.state.ready_confirm_samples = 2U;
    cfg.state.discharge_confirm_samples = 1U;

    /* MODE1：仅整流级开放，推导的预充/相位联锁生效。 */
    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(AppControl_SetMode(&app, MODE1_RECTIFIER));

    command.enable = true;
    command.dc_voltage_ref = 60.0f;
    raw.valid = true;
    raw.iin = 2048U;
    raw.vdc = 450U; /* 被动/外部预充到 45 V，超过运行欠压阈值。 */

    for (i = 1U; i <= 6000U; i++)
    {
        float t = (float)(i - 1U) * 50.0e-6f;
        float vin = 32.0f * 1.41421356237f
                  * sinf(2.0f * APP_PI_F * 50.0f * t);
        uint16_t counts = Test_FloatToRaw(vin, 2048.0f, 0.1f);

        raw.vin = counts;
        raw.vin_diag = counts;
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK(out.faults == PROTECTION_FAULT_NONE);
        if (out.state == RUN)
        {
            run_seen++;
        }
    }
    TEST_CHECK(run_seen > 0U);
    TEST_CHECK(out.state == RUN);
    TEST_CHECK(out.pwm_allowed);
    TEST_CHECK(out.rectifier.enabled);
    TEST_CHECK(!out.inverter.enabled);
    TEST_CHECK((out.rectifier.leg_a >= 0.02f)
               && (out.rectifier.leg_a <= 0.98f));
    TEST_CHECK((out.rectifier.leg_b >= 0.02f)
               && (out.rectifier.leg_b <= 0.98f));
    /* 整流换算把未连接的三相量置 0，不能误触发三相保护。 */
    TEST_CHECK(TEST_NEAR(out.measurement.ia, 0.0f, 1.0e-6f));
    TEST_CHECK(TEST_NEAR(out.measurement.vab, 0.0f, 1.0e-6f));
    TEST_CHECK(app.rectifier.vin_rms_v > 25.0f);
    /* 运行中不允许切换模式。 */
    TEST_CHECK(!AppControl_SetMode(&app, MODE2_INVERTER));

    /* MODE0：滤波 Vdc 连续落入容差后才开放逆变级。 */
    TEST_CHECK(AppControl_Init(&app));
    app.cfg = &cfg;
    TEST_CHECK(AppControl_SetMode(&app, MODE0_FULL));
    out = (AppOutput){0};
    raw.ia = 2048U;
    raw.ib = 2048U;
    raw.vab = 2048U;
    raw.vbc = 2048U;
    raw.vdc = 600U; /* 母线已在 60 V 目标 */

    for (i = 1U; i <= 6000U; i++)
    {
        float t = (float)(i - 1U) * 50.0e-6f;
        float vin = 32.0f * 1.41421356237f
                  * sinf(2.0f * APP_PI_F * 50.0f * t);
        uint16_t counts = Test_FloatToRaw(vin, 2048.0f, 0.1f);

        raw.vin = counts;
        raw.vin_diag = counts;
        raw.sequence = i;
        TEST_CHECK(AppControl_Step(&app, &raw, &command, &out));
        TEST_CHECK(out.faults == PROTECTION_FAULT_NONE);
        if (out.pwm_allowed && out.rectifier.enabled
            && !out.inverter.enabled)
        {
            saw_rect_only = true;
        }
        if (out.pwm_allowed && out.rectifier.enabled
            && out.inverter.enabled)
        {
            saw_full = true;
        }
    }
    TEST_CHECK(saw_rect_only);
    TEST_CHECK(saw_full);
    TEST_CHECK(app.mode0_inverter_ready);
    TEST_CHECK(app.rectifier_dc_ready);
    TEST_CHECK(TEST_NEAR(out.measurement.vdc, 60.0f, 0.2f));
    TEST_CHECK((out.inverter.a >= 0.02f) && (out.inverter.a <= 0.98f));
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
        {"inverter-only ADC path", Test_InverterMeasurementPath},
        {"rectifier-only ADC path", Test_RectifierMeasurementPath},
        {"phase recurrence", Test_PhaseRecurrence},
        {"PI anti-windup", Test_PiAntiWindup},
        {"Tustin QPR", Test_Qpr},
        {"Tustin notch", Test_Notch},
        {"SVPWM and rectifier PWM", Test_Modulation},
        {"SVPWM Vdc normalization envelope", Test_SvpwmVdcNormalization},
        {"inverter cascaded loops", Test_InverterCascade},
        {"rectifier synchronized loop", Test_RectifierControl},
        {"protection latch", Test_Protection},
        {"power state machine", Test_PowerState},
        {"board configuration gates", Test_BoardConfigGates},
        {"board mode validation", Test_BoardModeValidation},
        {"inverter-only app flow", Test_InverterOnlyAppFlow},
        {"MODE2 fixed Vdc no envelope", Test_Mode2FixedVdcNoEnvelope},
        {"rectifier and full mode flow", Test_RectifierAndFullModeFlow},
        {"public API and Vdc setting", Test_PublicApiAndVdcSetting},
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
