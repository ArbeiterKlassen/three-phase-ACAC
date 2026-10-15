#include "board_runtime.h"

#include "adc.h"
#include "app_control.h"
#include "key_board.h"
#include "main.h"
#include "oled.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define BOARD_PWM_PERIOD_COUNTS       4200U
#define BOARD_KEY_SCAN_PERIOD_MS      10U
#define BOARD_DISPLAY_PERIOD_MS       100U
#define BOARD_LONG_PRESS_MS           2000U
#define BOARD_KEY_DEBOUNCE_SAMPLES    3U
#define BOARD_FAULT_PAGE_COUNT        5U
#define BOARD_ADC_TRIGGER_INV_COMPARE 3447U
#define BOARD_ADC_TRIGGER_RECT_COMPARE 4087U

#define BOARD_INV_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | \
     TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define BOARD_RECT_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE)

typedef struct
{
    app_mode_t mode;
    inverter_run_mode_t inverter_run_mode;
    app_state_t state;
    fault_flags_t faults;
    float output_frequency_hz;
    float output_line_voltage_reference_v;
    float dc_voltage_reference_v;
    float dc_voltage_v;
    float iin_a;
    float ia_a;
    float ib_a;
    float vab_v;
    float vbc_v;
    uint16_t iin_raw;
    bool pwm_allowed;
    bool rectifier_dc_ready;
} BoardDisplayStatus;

static volatile uint32_t s_adc_dma[ADC_DMA_WORD_COUNT];
static uint32_t s_sample_sequence;
static volatile uint32_t s_max_fast_cycles;
static AppPwmCommand s_delayed_pwm;
static volatile bool s_delayed_pwm_valid;
static volatile bool s_trip_latched;

static uint8_t s_key_candidate;
static uint8_t s_key_stable;
static uint8_t s_key_count;
static bool s_key_long_handled;
static uint32_t s_key_pressed_at;
static uint32_t s_last_key_scan;
static uint32_t s_last_display;
static uint8_t s_fault_display_page;
static AppFaultSnapshot s_fault_display_snapshot;

static uint32_t BoardRuntime_DutyToCompare(float duty)
{
    if (duty <= 0.0f)
    {
        return 0U;
    }
    if (duty >= 1.0f)
    {
        return BOARD_PWM_PERIOD_COUNTS - 1U;
    }
    return (uint32_t)(duty * (float)(BOARD_PWM_PERIOD_COUNTS - 1U) + 0.5f);
}

static void BoardRuntime_SelectAdcTrigger(app_mode_t mode)
{
    /*
     * ADC=21 MHz and TIM8=168 MHz.  In rectifier-only mode, rank 1 is
     * PA5/PC5, so trigger 14 ADC clocks (112 TIM counts) before the
     * carrier peak.  In inverter/full modes, retain the existing rank 3
     * alignment: (2 * (28 + 12) + 14) ADC clocks = 752 TIM counts.
     */
    TIM8->CCR4 = (mode == MODE2_RECTIFIER)
        ? BOARD_ADC_TRIGGER_RECT_COMPARE
        : BOARD_ADC_TRIGGER_INV_COMPARE;
}

static uint32_t BoardRuntime_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

static void BoardRuntime_ExitCritical(uint32_t primask)
{
    __DMB();
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static bool BoardRuntime_BreakInputActive(void)
{
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET)
        || (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);
}

static bool BoardRuntime_BreakFlagSet(void)
{
    return ((TIM1->SR & TIM_SR_BIF) != 0U)
        || ((TIM8->SR & TIM_SR_BIF) != 0U);
}

static bool BoardRuntime_TripPending(void)
{
    return s_trip_latched
        || BoardRuntime_BreakInputActive()
        || BoardRuntime_BreakFlagSet();
}

static void BoardRuntime_DisableInverter(void)
{
    const uint32_t neutral = BOARD_PWM_PERIOD_COUNTS / 2U;

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    TIM1->CCER &= ~BOARD_INV_CCER_MASK;
    TIM1->CCR1 = neutral;
    TIM1->CCR2 = neutral;
    TIM1->CCR3 = neutral;
}

static void BoardRuntime_DisableRectifier(void)
{
    const uint32_t neutral = BOARD_PWM_PERIOD_COUNTS / 2U;

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    TIM8->CCER &= ~BOARD_RECT_CCER_MASK;
    TIM8->CCR1 = neutral;
    TIM8->CCR2 = neutral;
}

static void BoardRuntime_DisableAllPwm(void)
{
    BoardRuntime_DisableInverter();
    BoardRuntime_DisableRectifier();
}

static void BoardRuntime_LatchTrip(fault_flags_t fault)
{
    s_trip_latched = true;
    __DMB();
    BoardRuntime_DisableAllPwm();
    s_delayed_pwm_valid = false;
    App_TripFromISR(fault);
}

static bool BoardRuntime_ApplyDelayedPwm(const AppPwmCommand *command)
{
    uint32_t primask;
    bool trip_detected;

    if (command == NULL)
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return false;
    }

    /*
     * BKIN硬件在关中断期间仍可异步清MOE。这里把最终故障检查、CCR/CCER
     * 更新和MOE置位放进极短临界区，并在置位后再次检查BIF，避免Break
     * 回调返回后DMA ISR用旧命令重新开启功率输出。
     */
    primask = BoardRuntime_EnterCritical();
    trip_detected = BoardRuntime_TripPending();
    if (!trip_detected && command->inverter_enable_request)
    {
        TIM1->CCR1 = BoardRuntime_DutyToCompare(command->inverter_a);
        TIM1->CCR2 = BoardRuntime_DutyToCompare(command->inverter_b);
        TIM1->CCR3 = BoardRuntime_DutyToCompare(command->inverter_c);
        TIM1->CCER |= BOARD_INV_CCER_MASK;
        __HAL_TIM_MOE_ENABLE(&htim1);
    }
    else
    {
        BoardRuntime_DisableInverter();
    }

    trip_detected = trip_detected || BoardRuntime_TripPending();
    if (!trip_detected && command->rectifier_enable_request)
    {
        /*
         * Rectifier power-stage routing:
         *   A_HI/A_LI = TIM8 CH2/CH2N (PC7/PB0), normal polarity.
         *   B_HI/B_LI = TIM8 CH1N/CH1 (PA7/PC6), physically swapped.
         * TIM8 CH1 uses PWM2 so CH1N has the same carrier phase as a normal
         * PWM1 high-side output.  Both compare registers therefore receive
         * their logical high-side duties directly.
         */
        TIM8->CCR2 = BoardRuntime_DutyToCompare(command->rectifier_a);
        TIM8->CCR1 = BoardRuntime_DutyToCompare(command->rectifier_b);
        TIM8->CCER |= BOARD_RECT_CCER_MASK;
        __HAL_TIM_MOE_ENABLE(&htim8);
    }
    else
    {
        /*
         * TIM8 继续计数并由 CH4 产生 ADC1 触发；功率通道保持关闭，
         * CH1/CH2 比较值回到 50% 中性点。
         */
        BoardRuntime_DisableRectifier();
    }

    if (BoardRuntime_TripPending())
    {
        s_trip_latched = true;
        BoardRuntime_DisableAllPwm();
        trip_detected = true;
    }
    BoardRuntime_ExitCritical(primask);

    if (trip_detected)
    {
        App_TripFromISR(PROTECTION_FAULT_EXTERNAL);
    }
    return !trip_detected;
}

static void BoardRuntime_ProcessAdcFrame(const uint32_t pairs[ADC_DUAL_RANK_COUNT])
{
    const AppPwmCommand *command;
    const AppStatus *status;
    uint32_t started;
    uint32_t elapsed;

    started = DWT->CYCCNT;
    s_sample_sequence++;

    if (!App_ApplyAdcPairs(pairs, s_sample_sequence))
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return;
    }

    App_FastStep20kHz();
    status = App_GetStatus();
    command = App_GetPwmCommand();

    if ((status == NULL) || (command == NULL)
        || !status->board_config_valid || !status->pwm_allowed)
    {
        BoardRuntime_DisableAllPwm();
        s_delayed_pwm_valid = false;
    }
    else
    {
        /*
         * 当前采样计算出的占空比留到下一帧装载，与仿真中的一周期
         * 计算延迟一致；第一次进入 RUN 仍保持逆变 MOE 关闭。
         */
        if (s_delayed_pwm_valid)
        {
            if (!BoardRuntime_ApplyDelayedPwm(&s_delayed_pwm))
            {
                s_delayed_pwm_valid = false;
                return;
            }
        }
        else
        {
            BoardRuntime_DisableAllPwm();
        }
        if (!s_trip_latched)
        {
            s_delayed_pwm = *command;
            s_delayed_pwm_valid = true;
        }
    }

    elapsed = DWT->CYCCNT - started;
    if (elapsed > s_max_fast_cycles)
    {
        s_max_fast_cycles = elapsed;
    }
    if (elapsed > (SystemCoreClock / 20000U))
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_LOOP_OVERRUN);
    }
}

static void BoardRuntime_CopyAndProcessAdcFrame(uint32_t offset)
{
    uint32_t pairs[ADC_DUAL_RANK_COUNT];
    uint32_t rank;

    for (rank = 0U; rank < ADC_DUAL_RANK_COUNT; rank++)
    {
        pairs[rank] = s_adc_dma[offset + rank];
    }
    BoardRuntime_ProcessAdcFrame(pairs);
}

static bool BoardRuntime_TryResetFault(void)
{
    const AppStatus *status = App_GetStatus();
    uint32_t primask;
    bool reset = false;

    if ((status == NULL) || (status->state != FAULT_LATCH))
    {
        return false;
    }

    primask = BoardRuntime_EnterCritical();
    BoardRuntime_DisableAllPwm();
    s_delayed_pwm_valid = false;

    /*
     * 仅在两路FAULT/BKIN均已释放时清除锁存。清BIF后再次检查输入和标志，
     * 若复位窗口内出现新故障，硬件仍会保持MOE关闭并在退出临界区后进ISR。
     */
    if (!BoardRuntime_BreakInputActive())
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
        HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
        HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
        __DMB();
        if (!BoardRuntime_BreakInputActive() && !BoardRuntime_BreakFlagSet())
        {
            s_trip_latched = false;
            App_ResetFault();
            s_fault_display_page = 0U;
            s_fault_display_snapshot.valid = false;
            reset = true;
        }
    }
    BoardRuntime_ExitCritical(primask);
    return reset;
}

static void BoardRuntime_SelectMode(app_mode_t mode)
{
    const AppStatus *status = App_GetStatus();
    uint32_t primask;
    bool request_allowed = false;

    if (status == NULL)
    {
        return;
    }

    primask = BoardRuntime_EnterCritical();
    if (((status->state == OFF) || (status->state == FAULT_LATCH))
        && !status->pwm_allowed)
    {
        /*
         * OFF and a latched FAULT both have PWM disabled. Keep both stages
         * quiescent while selecting a mode and discard the previous-frame
         * PWM command. Mode selection never clears the latched fault.
         */
        BoardRuntime_DisableAllPwm();
        s_delayed_pwm_valid = false;
        request_allowed = true;
    }
    BoardRuntime_ExitCritical(primask);

    if (request_allowed)
    {
        if (App_RequestMode(mode))
        {
            primask = BoardRuntime_EnterCritical();
            BoardRuntime_SelectAdcTrigger(mode);
            BoardRuntime_ExitCritical(primask);
        }
    }
}

static void BoardRuntime_HandleShortKey(uint8_t key)
{
    const AppStatus *status = App_GetStatus();

    switch (key)
    {
    case 14U: /* 0: full AC-AC chain */
        BoardRuntime_SelectMode(MODE0_FULL);
        break;
    case 1U: /* 1: inverter only */
        BoardRuntime_SelectMode(MODE1_INVERTER);
        break;
    case 2U: /* 2: rectifier only */
        BoardRuntime_SelectMode(MODE2_RECTIFIER);
        break;
    case 4U: /* A */
        if (status != NULL)
        {
            (void)App_SetOutputFrequency(
                (status->output_frequency_hz < 45.0f) ? 60U : 30U);
        }
        break;
    case 8U: /* B: 输出线电压 -0.1 V（赛题 32V±0.1V 现场微调） */
        if (status != NULL)
        {
            (void)App_SetOutputLineVoltageReference(
                status->output_line_voltage_reference_v - 0.1f);
        }
        break;
    case 12U: /* C: 输出线电压 +0.1 V */
        if (status != NULL)
        {
            (void)App_SetOutputLineVoltageReference(
                status->output_line_voltage_reference_v + 0.1f);
        }
        break;
    case 16U: /* D */
        App_RequestStop();
        BoardRuntime_DisableAllPwm();
        s_delayed_pwm_valid = false;
        if ((status != NULL) && (status->state == FAULT_LATCH)
            && status->fault_snapshot.valid
            && ((status->fault_snapshot.faults
                 & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL))
        {
            s_fault_display_page =
                (uint8_t)((s_fault_display_page + 1U)
                          % BOARD_FAULT_PAGE_COUNT);
        }
        break;
    default:
        break;
    }
}

static void BoardRuntime_ScanKeys(uint32_t now)
{
    uint8_t raw = (uint8_t)Read_Key();

    if (raw != s_key_candidate)
    {
        s_key_candidate = raw;
        s_key_count = 1U;
    }
    else if (s_key_count < BOARD_KEY_DEBOUNCE_SAMPLES)
    {
        s_key_count++;
    }

    if ((s_key_count >= BOARD_KEY_DEBOUNCE_SAMPLES)
        && (s_key_stable != s_key_candidate))
    {
        s_key_stable = s_key_candidate;
        s_key_pressed_at = now;
        s_key_long_handled = false;
        if (s_key_stable != 0U)
        {
            BoardRuntime_HandleShortKey(s_key_stable);
        }
    }

    if (!s_key_long_handled
        && ((s_key_stable == 15U) || (s_key_stable == 13U))
        && ((now - s_key_pressed_at) >= BOARD_LONG_PRESS_MS))
    {
        if (s_key_stable == 15U) /* # */
        {
            App_RequestStart();
        }
        else /* * */
        {
            (void)BoardRuntime_TryResetFault();
        }
        s_key_long_handled = true;
    }
}

static const char *BoardRuntime_StateText(app_state_t state)
{
    static const char *const names[] =
    {
        "OFF", "SELF_TEST", "PRECHARGE", "READY",
        "RUN", "FAULT", "DISCHARGE"
    };

    return ((uint32_t)state < 7U) ? names[(uint32_t)state] : "UNKNOWN";
}

static const char *BoardRuntime_ModeText(app_mode_t mode)
{
    if (mode == MODE0_FULL)
    {
        return "FULL";
    }
    if (mode == MODE1_INVERTER)
    {
        return "INV";
    }
    if (mode == MODE2_RECTIFIER)
    {
        return "RECT";
    }
    return "MODE?";
}

static const char *BoardRuntime_ControlModeText(inverter_run_mode_t mode)
{
    if (mode == INVERTER_CLOSED_LOOP)
    {
        return "CTRL:CL";
    }
    if (mode == INVERTER_OPEN_LOOP)
    {
        return "CTRL:OL";
    }
    return "CTRL:??";
}

static uint32_t BoardRuntime_FirstFault(uint32_t faults)
{
    uint32_t bit = 0U;

    if (faults == 0U)
    {
        return 0U;
    }
    while (((faults >> bit) & 1U) == 0U)
    {
        bit++;
    }
    return bit + 1U;
}

static const char *BoardRuntime_FaultText(uint32_t fault_code)
{
    static const char *const names[] =
    {
        "NONE",
        "ADC INVALID",
        "SAMPLE STALE",
        "INPUT OC",
        "PHASE OC",
        "INPUT OV",
        "DC OV",
        "DC UV",
        "OUTPUT OV",
        "EXTERNAL/BKIN",
        "LOOP OVERRUN",
        "CONTROL",
        "DC SET MISMATCH"
    };

    return (fault_code < (sizeof(names) / sizeof(names[0])))
        ? names[fault_code] : "UNKNOWN";
}

static uint32_t BoardRuntime_ScaledMagnitude(float value,
                                             float scale,
                                             uint32_t maximum)
{
    uint32_t scaled;

    if (!isfinite(value))
    {
        return 0U;
    }
    value = fabsf(value);
    scaled = (uint32_t)(value * scale + 0.5f);
    return (scaled > maximum) ? maximum : scaled;
}

static void BoardRuntime_ShowSignedHundredths(uint8_t x,
                                              uint8_t y,
                                              float value)
{
    uint32_t scaled =
        BoardRuntime_ScaledMagnitude(value, 100.0f, 999U);

    OLED_ShowChar(x, y, (value < 0.0f) ? '-' : '+', 8, 1);
    OLED_ShowNum(x + 6U, y, scaled / 100U, 1, 8, 1);
    OLED_ShowChar(x + 12U, y, '.', 8, 1);
    OLED_ShowNum(x + 18U, y, scaled % 100U, 2, 8, 1);
}

static void BoardRuntime_ShowSignedTenths(uint8_t x,
                                         uint8_t y,
                                         float value)
{
    uint32_t scaled =
        BoardRuntime_ScaledMagnitude(value, 10.0f, 999U);

    OLED_ShowChar(x, y, (value < 0.0f) ? '-' : '+', 8, 1);
    OLED_ShowNum(x + 6U, y, scaled / 10U, 2, 8, 1);
    OLED_ShowChar(x + 18U, y, '.', 8, 1);
    OLED_ShowNum(x + 24U, y, scaled % 10U, 1, 8, 1);
}

static void BoardRuntime_ShowUnsignedTenths(uint8_t x,
                                           uint8_t y,
                                           float value)
{
    uint32_t scaled;

    if (!isfinite(value) || (value < 0.0f))
    {
        value = 0.0f;
    }
    scaled = (uint32_t)(value * 10.0f + 0.5f);
    if (scaled > 999U)
    {
        scaled = 999U;
    }
    OLED_ShowNum(x, y, scaled / 10U, 2, 8, 1);
    OLED_ShowChar(x + 12U, y, '.', 8, 1);
    OLED_ShowNum(x + 18U, y, scaled % 10U, 1, 8, 1);
}

static void BoardRuntime_ShowUnsignedThousandths(uint8_t x,
                                                 uint8_t y,
                                                 float value)
{
    uint32_t scaled;

    if (!isfinite(value) || (value < 0.0f))
    {
        value = 0.0f;
    }
    scaled = (uint32_t)(value * 1000.0f + 0.5f);
    if (scaled > 9999U)
    {
        scaled = 9999U;
    }
    OLED_ShowNum(x, y, scaled / 1000U, 1, 8, 1);
    OLED_ShowChar(x + 6U, y, '.', 8, 1);
    OLED_ShowNum(x + 12U, y, scaled % 1000U, 3, 8, 1);
}

static void BoardRuntime_ShowUnsignedInteger3(uint8_t x,
                                              uint8_t y,
                                              float value)
{
    uint32_t scaled =
        BoardRuntime_ScaledMagnitude(value, 1.0f, 999U);

    OLED_ShowNum(x, y, scaled, 3, 8, 1);
}

static float BoardRuntime_CalibrateRaw(
    const ChannelCalibration *calibration,
    uint16_t raw)
{
    if ((calibration == NULL)
        || !isfinite(calibration->offset_count)
        || !isfinite(calibration->gain))
    {
        return 0.0f;
    }
    return ((float)raw - calibration->offset_count)
         * calibration->gain;
}

static const AppFaultRawSample *BoardRuntime_HistoryAtLag(
    const AppFaultSnapshot *snapshot,
    uint8_t lag)
{
    if ((snapshot == NULL) || (lag == 0U)
        || (lag > snapshot->fast_history_count))
    {
        return NULL;
    }
    return &snapshot->fast_history[
        snapshot->fast_history_count - lag];
}

static float BoardRuntime_HistoryPhaseMax(
    const AppFaultRawSample *sample,
    const MeasurementCalibration *calibration)
{
    float vab;
    float vbc;
    float va;
    float vb;
    float vc;
    float maximum;

    if ((sample == NULL) || (calibration == NULL))
    {
        return 0.0f;
    }
    vab = BoardRuntime_CalibrateRaw(&calibration->vab,
                                    sample->pc2_raw);
    vbc = BoardRuntime_CalibrateRaw(&calibration->vbc,
                                    sample->pc3_raw);
    va = (2.0f * vab + vbc) / 3.0f;
    vb = (vbc - vab) / 3.0f;
    vc = -(vab + 2.0f * vbc) / 3.0f;
    maximum = fabsf(va);
    if (fabsf(vb) > maximum)
    {
        maximum = fabsf(vb);
    }
    return (fabsf(vc) > maximum) ? fabsf(vc) : maximum;
}

static float BoardRuntime_VectorMagnitude(AlphaBeta vector)
{
    float square = vector.alpha * vector.alpha
                 + vector.beta * vector.beta;

    return (isfinite(square) && (square > 0.0f))
        ? sqrtf(square) : 0.0f;
}

static void BoardRuntime_ShowFaultFooter(void)
{
    OLED_ShowString(0, 56, (u8 *)"D:NEXT *2S:RST", 8, 1);
}

static void BoardRuntime_ShowInputOcSnapshot(
    const AppFaultSnapshot *snapshot)
{
    const BoardPowerConfig *cfg = BoardPower_GetConfig();

    OLED_ShowString(0, 0, (u8 *)"FAULT INPUT OC", 8, 1);
    OLED_ShowString(0, 8, (u8 *)"IIN:", 8, 1);
    BoardRuntime_ShowSignedHundredths(24, 8, snapshot->iin_a);
    OLED_ShowString(72, 8, (u8 *)"R:", 8, 1);
    OLED_ShowNum(84, 8, snapshot->iin_raw, 4, 8, 1);

    OLED_ShowString(0, 16, (u8 *)"S:", 8, 1);
    OLED_ShowNum(12, 16,
                 snapshot->sample_sequence % 1000000UL,
                 6, 8, 1);

    OLED_ShowString(0, 24, (u8 *)"PRE:", 8, 1);
    if (snapshot->previous_iin_valid)
    {
        BoardRuntime_ShowSignedHundredths(
            24, 24, snapshot->previous_iin_a);
        OLED_ShowString(72, 24, (u8 *)"R:", 8, 1);
        OLED_ShowNum(84, 24, snapshot->previous_iin_raw,
                     4, 8, 1);
        OLED_ShowString(0, 32, (u8 *)"PS:", 8, 1);
        OLED_ShowNum(18, 32,
                     snapshot->previous_iin_sequence % 1000000UL,
                     6, 8, 1);
    }
    else
    {
        OLED_ShowString(24, 24, (u8 *)"N/A", 8, 1);
    }

    OLED_ShowString(0, 40, (u8 *)"VDC:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(24, 40, snapshot->vdc_v);
    OLED_ShowString(66, 40, (u8 *)"LIM:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(
        90, 40,
        (cfg != NULL) ? cfg->protection.input_current_abs_max_a : 0.0f);
    OLED_ShowString(0, 48, (u8 *)"PA5=ADC1 RANK1", 8, 1);
    OLED_ShowString(0, 56, (u8 *)"*HOLD2S RESET", 8, 1);
}

static void BoardRuntime_ShowInputOvSnapshot(
    const AppFaultSnapshot *snapshot)
{
    const BoardPowerConfig *cfg = BoardPower_GetConfig();

    OLED_ShowString(0, 0, (u8 *)"FAULT INPUT OV", 8, 1);
    OLED_ShowString(0, 8, (u8 *)"VIN:", 8, 1);
    BoardRuntime_ShowSignedTenths(24, 8, snapshot->vin_v);
    OLED_ShowString(72, 8, (u8 *)"R:", 8, 1);
    OLED_ShowNum(84, 8, snapshot->vin_raw, 4, 8, 1);

    OLED_ShowString(0, 16, (u8 *)"S:", 8, 1);
    OLED_ShowNum(12, 16,
                 snapshot->sample_sequence % 1000000UL,
                 6, 8, 1);

    OLED_ShowString(0, 24, (u8 *)"PRE:", 8, 1);
    if (snapshot->previous_vin_valid)
    {
        BoardRuntime_ShowSignedTenths(
            24, 24, snapshot->previous_vin_v);
        OLED_ShowString(72, 24, (u8 *)"R:", 8, 1);
        OLED_ShowNum(84, 24, snapshot->previous_vin_raw,
                     4, 8, 1);
        OLED_ShowString(0, 32, (u8 *)"PS:", 8, 1);
        OLED_ShowNum(18, 32,
                     snapshot->previous_vin_sequence % 1000000UL,
                     6, 8, 1);
    }
    else
    {
        OLED_ShowString(24, 24, (u8 *)"N/A", 8, 1);
    }

    OLED_ShowString(0, 40, (u8 *)"VDC:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(24, 40, snapshot->vdc_v);
    OLED_ShowString(66, 40, (u8 *)"LIM:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(
        90, 40,
        (cfg != NULL) ? cfg->protection.input_voltage_abs_max_v : 0.0f);
    OLED_ShowString(0, 48, (u8 *)"PC5=ADC2 RANK1", 8, 1);
    OLED_ShowString(0, 56, (u8 *)"*HOLD2S RESET", 8, 1);
}

static float BoardRuntime_OutputOvTripValue(
    const AppFaultSnapshot *snapshot,
    u8 *phase)
{
    float value = snapshot->va_v;

    *phase = 'A';
    if (fabsf(snapshot->vb_v) > fabsf(value))
    {
        value = snapshot->vb_v;
        *phase = 'B';
    }
    if (fabsf(snapshot->vc_v) > fabsf(value))
    {
        value = snapshot->vc_v;
        *phase = 'C';
    }
    return value;
}

static void BoardRuntime_ShowOutputOvSnapshot(
    const AppFaultSnapshot *snapshot)
{
    u8 phase;
    float trip_value =
        BoardRuntime_OutputOvTripValue(snapshot, &phase);

    OLED_ShowString(0, 0, (u8 *)"FAULT OUTPUT OV", 8, 1);
    OLED_ShowString(
        0, 8,
        (u8 *)BoardRuntime_ControlModeText(
            snapshot->inverter_run_mode),
        8, 1);
    OLED_ShowString(66, 8, (u8 *)"SET:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(
        90, 8, snapshot->dc_voltage_reference_v);

    OLED_ShowString(0, 16, (u8 *)"TRIP:", 8, 1);
    OLED_ShowChar(30, 16, phase, 8, 1);
    BoardRuntime_ShowSignedTenths(42, 16, trip_value);
    OLED_ShowString(78, 16, (u8 *)"S:", 8, 1);
    OLED_ShowNum(90, 16,
                 snapshot->sample_sequence % 1000000UL,
                 6, 8, 1);

    OLED_ShowString(0, 24, (u8 *)"VA:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 24, snapshot->va_v);
    OLED_ShowString(66, 24, (u8 *)"VB:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 24, snapshot->vb_v);

    OLED_ShowString(0, 32, (u8 *)"VC:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 32, snapshot->vc_v);
    OLED_ShowString(66, 32, (u8 *)"PWM:OFF", 8, 1);

    OLED_ShowString(0, 40, (u8 *)"AB:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 40, snapshot->vab_v);
    OLED_ShowString(66, 40, (u8 *)"BC:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 40, snapshot->vbc_v);

    OLED_ShowString(0, 48, (u8 *)"R2:", 8, 1);
    OLED_ShowNum(18, 48, snapshot->pc2_raw, 4, 8, 1);
    OLED_ShowString(66, 48, (u8 *)"R3:", 8, 1);
    OLED_ShowNum(84, 48, snapshot->pc3_raw, 4, 8, 1);

    OLED_ShowString(0, 56, (u8 *)"M:", 8, 1);
    OLED_ShowNum(12, 56, snapshot->faults, 4, 8, 1);
    OLED_ShowString(42, 56, (u8 *)"D:NEXT *2S:RST", 8, 1);
}

static void BoardRuntime_ShowOutputOvVoltageHistory(
    const AppFaultSnapshot *snapshot)
{
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U};
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const AppFaultRawSample *sample;
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE N VMAX R2 R3", 8, 1);
    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++)
    {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowNum(0, y, lags[row], 2, 8, 1);
        sample = BoardRuntime_HistoryAtLag(snapshot, lags[row]);
        if ((sample == NULL) || (cfg == NULL))
        {
            OLED_ShowString(18, y, (u8 *)"N/A", 8, 1);
            continue;
        }
        BoardRuntime_ShowUnsignedTenths(
            18, y,
            BoardRuntime_HistoryPhaseMax(sample, &cfg->measurement));
        OLED_ShowNum(60, y, sample->pc2_raw, 4, 8, 1);
        OLED_ShowNum(96, y, sample->pc3_raw, 4, 8, 1);
    }
    BoardRuntime_ShowFaultFooter();
}

static void BoardRuntime_ShowOutputOvCurrentHistory(
    const AppFaultSnapshot *snapshot)
{
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U};
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const AppFaultRawSample *sample;
    float ia;
    float ib;
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE N   IA      IB", 8, 1);
    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++)
    {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowNum(0, y, lags[row], 2, 8, 1);
        sample = BoardRuntime_HistoryAtLag(snapshot, lags[row]);
        if ((sample == NULL) || (cfg == NULL))
        {
            OLED_ShowString(18, y, (u8 *)"N/A", 8, 1);
            continue;
        }
        /* PC0 is Ib and PC1 is Ia on the confirmed hardware mapping. */
        ia = BoardRuntime_CalibrateRaw(&cfg->measurement.ia,
                                       sample->pc1_raw);
        ib = BoardRuntime_CalibrateRaw(&cfg->measurement.ib,
                                       sample->pc0_raw);
        BoardRuntime_ShowSignedHundredths(18, y, ia);
        BoardRuntime_ShowSignedHundredths(72, y, ib);
    }
    OLED_ShowString(0, 48, (u8 *)"R0:", 8, 1);
    OLED_ShowNum(18, 48, snapshot->trigger_raw.pc0_raw, 4, 8, 1);
    OLED_ShowString(66, 48, (u8 *)"R1:", 8, 1);
    OLED_ShowNum(84, 48, snapshot->trigger_raw.pc1_raw, 4, 8, 1);
    BoardRuntime_ShowFaultFooter();
}

static void BoardRuntime_ShowOutputOvControl(
    const AppFaultSnapshot *snapshot)
{
    const AppFaultControlSample *control =
        &snapshot->previous_control;
    AlphaBeta error;
    float reference;
    float measured;
    float voltage_error;
    float current_reference;
    float current_measured;
    float voltage_command;

    OLED_ShowString(0, 0, (u8 *)"CTRL PREV S:", 8, 1);
    OLED_ShowNum(72, 0,
                 control->sample_sequence % 1000000UL,
                 6, 8, 1);
    if (!control->valid)
    {
        OLED_ShowString(0, 16, (u8 *)"NO CONTROL DATA", 8, 1);
        BoardRuntime_ShowFaultFooter();
        return;
    }

    error.alpha = control->voltage_reference.alpha
                - control->voltage_measured.alpha;
    error.beta = control->voltage_reference.beta
               - control->voltage_measured.beta;
    reference =
        BoardRuntime_VectorMagnitude(control->voltage_reference);
    measured =
        BoardRuntime_VectorMagnitude(control->voltage_measured);
    voltage_error = BoardRuntime_VectorMagnitude(error);
    current_reference =
        BoardRuntime_VectorMagnitude(control->current_reference);
    current_measured =
        BoardRuntime_VectorMagnitude(control->current_measured);
    voltage_command =
        BoardRuntime_VectorMagnitude(control->voltage_command);

    OLED_ShowString(0, 8, (u8 *)"VR:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(18, 8, reference);
    OLED_ShowString(66, 8, (u8 *)"VM:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(84, 8, measured);

    OLED_ShowString(0, 16, (u8 *)"VE:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(18, 16, voltage_error);
    OLED_ShowString(66, 16, (u8 *)"IR:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        84, 16, current_reference);

    OLED_ShowString(0, 24, (u8 *)"IM:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        18, 24, current_measured);
    OLED_ShowString(66, 24, (u8 *)"VC:", 8, 1);
    BoardRuntime_ShowUnsignedInteger3(84, 24, voltage_command);

    OLED_ShowString(0, 32, (u8 *)"DA:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        18, 32, control->duty_a);
    OLED_ShowString(66, 32, (u8 *)"DB:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        84, 32, control->duty_b);

    OLED_ShowString(0, 40, (u8 *)"DC:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        18, 40, control->duty_c);
    OLED_ShowString(66, 40, (u8 *)"SC:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(
        84, 40, control->applied_scale);

    OLED_ShowString(0, 48, (u8 *)"SAT:", 8, 1);
    OLED_ShowString(
        24, 48,
        (u8 *)(control->saturated ? "YES" : "NO"),
        8, 1);
    BoardRuntime_ShowFaultFooter();
}

static void BoardRuntime_ShowOutputOvCycleTrend(
    const AppFaultSnapshot *snapshot)
{
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U, 32U};
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(
        24, 0, snapshot->partial_cycle_peak_v);
    OLED_ShowString(60, 0, (u8 *)"N:", 8, 1);
    OLED_ShowNum(72, 0, snapshot->partial_cycle_samples,
                 4, 8, 1);

    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++)
    {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowString(0, y, (u8 *)"C-", 8, 1);
        OLED_ShowNum(12, y, lags[row], 2, 8, 1);
        OLED_ShowChar(24, y, ':', 8, 1);
        if (lags[row] <= snapshot->cycle_peak_count)
        {
            BoardRuntime_ShowUnsignedTenths(
                30, y, snapshot->cycle_peak_v[lags[row] - 1U]);
        }
        else
        {
            OLED_ShowString(30, y, (u8 *)"N/A", 8, 1);
        }
    }
    BoardRuntime_ShowFaultFooter();
}

static void BoardRuntime_ShowOutputOvPage(
    const AppFaultSnapshot *snapshot)
{
    switch (s_fault_display_page)
    {
    case 1U:
        BoardRuntime_ShowOutputOvVoltageHistory(snapshot);
        break;
    case 2U:
        BoardRuntime_ShowOutputOvCurrentHistory(snapshot);
        break;
    case 3U:
        BoardRuntime_ShowOutputOvControl(snapshot);
        break;
    case 4U:
        BoardRuntime_ShowOutputOvCycleTrend(snapshot);
        break;
    default:
        BoardRuntime_ShowOutputOvSnapshot(snapshot);
        break;
    }
}

static void BoardRuntime_RefreshDisplay(void)
{
    BoardDisplayStatus status;
    const AppStatus *shared = App_GetStatus();
    uint32_t primask;
    uint32_t fault_code;
    bool diagnostic_snapshot_available;

    if (shared == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    status.mode = shared->mode;
    status.inverter_run_mode = shared->inverter_run_mode;
    status.state = shared->state;
    status.faults = shared->faults;
    status.output_frequency_hz = shared->output_frequency_hz;
    status.output_line_voltage_reference_v =
        shared->output_line_voltage_reference_v;
    status.dc_voltage_reference_v = shared->dc_voltage_reference_v;
    status.dc_voltage_v = shared->dc_voltage_v;
    status.iin_a = shared->iin_a;
    status.ia_a = shared->ia_a;
    status.ib_a = shared->ib_a;
    status.vab_v = shared->vab_v;
    status.vbc_v = shared->vbc_v;
    status.iin_raw = shared->iin_raw;
    status.pwm_allowed = shared->pwm_allowed;
    status.rectifier_dc_ready = shared->rectifier_dc_ready;
    diagnostic_snapshot_available =
        shared->fault_snapshot.valid
        && ((shared->fault_snapshot.faults
             & (PROTECTION_FAULT_INPUT_OVERCURRENT
                | PROTECTION_FAULT_INPUT_OVERVOLTAGE
                | PROTECTION_FAULT_OUTPUT_OVERVOLTAGE)) != 0UL);
    if (diagnostic_snapshot_available
        && (!s_fault_display_snapshot.valid
            || (s_fault_display_snapshot.sample_sequence
                != shared->fault_snapshot.sample_sequence)))
    {
        s_fault_display_snapshot = shared->fault_snapshot;
        s_fault_display_page = 0U;
    }
    else if (!diagnostic_snapshot_available)
    {
        s_fault_display_snapshot.valid = false;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    fault_code = BoardRuntime_FirstFault(status.faults);

    OLED_Clear();
    if (s_fault_display_snapshot.valid)
    {
        if ((s_fault_display_snapshot.faults
             & PROTECTION_FAULT_INPUT_OVERCURRENT) != 0UL)
        {
            BoardRuntime_ShowInputOcSnapshot(
                &s_fault_display_snapshot);
        }
        else if ((s_fault_display_snapshot.faults
             & PROTECTION_FAULT_INPUT_OVERVOLTAGE) != 0UL)
        {
            BoardRuntime_ShowInputOvSnapshot(
                &s_fault_display_snapshot);
        }
        else
        {
            BoardRuntime_ShowOutputOvPage(&s_fault_display_snapshot);
        }
        OLED_Refresh();
        return;
    }
    s_fault_display_page = 0U;

    OLED_ShowString(0, 0,
                    (u8 *)BoardRuntime_ModeText(status.mode), 8, 1);
    OLED_ShowString(30, 0,
                    (u8 *)BoardRuntime_StateText(status.state), 8, 1);
    OLED_ShowNum(96, 0,
                 (uint32_t)(status.output_frequency_hz + 0.5f),
                 2, 8, 1);
    OLED_ShowString(108, 0, (u8 *)"Hz", 8, 1);

    if ((status.mode == MODE2_RECTIFIER) && (status.state == OFF))
    {
        OLED_ShowString(0, 8, (u8 *)"IIN:", 8, 1);
        BoardRuntime_ShowSignedHundredths(24, 8, status.iin_a);
        OLED_ShowString(72, 8, (u8 *)"R:", 8, 1);
        OLED_ShowNum(84, 8, status.iin_raw, 4, 8, 1);
    }
    else
    {
        OLED_ShowString(0, 8, (u8 *)"IA:", 8, 1);
        BoardRuntime_ShowSignedHundredths(18, 8, status.ia_a);
        OLED_ShowString(66, 8, (u8 *)"IB:", 8, 1);
        BoardRuntime_ShowSignedHundredths(84, 8, status.ib_a);
    }

    OLED_ShowString(0, 16, (u8 *)"AB:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 16, status.vab_v);
    OLED_ShowString(66, 16, (u8 *)"BC:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 16, status.vbc_v);

    OLED_ShowString(
        0, 24,
        (u8 *)BoardRuntime_ControlModeText(status.inverter_run_mode),
        8, 1);
    OLED_ShowString(66, 24, (u8 *)"SET:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(90, 24,
                                    status.dc_voltage_reference_v);

    if (status.mode == MODE2_RECTIFIER)
    {
        OLED_ShowString(0, 32, (u8 *)"VDC:", 8, 1);
        BoardRuntime_ShowUnsignedTenths(24, 32, status.dc_voltage_v);
    }
    else
    {
        OLED_ShowString(0, 32, (u8 *)"OUT:", 8, 1);
        BoardRuntime_ShowUnsignedTenths(
            24, 32, status.output_line_voltage_reference_v);
    }
    OLED_ShowString(66, 32, (u8 *)"FLT:", 8, 1);
    OLED_ShowNum(90, 32, fault_code, 2, 8, 1);

    if ((status.mode == MODE0_FULL)
        || (status.mode == MODE2_RECTIFIER))
    {
        OLED_ShowString(0, 40, (u8 *)"RECT:", 8, 1);
        OLED_ShowString(
            30, 40,
            (u8 *)(status.rectifier_dc_ready ? "RDY" : "WAIT"),
            8, 1);
    }
    else
    {
        OLED_ShowString(0, 40, (u8 *)"K:", 8, 1);
        BoardRuntime_ShowUnsignedThousandths(
            12, 40, APP_INVERTER_OUTPUT_LINE_RMS_RATIO);
    }
    OLED_ShowString(66, 40,
                    (u8 *)(status.pwm_allowed ? "PWM:ON" : "PWM:OFF"),
                    8, 1);
    if (fault_code != 0U)
    {
        OLED_ShowString(0, 48, (u8 *)"ERR:", 8, 1);
        OLED_ShowString(24, 48,
                        (u8 *)BoardRuntime_FaultText(fault_code), 8, 1);
        OLED_ShowString(0, 56, (u8 *)"*HOLD2S RESET", 8, 1);
    }
    else
    {
        OLED_ShowString(0, 48, (u8 *)"0:F 1:I 2:R A:Hz", 8, 1);
        OLED_ShowString(0, 56, (u8 *)"B:- C:+ #2S D:STOP", 8, 1);
    }
    OLED_Refresh();
}

void BoardRuntime_Init(void)
{
    const uint32_t neutral = BOARD_PWM_PERIOD_COUNTS / 2U;
    uint32_t primask;

    s_trip_latched = true;
    s_delayed_pwm_valid = false;
    s_fault_display_page = 0U;
    s_fault_display_snapshot.valid = false;
    BoardRuntime_DisableAllPwm();
    TIM1->CCR1 = neutral;
    TIM1->CCR2 = neutral;
    TIM1->CCR3 = neutral;
    TIM8->CCR1 = neutral;
    TIM8->CCR2 = neutral;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    App_Init();
    Key_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_Refresh();

    /*
     * TIM1初始化阶段不打开BIE，避免TIM8句柄尚未就绪时进入全局关断回调。
     * 到这里两路定时器和App均已初始化，再统一清标志并使能最高优先级Break。
     */
    primask = BoardRuntime_EnterCritical();
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
    s_trip_latched = BoardRuntime_BreakInputActive()
                  || BoardRuntime_BreakFlagSet();
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
    BoardRuntime_ExitCritical(primask);

    if (BoardRuntime_TripPending())
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_EXTERNAL);
    }

    /*
     * 双ADC中先使能从ADC2，再启动ADC1多模式DMA；TIM8先运行，
     * 随后由TIM1更新事件同步复位。门极输出通道仍保持禁用。
     */
    if ((HAL_ADC_Start(&hadc2) != HAL_OK)
        || (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)s_adc_dma,
                                         ADC_DMA_WORD_COUNT) != HAL_OK))
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return;
    }

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim8);
    __HAL_TIM_ENABLE(&htim1);
}

void BoardRuntime_BackgroundTask(void)
{
    uint32_t now = HAL_GetTick();

    App_BackgroundTask();
    if ((now - s_last_key_scan) >= BOARD_KEY_SCAN_PERIOD_MS)
    {
        s_last_key_scan = now;
        BoardRuntime_ScanKeys(now);
    }
    if ((now - s_last_display) >= BOARD_DISPLAY_PERIOD_MS)
    {
        s_last_display = now;
        BoardRuntime_RefreshDisplay();
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BoardRuntime_CopyAndProcessAdcFrame(0U);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BoardRuntime_CopyAndProcessAdcFrame(ADC_DUAL_RANK_COUNT);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC1) || (hadc->Instance == ADC2))
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
    }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) || (htim->Instance == TIM8))
    {
        /*
         * BKIN已由定时器硬件先清MOE；这里再全局封锁两级并锁存故障，
         * 禁止AOE或软件自动恢复。
         */
        BoardRuntime_LatchTrip(PROTECTION_FAULT_EXTERNAL);
    }
}
