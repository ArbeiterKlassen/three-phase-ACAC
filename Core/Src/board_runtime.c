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
#define BOARD_PWM_NEUTRAL_COUNTS      (BOARD_PWM_PERIOD_COUNTS / 2U)
#define BOARD_KEY_SCAN_PERIOD_MS      10U
#define BOARD_DISPLAY_PERIOD_MS       100U
#define BOARD_LONG_PRESS_MS           2000U
#define BOARD_KEY_DEBOUNCE_SAMPLES    3U
/*
 * ADC 帧看门狗：RUN 中 TIM8/ADC/DMA 中断链停摆时 DMA 半/全满回调不再
 * 到来，PWM 会保持最后 CCR 输出且 LOOP_OVERRUN 无法发现“帧不来”。
 * 后台按 HAL_GetTick() 毫秒时基检查最近一帧时间戳，首帧后看门狗
 * 才武装，超过 5 ms 无新帧即锁存测量故障并关断两级 PWM。
 */
#define BOARD_ADC_FRAME_WATCHDOG_MS   5U

#define BOARD_INV_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | \
     TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define BOARD_RECT_CCER_MASK \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE)

static volatile uint32_t s_adc_dma[ADC_DMA_WORD_COUNT];
static uint32_t s_sample_sequence;
static volatile uint32_t s_max_fast_cycles;
static AppPwmCommand s_delayed_pwm;
static volatile bool s_delayed_pwm_valid;
static volatile bool s_trip_latched;
static volatile uint32_t s_last_adc_frame_ms;
static volatile bool s_adc_watchdog_armed;

static uint8_t s_key_candidate;
static uint8_t s_key_stable;
static uint8_t s_key_count;
static bool s_key_long_handled;
static uint32_t s_key_pressed_at;
static uint32_t s_last_key_scan;
static uint32_t s_last_display;

static uint32_t BoardRuntime_DutyToCompare(float duty)
{
    /* NaN/Inf 不能直接转整数（UB），按 50% 中性占空比输出。 */
    if (!isfinite(duty))
    {
        return BOARD_PWM_NEUTRAL_COUNTS;
    }
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
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    TIM1->CCER &= ~BOARD_INV_CCER_MASK;
    /*
     * OC preload 下停机前的旧 CCR 仍锁存在影子寄存器里，MOE 重新使能后会
     * 原样输出一拍。关闭输出的同时把 CCR 写回 50% 中性值（上下桥臂同开
     * 同关、线电压为零），下次使能首拍即为安全中性电平。
     */
    TIM1->CCR1 = BOARD_PWM_NEUTRAL_COUNTS;
    TIM1->CCR2 = BOARD_PWM_NEUTRAL_COUNTS;
    TIM1->CCR3 = BOARD_PWM_NEUTRAL_COUNTS;
}

static void BoardRuntime_DisableRectifier(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    TIM8->CCER &= ~BOARD_RECT_CCER_MASK;
    /* 同逆变级：CCR 写回 50% 中性值；TIM8 保持计数，不影响 ADC 触发。 */
    TIM8->CCR1 = BOARD_PWM_NEUTRAL_COUNTS;
    TIM8->CCR2 = BOARD_PWM_NEUTRAL_COUNTS;
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

    /*
     * 整流 TIM8 映射严格为 rectifier_a→CCR2(桥臂A,PC7/PB0)、
     * rectifier_b→CCR1(桥臂B,PC6/PA7)。使能时先 CCER 后 MOE；
     * 禁用时只清 MOE/CCER，TIM8 保持计数以维持 ADC1 外部触发。
     */
    if (!trip_detected && command->rectifier_enable_request)
    {
        TIM8->CCR2 = BoardRuntime_DutyToCompare(command->rectifier_a);
        TIM8->CCR1 = BoardRuntime_DutyToCompare(command->rectifier_b);
        TIM8->CCER |= BOARD_RECT_CCER_MASK;
        __HAL_TIM_MOE_ENABLE(&htim8);
    }
    else
    {
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

    /* 每处理一帧（无论解包成败）都喂狗；首帧后看门狗武装。 */
    s_last_adc_frame_ms = HAL_GetTick();
    s_adc_watchdog_armed = true;

    if (!App_ApplyAdcPairs(pairs, s_sample_sequence))
    {
        /*
         * 解包失败不能只记录 isr_faults 就返回：App_FastStep20kHz 不再
         * 运行时故障到不了 FAULT_LATCH，OLED 无显示且 * 复位被拒绝。
         * 这里先锁存测量故障，再补一拍快速环推进保护/状态机/状态发布；
         * 因 frame_pending 未置位，App_FastStep20kHz 会强制 raw.valid
         * 为 false，不会用旧帧继续控制。
         */
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        App_FastStep20kHz();
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
            reset = true;
        }
    }
    BoardRuntime_ExitCritical(primask);
    return reset;
}

static void BoardRuntime_HandleShortKey(uint8_t key)
{
    const AppStatus *status = App_GetStatus();
    bool mode_changed = false;

    switch (key)
    {
    case 14U: /* 0 */
        mode_changed = App_RequestMode(MODE0_FULL);
        break;
    case 1U: /* 1 */
        mode_changed = App_RequestMode(MODE1_RECTIFIER);
        break;
    case 2U: /* 2 */
        mode_changed = App_RequestMode(MODE2_INVERTER);
        break;
    case 4U: /* A */
        if (status != NULL)
        {
            (void)App_SetOutputFrequency(
                (status->output_frequency_hz < 45.0f) ? 60U : 30U);
        }
        break;
    case 8U: /* B: Vdc - 1 V */
        if (status != NULL)
        {
            (void)App_SetDcVoltageReference(
                status->dc_voltage_reference_v - 1.0f);
        }
        break;
    case 12U: /* C: Vdc + 1 V */
        if (status != NULL)
        {
            (void)App_SetDcVoltageReference(
                status->dc_voltage_reference_v + 1.0f);
        }
        break;
    case 16U: /* D */
        App_RequestStop();
        BoardRuntime_DisableAllPwm();
        s_delayed_pwm_valid = false;
        break;
    default:
        break;
    }

    if (mode_changed)
    {
        /*
         * 模式只在 OFF 切换，但仍显式关闭两级 PWM 并作废延迟命令，
         * 避免旧模式的 CCR/使能请求被装到新模式对应的定时器。
         */
        BoardRuntime_DisableAllPwm();
        s_delayed_pwm_valid = false;
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
    switch (mode)
    {
    case MODE0_FULL:
        return "FULL";
    case MODE1_RECTIFIER:
        return "RECT";
    case MODE2_INVERTER:
        return "INV";
    default:
        return "???";
    }
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

static void BoardRuntime_ShowUnsignedHundredths(uint8_t x,
                                               uint8_t y,
                                               float value)
{
    uint32_t scaled;

    if (!isfinite(value) || (value < 0.0f))
    {
        value = 0.0f;
    }
    scaled = (uint32_t)(value * 100.0f + 0.5f);
    if (scaled > 999U)
    {
        scaled = 999U;
    }
    OLED_ShowNum(x, y, scaled / 100U, 1, 8, 1);
    OLED_ShowChar(x + 6U, y, '.', 8, 1);
    OLED_ShowNum(x + 12U, y, scaled % 100U, 2, 8, 1);
}

static void BoardRuntime_RefreshDisplay(void)
{
    AppStatus status;
    const AppStatus *shared = App_GetStatus();
    uint32_t primask;
    uint32_t fault_code;

    if (shared == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    status = *shared;
    if (primask == 0U)
    {
        __enable_irq();
    }
    fault_code = BoardRuntime_FirstFault(status.faults);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)BoardRuntime_ModeText(status.mode), 8, 1);
    OLED_ShowString(30, 0,
                    (u8 *)BoardRuntime_StateText(status.state), 8, 1);
    OLED_ShowNum(96, 0,
                 (uint32_t)(status.output_frequency_hz + 0.5f),
                 2, 8, 1);
    OLED_ShowString(108, 0, (u8 *)"Hz", 8, 1);

    OLED_ShowString(0, 8, (u8 *)"IA:", 8, 1);
    BoardRuntime_ShowSignedHundredths(18, 8, status.ia_a);
    OLED_ShowString(66, 8, (u8 *)"IB:", 8, 1);
    BoardRuntime_ShowSignedHundredths(84, 8, status.ib_a);

    OLED_ShowString(0, 16, (u8 *)"AB:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 16, status.vab_v);
    OLED_ShowString(66, 16, (u8 *)"BC:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 16, status.vbc_v);

    OLED_ShowString(0, 24, (u8 *)"VDC:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(24, 24, status.dc_voltage_v);
    OLED_ShowString(66, 24, (u8 *)"SET:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(90, 24,
                                    status.dc_voltage_reference_v);

    OLED_ShowString(0, 32, (u8 *)"OUT:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(
        24, 32, status.output_line_voltage_reference_v);
    OLED_ShowString(66, 32, (u8 *)"FLT:", 8, 1);
    OLED_ShowNum(90, 32, fault_code, 2, 8, 1);

    if (status.mode != MODE2_INVERTER)
    {
        OLED_ShowString(0, 40,
                        (u8 *)(status.rectifier_dc_ready
                               ? "R:RDY" : "R:---"),
                        8, 1);
    }
    else
    {
        OLED_ShowString(0, 40, (u8 *)"K:", 8, 1);
        BoardRuntime_ShowUnsignedHundredths(
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
        OLED_ShowString(0, 48, (u8 *)"B:-1V C:+1V", 8, 1);
        OLED_ShowString(0, 56, (u8 *)"#HOLD2S D:STOP", 8, 1);
    }
    OLED_Refresh();
}

void BoardRuntime_Init(void)
{
    const uint32_t neutral = BOARD_PWM_NEUTRAL_COUNTS;
    uint32_t primask;

    s_trip_latched = true;
    s_delayed_pwm_valid = false;
    s_last_adc_frame_ms = 0U;
    s_adc_watchdog_armed = false;
    BoardRuntime_DisableAllPwm();
    TIM1->CCR1 = neutral;
    TIM1->CCR2 = neutral;
    TIM1->CCR3 = neutral;
    TIM8->CCR1 = neutral;
    TIM8->CCR2 = neutral;

    /*
     * 使能 TIM1 CH1-3 和 TIM8 CH1/CH2 的 OC preload：CCR 在 ADC ISR 中写入
     * 后，统一在下一个 update 事件转移，与“延迟一周期装载”逻辑保持
     * 一致，避免载波中途跳变。每帧只写一次 CCR，不存在双缓冲错拍。
     * TIM8 CH4 是 ADC 触发基准，保持原有配置不动。
     */
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

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

    /*
     * ADC 帧看门狗：独立于 OLED/键盘周期，每轮后台都检查。首帧到达前
     * 不武装，启动阶段不会误触发；武装后超过 5 ms 无新帧说明 TIM8/
     * ADC/DMA 中断链停摆，PWM 可能保持旧 CCR 输出，必须锁存故障。
     * 停帧意味着此后不会再有 ADC 回调推进状态机，这里在锁存后补一次
     * App_FastStep20kHz()，让状态机进入 FAULT_LATCH 并发布故障状态
     * （OLED 显示、TryResetFault 解锁）。s_trip_latched 已置位时跳过，
     * 避免后台每轮重复 LatchTrip/FastStep；复位后 s_trip_latched 清除
     * 而时间戳仍旧，若采样链未恢复下一轮会自然再次触发。
     * BoardRuntime_LatchTrip() 本身不变：它也被高优先级 Break ISR 调用，
     * 不能在通用路径里跑 FastStep。
     */
    if (s_adc_watchdog_armed
        && !s_trip_latched
        && ((now - s_last_adc_frame_ms) > BOARD_ADC_FRAME_WATCHDOG_MS))
    {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        App_FastStep20kHz();
    }

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
