#include "acdc_board_runtime.h"

#include <stddef.h>

#include "adc.h"
#include "acdc_application.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

#define ACDC_PWM_PERIOD_COUNTS (4200U)
#define ACDC_PWM_NEUTRAL_COUNTS (ACDC_PWM_PERIOD_COUNTS / 2U)
#define ACDC_ADC_FRAME_COUNT (2U)
#define ACDC_ADC_WORD_COUNT (ACDC_ADC_FRAME_COUNT * ACDC_ADC_RANK_COUNT)

#define ACDC_ACTIVE_CCER_MASK (TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define ACDC_ALL_POWER_CCER_MASK                                                                                       \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)

static AcdcApplication s_application;
static uint32_t s_adc_dma[ACDC_ADC_WORD_COUNT];
static bool s_key_was_down;
static uint32_t s_key_change_tick;
static bool s_key_accepted;

static uint32_t AcdcBoard_EnterCritical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void AcdcBoard_ExitCritical(uint32_t primask) {
    if ((primask & 1U) == 0U) __enable_irq();
}

/*
 * 门极关闭顺序必须是“先 MOE、后通道使能”。若反过来，在两步之间仍可能
 * 有一个已经排队的高侧脉冲。CCR 回中点是额外保险，不是主要关断手段。
 */
void AcdcBoard_EmergencyStop(void) {
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM8EN) != 0U) {
        TIM8->BDTR &= ~TIM_BDTR_MOE;
        TIM8->CCER &= ~ACDC_ALL_POWER_CCER_MASK;
        TIM8->CCR1 = ACDC_PWM_NEUTRAL_COUNTS;
        TIM8->CCR2 = ACDC_PWM_NEUTRAL_COUNTS;
    }
}

/* 实际功率半桥固定为 PC7(HI)/PB0(LI)，即 TIM8 CH2/CH2N、PWM1。 */
static bool AcdcBoard_ConfigureHalfBridgeChannels(void) {
    TIM_OC_InitTypeDef oc = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = ACDC_PWM_NEUTRAL_COUNTS,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };

    if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_2) != HAL_OK) return false;

    /* CH4 不出引脚，只给 ADC 提供中心对齐周期内的单次触发。 */
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = 3447U;
    return HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_4) == HAL_OK;
}

static bool AcdcBoard_BreakInputActive(void) {
    return HAL_GPIO_ReadPin(RECT_BKIN_GPIO_Port, RECT_BKIN_Pin) == GPIO_PIN_SET;
}

static void AcdcBoard_ApplyOutput(const AcdcControlOutput *out) {
    if (out == NULL) return;
#if !ACDC_HARDWARE_COMMISSIONED
    /* 编译期门禁的第二道保险：即使应用层状态异常也不允许写 CCER/MOE。 */
    AcdcBoard_EmergencyStop();
    return;
#else
    uint32_t high_side_counts;
    bool allow_gate;

    if (out->gates_enabled && AcdcBoard_BreakInputActive()) {
        AcdcApp_LatchExternalFault(&s_application);
        AcdcBoard_EmergencyStop();
        return;
    }
    allow_gate =
        out->valid && out->gates_enabled && (s_application.state == ACDC_STATE_RUN) && !AcdcBoard_BreakInputActive();
    if (!allow_gate) {
        AcdcBoard_EmergencyStop();
        return;
    }

    high_side_counts = (uint32_t)((1.0f - out->low_side_duty) * (float)ACDC_PWM_PERIOD_COUNTS + 0.5f);
    if (high_side_counts >= ACDC_PWM_PERIOD_COUNTS) high_side_counts = ACDC_PWM_PERIOD_COUNTS - 1U;

    TIM8->CCR2 = high_side_counts;
    TIM8->CCR1 = ACDC_PWM_NEUTRAL_COUNTS;
    TIM8->CCER |= ACDC_ACTIVE_CCER_MASK;
    __HAL_TIM_MOE_ENABLE(&htim8);
#endif
}

static void AcdcBoard_ProcessFrame(uint32_t offset) {
    AcdcControlOutput out;

    if ((offset + ACDC_ADC_RANK_COUNT) > ACDC_ADC_WORD_COUNT) return;
    AcdcApp_FastStep(&s_application, &s_adc_dma[offset], &out);
    AcdcBoard_ApplyOutput(&out);
}

/* DMA 半满/全满各对应一帧，避免 DMA 写当前帧时 CPU 读同一块内存。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if ((hadc != NULL) && (hadc->Instance == ADC1)) AcdcBoard_ProcessFrame(0U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if ((hadc != NULL) && (hadc->Instance == ADC1)) AcdcBoard_ProcessFrame(ACDC_ADC_RANK_COUNT);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if ((hadc != NULL) && ((hadc->Instance == ADC1) || (hadc->Instance == ADC2))) {
        AcdcApp_LatchMeasurementFault(&s_application);
        AcdcBoard_EmergencyStop();
    }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
    if ((htim != NULL) && (htim->Instance == TIM8)) {
        AcdcBoard_EmergencyStop();
        AcdcApp_LatchExternalFault(&s_application);
    }
}

void AcdcBoard_Init(void) {
    uint32_t primask;

    /* 上电先关门，再碰任何可能拉高驱动输入的引脚。 */
    AcdcBoard_EmergencyStop();
    AcdcApp_Init(&s_application);
    if (!AcdcBoard_ConfigureHalfBridgeChannels()) {
        AcdcApp_LatchControlFault(&s_application);
        return;
    }

    TIM8->CCR1 = ACDC_PWM_NEUTRAL_COUNTS;
    TIM8->CCR2 = ACDC_PWM_NEUTRAL_COUNTS;
    TIM8->CCER &= ~ACDC_ALL_POWER_CCER_MASK;
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);

    primask = AcdcBoard_EnterCritical();
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    if (AcdcBoard_BreakInputActive()) AcdcApp_LatchExternalFault(&s_application);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
    AcdcBoard_ExitCritical(primask);

    if ((HAL_ADC_Start(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_MultiModeStart_DMA(&hadc1, s_adc_dma, ACDC_ADC_WORD_COUNT) != HAL_OK)) {
        AcdcApp_LatchMeasurementFault(&s_application);
        AcdcBoard_EmergencyStop();
        return;
    }

    /* TIM8 只负责时基和 ADC 触发；两套功率通道的 CCER 仍保持关闭。 */
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim8);
}

void AcdcBoard_BackgroundTask(void) {
    const uint32_t now = HAL_GetTick();
    const bool key_down = HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET;
    AcdcSnapshot snapshot;
    bool led_on;

    if (key_down != s_key_was_down) {
        s_key_was_down = key_down;
        s_key_change_tick = now;
        s_key_accepted = false;
    }
    if (key_down && !s_key_accepted && ((now - s_key_change_tick) >= 30U)) {
        uint32_t primask = AcdcBoard_EnterCritical();
        AcdcApp_ToggleRequest(&s_application);
        AcdcBoard_EmergencyStop();
        /*
         * 清故障后允许重新 arm，但必须清掉上一次 BKIN 留下的标志和
         * NVIC pending；若故障源仍在，下一拍仍会重新锁存。
         */
        __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
        HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
        AcdcBoard_ExitCritical(primask);
        s_key_accepted = true;
    }

    AcdcBoard_GetSnapshot(&snapshot);
    switch (snapshot.state) {
    case ACDC_STATE_RUN:
        led_on = true;
        break;
    case ACDC_STATE_FAULT:
        led_on = ((now / 100U) & 1U) != 0U;
        break;
    case ACDC_STATE_LOCKED:
        led_on = ((now / 100U) & 1U) != 0U;
        break;
    default:
        led_on = ((now / 500U) & 1U) != 0U;
        break;
    }
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void AcdcBoard_GetSnapshot(AcdcSnapshot *snapshot) {
    uint32_t primask;
    if (snapshot == NULL) return;
    primask = AcdcBoard_EnterCritical();
    AcdcApp_GetSnapshot(&s_application, snapshot);
    AcdcBoard_ExitCritical(primask);
}
