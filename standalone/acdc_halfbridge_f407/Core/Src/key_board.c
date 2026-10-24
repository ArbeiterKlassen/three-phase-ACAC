/*
 * key_board.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Administrator
 */
#include "key_board.h"

static void Key_WaitRowSettled(void) {
    uint32_t settle_cycles = SystemCoreClock / 1000000U;
    uint32_t started;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        __NOP();
        __NOP();
        return;
    }
    if (settle_cycles == 0U) settle_cycles = 1U;

    started = DWT->CYCCNT;
    while ((DWT->CYCCNT - started) < settle_cycles) __NOP();
}

static void Key_SetAllRowsLow(void) {
    GPIOE->BSRR = (GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4) << 16;
    GPIOD->BSRR = GPIO_PIN_11 << 16;
}

void Key_init(void) {
    Key_SetAllRowsLow();
}

int Read_Key(void) {
    static GPIO_TypeDef *const row_port[4] = {GPIOD, GPIOE, GPIOE, GPIOE};
    static const uint16_t row_pin[4] = {GPIO_PIN_11, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_4};
    static const uint16_t col_pin[4] = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_7, GPIO_PIN_10};
    uint32_t columns;
    int row;
    int col;

    for (row = 0; row < 4; ++row) {
        Key_SetAllRowsLow();
        row_port[row]->BSRR = row_pin[row];

        /* Wait about 1 us without depending on the SysTick time base. */
        Key_WaitRowSettled();
        columns = GPIOD->IDR;

        for (col = 0; col < 4; ++col) {
            if ((columns & col_pin[col]) != 0U) {
                Key_SetAllRowsLow();
                return row * 4 + col + 1;
            }
        }
    }

    Key_SetAllRowsLow();
    return 0;
}
