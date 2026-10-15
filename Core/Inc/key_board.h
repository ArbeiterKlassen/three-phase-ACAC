/*
 * key_board.h
 *
 *  Created on: Jun 29, 2025
 *      Author: Administrator
 */

#ifndef INC_KEY_BOARD_H_
#define INC_KEY_BOARD_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f4xx.h"

void Key_init(void);
int Read_Key(void);
#endif /* INC_KEY_BOARD_H_ */
