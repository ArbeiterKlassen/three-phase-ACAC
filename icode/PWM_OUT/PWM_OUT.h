/*
 * PWM_OUT.h
 *
 *  Created on: Jul 22, 2024
 *      Author: lly
 */

#ifndef PWM_OUT_PWM_OUT_H_
#define PWM_OUT_PWM_OUT_H_

#include "stm32f4xx_hal.h""
void PWM_Output();
void inv_Clarke_F(float Ualpha, float Ubeta, float *w1, float *w2, float *w3);
#endif /* PWM_OUT_PWM_OUT_H_ */
