/*
 * my_adc.h
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */

#ifndef MY_ADC_MY_ADC_H_
#define MY_ADC_MY_ADC_H_

#define ADC_num 4
void ADC_To_adx();
void ADC_ad5_avg_Get();
void ADC_Real_Value_Get();
void ADC_Queue_Save();
void ADC_Sine_Calculate();
void ADC_Park_Transform();
void ADC_Paremeters_Calc();
void ADC_DAC_get_and_show2();
void ADC_Clark_Transform();
void Clarke_F(float Ua, float Ub, float Uc, float *Ualpha, float *Ubeta);


#endif /* MY_ADC_MY_ADC_H_ */
