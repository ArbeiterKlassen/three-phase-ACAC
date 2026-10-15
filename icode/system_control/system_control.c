/*
 * system_control.c
 *
 *  Created on: Jul 22, 2024
 *      Author: lly
 */
#include"system_control.h"
#include"../../icode/my_define/my_extern_define.h"
#include"main.h"
#include"tim.h"
int flag_Protect=0;
void System_Protect1(void){
	if(flag_Start_Running==1&&flag_Protect==0&&anjian==1){
			if(thetas<0.03f+PI/2&&thetas>-0.03f+PI/2){
				PID_init(&pid_I1d,2,400,0,2,-2,0,Control_Time);
				PID_init(&pid_I1q,2,400,0,2,-2,0,Control_Time);
				PID_init(&pid_Uout,0.001,40,0,2,-2,0,Control_Time);
				//整流开关打开
				  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
				  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
				  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
				  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
				  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
				  flag_Start_Running=2;
			}
	}
}
