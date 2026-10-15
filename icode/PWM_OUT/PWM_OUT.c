/*
 * PWM_OUT.c
 *
 *  Created on: Jul 22, 2024
 *      Author: lly
 */

#include"PWM_OUT.h"
#include"arm_math.h"
#include"../../icode/my_define/my_extern_define.h"
#include"main.h"
#include"tim.h"
float D1R_T,D2R_T,D3R_T;
void PWM_Output(){
	//    	开环测试
//	    	DdR=0.5,DqR=0;
//	DdI=0.5;DqI=0;
	//占空比Park逆变换
	//arm_inv_park_f32(DdR,DqR,&DalphaR,&DbetaR,sin_thetas,cos_thetas);
	//if(DalphaR>0.985)DalphaR=0.985;
	//	else if(DalphaR<-0.985)DalphaR=-0.985;
	//inv_Clarke_F(DalphaR,DbetaR,&D1R_T,&D2R_T,&D3R_T);
	//单极性倍频输出
	D1R=2100*(boa+1);
	D2R=2100*(bob+1);
	D3R=2100*(boc+1);
	/*TIM1->CCR1=Tim1_ARR*D1R;Tim1_ARR*D2R
	TIM1->CCR2=Tim1_ARR*D2R;Tim1_ARR*D1R;*/
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,D1R);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,D2R);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,D3R);
}

