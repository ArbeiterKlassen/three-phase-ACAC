/*
 * my_extern_define.h
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */

#ifndef MY_DEFINE_MY_EXTERN_DEFINE_H_
#define MY_DEFINE_MY_EXTERN_DEFINE_H_
#include"../../icode/duilie/duilie.h"
#include"../../icode/PLL/pid.h"
#include "stm32f4xx_hal.h"
#include"stdint.h"
extern SqQueue U1_queue;
extern SqQueue I1_queue;
extern int time_Tim1;
extern pid_p PLL;
extern pid_p pid_Udc,pid_I1d,pid_I1q,pid_Uout;
extern float Ua,Ub,Uc,Ia,Ib,Ic,Uapha,Ubeta,Iapha,Ibeta,U1xiao,I1xiao,dcxiao,sinphi,cosphi,pianzhiU_sum,pianzhiU,pianzhiI_sum,pianzhiI;
extern float thetas,boa,bob,boc;
extern float sin_thetas,DalphaR,DbetaR,DalphaR_Udc,DbetaR_Udc,D1R,D2R,D3R,D4R;
extern float tiaozhidu,tiaozhidu_derta,fq,cos_thetas,U1d,U1q,I1d,I1q,PF_set,ref_I1d,ref_I1q,I1_ref,rom_in,UId,UIq,Udc,Udcd,DdI,DdR,DqR,Xs,Udc_zhen;
extern int choose_inductance_capacity,MODE;
extern uint16_t sinData[400];
extern int time_Tim1,m,flag_Start_Running;
#define APB1_Timer_Clock (80e6)
#define Tim1_Pre (10)
#define Tim1_ARR (8400)
#define PWM_Output_Frequency (20e3)
#define Double_PWM_Frequency  (40e3)
#define Control_Frequency (20e3)
#define Control_Time (50e-6)
#define ADCTimesEachCycle (400.0f)
#define PI 3.1415926
#endif /* MY_DEFINE_MY_EXTERN_DEFINE_H_ */
