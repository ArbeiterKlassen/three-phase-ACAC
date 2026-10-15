/*
 * my_adc.c
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */


#include "main.h"
#include"my_adc.h"
#include"math.h"
#include"arm_math.h"
#include"../../icode/my_define/my_define.h"
#include"../../icode/my_define/my_extern_define.h"
#include"../../icode/pll/pid.h"
#include"../../icode/begin_dcac/begin_dcac.h"
uint8_t i;
float32_t V_in_alpha=0.0f,V_in_beta=0.0f;
float32_t Vdelay[100];
int count_fifo=0;
int adc1,adc2,adc3,adc4,adc5,adc6,adc7,adc8;
int h=0,point;
float sum_va=0,sum_ia=0,sum_vb=0,sum_ib=0,sum_vc=0,sum_ic=0,sum_dc,pianzhiU_sum=0,pianzhiU=0,pianzhiI_sum=0,pianzhiI=0;

void ADC_DAC_get_and_show2()
{
	ADC_To_adx();//adc赋值
	ADC_Real_Value_Get();
	//ADC_Queue_Save();
	//ADC_Clark_Transform();
	//ADC_Sine_Calculate();
	//ADC_Park_Transform();
	/*ADC_Paremeters_Calc();*/
}
void ADC_To_adx()
{
	adc1=adc_V[0];
	adc2=adc_V[1];
	adc3=adc_V[2];
	adc4=adc_V[3];
	adc5=adc_V[4];
	adc6=adc_V[5];
	adc7=adc_V[6];
	adc8=adc_V[7];
}
//在开始前控制进行输出电流采样
/*const int ADC5_AVG_NUM = 2000;//0.1s
const float ADC5_AVG_Weight = 50e-5;//=1/2000
float ad5_sum=0;
int ad5_avg=0;
int time=0;
int ad5_avg_flag=0;*/
//滤波
/*void ADC_Real_Value_Get(){
	//	Us ad1-ad2 10kHz滤波器滤�??
	//	Us=0.8f*Us+0.2f*(ad1-ad2)/4096.0f*3.3f*18.292f*修正
		U1=0.8f*U1+0.2f*(ad1-ad2)*0.015032f;

	//	Is ad3-ad4 10kHz滤波器滤�??
	//	Is=0.8f*Is+0.2f*((ad3-ad3_avg)/4096.0f*3.3f)*1.6835f*1.19;
//		Is=0.8f*Is+0.2f*(ad3-ad4)*0.001614f;


	//	IoIV ad5-ad6 10kHz滤波 注意是负的！！！这个地方的符号要细致的看
	//	IoIV=0.8f*Is+0.2f*((ad5-ad6)/4096.0f*3.3f)*3.7037f*1.26;
		I1=0.8f*I1+0.2f*(ad5-ad6-ad5_avg)*-0.00343f;
	//	Udc ad7-ad8
		//对母线直流电压用4kHz的低通滤波器进行滤波，对�????50Hz幅�?�差0.9999，角�????1°不到，应该可用，如果不行就用8kHz
	//	Udc=0.95f*Udc+0.05f*(ad4-ad5)/4096.0f*3.3f*26.49989f;
		Udc=0.95f*Udc+0.05f*(ad7-ad8)*0.021350f-0.01f;
}*/
void ADC_Real_Value_Get()
{
	Ua=-32.3906*(adc1*3.3/4096.0-1.633)*0.9492*1.3559;
   /* Ia=-1*(adc2/4096.0*3.3-1.648)*3.5796;
    Ub=-32.3906*(adc3*3.3/4096.0-1.6493)*1.2837;
    Ib=-1*(adc4/4096.0*3.3-1.656)*4.7086;
    Uc=-32.3906*(adc5*3.3/4096.0-1.6367)*1.2892;
    Ic=-1*(adc6/4096.0*3.3-1.644)*4.7097;
    Udc_zhen=18.163*adc7/4096.0*3.3;
	//U1=(adc1*3.3/4096.0-1.635)*42.49/1.035;
	//U1=0.8f*U1+0.2*31.1334*1.0451*(adc1*3.3/4096.0-1.5);
	//U1=Low_pass_filter(&low_pass_filter,U1);
	/*U1=Low_pass_filter(&low_pass_filter,U1);*/
	//adc1=Low_pass_filter(&low_pass_filter,adc1);
	//U1=(adc1/4096.0-1.647)*31.1334;
	//I1=2.43*(adc2/4096.0*3.3-1.62)/0.98995;
	//I1=Low_pass_filter(&low_pass_filter,I1);
	//Udc=(adc3-adc4)/4096.0*3.3*16;
	/*if(47.38<=Udc_zhen)
	{
		Udc=Udc_zhen*1.0038;
	}
	if(Udc_zhen<47.38)
	{
		Udc=Udc_zhen;
	}*/
	point=400;
	pianzhiU_sum=pianzhiU_sum+adc1*3.3/4096.0;
//	pianzhiI_sum=pianzhiI_sum+adc4/4096.0*3.3;
	sum_va=sum_va+Ua*Ua;
//	sum_ia=sum_ia+Ic*Ic;
//	sum_dc=sum_dc+Udc*Udc;
	h++;
	if(h>=point)
	{
	  arm_sqrt_f32(sum_va/point,&U1xiao);
//	  arm_sqrt_f32(sum_ia/point,&I1xiao);
	 // arm_sqrt_f32(sum_dc/point,&dcxiao);
	  pianzhiU=pianzhiU_sum/400.0f;
	 // pianzhiI=pianzhiI_sum/200.f;
	  pianzhiU_sum=0;
	 // pianzhiI_sum=0;
	  h=0;
	  sum_va=0;
	  diaoderta=PID_realize(&pid_Udc,15,U1xiao);
	  diao=diao+diaoderta;
	  if(diao>0.998) diao=0.998;
	  if(diao<0.2)diao=0.2;
//	  sum_ia=0;
//	  sum_dc=0;
	}
}

void ADC_Queue_Save(){
	enQueue(&U1_queue,Ua);
}
void ADC_Clark_Transform()
{
	 Clarke_F(Ua, Ub, Uc, &Uapha, &Ubeta);
	 Clarke_F(Ia, Ib, Ic, &Iapha, &Ibeta);
}
void ADC_Sine_Calculate(){
	//进行正弦运算 注意角度是弧度制 1.8us
	sin_thetas=arm_sin_f32(thetas);
    cos_thetas= arm_cos_f32(thetas);
}
void ADC_Park_Transform(){
	//进行PARK运算
	arm_park_f32(Ua,U1_queue.data[(U1_queue.rear-50+maxsize)%maxsize],
			&U1d,&U1q,sin_thetas,cos_thetas);
	arm_park_f32(Iapha,Ibeta,
			&I1d,&I1q,sin_thetas,cos_thetas);
}


/*void ADC_Paremeters_Calc(){
	U1rms_screen=0.999f*U1rms_screen+0.001f*U1d*0.7071f;
	I1d_screen=0.999f*I1d_screen+0.001f*I1d;
	I1q_screen=0.999f*I1q_screen+0.001f*I1q;
	arm_sqrt_f32(I1d_screen*I1d_screen+I1q_screen*I1q_screen,&I1rms_screen);
//	PF=I1d_screen/I1rms;
	I1rms_screen=I1rms_screen*0.7071f;
//	I1rms_screen=I1rms_screen*0.7071f*0.8931f+0.2386f;
}*/

void Clarke_F(float Ua, float Ub, float Uc, float *Ualpha, float *Ubeta)
{
	*Ualpha = (0.6666666667f) * (Ua-(Ub+Uc)/2.0f);
	*Ubeta = (0.5773502692f) * (Ub-Uc);
}
