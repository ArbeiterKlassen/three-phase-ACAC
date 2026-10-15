/*
 * pid.c
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */
#include"pid.h"
#include"main.h"
#include"../../icode/my_define/my_extern_define.h"

void PID_Limit(pid_p *pid)
{
	if(pid->result > pid->max_output)
	{
		pid->result=pid->max_output;
	}
	else if(pid->result < pid->min_output){
		pid->result < pid->min_output;

	}
}
void PID_init(pid_p *pid,float kp,float ki, float kd,float max_output,float min_output, float initial_result,float Ts)
{
    //printf("PID_init begin \n");
    pid->REF= 0.0f;		  	// ????????
    pid->ACTUAL= 0.0f;			// adc?????
    pid->err= 0.0f;				    // ???????????
    pid->err_last=0.0f;			    // ??????
    pid->integral= 0.0f;			  	// ???
    pid->Kp= kp;				    // ????
    pid->Ki= ki*Ts;				    // ????
    pid->Kd= kd;				    // ????
    pid->result=0;
    //printf("PID_init end \n");
    pid->max_output=max_output;
    pid->min_output=min_output;
    pid->max_output_integral=pid->max_output/pid->Ki;
    pid->min_out_integral=pid->min_output/pid->Ki;
    pid->integral=initial_result/ki;
    pid->result = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * ( pid->err - pid->err_last);
    PID_Limit(pid);
}

float PID_realize(pid_p *pid, float vref, float v_now)
{

    pid->REF = vref;			// ???????
    pid->ACTUAL = v_now;	// ?????? = ADC_Value * 3.3f/ 4096
    pid->err =   pid->REF - pid->ACTUAL;	//????
    /*pid->result = pid->Kp* pid->err +pid->Ki * pid->integral + 0* ( pid->err - pid->err_last);*/
   /* wu=488.7*(pid->err - pid->err_last)+0.3472*pid->err;*/
    /*wu=0.000003*pid->integral+100*( pid->err);*/
    /*pid->result=wu;*/
   wu=pid->Kp*(pid->err - pid->err_last)+pid->Ki*pid->err;
   /*if(wu>2*3.1416)
   {
	   wu=2*3.1416;
   }
   if(wu<-2*3.1416)
   {
	   wu=-2*3.1416;
   }*/
    pid->result=wu;
    pid->integral += pid->err;
    pid->err_last = pid->err;
    //???????
   PID_Limit(pid);
    return pid->result;
}

//吉吉国王
void PidDeltaInit(PID_DELTA *pp, float setPoint, float min, float max,
		float kp, float ki, float kd)
{
    pp->setPoint = setPoint;

    pp->min_result = min;
    pp->max_result = max;
	pp->Proportion = kp;
	pp->Integral = ki;
	pp->Derivative = kd;

	pp->LastError = 0.0f;
	pp->PreError = 0.0f;
}
void PidDeltaInit1(PID_DELTA *pp)
{
	pp->LastError = 0.0f;
	pp->PreError = 0.0f;
}

/* ����ʽ PID �㷨������Ҫ���ƵĲ��ǿ������ľ���ֵ�����ǿ�������������
 * �������: PID �Ŀ��Ƴ����ͺ����ֵ
 * ����ֵ:   ������� delta_Uk_temp */
float PidDeltaCal(PID_DELTA *pp, float ThisPoint)
{
	float ThisError;                                           /* Error[k] ��һ��ʵ��ֵ���趨ֵ����� */
//	double pError, dError, iError;                              /* PID ��������Ӧ�������� */
	float pError, iError;                              /* PID ��������Ӧ�������� */

	float delta_Uk_temp;                                       /* ��u[k] */
	float max,min;
	max=pp->max_result;
	min=pp->min_result;

	ThisError = pp->setPoint - ThisPoint;                       /* Error[k] */
	pError = ThisError - pp->LastError;                         /* Error[k] - Error[k-1] */
	iError = ThisError;                                         /* Error[k] */
//	dError = ThisError - 2 * (pp->LastError) + pp->PreError;    /* Error[k] - 2 * Error[k-1] + Error[k-2] */

	/* ����������������: ��u[k] = ��u[k] - ��u[k-1] */
//	delta_Uk_temp = (pp->Proportion * pError                    /* Proportion = Kp */
//                    + pp->Integral * iError                     /*   Integral = Ki = Kp * T / Ti */
//                    + pp->Derivative * dError);                 /* Derivative = Kd = Kp * Td / T */
	delta_Uk_temp = pp->Proportion * pError  + pp->Integral * iError ;                  /* Proportion = Kp */

	if (delta_Uk_temp >max || delta_Uk_temp < min)
	{                                                           /* �����������������, ���洢��� */
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return (delta_Uk_temp > max)? max : min;
	}
	else
	{	                                                        /* ���������������, �洢��������´μ��� */
		pp->PreError = pp->LastError;
		pp->LastError = ThisError;
		return delta_Uk_temp;
	}
}
float PidDeltaCal1(PID_DELTA *pp, float ThisPoint)
{
	float ThisError,delta_Uk_temp,max,min;
	max=pp->max_result;
	min=pp->min_result;
	ThisError = pp->setPoint - ThisPoint;
	delta_Uk_temp= pp->Proportion*(ThisError - pp->LastError)+pp->Integral*ThisError;
	/*delta_Uk_temp=100*(ThisError - pp->LastError)+100*ThisError;*/
	pp->PreError = pp->LastError;
	pp->LastError = ThisError;
	if (delta_Uk_temp >max || delta_Uk_temp < min)
		return (delta_Uk_temp > max)? max : min;
	else
		return delta_Uk_temp;
}
float PidDeltaCal2(PID_DELTA *pp, float ThisPoint,float setpoint,float max,float min,float kp,float ki)
{
	float ThisError,delta_Uk_temp;
	ThisError = setpoint - ThisPoint;
	delta_Uk_temp= kp*(ThisError - pp->LastError)+ki*ThisError;
	pp->PreError = pp->LastError;
	pp->LastError = ThisError;
	if (delta_Uk_temp >max || delta_Uk_temp < min)
		return (delta_Uk_temp > max)? max : min;
	else
		return delta_Uk_temp;




}

/* ��̬���� PID ���� */
void adjust_pid(PID_DELTA *pp, float kp, float ki, float kd)
{
	pp->Proportion = kp;
	pp->Integral = ki;
	pp->Derivative = kd;
}

/* ������ʼ�趨ֵ */
void adjust_pid_setPoint(PID_DELTA *pp, float newSetPoint)
{
	pp->setPoint = newSetPoint;
}

/* ��̬�������������������Ʒ�Χ */
void adjust_pid_limit(PID_DELTA *pp, float min, float max)
{
	pp->min_result = min;
	pp->max_result = max;
}
