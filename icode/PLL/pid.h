/*
 * pid.h
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */

#ifndef PLL_PID_H_
#define PLL_PID_H_
typedef struct
{
    float REF;	  	//?????
    float ACTUAL;	//?????
    float err;			    //?????
    float err_last;		  	//????????
    float Kp,Ki,Kd;		  	//????????????
    float result;		    //pid????
    float integral;		  	//?????
    float max_output,min_output,initial_result;
    float max_output_integral,min_out_integral;
}pid_p;
typedef struct PID_DELTA
{
	float setPoint;                        /* ��ʼ�趨ֵ */

	float min_result;                      /* ���������������ֵ */
	float max_result;                      /* ��������������Сֵ */

	float Proportion;                      /* ����ϵ�� */
	float Integral;                        /* ����ϵ�� */
	float Derivative;                      /* ΢��ϵ�� */

	float LastError;                       /* ǰһ�����Error[k-1] */
	float PreError;                        /* ǰ�������Error[k-2] */

} PID_DELTA;

extern void PidDeltaInit(PID_DELTA *pp, float setPoint, float min, float max, float kp, float ki, float kd);
extern void PidDeltaInit1(PID_DELTA *pp);
extern float PidDeltaCal(PID_DELTA *pp, float ThisPoint);
extern float PidDeltaCal1(PID_DELTA *pp, float ThisPoint);
extern float PidDeltaCal2(PID_DELTA *pp, float ThisPoint,float setpoint,float max,float min,float kp,float ki);
extern void adjust_pid(PID_DELTA *pp, float kp, float ki, float kd);
extern void adjust_pid_limit(PID_DELTA *pp, float min, float max);
extern void adjust_pid_setPoint(PID_DELTA *pp, float newSetPoint);

void PID_Limit(pid_p *pid);
void PID_init(pid_p *pid,float kp,float ki, float kd,float max_output,float min_output, float initial_result,float Ts);
float PID_realize(pid_p *pid, float vref, float v_now);


#endif /* PLL_PID_H_ */
