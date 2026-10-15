/*
 * pi.c
 *
 *  Created on: Jul 22, 2024
 *      Author: lly
 */
#include "main.h"
#include"pid.h"
#include"../../icode/my_define/my_extern_define.h"
pid_p pid_I1d,pid_I1q,pid_Udc,pid_Uout;
float UId_delta=0,UIq_delta=0,Udc_delta,ref_I1d_deta;
PID_DELTA Id1_PID_Stru  = {2,    /* setPoint */
                            -2,     /* min_result */
                            2,      /* max_result */
                            0.0004,      /* Proportion */  //0.4
                            0.1,     /* Integral */ //   0.3572可用
                            0,      /* Derivative */};
PID_DELTA Iq1_PID_Stru  = {2,    /* setPoint */
                            -2,     /* min_result */
                            2,      /* max_result */
							0.0004,      /* Proportion */  //0.4
							0.1,     /* Integral */ //   0.3572可用
                            0,      /* Derivative */};


/*控制设计*/

void PI_Control(){
	//逆变电压环
  /*UId_delta=PidDeltaCal2(&Id1_PID_Stru,I1d,ref_I1d,2,-2,0.02f,0.4f);
	//UId_delta=PID_realize(&pid_I1d,ref_I1d,I1d);
	UId=UId+UId_delta;
	//UIq_delta=PID_realize(&pid_I1q,ref_I1q,I1q);
	UIq_delta=PidDeltaCal2(&Iq1_PID_Stru,I1q,ref_I1q,2,-2,0.02f,0.4f);
	UIq=UIq+UIq_delta;
	DdR=(U1d+I1q*Xs-UId)/30;
	DqR=(U1q-I1d*Xs-UIq)/30;*/
	//UIq_delta=PidDeltaCal2(&Id1_PID_Stru,I1q,ref_I1d,2,-2,2,0.0044f);
		//UId_delta=PID_realize(&pid_I1d,ref_I1d,I1d);
		//UIq=UIq+UIq_delta;
		//UIq_delta=PID_realize(&pid_I1q,ref_I1q,I1q);
		//UId_delta=PidDeltaCal2(&Iq1_PID_Stru,I1d,ref_I1q,2,-2,2,0.0044f);
		//UId=UId+UId_delta;
	//前馈和限制
	ref_I1d_deta=PID_realize(&pid_Uout,50,Udc);
	ref_I1d=ref_I1d+ref_I1d_deta;
	if(ref_I1d>10)
	{
		ref_I1d=10;
	}
	if(ref_I1d<0.2)
	{
		ref_I1d=0.2;
	}
	UId_delta=PID_realize(&pid_I1d,2.2624,I1d);
	UIq_delta=PID_realize(&pid_I1q,1.6968,I1q);
	UId=UId+UId_delta;
	UIq=UIq+UIq_delta;
	if(UId<-50)UId=-50;
	DdR=(U1d+I1q*Xs-UId)/(dcxiao);
	DqR=(U1q-I1d*Xs-UIq)/(dcxiao);
	if(DdR>0.97)DdR=0.97;
		else if(DdR<-0.95)DdR=-0.95;
		if(DqR>0.95)DqR=0.95;
		else if(DqR<-0.95)DqR=-0.95;


//    限制DdDq
//	    	if(Dd>0.8)Dd=0.8;
//	    	else if (Dd<-0.8) Dd=-0.8;
//	    	if(Dq>0.8)Dq=0.8;
//	    	else if (Dq<-0.8) Dq=-0.8;

//    	开环测试
//    	D3d=0.5;D3q=0;
}

