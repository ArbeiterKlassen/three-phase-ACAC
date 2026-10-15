/*
 * begin_dcac.c
 *
 *  Created on: Jun 26, 2025
 *      Author: lly
 */
#include"../../icode/my_define/my_extern_define.h"
#include"begin_dcac.h"
#include"main.h"
#include"tim.h"
#include"arm_math.h"

#include"../../icode/pll/pid.h"
int q=0,l;
float buchang=1,temp=0;
int A,B,C,N;
float U1_kong,U2_kong,U3_kong,T1,T2,T3,T4,T5,T6,T7,boa,bob,boc,diao=0.95f,fd,diaoderta=0;
void begin_dcac()
{
	if(anjiandcac==0)
	{
		  PID_init(&pid_Udc,0.01,10,0,0.01,-0.01,0,Control_Time);
		  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		  anjiandcac=2;
	}
}
void pidcac()
{

	fq=0.9*arm_sin_f32(f*3.1415926*2*buchang/20000.0f);
	fd=0.9*arm_cos_f32(f*3.1415926*2*buchang/20000.0f);
	buchang++;
	if(buchang>=20000)
	{
	   buchang=1;
	}
}
void spwm_nibian()
{
	inv_Clarke_F(fd,fq,&boa,&bob,&boc);
}
void inv_Clarke_F(float Ualpha, float Ubeta, float *w1, float *w2, float *w3)
{
    *w1 =1.0f * Ualpha;
    *w2 =-0.5*Ualpha + 0.866025* Ubeta;
    *w3 =-0.5*Ualpha - 0.866025* Ubeta;
}

/*void kongjianshiliangnibian()
{
	T1=0;T2=0;T3=0;T4=0;T5=0;T6=0;T7=0;
	U1_kong=diao/1.732f*fq;
	U2_kong=diao/1.732f*(0.866*fd-0.5*fq);
	U3_kong=diao/1.732f*(-0.866*fd-0.5*fq);
	if(U1_kong>0)
		A=1;
	else A=0;
	if(U2_kong>0)
		B=1;
	else B=0;
	if(U3_kong>0)
	    C=1;
	else C=0;
	N=A+2*B+4*C;
	switch (N)
		{//扇区1
		  case 3:
			  T4=1.732*U2_kong;
			  T6=1.732*U1_kong;
			  if((T4+T6)>1)
			  {
				  temp=T4+T6;
				  T4=T4/temp;
				  T6=T6/temp;

			  }
			  T7=(1-T4-T6)/2;
			  break;
			  //扇区2
		  case 1:
			  T2=-1.732*U2_kong;
			  T6=-1.732*U3_kong;

			  if((T2+T6)>1)
			  {
			  	temp=T2+T6;
			  	 T2=T2/temp;
			      T6=T6/temp;

			   }
			  T7=(1-T2-T6)/2;
			  break;
			  //扇区3
		  case 5:
				  T2=1.732*U1_kong;
				  T3=1.732*U3_kong;

				  if((T2+T3)>1)
				  {
				  	temp=T2+T3;
				  	T2=T2/temp;
				    T3=T3/temp;

				   }
				  T7=(1-T2-T3)/2;
				  break;
		//扇区4
		  case 4:
					  T1=-1.732*U1_kong;
					  T3=-1.732*U2_kong;

					  if((T1+T3)>1)
					  {
					  	temp=T1+T3;
					  	T1=T1/temp;
					    T3=T3/temp;

					   }
					  T7=(1-T1-T3)/2;
					  break;
		//扇区5
		  case 6:
					  T1=1.732*U3_kong;
					  T5=1.732*U2_kong;

					  if((T5+T1)>1)
					  {
					  	temp=T1+T5;
					  	T1=T1/temp;
					    T5=T5/temp;

					   }
					  T7=(1-T5-T1)/2;
					  break;
		//扇区6
		  case 2:
			       T4=-1.732*U3_kong;
				   T5=-1.732*U1_kong;

						  if((T5+T4)>1)
						  {
						  	temp=T4+T5;
						  	T4=T4/temp;
						    T5=T5/temp;

						   }
						  T7=(1-T5-T4)/2;
						  break;
		default:
	               break;

		}
		boa=T7+T4+T6+T5;
		bob=T7+T6+T2+T3;
		boc=T7+T3+T5+T1;
		/*switch (N)
			{//扇区1
			  case 3:
				  T4=1.732*50e-6*U2_kong;
				  T6=1.732*50e-6*U1_kong;
				  if((T4+T6)>50e-6)
				  {
					  temp=T4+T6;
					  T4=T4*50e-6/temp;
					  T6=T6*50e-6/temp;

				  }
				  T7=(50e-6-T4-T6)/2;
				  break;
				  //扇区2
			  case 1:
				  T2=-1.732*50e-6*U2_kong;
				  T6=-1.732*50e-6*U3_kong;

				  if((T2+T6)>50e-6)
				  {
				  	temp=T2+T6;
				  	 T2=T2*50e-6/temp;
				      T6=T6*50e-6/temp;

				   }
				  T7=(50e-6-T2-T6)/2;
				  break;
				  //扇区3
			  case 5:
					  T2=1.732*50e-6*U1_kong;
					  T3=1.732*50e-6*U3_kong;

					  if((T2+T3)>50e-6)
					  {
					  	temp=T2+T3;
					  	T2=T2*50e-6/temp;
					    T3=T3*50e-6/temp;

					   }
					  T7=(50e-6-T2-T3)/2;
					  break;
			//扇区4
			  case 4:
						  T1=-1.732*50e-6*U1_kong;
						  T3=-1.732*50e-6*U2_kong;

						  if((T1+T3)>50e-6)
						  {
						  	temp=T1+T3;
						  	T1=T1*50e-6/temp;
						    T3=T3*50e-6/temp;

						   }
						  T7=(50e-6-T1-T3)/2;
						  break;
			//扇区5
			  case 6:
						  T1=1.732*50e-6*U3_kong;
						  T5=1.732*50e-6*U2_kong;

						  if((T5+T1)>50e-6)
						  {
						  	temp=T1+T5;
						  	T1=T1*50e-6/temp;
						    T5=T5*50e-6/temp;

						   }
						  T7=(50e-6-T5-T1)/2;
						  break;
			//扇区6
			  case 2:
				       T4=-1.732*50e-6*U3_kong;
					   T5=-1.732*50e-6*U1_kong;

							  if((T5+T4)>50e-6)
							  {
							  	temp=T4+T5;
							  	T4=T4*50e-6/temp;
							    T5=T5*50e-6/temp;

							   }
							  T7=(50e-6-T5-T4)/2;
							  break;
			default:
		               break;

			}
			boa=(T7+T4+T6+T5)*20000;
			bob=(T7+T6+T2+T3)*20000;
			boc=(T7+T3+T5+T1)*20000;


}*/
