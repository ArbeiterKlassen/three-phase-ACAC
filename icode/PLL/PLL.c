/*
 * PLL.c
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */
#include"PLL.h"
#include"main.h"
#include"../../icode/my_define/my_extern_define.h"
#include"pid.h"
float ws=314.1592654,fs;
pid_p PLL;
int time_Point_PLL =10;
int flag_PLL=0;
int flag_Start_Running=0;
int mm=0;
PID_DELTA PLL_PID_Stru  = {0,    /* setPoint */
                            -0.5,     /* min_result */
                            0.5,      /* max_result */
                           0.4887,      /* Proportion */  //0.4887
                            0.3472,     /* Integral */ //   0.3572可用
                            0, 0,0,     /* Derivative */};



void PLL_Realize(){
	//    	PLL锁相，先得角速度再积分，角度大于2pi则减
	//    	ws加一个滤波器，滤出高频分量，用2kHz滤波器 4us
	     /* ws=0.9f*ws+0.1f*PID_realize(&PLL,0,-U1q);*/
	       /* wu=PID_realize(&PLL,0,U1q);*/
	        /*ws=100*PI+wu;*/
	        /*if(ws>320)
	        {
	        	ws=320;
	        }
	        if(ws<300)
	        {
	        	ws=300;
	        }*/
	       /* wu=PidDeltaCal1(&PLL_PID_Stru,-U1q);*/
	       ws=ws+PID_realize(&PLL,0,-U1q);
	        /*if(ws<0) ws=0;
			  if(ws>3142) ws=3142;*/
            if(ws<309)
            {
            	ws=309;
            }
	    	thetas+=ws*100e-6;
	    	if(thetas>=2*PI){thetas-=2*PI;}
	//    	判断PLL锁相锁成功没有，每1/50s检测一次，设定裕度为0.5，检测10次，如果都通过就锁相成功
	//    	如果锁相成功，则初始化电压电流环，并且打开开关。要在过零点打开开关
	    	PLL_Success_Detect();
}

void PLL_Success_Detect(void){
	//判断PLL锁相锁成功没有，每1/50s检测一次，设定裕度为0.5，检测10次，如果都通过就锁相成功
	//如果锁相成功，则初始化电压电流环，并且打开开关。要在过零点打开开关
	if(time_Point_PLL>=Control_Frequency)
		time_Point_PLL-=Control_Frequency;
	if(flag_Start_Running==0){
		if(flag_PLL<10){
			if(time_Tim1==time_Point_PLL){
				if((((U1q<0.9&&U1q>0)||(U1q>-0.9&&U1q<0)))&&(U1d>20||U1d<-20)){
					time_Point_PLL+=400;
					flag_PLL+=1;
				}
				else{
					flag_PLL-=5;
					if(flag_PLL<0)
						flag_PLL=0;
				}
			}
		}
		else{
			flag_Start_Running=1;
			mm=1;
		}
	}


}
