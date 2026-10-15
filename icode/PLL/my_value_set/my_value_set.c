/*
 * my_value_set.c
 *
 *  Created on: Jul 22, 2024
 *      Author: lly
 */

#include"my_value_set.h"
#include"../../../icode/my_define/my_extern_define.h"
#include"main.h"
#include"math.h"
int choose_inductance_capacity = 1;//电感为1电容为-1
float I1_ref =1.1;
int Mode=1;//Mode 为1或其他 则是CC，Mode为2，CR，为3，CP
float ref_Udc=24;
void Control_Value_Set()
{
	   /*if((PF_set<=1)&&(PF_set>=0.95)&&choose_inductance_capacity==1)
		cosphi=PF_set;
	   if((PF_set<=0.95)&&(PF_set>0.85)&&choose_inductance_capacity==1)
	  		cosphi=PF_set-0.012;
	    if(0.75<PF_set&&PF_set<=0.85&&choose_inductance_capacity==1)
	    {  	cosphi=PF_set-0.015-0.006;}
	    if(0.65<PF_set&&PF_set<=0.75&&choose_inductance_capacity==1)
	    {
	    	cosphi=PF_set-0.02;
	    }
	    if(0.55<PF_set&&PF_set<=0.65&&choose_inductance_capacity==1)
	    	    {
	    	    	cosphi=PF_set-0.022;
	    	    }
	    if(PF_set<=0.55&&choose_inductance_capacity==1)
	   	    	    {
	   	    	    	cosphi=PF_set-0.03+0.01;
	   	    	    }

	    if((PF_set<=1)&&(PF_set>=0.95)&&choose_inductance_capacity==-1)
	    		cosphi=PF_set;
	    	   if((PF_set<=0.95)&&(PF_set>0.85)&&choose_inductance_capacity==-1)
	    	  		cosphi=PF_set+0.03;
	    	    if(0.75<=PF_set&&PF_set<=0.85&&choose_inductance_capacity==-1)
	    	    {  	cosphi=PF_set+0.03+0.002;}
	    	    if(0.65<=PF_set&&PF_set<0.75&&choose_inductance_capacity==-1)
	    	    {
	    	    	cosphi=PF_set+0.028+0.005+0.004;
	    	    }
	    	    if(0.55<=PF_set&&PF_set<0.65&&choose_inductance_capacity==-1)
	    	    	    {
	    	    	    	cosphi=PF_set+0.028+0.005;
	    	    	    }
	    	    if(PF_set<0.55&&choose_inductance_capacity==-1)
	    	   	    	    {
	    	   	    	    	cosphi=PF_set+0.025;
	    	   	    	    }
	  /* if(0.65<PF_set&&PF_set<=0.75)
	    {
	    	cosphi=PF_set-0.041;
	    }
	    if(0.55<PF_set&&PF_set<=0.65)
	    	    {
	    	    	cosphi=PF_set-0.028;
	    	    }
	    if(0.45<PF_set&&PF_set<=0.55)
	   	    	    {
	   	    	    	cosphi=PF_set-0.029;
	   	    	    }
	    if(PF_set<=0.45)
	    {
	    	cosphi=PF_set;
	    }*/

		//ref_I1d=2*Sqrt2;
		//if(mode==1)
		//{
		//	ref_I1d=1*Sqrt2;
		//}
		//sinphi=sqrt(1-cosphi*cosphi);
		ref_I1q=0;
}

