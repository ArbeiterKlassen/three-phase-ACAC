/*
 * my_clock.c
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */

#include"my_clock.h"
#include"../../icode/my_define/my_extern_define.h"
int time_Tim1=0;
void Clock_Timer(){
	time_Tim1++;
	if(time_Tim1>10000){
		time_Tim1-=10000;
	}
}
