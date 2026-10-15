/*
 * spwm.c
 *
 *  Created on: Jul 15, 2024
 *      Author: lly
 */
#include"spwm.h"
#include"../../icode/my_define/my_extern_define.h"
void get_sin_tab1( uint16_t point, uint16_t maxnum )
{
    uint16_t i = 0, j = 0, k = 0;
    float hd = 0.0;        	//弧度
    float fz = 0.0;       	//峰值
    uint16_t tem = 0;
    j = point / 2;			//水平线位置 单片机没有负电压
    hd = PI / j;        	// π/2 内每一个点对应的弧度值
    k = maxnum / 2;      	//最大值一半
    for( i = 0; i < point; i++ )
    {
		fz = k * sin( hd * i )+k;
        tem = ( uint16_t )fz ;
        sinData[i] = tem;
    }
}
