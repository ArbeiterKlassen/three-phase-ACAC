/*
 * my_extern_define.c
 *
 *  Created on: Jul 21, 2024
 *      Author: lly
 */
#include"my_extern_define.h"
SqQueue U1_queue={{},0,0};
SqQueue I1_queue={{},0,0};
float Ua,Ub,Uc,Ia,Ib,Ic,Uapha,Ubeta,Iapha,Ibeta,thetas,sinthetas,costhetas,U1xiao,I1xiao,dcxiao=42;
float U1d,U1q,I1d,I1q,Udc,DdI,DdR,DqR,Udc_zhen;
float Xs=2*PI*50*4000e-6;
float PF_set=1;
float sin_thetas,thetas,cos_thetas,ref_I1d=2.828,ref_I1q,DalphaR,DbetaR,D1R,D2R,D3R,D4R,DalphaR_Udc,DbetaR_Udc;
float sinphi=0,cosphi=0;
float rom_in=8,UId,UIq,Udcd,tiaozhidu=0.95,tiaozhidu_derta,fq;
int MODE=0;
uint16_t  sinData[400];
