/*
 * begin_dcac.h
 *
 *  Created on: Jun 26, 2025
 *      Author: lly
 */

#ifndef BEGIN_DCAC_BEGIN_DCAC_H_
#define BEGIN_DCAC_BEGIN_DCAC_H_
void begin_dcac();
void pidcac();
void kongjianshiliangnibian();
void spwm_nibian();
void inv_Clarke_F(float Ualpha, float Ubeta, float *w1, float *w2, float *w3);
extern float boa,bob,boc,diao,diaoderta;
#endif /* BEGIN_DCAC_BEGIN_DCAC_H_ */
