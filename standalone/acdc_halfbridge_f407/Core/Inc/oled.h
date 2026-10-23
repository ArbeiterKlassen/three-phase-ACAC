/*
 * oled.h
 *
 *  Created on: Jun 29, 2025
 *      Author: Administrator
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"
#include "stdlib.h"
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//-----------------OLED端口定义----------------

//#define OLED_SCL_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_0)//SCL
//#define OLED_SCL_Set() GPIO_SetBits(GPIOA,GPIO_Pin_0)

//#define OLED_SDA_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_1)//SDA
//#define OLED_SDA_Set() GPIO_SetBits(GPIOA,GPIO_Pin_1)

//#define OLED_RES_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//RES
//#define OLED_RES_Set() GPIO_SetBits(GPIOA,GPIO_Pin_2)

//#define OLED_DC_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_3)//DC
//#define OLED_DC_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_3)
//
//#define OLED_CS_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_4)//CS
//#define OLED_CS_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_4)
//****************复位*********************

#define OLED_RES_Clr()  HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin, GPIO_PIN_RESET)
#define OLED_RES_Set()  HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin, GPIO_PIN_SET)

//****************数据/命令***************

#define OLED_DC_Clr()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_DC_Set()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin, GPIO_PIN_SET)

//****************片选*********************

#define OLED_CS_Clr()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin, GPIO_PIN_RESET)
#define OLED_CS_Set()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port,OLED_CS_Pin, GPIO_PIN_SET)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ClearPoint(u8 x,u8 y);
void OLED_ColorTurn(u8 i);
void OLED_DisplayTurn(u8 i);
void OLED_WR_Byte(u8 dat,u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_ClearBuffer(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2,u8 mode);
void OLED_DrawCircle(u8 x,u8 y,u8 r);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1,u8 mode);
void OLED_ShowChar6x8(u8 x,u8 y,u8 chr,u8 mode);
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1,u8 mode);
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode);
void OLED_ScrollDisplay(u8 num,u8 space,u8 mode);
void OLED_ShowPicture(u8 x,u8 y,u8 sizex,u8 sizey,u8 BMP[],u8 mode);
void OLED_Init(void);


#endif /* INC_OLED_H_ */
