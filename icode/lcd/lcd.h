/*
 * lcd.h
 *
 *  Created on: Jul 15, 2024
 *      Author: lly
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f4xx.h"

#define LCD12864_CLR_SCLK     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_SCLK_PORT,LCD12864_SCLK_PIN,~LCD12864_SCLK_PIN)
#define LCD12864_SET_SCLK     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_SCLK_PORT,LCD12864_SCLK_PIN,LCD12864_SCLK_PIN)

#define LCD12864_CLR_SDA      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_SDA_PORT,LCD12864_SDA_PIN,~LCD12864_SDA_PIN)
#define LCD12864_SET_SDA      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_SDA_PORT,LCD12864_SDA_PIN,LCD12864_SDA_PIN)

#define LCD12864_CLR_RS       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_RS_PORT,LCD12864_RS_PIN,~LCD12864_RS_PIN)
#define LCD12864_SET_RS       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_RS_PORT,LCD12864_RS_PIN,LCD12864_RS_PIN)

#define LCD12864_CLR_RST      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_RST_PORT,LCD12864_RST_PIN,~LCD12864_RST_PIN)
#define LCD12864_SET_RST      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_RST_PORT,LCD12864_RST_PIN,LCD12864_RST_PIN)

#define LCD12864_CLR_CS       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_CS_PORT,LCD12864_CS_PIN,~LCD12864_CS_PIN)
#define LCD12864_SET_CS       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_CS_PORT,LCD12864_CS_PIN,LCD12864_CS_PIN)


#define LCD12864_CLR_ROM_IN   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_ROM_IN_PORT,LCD12864_ROM_IN_PIN,~LCD12864_ROM_IN_PIN)
#define LCD12864_SET_ROM_IN   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_ROM_IN_PORT,LCD12864_ROM_IN_PIN,LCD12864_ROM_IN_PIN)


#define LCD12864_CLR_ROM_SCK  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_ROM_SCK_PORT,LCD12864_ROM_SCK_PIN,~LCD12864_ROM_SCK_PIN)
#define LCD12864_SET_ROM_SCK  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_ROM_SCK_PORT,LCD12864_ROM_SCK_PIN,LCD12864_ROM_SCK_PIN)

#define LCD12864_CLR_ROM_CS   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
//ROM_GPIOPinWrite(LCD12864_ROM_CS_PORT,LCD12864_ROM_CS_PIN,~LCD12864_ROM_CS_PIN)
#define LCD12864_SET_ROM_CS   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
//ROM_GPIOPinWrite(LCD12864_ROM_CS_PORT,LCD12864_ROM_CS_PIN,LCD12864_ROM_CS_PIN)

#define LCD12864_GET_BIT_ROM_OUT  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)
//ROM_GPIOPinRead(LCD12864_ROM_OUT_PORT,LCD12864_ROM_OUT_PIN)

void lcd12864_init(void);
extern void DispClear();
void DispClear_line(uint8_t x,uint8_t y,uint8_t len);

void Display_128x64(uint8_t dp[1024]);
void Disp_GB2312_string_16x16(uint8_t x,uint8_t y,uint8_t *text,bool style);

void DispChar5x8(uint8_t x,uint8_t y,char str,bool style);
void DispString5x8(uint8_t x,uint8_t y,char *str,bool style);

//  ʾ     ַ
void DispExtern16x16(uint8_t x,uint8_t y,uint8_t *str,bool style);
//  ʾС
void DispNumber5x8(uint8_t page,uint8_t column,uint32_t data,bool style);
void DispNumber8x16(uint8_t page,uint8_t column,uint32_t data,bool style);

void DispFloat5x8(uint8_t x,uint8_t y,float dat,bool style,uint8_t len2);
void DispFloat8x16(uint8_t page,uint8_t column,float dat,bool style,uint8_t len2);

//      ͼ
void display_vertical_line( uint8_t y,uint8_t len,bool style);
void DispLineChart(uint8_t x,uint8_t y,char *data,bool style);

//63
void DispNumber5x8len(uint8_t page,uint8_t column,uint32_t data,bool style,uint8_t len);
void DispFloat5x8len(uint8_t page,uint8_t column,float dat,bool style,uint8_t len1,uint8_t len2);
//  ʱʱ
void delay_cycles(uint32_t cycles);
#endif /* LCD_LCD12864_ROM_ENABLE_H_ */



