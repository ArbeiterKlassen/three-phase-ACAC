#pragma once

#include "acdc_application.h"

/*
 * 板级适配层：只负责 ADC/DMA、TIM8 门极和按键/灯，不实现控制算法。
 * 所有控制计算在 AcdcApp_FastStep()，便于在 PC 上做单元测试。
 */

void AcdcBoard_Init(void);
void AcdcBoard_BackgroundTask(void);
void AcdcBoard_EmergencyStop(void);
void AcdcBoard_GetSnapshot(AcdcSnapshot *snapshot);
