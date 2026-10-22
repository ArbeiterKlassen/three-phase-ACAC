#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 双 ADC 同步 DMA 的四个 rank，每字低 16 位 ADC1、高 16 位 ADC2。 */
#define ACDC_ADC_RANK_COUNT (4U)

typedef struct {
    float iin_a; // 交流输入电流，有符号 [A]
    float vin_v; // 交流输入电压 [V]
    float vcf_v; // 二极管桥后 Cf 电压 [V]
    float vdc_v; // 母线电压 [V]
    bool valid;  // 四路均在标定窗口且为有限值
    bool iin_overrange;
    bool vin_overrange;
    bool vdc_overrange;
} AcdcMeasurement;

/*
 * 解一帧 ADC DMA：
 *   word0 = Iin / Vin
 *   word1 = 旧 Ib / Ia，本项目不用
 *   word2 = Vcf(PC2) / 旧 Vbc，本项目只用低 16 位
 *   word3 = Vdc / Vin 诊断，本项目只用低 16 位
 * 失败时清空 out，调用者不得把旧值当作新测量。
 */
bool AcdcMeasurement_Convert(const uint32_t frame[ACDC_ADC_RANK_COUNT], AcdcMeasurement *out);
