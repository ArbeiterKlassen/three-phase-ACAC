#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 双 ADC 同步 DMA 的四个 rank，每字低 16 位 ADC1、高 16 位 ADC2。 */
#define ACDC_ADC_RANK_COUNT (4U)

typedef struct {
    float iin_a; // 交流输入电流，有符号 [A]
    float vcf_v; // 二极管桥后 Cf 电压，也是唯一整流电压反馈 [V]
    float vdc_v; // 母线电压 [V]
    bool valid;  // 所需三路均在标定窗口且为有限值
    bool iin_overrange;
    bool vcf_overrange;
    bool vdc_overrange;
} AcdcMeasurement;

/*
 * 解一帧 ADC DMA：
 *   word0 = Iin / Vcf(PC5，1.9k/48.4k 分压)
 *   word1 = 旧 Ib / Ia，本项目不用
 *   word2 = 原 Vab(PC2) / 旧 Vbc，留给最终 ACAC 工程，本 ACDC 控制不用
 *   word3 = Vdc(PC4) / Vcf 诊断，本项目只用低 16 位
 * 失败时清空 out，调用者不得把旧值当作新测量。
 */
bool AcdcMeasurement_Convert(const uint32_t frame[ACDC_ADC_RANK_COUNT], AcdcMeasurement *out);
