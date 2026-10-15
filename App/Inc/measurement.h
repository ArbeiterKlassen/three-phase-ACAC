#pragma once

/*
 * ADC 解包、通道标定与三相量重构（三相逆变工程）。
 *
 * 换算统一为一条式子：物理量 = (原始码 − 零点码) × 每码比例。
 * gain 里已经含了传感器极性、互感器变比、运放增益和分压比，所以
 * **改硬件只需改 app_config.c 里对应通道的三个数**。
 *
 * 板上只采两相电流（Ia/Ib）与两路线电压（Vab/Vbc），第三相按
 * 三线制关系重构：ic = −(ia+ib)、vca = −(vab+vbc)。
 *
 * 本工程只采逆变需要的通道：rank2（PC0/PC1 → Ib/Ia）与
 * rank3（PC2/PC3 → Vab/Vbc）。rank1/rank4 保留在 ADC 序列里只是为了
 * 维持既有的双 ADC + DMA 时序，其数值不参与换算。
 */

#include "app_types.h"

/* 单通道标定。物理量 = (原始码 − offset_count) × gain。 */
typedef struct {
    float offset_count; // 零输入对应的 ADC 原始码
    float gain;         // 每码对应的物理量（含传感器极性与分压比）
    uint16_t raw_min;   // 合法原始码下限；超出判"测量无效"
    uint16_t raw_max;   // 合法原始码上限
} ChannelCalibration;

/* 通道标定表：三相电流/线电压 + 母线电压。 */
typedef struct {
    ChannelCalibration ia;  // 三相 A 相电流 [A]
    ChannelCalibration ib;  // 三相 B 相电流 [A]
    ChannelCalibration vab; // 线电压 Vab [V]
    ChannelCalibration vbc; // 线电压 Vbc [V]
    ChannelCalibration vdc; // 直流母线电压 [V]
} MeasurementCalibration;

// 校验全部通道的零点、增益与原始码范围是否都合法
bool Measurement_CalibrationValid(const MeasurementCalibration *cal);

// 只校验本工程实际用到的 Ia/Ib/Vab/Vbc 四路（Vdc 通道暂未参与换算）
bool Measurement_InverterCalibrationValid(const MeasurementCalibration *cal);

// 解包一帧：只取 PC0/PC1 的 Ib/Ia 与 PC2/PC3 的 Vab/Vbc，其余 Rank 忽略
// **注意：本模式不填 raw.vdc**，Vdc 由 app_control.c 用用户 SET 值替代
// 返回指针有效
bool Measurement_UnpackInverterAdcPairs(const uint32_t pairs[4], uint32_t sequence, AdcRawFrame *out);

// 换算四路（Ia/Ib/Vab/Vbc）并重构第三相；Vdc 输出固定为 0
// （调制用的母线值由 app_control.c 决定，见那里的说明）
bool Measurement_ConvertInverter(const MeasurementCalibration *cal, const AdcRawFrame *raw, Measurements *out);

// 三相相电压的幅值不变 Clarke 变换，返回 αβ 电压 [V]；输入无效返回零向量
AlphaBeta Measurement_VoltageAlphaBeta(const Measurements *m);

// 三相电流的幅值不变 Clarke 变换，返回 αβ 电流 [A]；输入无效返回零向量
AlphaBeta Measurement_CurrentAlphaBeta(const Measurements *m);
