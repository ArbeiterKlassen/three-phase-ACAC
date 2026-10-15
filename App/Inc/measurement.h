#pragma once

/*
 * ADC 解包、通道标定与三相量重构。
 *
 * 换算统一为一条式子：物理量 = (原始码 − 零点码) × 每码比例。
 * gain 里已经含了传感器极性、互感器变比、运放增益和分压比，所以
 * **改硬件只需改 app_config.c 里对应通道的三个数**。
 *
 * 板上只采两相电流（Ia/Ib）与两路线电压（Vab/Vbc），第三相按
 * 三线制关系重构：ic = −(ia+ib)、vca = −(vab+vbc)。
 *
 * 三种模式用的通道不同，所以解包与换算各有一套：
 *   MODE0 全帧（四 Rank、含 Vin 冗余校验）
 *   MODE1 仅 Ib/Ia/Vab/Vbc
 *   MODE2 仅 Iin/Vin/Vdc
 */

#include "app_types.h"

/* 单通道标定。物理量 = (原始码 − offset_count) × gain。 */
typedef struct
{
    float offset_count;   // 零输入对应的 ADC 原始码
    float gain;           // 每码对应的物理量（含传感器极性与分压比）
    uint16_t raw_min;     // 合法原始码下限；超出判"测量无效"
    uint16_t raw_max;     // 合法原始码上限
} ChannelCalibration;

/* 七路通道标定表。前三路供整流级，中间四路供逆变级，vdc 两级共用。 */
typedef struct
{
    ChannelCalibration iin;   // 整流输入电流 [A]
    ChannelCalibration vin;   // 整流输入电压 [V]
    ChannelCalibration ia;    // 三相 A 相电流 [A]
    ChannelCalibration ib;    // 三相 B 相电流 [A]
    ChannelCalibration vab;   // 线电压 Vab [V]
    ChannelCalibration vbc;   // 线电压 Vbc [V]
    ChannelCalibration vdc;   // 直流母线电压 [V]
} MeasurementCalibration;

// 校验七路通道的零点、增益与原始码范围是否都合法
bool Measurement_CalibrationValid(const MeasurementCalibration *cal);

// 只校验逆变用到的 Ia/Ib/Vab/Vbc 四路（MODE1 用；Iin/Vin/Vdc 不参与）
bool Measurement_InverterCalibrationValid(const MeasurementCalibration *cal);

// 只校验整流用到的 Iin/Vin/Vdc 三路（MODE2 用；三相通道不参与）
bool Measurement_RectifierCalibrationValid(const MeasurementCalibration *cal);

// 解包四个双 ADC 的 CDR 字，并做 Vin 冗余一致性检查
// pairs 每项低 16 位为 ADC1、高 16 位为 ADC2，映射依次为：
//   Iin/Vin、Ib/Ia、Vab/Vbc、Vdc/Vin诊断
// 返回两路 Vin 差异未超容差；**失败时 out 仍是本帧数据但 valid=false**
bool Measurement_UnpackDualAdcPairs(const uint32_t pairs[4],
                                    uint32_t sequence,
                                    uint16_t vin_tolerance_counts,
                                    AdcRawFrame *out);

// MODE2 专用解包：只取 PA5/Iin、ADC2 Rank1 的 PC5/Vin 和 PC4/Vdc
// Rank4 那路 PC5 重复采样在单级台架上紧跟悬空通道，故**不用它做有效性门控**
// 返回指针有效
bool Measurement_UnpackRectifierAdcPairs(const uint32_t pairs[4],
                                         uint32_t sequence,
                                         AdcRawFrame *out);

// MODE1 专用解包：只取 PC0/PC1 的 Ib/Ia 与 PC2/PC3 的 Vab/Vbc，其余 Rank 忽略
// **注意：本模式不填 raw.vdc**，Vdc 由 app_control.c 用用户 SET 值替代
// 返回指针有效
bool Measurement_UnpackInverterAdcPairs(const uint32_t pairs[4],
                                        uint32_t sequence,
                                        AdcRawFrame *out);

// 全帧换算：七路都换算，并重构第三相
// 任一通道原始码越界或换算结果非有限值时返回 false
bool Measurement_Convert(const MeasurementCalibration *cal,
                         const AdcRawFrame *raw,
                         Measurements *out);

// 仅换算逆变四路并重构第三相；Iin/Vin/Vdc 输出固定为 0
bool Measurement_ConvertInverter(const MeasurementCalibration *cal,
                                 const AdcRawFrame *raw,
                                 Measurements *out);

// 仅换算整流三路；三相输出量固定为 0
bool Measurement_ConvertRectifier(const MeasurementCalibration *cal,
                                  const AdcRawFrame *raw,
                                  Measurements *out);

// 三相相电压的幅值不变 Clarke 变换，返回 αβ 电压 [V]；输入无效返回零向量
AlphaBeta Measurement_VoltageAlphaBeta(const Measurements *m);

// 三相电流的幅值不变 Clarke 变换，返回 αβ 电流 [A]；输入无效返回零向量
AlphaBeta Measurement_CurrentAlphaBeta(const Measurements *m);
