#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float offset_count;
    float gain;
    uint16_t raw_min;
    uint16_t raw_max;
} ChannelCalibration;

typedef struct
{
    ChannelCalibration iin;
    ChannelCalibration vin;
    ChannelCalibration ia;
    ChannelCalibration ib;
    ChannelCalibration vab;
    ChannelCalibration vbc;
    ChannelCalibration vdc;
} MeasurementCalibration;

/**
 * @brief 检查七个 ADC 通道的偏置、增益和原始码有效范围。
 * @param[in] cal 测量标定表。
 * @return 所有通道参数有限、增益非零且 raw_min 小于 raw_max 时返回 true。
 */
bool Measurement_CalibrationValid(const MeasurementCalibration *cal);

/**
 * @brief 检查逆变测试所需的 Ia、Ib、Vab、Vbc 四个通道标定。
 * @param[in] cal 测量标定表。
 * @return 四个逆变相关通道均有效时返回 true；Iin/Vin/Vdc 不参与判断。
 */
bool Measurement_InverterCalibrationValid(
    const MeasurementCalibration *cal);

/**
 * @brief 检查整流所需的 Iin、Vin、Vdc 三个通道标定。
 * @param[in] cal 测量标定表。
 * @return 三个整流相关通道均有效时返回 true；三相输出通道不参与判断。
 */
bool Measurement_RectifierCalibrationValid(
    const MeasurementCalibration *cal);

/**
 * @brief 解包四个 ADC 双重规则同步 CDR 值并执行 Vin 冗余一致性检查。
 * @param[in] pairs 四个 rank；每项低 16 位为 ADC1，高 16 位为 ADC2，
 *            映射依次为 Iin/Vin、Ib/Ia、Vab/Vbc、Vdc/Vin诊断。
 * @param[in] sequence 当前 DMA 完整帧序号。
 * @param[in] vin_tolerance_counts 两路 Vin 原始码允许的最大绝对差值。
 * @param[out] out 解包后的原始采样帧。
 * @return 指针有效且两路 Vin 差值不超过容差时返回 true。
 * @note 即使一致性检查失败，out 仍包含本帧数据，但 valid 为 false。
 */
bool Measurement_UnpackDualAdcPairs(const uint32_t pairs[4],
                                    uint32_t sequence,
                                    uint16_t vin_tolerance_counts,
                                    AdcRawFrame *out);

/**
 * @brief Unpack the three channels required by rectifier-only MODE2.
 * @param[in] pairs Four packed dual-ADC CDR rank values.
 * @param[in] sequence Complete DMA frame sequence number.
 * @param[out] out Unpacked raw frame.
 * @return true when the pointers are valid.
 * @note Uses PA5/Iin, ADC2 rank-1 PC5/Vin and PC4/Vdc. The repeated
 *       rank-4 PC5 diagnostic is retained in the frame but does not gate
 *       MODE2 validity; MODE0 continues to use the strict dual check.
 */
bool Measurement_UnpackRectifierAdcPairs(const uint32_t pairs[4],
                                         uint32_t sequence,
                                         AdcRawFrame *out);

/**
 * @brief 按现有 ADC rank 映射解包逆变测试采样帧。
 * @param[in] pairs 四个 ADC CDR 打包值。
 * @param[in] sequence DMA 完整帧序号。
 * @param[out] out 解包后的原始采样帧。
 * @return 指针有效时返回 true。
 * @note 仅提取 PC0/PC1/PC2/PC3 的 Ib/Ia/Vab/Vbc；其余 rank 被忽略。
 */
bool Measurement_UnpackInverterAdcPairs(const uint32_t pairs[4],
                                        uint32_t sequence,
                                        AdcRawFrame *out);

/**
 * @brief 将 ADC 原始码按通道标定转换为物理量并重构缺失的第三相。
 * @param[in] cal 通道标定表，换算关系为 (raw - offset_count) * gain。
 * @param[in] raw 有效的原始 ADC 帧。
 * @param[out] out 电流单位 A、电压单位 V 的完整测量结果。
 * @return 所有通道原始码在范围内且换算结果有限时返回 true。
 * @note 三线制下按 ia+ib+ic=0 和 vab+vbc+vca=0 重构第三相。
 */
bool Measurement_Convert(const MeasurementCalibration *cal,
                         const AdcRawFrame *raw,
                         Measurements *out);

/**
 * @brief 仅换算逆变测试所需的四个通道并重构第三相。
 * @param[in] cal 通道标定表。
 * @param[in] raw 有效的原始 ADC 帧。
 * @param[out] out 完整三相测量结果；Iin/Vin/Vdc 固定为 0。
 * @return 四个逆变通道换算及三相重构均有效时返回 true。
 */
bool Measurement_ConvertInverter(const MeasurementCalibration *cal,
                                 const AdcRawFrame *raw,
                                 Measurements *out);

/**
 * @brief 仅换算整流所需的 Iin、Vin、Vdc 三个通道。
 * @param[in] cal 通道标定表。
 * @param[in] raw 有效的原始 ADC 帧。
 * @param[out] out 整流测量结果；三相输出量固定为 0。
 * @return 三个整流通道换算结果均有限且原始码在范围内时返回 true。
 */
bool Measurement_ConvertRectifier(const MeasurementCalibration *cal,
                                  const AdcRawFrame *raw,
                                  Measurements *out);

/**
 * @brief 将三相相电压执行幅值不变 Clarke 变换。
 * @param[in] m 有效的三相测量值。
 * @return αβ 电压，单位 V；输入无效时返回零向量。
 */
AlphaBeta Measurement_VoltageAlphaBeta(const Measurements *m);

/**
 * @brief 将三相电流执行幅值不变 Clarke 变换。
 * @param[in] m 有效的三相测量值。
 * @return αβ 电流，单位 A；输入无效时返回零向量。
 */
AlphaBeta Measurement_CurrentAlphaBeta(const Measurements *m);

#ifdef __cplusplus
}
#endif

#endif
