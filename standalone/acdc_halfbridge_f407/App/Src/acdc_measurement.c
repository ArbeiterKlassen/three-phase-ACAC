#include "acdc_measurement.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "acdc_config.h"

/*
 * 2025 现场已标定的通道直接复用：
 *   PA5 ADC1_IN5  = Iin，TA ±5.9 A 量程；
 *   PC4 ADC1_IN14 = Vdc，单极性 0~67.9 V。
 *
 * Vcf 只使用 PC5（ADC2_IN15，原 ACDC Vin 位置）这一路，前端改为
 * 高压端 48.4 kΩ、ADC 节点对地 1.9 kΩ 的电阻分压：
 *   ratio = 1.9 / (1.9 + 48.4) = 0.037773...
 *   gain  = (3.3 / 4095) / ratio = 0.021334 V/码
 * ADC 引脚达到 3.3 V 时对应约 87.36 V；为避免 ADC 两端非线性，窗口取
 * 0..4031 码，对应约 0..86.0 V。
 */
const AcdcCalibration g_acdc_cal_iin = {
    .offset_count = 2084.7273f,
    .gain = 0.002919044f,
    .raw_min = 64U,
    .raw_max = 4031U,
};

const AcdcCalibration g_acdc_cal_vcf = {
    .offset_count = 0.0f,
    .gain = ACDC_VCF_GAIN_V_PER_CODE,
    .raw_min = 0U,
    .raw_max = 4031U,
};

const AcdcCalibration g_acdc_cal_vdc = {
    .offset_count = 4.5388f,
    .gain = 0.016861944f,
    .raw_min = 0U,
    .raw_max = 4031U,
};

/* 先检查原始码窗口，再线性换算；任何一步失败都返回 false。 */
static bool AcdcMeasurement_ConvertChannel(const AcdcCalibration *cal, uint16_t raw, float *physical) {
    float value;

    if ((cal == NULL) || (physical == NULL) || (raw < cal->raw_min) || (raw > cal->raw_max)) return false;
    value = ((float)raw - cal->offset_count) * cal->gain;
    if (!isfinite(value)) return false;
    *physical = value;
    return true;
}

bool AcdcMeasurement_Convert(const uint32_t frame[ACDC_ADC_RANK_COUNT], AcdcMeasurement *out) {
    uint16_t raw_iin;
    uint16_t raw_vcf;
    uint16_t raw_vdc;

    if ((frame == NULL) || (out == NULL)) return false;
    memset(out, 0, sizeof(*out));

    raw_iin = (uint16_t)(frame[0] & 0xFFFFU);
    raw_vcf = (uint16_t)((frame[0] >> 16U) & 0xFFFFU);
    raw_vdc = (uint16_t)(frame[3] & 0xFFFFU);
    out->iin_overrange = (raw_iin < g_acdc_cal_iin.raw_min) || (raw_iin > g_acdc_cal_iin.raw_max);
    out->vcf_overrange = (raw_vcf < g_acdc_cal_vcf.raw_min) || (raw_vcf > g_acdc_cal_vcf.raw_max);
    out->vdc_overrange = (raw_vdc < g_acdc_cal_vdc.raw_min) || (raw_vdc > g_acdc_cal_vdc.raw_max);

    if (!AcdcMeasurement_ConvertChannel(&g_acdc_cal_iin, raw_iin, &out->iin_a) ||
        !AcdcMeasurement_ConvertChannel(&g_acdc_cal_vcf, raw_vcf, &out->vcf_v) ||
        !AcdcMeasurement_ConvertChannel(&g_acdc_cal_vdc, raw_vdc, &out->vdc_v)) {
        const bool iin_overrange = out->iin_overrange;
        const bool vcf_overrange = out->vcf_overrange;
        const bool vdc_overrange = out->vdc_overrange;
        memset(out, 0, sizeof(*out));
        out->iin_overrange = iin_overrange;
        out->vcf_overrange = vcf_overrange;
        out->vdc_overrange = vdc_overrange;
        return false;
    }

    /* Cf 在正常桥式 Boost 中不应为负；轻微负偏置按 0 V 处理。 */
    if (out->vcf_v < 0.0f) out->vcf_v = 0.0f;
    if (out->vdc_v < 0.0f) out->vdc_v = 0.0f;
    out->valid = isfinite(out->iin_a) && isfinite(out->vcf_v) && isfinite(out->vdc_v);
    return out->valid;
}
