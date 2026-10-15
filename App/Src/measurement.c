/*
 * ADC 解包、通道标定与三相量重构（三相逆变工程）。
 *
 * 数据流（一次快环走一遍）：
 *
 *   DMA 双缓冲 → 4 个 32 位 CDR 字
 *     → 【解包】取出本工程要用的 4 路原始码（Ib/Ia/Vab/Vbc）
 *     → 【标定】每路 (原始码 − 零点) × 每码比例  → 物理量
 *     → 【重构】由两相量补出第三相
 *     → Measurements（供保护与控制使用）
 */

#include "measurement.h"

#include <math.h>

/*
 * 单通道标定是否可用：指针非空、零点与增益有限、增益非零、原始码区间有序。
 * 增益为 0 会让整条通道恒输出 0，必须当无效处理。
 */
static bool ChannelCalibration_Valid(const ChannelCalibration *cal) {
    return (cal != NULL) && isfinite(cal->offset_count) && isfinite(cal->gain) && (cal->gain != 0.0f) &&
           (cal->raw_min < cal->raw_max);
}

/*
 * 把原始码换算成物理量，并做量程守卫。
 *
 * 这是全工程唯一的"原始码 → 物理量"入口。守卫分两层：
 *   ① 原始码必须落在 [raw_min, raw_max] —— 越界直接判无效
 *   ② 换算结果必须有限
 *
 * **量程越界为什么不当成"读数很大"而是判无效**：raw_min/raw_max 划的是
 * 传感器可信区间。超出说明要么传感器坏了、要么电流超过了量程，
 * 这两种情况下读数的具体数值都不可信，所以宁可标记无效交保护处理。
 *
 * 返回换算是否成功；失败时不写 *value，由调用方保证输出被清零。
 */
static bool ChannelCalibration_Apply(const ChannelCalibration *cal, uint16_t raw, float *value) {
    if (!ChannelCalibration_Valid(cal) || (value == NULL) || (raw < cal->raw_min) || (raw > cal->raw_max)) return false;

    /* 核心换算：物理量 = (原始码 − 零点码) × 每码比例
     * gain 里已经含了传感器极性、互感器变比、运放增益、分压比——
     * 所以换硬件只需要改 app_config.c 里对应通道的三个数，这里不动。 */
    *value = ((float)raw - cal->offset_count) * cal->gain;
    return isfinite(*value);
}

/* 全部通道有效才通过（含暂未参与换算的 Vdc）。用在整体配置校验里 */
bool Measurement_CalibrationValid(const MeasurementCalibration *cal) {
    return (cal != NULL) && ChannelCalibration_Valid(&cal->ia) && ChannelCalibration_Valid(&cal->ib) &&
           ChannelCalibration_Valid(&cal->vab) && ChannelCalibration_Valid(&cal->vbc) &&
           ChannelCalibration_Valid(&cal->vdc);
}

/* 只查本工程实际用到的四路（Ia/Ib/Vab/Vbc） */
bool Measurement_InverterCalibrationValid(const MeasurementCalibration *cal) {
    return (cal != NULL) && ChannelCalibration_Valid(&cal->ia) && ChannelCalibration_Valid(&cal->ib) &&
           ChannelCalibration_Valid(&cal->vab) && ChannelCalibration_Valid(&cal->vbc);
}

/*
 * 解包一帧：只取逆变需要的两路。
 *
 * 硬件约定：每个 32 位 CDR 字低 16 位是 ADC1 的结果、高 16 位是 ADC2 的；
 * 两个 ADC 按相同 Rank 同步转换，所以同一个字里的两路是同一时刻采的
 * —— 相位重构的正确性依赖这一点。
 *
 * Rank 映射（与 .ioc 的 ADC 通道配置一一对应，改硬件要同步改这里）：
 *   Rank1  低Iin(PA5)  高Vin(PC5)      ← 本工程不使用
 *   Rank2  低Ib(PC0)   高Ia(PC1)       ← 使用
 *   Rank3  低Vab(PC2)  高Vbc(PC3)      ← 使用
 *   Rank4  低Vdc(PC4)  高Vin诊断(PC5)  ← 本工程不使用
 *
 * Rank2 与 Rank3 的"低/高"顺序（Ib 在前、Ia 在后）是跟着实际引脚走的，
 * 不是笔误。rank1/rank4 保留在 DMA 序列里是为了维持既有时序。
 */
bool Measurement_UnpackInverterAdcPairs(const uint32_t pairs[4], uint32_t sequence, AdcRawFrame *out) {
    AdcRawFrame raw = {0};

    if ((pairs == NULL) || (out == NULL)) return false;

    /* ADC1/PC0 是 Ib，ADC2/PC1 是 Ia；ADC1/PC2 是 Uab，ADC2/PC3 是 Ubc */
    raw.ib = (uint16_t)(pairs[1] & 0xFFFFUL);
    raw.ia = (uint16_t)(pairs[1] >> 16);
    raw.vab = (uint16_t)(pairs[2] & 0xFFFFUL);
    raw.vbc = (uint16_t)(pairs[2] >> 16);
    raw.sequence = sequence;
    raw.valid = true;
    *out = raw;
    return true;
}

/*
 * 由两相量补出第三相，并把线电压换算成相电压。
 *
 * 电流用三线制零和约束：ia + ib + ic = 0。
 * 电压同理：vab + vbc + vca = 0。
 *
 * 相电压由线电压反解（三相平衡时）：
 *   va = (2·vab + vbc) / 3
 *   vb = ( vbc − vab) / 3
 *   vc = −(vab + 2·vbc) / 3
 * 这三个式子来自 vab = va − vb、vbc = vb − vc 加上 va+vb+vc = 0 联立。
 * 注意它们假定三相平衡——不平衡负载下会有误差，这是采两路线电压的固有代价。
 */
static bool Measurement_ReconstructThreePhase(Measurements *m, uint32_t sequence) {
    m->ic = -(m->ia + m->ib);
    m->vca = -(m->vab + m->vbc);
    m->va = (2.0f * m->vab + m->vbc) / 3.0f;
    m->vb = (m->vbc - m->vab) / 3.0f;
    m->vc = -(m->vab + 2.0f * m->vbc) / 3.0f;
    m->sequence = sequence;
    m->valid = isfinite(m->ic) && isfinite(m->vca) && isfinite(m->va) && isfinite(m->vb) && isfinite(m->vc);
    return m->valid;
}

/*
 * 换算四路并重构第三相。
 *
 * Vdc 显式置 0 —— 母线通道（PC4）在本工程的测试台上没有接入，
 * 调制用的母线值由 app_control.c 用用户 SET 值提供（那里有完整说明），
 * 这里不臆造一个值出来。
 *
 * 任一路原始码越界或换算非有限 → 整体判无效（*out 被清零）。
 */
bool Measurement_ConvertInverter(const MeasurementCalibration *cal, const AdcRawFrame *raw, Measurements *out) {
    Measurements m = {0};
    bool ok;

    if ((cal == NULL) || (raw == NULL) || (out == NULL) || !raw->valid) {
        if (out != NULL) *out = m;
        return false;
    }

    ok = ChannelCalibration_Apply(&cal->ia, raw->ia, &m.ia) && ChannelCalibration_Apply(&cal->ib, raw->ib, &m.ib) &&
         ChannelCalibration_Apply(&cal->vab, raw->vab, &m.vab) && ChannelCalibration_Apply(&cal->vbc, raw->vbc, &m.vbc);

    if (ok) {
        m.vdc = 0.0f;
        (void)Measurement_ReconstructThreePhase(&m, raw->sequence);
    }

    *out = m;
    return m.valid;
}

/*
 * 三相相电压的幅值不变 Clarke 变换。
 *   α = (2/3)·(va − ½(vb + vc))
 *   β = (1/√3)·(vb − vc)
 * 系数取 2/3 与 1/√3 是"幅值不变"约定：平衡三相量的 αβ 峰值等于原来
 * 的相电压峰值。用等功率约定的话系数不同，两者不能混用。
 * 输入无效时返回零向量。
 */
AlphaBeta Measurement_VoltageAlphaBeta(const Measurements *m) {
    AlphaBeta out = {0};

    if ((m != NULL) && m->valid) {
        /* 幅值不变 Clarke 变换，平衡三相正弦量的 αβ 峰值保持不变。 */
        out.alpha = (2.0f / 3.0f) * (m->va - 0.5f * (m->vb + m->vc));
        out.beta = APP_1DIVSQRT3_F * (m->vb - m->vc);
    }
    return out;
}

/* 三相电流的幅值不变 Clarke 变换，与电压同式，单位 A */
AlphaBeta Measurement_CurrentAlphaBeta(const Measurements *m) {
    AlphaBeta out = {0};

    if ((m != NULL) && m->valid) {
        /* 与电压环使用相同的幅值不变 Clarke 约定。 */
        out.alpha = (2.0f / 3.0f) * (m->ia - 0.5f * (m->ib + m->ic));
        out.beta = APP_1DIVSQRT3_F * (m->ib - m->ic);
    }
    return out;
}
