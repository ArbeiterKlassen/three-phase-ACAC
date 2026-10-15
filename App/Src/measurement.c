/*
 * ADC 解包、通道标定与三相量重构。
 *
 * 数据流（一次快环走一遍）：
 *
 *   DMA 双缓冲 → 4 个 32 位 CDR 字
 *     → 【解包】按 Rank 拆成 8 个 16 位原始码
 *     → 【标定】每路 (原始码 − 零点) × 每码比例  → 物理量
 *     → 【重构】由两相量补出第三相
 *     → Measurements（供保护与控制使用）
 *
 * 三种模式用的通道不同，所以解包与标定各有三套：
 *   MODE0 全帧（四 Rank，含 Vin 冗余校验）
 *   MODE1 仅 Ib/Ia/Vab/Vbc
 *   MODE2 仅 Iin/Vin/Vdc
 * 分开写不是冗余——它让每个模式只依赖自己真正用到的通道，
 * 单级台架上没接的通道不会误伤模式校验。
 */

#include "measurement.h"

#include <math.h>

/*
 * 单通道标定是否可用：指针非空、零点与增益有限、增益非零、原始码区间有序。
 * 增益为 0 会让整条通道恒输出 0，必须当无效处理。
 */
static bool ChannelCalibration_Valid(const ChannelCalibration *cal)
{
    return (cal != NULL)
        && isfinite(cal->offset_count)
        && isfinite(cal->gain)
        && (cal->gain != 0.0f)
        && (cal->raw_min < cal->raw_max);
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
static bool ChannelCalibration_Apply(const ChannelCalibration *cal,
                                     uint16_t raw,
                                     float *value)
{
    if (!ChannelCalibration_Valid(cal)
        || (value == NULL)
        || (raw < cal->raw_min)
        || (raw > cal->raw_max))
    {
        return false;
    }

    /* 核心换算：物理量 = (原始码 − 零点码) × 每码比例
     * gain 里已经含了传感器极性、互感器变比、运放增益、分压比——
     * 所以换硬件只需要改 app_config.c 里对应通道的三个数，这里不动。 */
    *value = ((float)raw - cal->offset_count) * cal->gain;
    return isfinite(*value);
}

/* 七路全部有效才通过。用于 MODE0 的完整校验 */
bool Measurement_CalibrationValid(const MeasurementCalibration *cal)
{
    return (cal != NULL)
        && ChannelCalibration_Valid(&cal->iin)
        && ChannelCalibration_Valid(&cal->vin)
        && ChannelCalibration_Valid(&cal->ia)
        && ChannelCalibration_Valid(&cal->ib)
        && ChannelCalibration_Valid(&cal->vab)
        && ChannelCalibration_Valid(&cal->vbc)
        && ChannelCalibration_Valid(&cal->vdc);
}

/* 只查逆变四路（Ia/Ib/Vab/Vbc）。Iin/Vin/Vdc 不参与，单测台架没接也不影响 */
bool Measurement_InverterCalibrationValid(
    const MeasurementCalibration *cal)
{
    return (cal != NULL)
        && ChannelCalibration_Valid(&cal->ia)
        && ChannelCalibration_Valid(&cal->ib)
        && ChannelCalibration_Valid(&cal->vab)
        && ChannelCalibration_Valid(&cal->vbc);
}

/* 只查整流三路（Iin/Vin/Vdc）。三相输出通道不参与 */
bool Measurement_RectifierCalibrationValid(
    const MeasurementCalibration *cal)
{
    return (cal != NULL)
        && ChannelCalibration_Valid(&cal->iin)
        && ChannelCalibration_Valid(&cal->vin)
        && ChannelCalibration_Valid(&cal->vdc);
}

/*
 * 把 4 个 CDR 打包字拆成 8 个 16 位原始码。
 *
 * 硬件约定：每个 32 位字低 16 位是 ADC1 的结果、高 16 位是 ADC2 的。
 * 两个 ADC 按相同的 Rank 同步转换，所以同一个字里的两路是同一时刻采的
 * —— 这一点很重要，相位重构的正确性依赖它。
 *
 * Rank 映射（与 .ioc 的 ADC1/ADC2 通道配置一一对应，改硬件要同步改这里）：
 *   Rank1  低Iin(PA5)      高Vin(PC5)
 *   Rank2  低Ib(PC0)       高Ia(PC1)
 *   Rank3  低Vab(PC2)      高Vbc(PC3)
 *   Rank4  低Vdc(PC4)      高Vin诊断(PC5)
 *
 * 注意 Rank2 与 Rank3 的"低/高"顺序：与 Rank1 的 Iin/Vin 相反，
 * 这是跟着实际引脚走的，不是笔误。
 */
static void Measurement_UnpackPairs(const uint32_t pairs[4],
                                    uint32_t sequence,
                                    AdcRawFrame *raw)
{
    raw->iin = (uint16_t)(pairs[0] & 0xFFFFUL);
    raw->vin = (uint16_t)(pairs[0] >> 16);
    /* ADC1/PC0 是 Ib，ADC2/PC1 是 Ia */
    raw->ib = (uint16_t)(pairs[1] & 0xFFFFUL);
    raw->ia = (uint16_t)(pairs[1] >> 16);
    /* ADC1/PC2 是 Uab，ADC2/PC3 是 Ubc */
    raw->vab = (uint16_t)(pairs[2] & 0xFFFFUL);
    raw->vbc = (uint16_t)(pairs[2] >> 16);
    raw->vdc = (uint16_t)(pairs[3] & 0xFFFFUL);
    raw->vin_diag = (uint16_t)(pairs[3] >> 16);
    raw->sequence = sequence;
}

/*
 * MODE0 解包：拆包之后再校验一路冗余。
 *
 * Vin 在 Rank1 与 Rank4 各采了一次（同一引脚 PC5，中间隔了悬空的 PC3）。
 * 两次读数必须接近，否则说明采样保持时间不够或 DMA 帧错位——这类问题
 * 不会让数据"看起来不对"，只会让控制悄悄变差，所以值得专门查。
 *
 * **失败时 out 仍然被填充**（含 sequence），只是 valid=false；
 * 调用方据此走"测量无效"路径而不是拿到垃圾数据。
 */
bool Measurement_UnpackDualAdcPairs(const uint32_t pairs[4],
                                    uint32_t sequence,
                                    uint16_t vin_tolerance_counts,
                                    AdcRawFrame *out)
{
    AdcRawFrame raw = {0};
    uint16_t vin_delta;

    if ((pairs == NULL) || (out == NULL))
    {
        return false;
    }

    Measurement_UnpackPairs(pairs, sequence, &raw);
    /* 两次 Vin 采样的绝对差（无符号减法，先比大小防止回绕） */
    vin_delta = (raw.vin > raw.vin_diag)
        ? (uint16_t)(raw.vin - raw.vin_diag)
        : (uint16_t)(raw.vin_diag - raw.vin);
    raw.valid = vin_delta <= vin_tolerance_counts;
    *out = raw;
    return raw.valid;
}

bool Measurement_UnpackRectifierAdcPairs(const uint32_t pairs[4],
                                         uint32_t sequence,
                                         AdcRawFrame *out)
{
    AdcRawFrame raw = {0};

    if ((pairs == NULL) || (out == NULL))
    {
        return false;
    }

    /*
     * MODE2 uses PA5/Iin, the first PC5/Vin sample and PC4/Vdc.
     * The repeated PC5 rank follows an unused, floating PC3 channel on
     * the modular bench, so it is retained in the DMA frame but is not
     * used as a validity gate. Physical range checks remain in the
     * rectifier conversion and protection paths.
     */
    Measurement_UnpackPairs(pairs, sequence, &raw);
    raw.valid = true;
    *out = raw;
    return true;
}

bool Measurement_UnpackInverterAdcPairs(const uint32_t pairs[4],
                                        uint32_t sequence,
                                        AdcRawFrame *out)
{
    AdcRawFrame raw = {0};

    if ((pairs == NULL) || (out == NULL))
    {
        return false;
    }

    /*
     * Inverter-only hardware uses PC0/PC1 for Ib/Ia and PC2/PC3 for
     * Uab/Ubc. Rank 1 and rank 4 remain in the ADC sequence solely to
     * preserve the existing dual-ADC DMA timing; their values are ignored.
     */
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
static bool Measurement_ReconstructThreePhase(Measurements *m,
                                              uint32_t sequence)
{
    m->ic = -(m->ia + m->ib);
    m->vca = -(m->vab + m->vbc);
    m->va = (2.0f * m->vab + m->vbc) / 3.0f;
    m->vb = (m->vbc - m->vab) / 3.0f;
    m->vc = -(m->vab + 2.0f * m->vbc) / 3.0f;
    m->sequence = sequence;
    m->valid = isfinite(m->ic) && isfinite(m->vca)
            && isfinite(m->va) && isfinite(m->vb) && isfinite(m->vc);
    return m->valid;
}

/*
 * MODE0 全帧换算：七路都算，然后重构第三相。
 * 任一路越界或换算非有限 → 整体判无效（*out 被清零）。
 * 返回最终 valid。
 */
bool Measurement_Convert(const MeasurementCalibration *cal,
                         const AdcRawFrame *raw,
                         Measurements *out)
{
    Measurements m = {0};
    bool ok;

    if ((cal == NULL) || (raw == NULL) || (out == NULL) || !raw->valid)
    {
        if (out != NULL)
        {
            *out = m;
        }
        return false;
    }

    ok = ChannelCalibration_Apply(&cal->iin, raw->iin, &m.iin)
      && ChannelCalibration_Apply(&cal->vin, raw->vin, &m.vin)
      && ChannelCalibration_Apply(&cal->ia, raw->ia, &m.ia)
      && ChannelCalibration_Apply(&cal->ib, raw->ib, &m.ib)
      && ChannelCalibration_Apply(&cal->vab, raw->vab, &m.vab)
      && ChannelCalibration_Apply(&cal->vbc, raw->vbc, &m.vbc)
      && ChannelCalibration_Apply(&cal->vdc, raw->vdc, &m.vdc);

    if (ok)
    {
        /* 只采了两相电流与两路线电压，第三相在这里补出来 */
        (void)Measurement_ReconstructThreePhase(&m, raw->sequence);
    }

    *out = m;
    return m.valid;
}

/*
 * MODE1 专用换算：只算逆变四路。
 * Iin/Vin/Vdc 显式置 0 —— 由 app_control.c 决定 MODE1 下母线用什么值
 * （默认用用户 SET 值），这里不臆造。
 */
bool Measurement_ConvertInverter(const MeasurementCalibration *cal,
                                 const AdcRawFrame *raw,
                                 Measurements *out)
{
    Measurements m = {0};
    bool ok;

    if ((cal == NULL) || (raw == NULL) || (out == NULL) || !raw->valid)
    {
        if (out != NULL)
        {
            *out = m;
        }
        return false;
    }

    ok = ChannelCalibration_Apply(&cal->ia, raw->ia, &m.ia)
      && ChannelCalibration_Apply(&cal->ib, raw->ib, &m.ib)
      && ChannelCalibration_Apply(&cal->vab, raw->vab, &m.vab)
      && ChannelCalibration_Apply(&cal->vbc, raw->vbc, &m.vbc);

    if (ok)
    {
        m.iin = 0.0f;
        m.vin = 0.0f;
        m.vdc = 0.0f;
        (void)Measurement_ReconstructThreePhase(&m, raw->sequence);
    }

    *out = m;
    return m.valid;
}

/*
 * MODE2 专用换算：只算 Iin/Vin/Vdc，三相输出量保持 0。
 * 单级整流台架上三相通道本来就悬空，不参与有效性判断。
 */
bool Measurement_ConvertRectifier(const MeasurementCalibration *cal,
                                  const AdcRawFrame *raw,
                                  Measurements *out)
{
    Measurements m = {0};
    bool ok;

    if ((cal == NULL) || (raw == NULL) || (out == NULL) || !raw->valid)
    {
        if (out != NULL)
        {
            *out = m;
        }
        return false;
    }

    ok = ChannelCalibration_Apply(&cal->iin, raw->iin, &m.iin)
      && ChannelCalibration_Apply(&cal->vin, raw->vin, &m.vin)
      && ChannelCalibration_Apply(&cal->vdc, raw->vdc, &m.vdc);

    if (ok)
    {
        m.sequence = raw->sequence;
        m.valid = isfinite(m.iin) && isfinite(m.vin) && isfinite(m.vdc);
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
AlphaBeta Measurement_VoltageAlphaBeta(const Measurements *m)
{
    AlphaBeta out = {0};

    if ((m != NULL) && m->valid)
    {
        /* 幅值不变 Clarke 变换，平衡三相正弦量的 αβ 峰值保持不变。 */
        out.alpha = (2.0f / 3.0f) * (m->va - 0.5f * (m->vb + m->vc));
        out.beta = APP_INV_SQRT3_F * (m->vb - m->vc);
    }
    return out;
}

/* 三相电流的幅值不变 Clarke 变换，与电压同式，单位 A */
AlphaBeta Measurement_CurrentAlphaBeta(const Measurements *m)
{
    AlphaBeta out = {0};

    if ((m != NULL) && m->valid)
    {
        /* 与电压环使用相同的幅值不变 Clarke 约定。 */
        out.alpha = (2.0f / 3.0f) * (m->ia - 0.5f * (m->ib + m->ic));
        out.beta = APP_INV_SQRT3_F * (m->ib - m->ic);
    }
    return out;
}
