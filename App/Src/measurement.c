#include "measurement.h"

#include <math.h>

static bool ChannelCalibration_Valid(const ChannelCalibration *cal)
{
    return (cal != NULL)
        && isfinite(cal->offset_count)
        && isfinite(cal->gain)
        && (cal->gain != 0.0f)
        && (cal->raw_min < cal->raw_max);
}

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

    /* gain 自带传感器极性和“每计数对应物理量”的比例。 */
    *value = ((float)raw - cal->offset_count) * cal->gain;
    return isfinite(*value);
}

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

bool Measurement_InverterCalibrationValid(
    const MeasurementCalibration *cal)
{
    return (cal != NULL)
        && ChannelCalibration_Valid(&cal->ia)
        && ChannelCalibration_Valid(&cal->ib)
        && ChannelCalibration_Valid(&cal->vab)
        && ChannelCalibration_Valid(&cal->vbc);
}

bool Measurement_RectifierCalibrationValid(
    const MeasurementCalibration *cal)
{
    return (cal != NULL)
        && ChannelCalibration_Valid(&cal->iin)
        && ChannelCalibration_Valid(&cal->vin)
        && ChannelCalibration_Valid(&cal->vdc);
}

static void Measurement_UnpackPairs(const uint32_t pairs[4],
                                    uint32_t sequence,
                                    AdcRawFrame *raw)
{
    raw->iin = (uint16_t)(pairs[0] & 0xFFFFUL);
    raw->vin = (uint16_t)(pairs[0] >> 16);
    /* ADC1/PC0 is Ib; ADC2/PC1 is Ia. */
    raw->ib = (uint16_t)(pairs[1] & 0xFFFFUL);
    raw->ia = (uint16_t)(pairs[1] >> 16);
    /* ADC1/PC2 is Uab; ADC2/PC3 is Ubc. */
    raw->vab = (uint16_t)(pairs[2] & 0xFFFFUL);
    raw->vbc = (uint16_t)(pairs[2] >> 16);
    raw->vdc = (uint16_t)(pairs[3] & 0xFFFFUL);
    raw->vin_diag = (uint16_t)(pairs[3] >> 16);
    raw->sequence = sequence;
}

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
        /* 仅采两相量，第三相由三线制零和约束重构。 */
        (void)Measurement_ReconstructThreePhase(&m, raw->sequence);
    }

    *out = m;
    return m.valid;
}

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
