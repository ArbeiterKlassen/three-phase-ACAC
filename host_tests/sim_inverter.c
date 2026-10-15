/*
 * 三相逆变级（DCAC）的闭环仿真 —— 复现省赛现场"MODE1、母线直入 >60 V、
 * 输出峰值只有 3 V 多"的现象。
 *
 * 被控对象：三相全桥 + LC 滤波器 + 星形阻性负载
 *     van = (duty_a − 0.5)·Vdc                    （相对母线中点）
 *     Lf·di_a/dt = van − vCa − vN,  vN = (vCa+vCb+vCc)/3
 *     Cf·dvCa/dt = i_a − vCa/R
 * 控制器测量的量：vab = vCa − vCb, vbc = vCb − vCc, ia, ib（与板级一致）
 *
 * 用法：sim_inverter.exe [pc4_connected]
 *   默认 1：PC4 接到母线（Vdc 实测正常）
 *   传 0  ：PC4 悬空（MODE1 的 unpack 不填 vdc，实测为 0）
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "app_control.h"
#include "app_config.h"

#define SIM_VBUS_V      61.0f     /* 直入的直流母线 */
#define SIM_LF_H        0.0010f   /* 滤波电感 1 mH */
#define SIM_CF_F        10.0e-6f  /* 滤波电容 10 uF（星形） */
#define SIM_R_LOAD      120.0f    /* 每相负载电阻 */
#define SIM_SECONDS     3.0f

static uint16_t RawOf(const ChannelCalibration *cal, float physical)
{
    float raw = cal->offset_count + physical / cal->gain;

    if (raw < 0.0f) {
        raw = 0.0f;
    }
    if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)(raw + 0.5f);
}

static uint32_t Pair(uint16_t adc1, uint16_t adc2)
{
    return ((uint32_t)adc1 & 0xFFFFu) | (((uint32_t)adc2 & 0xFFFFu) << 16);
}

int main(int argc, char **argv)
{
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const float ts = cfg->inverter.sample_time_s;
    const uint32_t total = (uint32_t)(SIM_SECONDS / ts);
    const int pc4_connected = (argc > 1) ? atoi(argv[1]) : 1;

    float t = 0.0f;
    float ia = 0.0f, ib = 0.0f, ic = 0.0f;
    float vca = 0.0f, vcb = 0.0f, vcc = 0.0f;
    uint32_t seq;

    printf("=== DCAC (MODE1) 闭环仿真 ===\n");
    printf("母线 %.0f V 直入   Lf=%.1f mH  Cf=%.0f uF  R=%.0f ohm/相\n",
           SIM_VBUS_V, SIM_LF_H * 1000.0f, SIM_CF_F * 1e6f, SIM_R_LOAD);
    printf("输出目标: 线电压 %.0f Vrms = %.1f V 峰值\n",
           cfg->output_line_voltage_rms, cfg->output_line_voltage_rms * 1.41421f);
    printf("PC4(Vdc 实测): %s\n\n", pc4_connected ? "接通" : "悬空");
    printf("   t    state pwmOK  |vab|峰值   |vca|峰值   参考相电压rms  gate\n");

    App_Init();
    (void)App_RequestMode(MODE1_INVERTER);
    App_RequestStart();

    {
        float peak_ab = 0.0f, peak_ca = 0.0f;
        uint32_t win = 0U;
        const uint32_t win_len = (uint32_t)(0.20f / ts);

        for (seq = 1U; seq <= total; seq++) {
            const AppPwmCommand *pwm;
            const AppStatus *status;
            uint32_t pairs[4];
            float vab = vca - vcb;
            float vbc = vcb - vcc;
            float vN = (vca + vcb + vcc) / 3.0f;
            float vdc_meas = pc4_connected ? SIM_VBUS_V : 0.0f;
            float da, db, dc;

            pairs[0] = Pair(RawOf(&cfg->measurement.iin, 0.0f),
                            RawOf(&cfg->measurement.vin, 0.0f));
            pairs[1] = Pair(RawOf(&cfg->measurement.ib, ib),
                            RawOf(&cfg->measurement.ia, ia));
            pairs[2] = Pair(RawOf(&cfg->measurement.vab, vab),
                            RawOf(&cfg->measurement.vbc, vbc));
            pairs[3] = Pair(RawOf(&cfg->measurement.vdc, vdc_meas),
                            RawOf(&cfg->measurement.vin, 0.0f));

            (void)App_ApplyAdcPairs(pairs, seq);
            App_FastStep20kHz();

            pwm = App_GetPwmCommand();
            status = App_GetStatus();

            da = pwm->inverter_a; db = pwm->inverter_b; dc = pwm->inverter_c;
            if (!pwm->inverter_enable_request) {
                da = db = dc = 0.5f;      /* 门极关断 → 桥臂输出中点 */
            }

            /* LC 滤波器 + 星形负载 */
            {
                float van = (da - 0.5f) * SIM_VBUS_V;
                float vbn = (db - 0.5f) * SIM_VBUS_V;
                float vcn = (dc - 0.5f) * SIM_VBUS_V;

                ia += ((van - vca - vN) / SIM_LF_H) * ts;
                ib += ((vbn - vcb - vN) / SIM_LF_H) * ts;
                ic += ((vcn - vcc - vN) / SIM_LF_H) * ts;
                vca += ((ia - vca / SIM_R_LOAD) / SIM_CF_F) * ts;
                vcb += ((ib - vcb / SIM_R_LOAD) / SIM_CF_F) * ts;
                vcc += ((ic - vcc / SIM_R_LOAD) / SIM_CF_F) * ts;
            }
            t += ts;

            if (fabsf(vab) > peak_ab) peak_ab = fabsf(vab);
            if (fabsf(vca) > peak_ca) peak_ca = fabsf(vca);
            if (++win >= win_len) {
                printf("%6.2f   %3d    %d    %7.2f     %7.2f      %6.2f        %d\n",
                       t, (int)status->state, (int)status->pwm_allowed,
                       peak_ab, peak_ca, status->output_line_voltage_reference_v,
                       (int)pwm->inverter_enable_request);
                win = 0U; peak_ab = 0.0f; peak_ca = 0.0f;
            }
        }
    }

    {
        const AppStatus *status = App_GetStatus();
        printf("\n结束: state=%d faults=0x%08lX  输出参考=%.2f V(rms 线)\n",
               (int)status->state, (unsigned long)status->faults,
               status->output_line_voltage_reference_v);
    }
    return 0;
}
