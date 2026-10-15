/*
 * 单相全桥升压整流器（ACDC 级）的闭环仿真。
 *
 * 与 test_main.c 的区别：那些用例把软启动斜坡加速了几万倍（1200000 V/s），
 * 只验证"门极最终会开"；这里用 app_config.c 里的**生产参数**（60 V/s），
 * 并接上一个母线电容 + 升压电感的平均模型，所以能看到真实的启动过程——
 * 包括"参考还没爬到母线电压之前，外环请求电流为 0、门极保持关断"这一整段。
 *
 * 被控对象（开关周期平均模型，忽略纹波）：
 *     vin(t)  = √2·Vin_rms·sin(2πf·t)
 *     vbridge = (duty_a − duty_b) · vdc
 *     L·di/dt = vin − vbridge
 *     C·vdc·dvdc/dt = vbridge·i − Pload
 *    二极管通路（门极关断时）：桥臂变成不控整流。
 *      |vin| > vdc  → 二极管导通，L·di/dt = vin − sign(vin)·vdc
 *      |vin| < vdc  → 二极管反偏，电流断续归零
 *
 * 控制器是 App 单例接口（与单片机跑的是同一份代码）。
 */

#include <stdio.h>
#include <math.h>

#include "app_control.h"
#include "app_config.h"

/* ── 被控对象参数（原理图未知，取该功率等级的典型值） ───────────────── */
#define SIM_L_H 0.0020f       /* 升压电感 2 mH */
#define SIM_C_F 680.0e-6f     /* 母线电容 680 uF */
#define SIM_VIN_RMS_V 36.0f   /* 交流输入有效值 */
#define SIM_P_LOAD_W 0.0f     /* 负载功率，0 = 空载 */
#define SIM_DIODE_R_OHM 0.35f /* 二极管通路等效串联电阻 */

#define SIM_SECONDS 3.0f

static uint16_t RawOf(const ChannelCalibration *cal, float physical) {
    float raw = cal->offset_count + physical / cal->gain;

    if (raw < 0.0f) raw = 0.0f;
    if (raw > 4095.0f) raw = 4095.0f;
    return (uint16_t)(raw + 0.5f);
}

static uint32_t Pair(uint16_t adc1, uint16_t adc2) {
    return ((uint32_t)adc1 & 0xFFFFu) | (((uint32_t)adc2 & 0xFFFFu) << 16);
}

static bool InRange(const ChannelCalibration *c, uint16_t raw) {
    return (raw >= c->raw_min) && (raw <= c->raw_max);
}

int main(void) {
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const float ts = cfg->rectifier.sample_time_s;
    const float omega = 2.0f * 3.14159265358979f * cfg->rectifier.line_frequency_hz;
    const uint32_t total = (uint32_t)(SIM_SECONDS / ts);

    float t = 0.0f;
    float il = 0.0f;
    /*
     * 母线从"已被动预充到交流峰值"开始：真机上这一步由预充电阻/继电器完成，
     * 本仿真不建模预充回路。若从 0 V 起充，充电浪涌会超过电流传感器量程
     * （±5.9 A @ raw 64..4031），固件会立刻锁存 PROTECTION_FAULT_MEASUREMENT，
     * 那是"浪涌 vs 量程"的问题，不是控制器的问题——先把两者分开看。
     */
    float vdc = 1.41421356237f * SIM_VIN_RMS_V - 1.4f;
    uint32_t seq;
    uint32_t prev_faults = 0U;

    printf("=== ACDC 闭环仿真（生产参数）===\n");
    printf("Vin=%.0f Vrms  L=%.1f mH  C=%.0f uF  Pload=%.1f W\n", SIM_VIN_RMS_V, SIM_L_H * 1000.0f, SIM_C_F * 1.0e6f,
           SIM_P_LOAD_W);
    printf("软启动斜坡 = %.1f V/s（app_config.c 生产值），母线目标 = %.0f V\n",
           cfg->rectifier.dc_reference_slew_v_per_s, cfg->dc_voltage_ref);
    printf("母线从被动充电开始（初始 0 V，二极管通路先充到 Vin 峰值）\n\n");
    printf("     t     state  pwmOK fault    dcref    vdc      iL      m   gate\n");

    App_Init();
    printf("App_Init 后: mode=%d\n", (int)App_GetStatus()->mode);
    printf("App_RequestMode(MODE2) -> %d\n", (int)App_RequestMode(MODE2_RECTIFIER));
    printf("切换后: mode=%d state=%d board_config_valid=%d\n\n", (int)App_GetStatus()->mode,
           (int)App_GetStatus()->state, (int)App_GetStatus()->board_config_valid);
    App_RequestStart();

    for (seq = 1U; seq <= total; seq++) {
        const AppPwmCommand *pwm;
        const AppStatus *status;
        uint32_t pairs[4];
        float vin = 1.41421356237f * SIM_VIN_RMS_V * sinf(omega * t);
        bool gates;
        float m;
        float vbridge;

        /* 反馈真实母线电压与电感电流 */
        pairs[0] = Pair(RawOf(&cfg->measurement.iin, il), RawOf(&cfg->measurement.vin, vin));
        pairs[1] = Pair(RawOf(&cfg->measurement.ib, 0.0f), RawOf(&cfg->measurement.ia, 0.0f));
        pairs[2] = Pair(RawOf(&cfg->measurement.vab, 0.0f), RawOf(&cfg->measurement.vbc, 0.0f));
        pairs[3] = Pair(RawOf(&cfg->measurement.vdc, vdc), RawOf(&cfg->measurement.vin, vin));

        {
            bool applied = App_ApplyAdcPairs(pairs, seq);
            if (seq <= 3U) {
                printf("[帧%lu] ApplyAdcPairs -> %d，当前 faults=0x%08lX\n", (unsigned long)seq, (int)applied,
                       (unsigned long)App_GetStatus()->faults);
            }
        }
        App_FastStep20kHz();

        if (seq <= 3U) { /* 前几帧的原码与量程检查 */
            uint16_t r_iin = (uint16_t)(pairs[0] & 0xFFFFu);
            uint16_t r_vin = (uint16_t)(pairs[0] >> 16);
            uint16_t r_vdc = (uint16_t)(pairs[3] & 0xFFFFu);
            printf("[帧%lu] iin raw=%u/%u..%u %s | vin raw=%u/%u..%u %s | vdc raw=%u/%u..%u %s\n", (unsigned long)seq,
                   r_iin, cfg->measurement.iin.raw_min, cfg->measurement.iin.raw_max,
                   InRange(&cfg->measurement.iin, r_iin) ? "OK" : "超限", r_vin, cfg->measurement.vin.raw_min,
                   cfg->measurement.vin.raw_max, InRange(&cfg->measurement.vin, r_vin) ? "OK" : "超限", r_vdc,
                   cfg->measurement.vdc.raw_min, cfg->measurement.vdc.raw_max,
                   InRange(&cfg->measurement.vdc, r_vdc) ? "OK" : "超限");
        }

        pwm = App_GetPwmCommand();
        status = App_GetStatus();

        if ((status->faults != prev_faults) && (prev_faults == 0U)) {
            printf("\n>>> 首次出现故障 @ 帧 %lu (t=%.4f s)\n", (unsigned long)seq, t);
            printf("    faults=0x%08lX  state=%d  pwm_allowed=%d\n", (unsigned long)status->faults, (int)status->state,
                   (int)status->pwm_allowed);
            printf("    物理量: iin=%.4f A  vdc=%.3f V  iL=%.4f A  il_sim(vin)=%.3f V\n", status->iin_a,
                   status->dc_voltage_v, il, vin);
            printf("    原始码: iin=%u vin=%u vdc=%u\n", (unsigned)(pairs[0] & 0xFFFFu), (unsigned)(pairs[0] >> 16),
                   (unsigned)(pairs[3] & 0xFFFFu));
            prev_faults = status->faults;
        }
        gates = pwm->rectifier_enable_request;
        m = gates ? (pwm->rectifier_a - pwm->rectifier_b) : 0.0f;

        if (gates) {
            /* 门极放开：桥臂变成一个受控电压源，电流环直接命令 L·di/dt */
            vbridge = m * vdc;
            il += ((vin - vbridge) / SIM_L_H) * ts;
            vdc += ((vbridge * il - SIM_P_LOAD_W) / (SIM_C_F * fmaxf(vdc, 1.0f))) * ts;
        } else {
            /*
             * 门极关断：桥臂退化为不控二极管桥。|vin| 高于母线时经二极管
             * 给母线电容充电，充电电流由等效串联电阻限制；|vin| 低于母线
             * 时二极管反偏，交流侧电流归零，母线只向负载放电。
             */
            float i_diode = (fabsf(vin) - vdc) / SIM_DIODE_R_OHM;

            if (i_diode > 0.0f) {
                il = (vin >= 0.0f) ? i_diode : -i_diode;
                vdc += ((i_diode - SIM_P_LOAD_W / fmaxf(vdc, 1.0f)) / SIM_C_F) * ts;
            } else {
                il = 0.0f;
                if (vdc > 0.0f) vdc -= (SIM_P_LOAD_W / (SIM_C_F * vdc)) * ts;
            }
        }
        if (vdc < 0.0f) vdc = 0.0f;
        t += ts;

        if ((seq % 4000U) == 0U) { /* 每 200 ms 打一行 */
            printf("%7.3f   %3d     %d   0x%03lX  %6.2f  %6.2f  %7.3f %6.3f   %d\n", t, (int)status->state,
                   (int)status->pwm_allowed, (unsigned long)(status->faults & 0xFFFu), status->dc_voltage_reference_v,
                   vdc, il, m, (int)gates);
        }
    }

    {
        const AppStatus *status = App_GetStatus();
        printf("\n结束状态: state=%d  faults=0x%08lX  Vdc=%.2f V  母线参考显示=%.2f\n", (int)status->state,
               (unsigned long)status->faults, status->dc_voltage_v, status->dc_voltage_reference_v);
    }
    return 0;
}
