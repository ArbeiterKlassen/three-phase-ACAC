/*
 * 单相 AC-DC 降级拓扑仿真：
 *
 *   AC ─► 4×肖特基全桥 ─► Lf ─► 节点 Cf ─► Lb ─► 单半桥 ─► Cbus ─► 负载
 *                                  │                    │
 *                                  └─ Cf 下端接回流       └─ 上管/下管互补
 *
 * 这里仿真的是“只有一个半桥模块”时能搭出的二极管整流 + Boost：
 *   - 四只二极管负责 AC→DC 极性翻转；
 *   - Lf/Cf 是桥后低通，Cf 同时是 Boost 的输入储能电容；
 *   - 半桥的下管把能量存进 Lb，上管（或上管体二极管）把能量送到 Cbus；
 *   - 半桥上下管互补，死区、开关损耗和 EMI 不进入这个平均模型。
 *
 * 为什么必须闭环：
 *   Cbus 有电，但输入侧还会受到二极管换流、LC 谐振和负载扰动。
 *   如果直接固定占空比，母线会随输入峰值、负载和元件误差漂移，不能作为
 *   “稳定 62 V 母线”的方案。这里的闭环不是 Decorator，而是把实际
 *   RectifierControl 的慢/快双环思路换成适合单半桥的平均电流控制：
 *
 *     1 kHz 母线 PI + 100 Hz notch ─► 输入电流 RMS 指令 Iamp
 *     20 kHz 电流 PI             ─► Lb 电压指令 vL
 *     d = 1 − (vCf − vL)/Vbus    ─► 下管占空比，上管 1−d
 *
 * 重要限制：
 *   - 这是开关周期平均模型；2 µs 只用于解 LC/控制动态，不代表真的仿真了
 *     20 kHz 的每个开关边沿；
 *   - 二极管采用恒压降模型，每只 0.45 V，整流通路按两只串联计；
 *   - 电感只有 DCR，没有磁芯损耗；半桥没有开关/导通损耗；
 *   - 默认电流上限 3.8 A RMS 来自旧固件的 TA 通道；该上限必须和硬件
 *     实测量程一起核对，不能为了“让仿真通过”随手调大。
 *
 * 编译：
 *   gcc -std=c11 -O2 -Wall -Wextra -I App\Inc ^
 *       host_tests\sim_acdc_halfbridge.c -lm -o sim_acdc_halfbridge.exe
 *
 * 运行：
 *   sim_acdc_halfbridge.exe
 *   sim_acdc_halfbridge.exe 36 1.8 10 2
 *   sim_acdc_halfbridge.exe 36 1.8 10 2 --ilimit=3.8 --csv=wave.csv
 *
 * 位置参数：
 *   [Vac_rms] [Iload_A] [Cf_uF] [Lf_mH]
 *
 * 可选参数：
 *   --lb-mh=...       Boost 电感 [mH]，默认 1
 *   --cbus-uf=...     母线电容 [uF]，默认 1000
 *   --ilimit=...      输入电流 RMS 上限 [A]，默认 3.8
 *   --vref=...        母线目标 [V]，默认 62
 *   --duration=...    仿真终止时间 [s]，默认 3.9
 *   --stats-start=... 统计窗口起点 [s]，默认 3.5
 *   --trace           打印一个工频周期的控制量
 *   --trace-start=... 跟踪打印起点 [s]，默认 3.52
 *   --csv=文件名      输出统计窗口内的控制/功率波形
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_F 3.14159265358979323846
#define SQRT2_F 1.41421356237309504880

/* ── 控制频率与仿真步长 ────────────────────────────────────────────── */
#define DT_S 2.0e-6
#define OUTER_DIVIDER 500U      /* 1 kHz */
#define INNER_DIVIDER 25U       /* 20 kHz */
#define RMS_WINDOW_SAMPLES 400U /* 20 ms @ 20 kHz */

/* ── 与旧固件同量级的外环参数 ──────────────────────────────────────── */
#define K_P_BUS 1.0
#define K_I_BUS 1.0

/*
 * 板上只有交流输入电流传感器，没有 Lb 电流传感器。因此实际可实现的控制
 * 必须直接拿整流后的 Iin 做反馈，而不是控制一个不可测的 iLb。
 */
#define K_P_INPUT_CURRENT 0.4
#define K_I_INPUT_CURRENT 50.0
#define INPUT_VOLTAGE_INTEGRAL_LIMIT 150.0

/* ── 功率级默认值（没有实物参数时使用，报告必须标注为假设）────────── */
#define VF_SCHOTTKY_V 0.45
#define R_LF_OHM 0.050
#define R_LB_OHM 0.050
#define L_B_DEFAULT_H 1.0e-3
#define C_BUS_DEFAULT_F 1000.0e-6
#define F_LINE_HZ 50.0
#define V_BUS_REF_DEFAULT_V 62.0
#define V_BUS_SLEW_V_PER_S 60.0
#define I_AMP_LIMIT_DEFAULT_A 3.8
#define DUTY_MIN 0.05
#define DUTY_MAX 0.95
#define DUTY_EPS 1.0e-9

#define DEFAULT_T_END_S 3.9
#define DEFAULT_STATS_START_S 3.5
#define STATS_SECONDS 0.4 /* 20 个工频周期，正好覆盖 100 Hz 纹波 */

#define HARMONICS 15U

typedef struct {
    double vac_rms_v;
    double i_load_a;
    double c_f_f;
    double l_f_h;
    double l_b_h;
    double c_bus_f;
    double i_amp_limit_a;
    double v_bus_ref_v;
    double t_end_s;
    double stats_start_s;
    double trace_start_s;
    bool trace;
    const char *csv_path;
} SimConfig;

typedef struct {
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;
    double s1;
    double s2;
} Notch;

typedef struct {
    double t;
    double i_lf;
    double v_cf;
    double i_lb;
    double v_bus;

    /* 控制状态 */
    bool control_ready;
    bool rms_valid;
    double vcf_sq_window[RMS_WINDOW_SAMPLES];
    double vcf_sq_sum;
    uint32_t rms_index;
    uint32_t rms_count;
    double vcf_rms;
    Notch bus_notch;
    double v_bus_filtered;
    double v_ref;
    double i_amp;
    double i_amp_integral;
    double current_integral;
    double i_ref;
    double i_lb_ref;
    double v_cf_prev;
    bool v_cf_prev_valid;
    double duty;

    uint32_t step;

    /* 统计窗口 */
    uint32_t count;
    double sum_vac_sq;
    double sum_iac_sq;
    double sum_p;
    double sum_vbus;
    double vbus_min;
    double vbus_max;
    double duty_min;
    double duty_max;
    double i_lb_peak;
    double i_lf_peak;
    double global_i_lb_peak;
    double global_i_lf_peak;
    double global_vbus_peak;
    uint32_t conduct_count;
    uint32_t current_limit_count;
    double fund_cos[HARMONICS + 1U];
    double fund_sin[HARMONICS + 1U];

    FILE *csv;
} Sim;

static double clampd(double value, double low, double high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static bool parse_double_arg(const char *text, double *out) {
    char *end = NULL;
    double value;
    if ((text == NULL) || (out == NULL)) return false;
    value = strtod(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(value)) return false;
    *out = value;
    return true;
}

static bool parse_option_value(int argc, char **argv, int *index, const char *name, double *out) {
    size_t n = strlen(name);
    const char *arg = argv[*index];
    if (strncmp(arg, name, n) != 0) return false;
    if (arg[n] == '=') return parse_double_arg(arg + n + 1U, out);
    if ((arg[n] == '\0') && (*index + 1 < argc)) {
        (*index)++;
        return parse_double_arg(argv[*index], out);
    }
    return false;
}

/*
 * 二阶 100 Hz 陷波器。母线上的二倍频纹波在理想单相 PFC 中本来就存在；
 * 如果不从电压外环里去掉，慢环会把纹波当成电流指令，造成周期性的
 * 电流幅值调制。公式与 App/Src/notch_filter.c 的 Tustin 实现相同。
 */
static bool notch_init(Notch *notch, double center_rad_s, double zero_damping, double pole_damping,
                       double sample_time_s) {
    double k;
    double w2;
    double d0;
    if ((notch == NULL) || !(center_rad_s > 0.0) || !(pole_damping > 0.0) ||
        !(zero_damping >= 0.0 && zero_damping <= pole_damping) || !(sample_time_s > 0.0)) {
        return false;
    }
    k = 2.0 / sample_time_s;
    w2 = center_rad_s * center_rad_s;
    d0 = k * k + 2.0 * pole_damping * center_rad_s * k + w2;
    if (!(d0 > 0.0) || !isfinite(d0)) return false;
    notch->b0 = (k * k + 2.0 * zero_damping * center_rad_s * k + w2) / d0;
    notch->b1 = 2.0 * (w2 - k * k) / d0;
    notch->b2 = (k * k - 2.0 * zero_damping * center_rad_s * k + w2) / d0;
    notch->a1 = notch->b1;
    notch->a2 = (k * k - 2.0 * pole_damping * center_rad_s * k + w2) / d0;
    notch->s1 = 0.0;
    notch->s2 = 0.0;
    return true;
}

static void notch_reset(Notch *notch, double input) {
    notch->s1 = (1.0 - notch->b0) * input;
    notch->s2 = (notch->b2 - notch->a2) * input;
}

static double notch_step(Notch *notch, double input) {
    double output = notch->b0 * input + notch->s1;
    double s1 = notch->b1 * input - notch->a1 * output + notch->s2;
    notch->s2 = notch->b2 * input - notch->a2 * output;
    notch->s1 = s1;
    return output;
}

/*
 * 400 点滑动 RMS，模拟固件在 20 kHz 快环里维护的一个完整 50 Hz 周期。
 * 不能把 vcf 当作理想正弦：桥后 Cf 上会有换流和 LC 动态，参考必须使用
 * 实际测得的 vcf 波形。
 */
static void update_vcf_rms(Sim *s) {
    double oldest = 0.0;
    double sample = s->v_cf * s->v_cf;
    if (s->rms_count >= RMS_WINDOW_SAMPLES) oldest = s->vcf_sq_window[s->rms_index];
    else s->rms_count++;
    s->vcf_sq_window[s->rms_index] = sample;
    s->vcf_sq_sum += sample - oldest;
    s->rms_index++;
    if (s->rms_index >= RMS_WINDOW_SAMPLES) s->rms_index = 0U;
    if (s->rms_count >= RMS_WINDOW_SAMPLES) {
        s->vcf_rms = sqrt(fmax(s->vcf_sq_sum / (double)RMS_WINDOW_SAMPLES, 0.0));
        s->rms_valid = isfinite(s->vcf_rms) && (s->vcf_rms > 1.0e-6);
    }
}

/* 1 kHz 母线外环；条件积分和固件 PiController 的抗饱和语义一致。 */
static void outer_loop(Sim *s, const SimConfig *cfg) {
    const double dt = (double)OUTER_DIVIDER * DT_S;
    const double error = s->v_ref - s->v_bus_filtered;
    const double integral_old = s->i_amp_integral;
    double unsaturated;
    double output;

    s->i_amp_integral += K_I_BUS * dt * error;
    unsaturated = K_P_BUS * error + s->i_amp_integral;
    output = clampd(unsaturated, 0.0, cfg->i_amp_limit_a);
    if (output != unsaturated) {
        /* 误差继续朝饱和方向时冻结，反向时允许积分器退出饱和。 */
        if (((unsaturated > cfg->i_amp_limit_a) && (error > 0.0)) || ((unsaturated < 0.0) && (error < 0.0)))
            s->i_amp_integral = integral_old;
    }
    s->i_amp_integral = clampd(s->i_amp_integral, 0.0, cfg->i_amp_limit_a);
    s->i_amp = output;
}

/*
 * 20 kHz 电流内环。平均 Boost 电感的动力学是：
 *
 *     Lb·diL/dt = vCf − R_Lb·iL − (1−d)·Vbus
 *
 * 把目标电感电压记为 vL，则可以反解出下管占空比：
 *
 *     d = 1 − (vCf − vL)/Vbus
 *
 * 因此 PI 输出的是“电感上的电压”，而不是直接输出占空比；这样 vCf
 * 和 Vbus 变化时，母线前馈会自动补偿一部分扰动。
 */
static void inner_loop(Sim *s, const SimConfig *cfg) {
    const double dt = (double)INNER_DIVIDER * DT_S;
    double error;
    double unsaturated_vl;
    double requested_duty;
    double duty;
    double integral_old;

    update_vcf_rms(s);

    if (!s->control_ready) {
        if ((s->t >= ((double)RMS_WINDOW_SAMPLES / (1.0 / dt))) && s->rms_valid && (s->v_bus >= 5.0)) {
            s->control_ready = true;
            s->v_ref = clampd(s->v_bus - 1.0, 0.0, cfg->v_bus_ref_v);
            s->i_amp = 0.0;
            s->i_amp_integral = 0.0;
            s->current_integral = 0.0;
            s->v_cf_prev = s->v_cf;
            s->v_cf_prev_valid = true;
            s->v_bus_filtered = s->v_bus;
            notch_reset(&s->bus_notch, s->v_bus);
        } else {
            s->duty = 0.0;
            s->current_integral = 0.0;
            return;
        }
    }

    /*
     * 实际 ACAC 板只给整流侧新增一路电压采样，放置于桥后 Vcf。
     * 因此电流参考必须由 Vcf 自身定形；不能再假设有独立的 |Vac| 采样。
     */
    s->i_ref = (s->vcf_rms > 1.0e-6) ? s->i_amp * fmax(s->v_cf, 0.0) / s->vcf_rms : 0.0;
    error = s->i_ref - s->i_lf;
    integral_old = s->current_integral;
    s->current_integral += K_I_INPUT_CURRENT * dt * error;
    s->current_integral = clampd(s->current_integral, -INPUT_VOLTAGE_INTEGRAL_LIMIT, INPUT_VOLTAGE_INTEGRAL_LIMIT);
    unsaturated_vl = K_P_INPUT_CURRENT * error + s->current_integral;
    requested_duty = 1.0 - (s->v_cf - unsaturated_vl) / fmax(s->v_bus, 1.0);
    duty = clampd(requested_duty, DUTY_MIN, DUTY_MAX);

    if (fabs(duty - requested_duty) > DUTY_EPS) {
        if (((requested_duty > DUTY_MAX) && (error > 0.0)) || ((requested_duty < DUTY_MIN) && (error < 0.0)))
            s->current_integral = integral_old;
    }
    s->duty = duty;
}

static void write_csv_sample(Sim *s, double v_ac, double i_ac) {
    if (s->csv == NULL) return;
    fprintf(s->csv, "%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", s->t, v_ac, i_ac, s->v_cf, s->i_lf,
            s->i_lb, s->v_bus, s->v_ref, s->v_bus_filtered, s->i_ref, s->duty);
}

static void sim_step(Sim *s, const SimConfig *cfg) {
    const double v_ac = SQRT2_F * cfg->vac_rms_v * sin(2.0 * PI_F * F_LINE_HZ * s->t);
    const double v_rect = fabs(v_ac) - 2.0 * VF_SCHOTTKY_V;
    double i_ac;
    double i_load;
    double v_l;

    /*
     * 整流桥 + Lf。二极管电流不能反向；当桥已经关断且 v_rect 低于
     * vCf 时保持零电流。这样会自然出现普通二极管整流器的零电流死区。
     */
    if ((s->i_lf > 0.0) || (v_rect > s->v_cf)) {
        s->i_lf += (v_rect - s->v_cf - R_LF_OHM * s->i_lf) / cfg->l_f_h * DT_S;
        if (s->i_lf < 0.0) s->i_lf = 0.0;
    } else {
        s->i_lf = 0.0;
    }

    /* 控制使能前只走 Boost 上管体二极管；20 ms 后闭环接管。 */
    if (!s->control_ready) s->duty = 0.0;
    s->step++;
    if ((s->step % INNER_DIVIDER) == 0U) inner_loop(s, cfg);
    if ((s->step % OUTER_DIVIDER) == 0U) {
        if (s->control_ready) {
            s->v_bus_filtered = notch_step(&s->bus_notch, s->v_bus);
            outer_loop(s, cfg);
        } else {
            s->i_amp = 0.0;
            s->i_amp_integral = 0.0;
        }
    }
    if (s->control_ready && (s->step % INNER_DIVIDER) == 0U) {
        s->v_ref += V_BUS_SLEW_V_PER_S * ((double)INNER_DIVIDER * DT_S);
        if (s->v_ref > cfg->v_bus_ref_v) s->v_ref = cfg->v_bus_ref_v;
    }

    /*
     * 半桥 Boost 的开关周期平均模型。下管占空比为 d，上管为 1−d；
     * 忽略死区时开关节点平均电压是 (1−d)·Vbus。
     */
    v_l = s->v_cf - R_LB_OHM * s->i_lb - (1.0 - s->duty) * s->v_bus;
    s->i_lb += v_l / cfg->l_b_h * DT_S;
    if (s->i_lb < 0.0) s->i_lb = 0.0;

    /* 恒流负载比“额定电阻”更接近后级 DCAC 的恒功率需求。 */
    i_load = (s->v_bus > 0.0) ? cfg->i_load_a : 0.0;
    s->v_bus += ((1.0 - s->duty) * s->i_lb - i_load) / cfg->c_bus_f * DT_S;
    if (s->v_bus < 0.0) s->v_bus = 0.0;

    s->v_cf += (s->i_lf - s->i_lb) / cfg->c_f_f * DT_S;
    if (s->v_cf < 0.0) s->v_cf = 0.0;

    /*
     * 以 20 kHz 记录统计量和绘图点。输入电流在 AC 侧随 v_ac 过零改变
     * 方向；桥后 i_lf 始终是非负的。
     */
    if ((s->step % INNER_DIVIDER) == 0U) {
        i_ac = (v_ac >= 0.0) ? s->i_lf : -s->i_lf;
        if (s->i_lb > s->global_i_lb_peak) s->global_i_lb_peak = s->i_lb;
        if (s->i_lf > s->global_i_lf_peak) s->global_i_lf_peak = s->i_lf;
        if (s->v_bus > s->global_vbus_peak) s->global_vbus_peak = s->v_bus;
        if ((s->t >= cfg->stats_start_s) && (s->t < cfg->stats_start_s + STATS_SECONDS)) {
            uint32_t h;
            s->count++;
            s->sum_vac_sq += v_ac * v_ac;
            s->sum_iac_sq += i_ac * i_ac;
            s->sum_p += v_ac * i_ac;
            s->sum_vbus += s->v_bus;
            if (s->v_bus < s->vbus_min) s->vbus_min = s->v_bus;
            if (s->v_bus > s->vbus_max) s->vbus_max = s->v_bus;
            if (s->duty < s->duty_min) s->duty_min = s->duty;
            if (s->duty > s->duty_max) s->duty_max = s->duty;
            if (s->i_lb > s->i_lb_peak) s->i_lb_peak = s->i_lb;
            if (s->i_lf > s->i_lf_peak) s->i_lf_peak = s->i_lf;
            if (s->i_lf > 1.0e-3) s->conduct_count++;
            if (s->i_amp >= (cfg->i_amp_limit_a - 1.0e-5)) s->current_limit_count++;
            for (h = 1U; h <= HARMONICS; h++) {
                const double w = 2.0 * PI_F * (double)h * F_LINE_HZ * s->t;
                s->fund_cos[h] += i_ac * cos(w);
                s->fund_sin[h] += i_ac * sin(w);
            }
        }
        if ((s->csv != NULL) && (s->t >= cfg->stats_start_s)) write_csv_sample(s, v_ac, i_ac);
        if (cfg->trace && (s->t >= cfg->trace_start_s) && (s->t <= cfg->trace_start_s + 0.025)) {
            printf("%8.3f %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %7.3f %7.2f %6.3f\n", s->t * 1000.0, s->v_ref,
                   s->v_bus_filtered, s->v_bus, s->v_cf, s->vcf_rms, s->i_amp, s->i_lb, s->i_ref, s->duty);
        }
    }

    s->t += DT_S;
}

static void print_usage(const char *program) {
    printf("用法: %s [Vac_rms] [Iload_A] [Cf_uF] [Lf_mH] [选项]\n", program);
    printf("选项: --lb-mh=N --cbus-uf=N --ilimit=N --vref=N --duration=S --stats-start=S "
           "--trace-start=S --trace --csv=FILE\n");
}

int main(int argc, char **argv) {
    SimConfig cfg = {
        .vac_rms_v = 36.0,
        .i_load_a = 1.8,
        .c_f_f = 10.0e-6,
        .l_f_h = 2.0e-3,
        .l_b_h = L_B_DEFAULT_H,
        .c_bus_f = C_BUS_DEFAULT_F,
        .i_amp_limit_a = I_AMP_LIMIT_DEFAULT_A,
        .v_bus_ref_v = V_BUS_REF_DEFAULT_V,
        .t_end_s = DEFAULT_T_END_S,
        .stats_start_s = DEFAULT_STATS_START_S,
        .trace_start_s = DEFAULT_STATS_START_S + 0.02,
        .trace = false,
        .csv_path = NULL,
    };
    Sim s = {0};
    int positional = 1;
    double value;
    int i;
    double v_rms;
    double i_rms;
    double p_in;
    double v_bus_mean;
    double p_out;
    double pf;
    double fund_amp = 0.0;
    double harm_sq = 0.0;
    double thd;
    uint32_t h;

    /* 先解析四个兼容旧脚本的位置参数，再处理命名选项。 */
    for (; (positional < argc) && (argv[positional][0] != '-') && (positional <= 4); positional++) {
        if (!parse_double_arg(argv[positional], &value)) {
            print_usage(argv[0]);
            return 2;
        }
        if (positional == 1) cfg.vac_rms_v = value;
        else if (positional == 2) cfg.i_load_a = value;
        else if (positional == 3) cfg.c_f_f = value * 1.0e-6;
        else if (positional == 4) cfg.l_f_h = value * 1.0e-3;
    }
    for (i = positional; i < argc; i++) {
        if (strcmp(argv[i], "--trace") == 0) {
            cfg.trace = true;
        } else if (strncmp(argv[i], "--csv=", 6U) == 0) {
            cfg.csv_path = argv[i] + 6;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (parse_option_value(argc, argv, &i, "--lb-mh", &value)) {
            cfg.l_b_h = value * 1.0e-3;
        } else if (parse_option_value(argc, argv, &i, "--cbus-uf", &value)) {
            cfg.c_bus_f = value * 1.0e-6;
        } else if (parse_option_value(argc, argv, &i, "--ilimit", &value)) {
            cfg.i_amp_limit_a = value;
        } else if (parse_option_value(argc, argv, &i, "--vref", &value)) {
            cfg.v_bus_ref_v = value;
        } else if (parse_option_value(argc, argv, &i, "--duration", &value)) {
            cfg.t_end_s = value;
        } else if (parse_option_value(argc, argv, &i, "--stats-start", &value)) {
            cfg.stats_start_s = value;
        } else if (parse_option_value(argc, argv, &i, "--trace-start", &value)) {
            cfg.trace_start_s = value;
        } else {
            fprintf(stderr, "未知参数: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    if (!(cfg.vac_rms_v >= 5.0 && cfg.vac_rms_v <= 60.0) || !(cfg.i_load_a > 0.0 && cfg.i_load_a <= 4.0) ||
        !(cfg.c_f_f >= 0.5e-6 && cfg.c_f_f <= 470.0e-6) || !(cfg.l_f_h >= 0.1e-3 && cfg.l_f_h <= 20.0e-3) ||
        !(cfg.l_b_h >= 0.1e-3 && cfg.l_b_h <= 20.0e-3) || !(cfg.c_bus_f >= 100.0e-6) ||
        !(cfg.i_amp_limit_a > 0.1 && cfg.i_amp_limit_a <= 10.0) || !(cfg.v_bus_ref_v > cfg.vac_rms_v * SQRT2_F) ||
        !(cfg.t_end_s > cfg.stats_start_s) || !(cfg.stats_start_s >= 0.1)) {
        fprintf(stderr, "参数越界：Vac 5..60 Vrms，Iload 0..4 A，Cf 0.5..470 uF，Lf/Lb 0.1..20 mH，"
                        "Vbus_ref 必须大于输入峰值\n");
        return 2;
    }

    s.vbus_min = 1.0e100;
    s.vbus_max = -1.0e100;
    s.duty_min = 1.0e100;
    s.duty_max = -1.0e100;
    if (!notch_init(&s.bus_notch, 2.0 * 2.0 * PI_F * F_LINE_HZ, 0.0, 50.0 / (4.0 * PI_F * 100.0),
                    (double)OUTER_DIVIDER * DT_S)) {
        fprintf(stderr, "陷波器初始化失败\n");
        return 2;
    }
    if (cfg.csv_path != NULL) {
        s.csv = fopen(cfg.csv_path, "w");
        if (s.csv == NULL) {
            perror("创建 CSV 失败");
            return 2;
        }
        fprintf(s.csv, "time_s,vac_v,iac_a,vcf_v,ilf_a,ilb_a,vbus_v,vref_v,vbus_filtered_v,iref_a,d_low\n");
    }

    printf("=== ACDC: 4×肖特基桥 + LC + 单半桥 Boost（平均模型）===\n");
    printf("拓扑参数: Lf=%.2f mH, Cf=%.1f uF, Lb=%.2f mH, Cbus=%.0f uF, Vf=%.2f V/只\n", cfg.l_f_h * 1.0e3,
           cfg.c_f_f * 1.0e6, cfg.l_b_h * 1.0e3, cfg.c_bus_f * 1.0e6, VF_SCHOTTKY_V);
    printf("工况: Vac=%.1f Vrms/%.0f Hz, Iload=%.2f A, Vbus_ref=%.1f V, Iin_limit=%.2f A RMS\n", cfg.vac_rms_v,
           F_LINE_HZ, cfg.i_load_a, cfg.v_bus_ref_v, cfg.i_amp_limit_a);
    if (cfg.trace) printf("\n   t(ms)   v_ref vbus_flt   v_bus    v_cf vcf_rms  i_amp    i_lb   i_ref duty_d\n");

    while (s.t < cfg.stats_start_s + STATS_SECONDS && s.t < cfg.t_end_s) sim_step(&s, &cfg);

    if (s.csv != NULL) fclose(s.csv);

    if (s.count == 0U) {
        fprintf(stderr, "统计窗口没有样本\n");
        return 2;
    }
    v_rms = sqrt(s.sum_vac_sq / (double)s.count);
    i_rms = sqrt(s.sum_iac_sq / (double)s.count);
    p_in = s.sum_p / (double)s.count;
    v_bus_mean = s.sum_vbus / (double)s.count;
    p_out = v_bus_mean * cfg.i_load_a;
    pf = (v_rms * i_rms > 1.0e-12) ? p_in / (v_rms * i_rms) : 0.0;

    for (h = 1U; h <= HARMONICS; h++) {
        const double amp = 2.0 * sqrt(s.fund_cos[h] * s.fund_cos[h] + s.fund_sin[h] * s.fund_sin[h]) / (double)s.count;
        if (h == 1U) fund_amp = amp;
        else harm_sq += amp * amp;
    }
    thd = (fund_amp > 1.0e-9) ? 100.0 * sqrt(harm_sq) / fund_amp : 0.0;

    printf("\n--- %.1f..%.1f s 稳态统计（%u 个 20 kHz 点）---\n", cfg.stats_start_s, cfg.stats_start_s + STATS_SECONDS,
           (unsigned)s.count);
    printf("母线: 平均 %.2f V, 纹波 %.2f Vpp (%.1f%%), 范围 %.2f..%.2f V\n", v_bus_mean, s.vbus_max - s.vbus_min,
           100.0 * (s.vbus_max - s.vbus_min) / fmax(v_bus_mean, 1.0), s.vbus_min, s.vbus_max);
    printf("输入: %.2f Vrms, %.2f Arms, Pin %.1f W, PF %.3f, 电流 THD(h2..h%u) %.1f%%\n", v_rms, i_rms, p_in, pf,
           HARMONICS, thd);
    printf("输出: %.1f W, 效率(仅铜损+二极管压降) %.1f%%\n", p_out, 100.0 * p_out / ((p_in > 1.0e-9) ? p_in : 1.0e-9));
    printf("控制: Iamp 限幅占用 %.1f%%, 二极管导通 %.1f%%, ILb峰值 %.2f A, ILf峰值 %.2f A, d=%.3f..%.3f\n",
           100.0 * (double)s.current_limit_count / (double)s.count, 100.0 * (double)s.conduct_count / (double)s.count,
           s.i_lb_peak, s.i_lf_peak, s.duty_min, s.duty_max);
    printf("判定: %s（母线均值误差 %.2f V，目标带按 ±2%% = %.2f V）\n",
           (fabs(v_bus_mean - cfg.v_bus_ref_v) <= 0.02 * cfg.v_bus_ref_v) ? "达到母线目标" : "未达到母线目标",
           v_bus_mean - cfg.v_bus_ref_v, 0.02 * cfg.v_bus_ref_v);
    printf("上电风险: 全过程中 ILb峰值 %.2f A, ILf峰值 %.2f A, Vbus峰值 %.2f V（理想源、无限流）\n", s.global_i_lb_peak,
           s.global_i_lf_peak, s.global_vbus_peak);
    printf("RESULT vac=%.6f iload=%.6f cf_uf=%.6f lf_mh=%.6f ilimit=%.6f vbus_mean=%.6f "
           "vbus_ripple=%.6f iin_rms=%.6f pin=%.6f pf=%.6f thd=%.6f pout=%.6f eff_pct=%.6f "
           "iamp_sat_pct=%.6f diode_conduct_pct=%.6f ilb_peak=%.6f ilf_peak=%.6f "
           "duty_min=%.6f duty_max=%.6f reached=%d startup_ilb_peak=%.6f startup_ilf_peak=%.6f "
           "startup_vbus_peak=%.6f\n",
           cfg.vac_rms_v, cfg.i_load_a, cfg.c_f_f * 1.0e6, cfg.l_f_h * 1.0e3, cfg.i_amp_limit_a, v_bus_mean,
           s.vbus_max - s.vbus_min, i_rms, p_in, pf, thd, p_out, 100.0 * p_out / ((p_in > 1.0e-9) ? p_in : 1.0e-9),
           100.0 * (double)s.current_limit_count / (double)s.count, 100.0 * (double)s.conduct_count / (double)s.count,
           s.i_lb_peak, s.i_lf_peak, s.duty_min, s.duty_max,
           (fabs(v_bus_mean - cfg.v_bus_ref_v) <= 0.02 * cfg.v_bus_ref_v) ? 1 : 0, s.global_i_lb_peak,
           s.global_i_lf_peak, s.global_vbus_peak);
    return 0;
}
