#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "acdc_config.h"
#include "acdc_measurement.h"

typedef struct {
    float kp;
    float ki;
    float sample_time_s;
    float integrator;
    float out_min;
    float out_max;
} AcdcPi;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float s1;
    float s2;
} AcdcNotch;

typedef struct {
    float vbus_target_v;
    float vbus_min_v;
    float vbus_max_v;
    float vcf_rms_min_v;
    float vcf_abs_max_v;
    float iin_abs_max_a;
    float iin_command_max_a;
    float vbus_slew_v_per_s;
    float duty_min;
    float duty_max;
} AcdcControlConfig;

typedef struct {
    AcdcControlConfig cfg;
    AcdcPi bus_pi;
    AcdcNotch bus_notch;
    float vcf_square_window[ACDC_RMS_WINDOW_SAMPLES];
    float vcf_square_sum;
    uint32_t vcf_square_index;
    uint32_t vcf_square_count;
    uint32_t outer_count;
    float vcf_rms_v;
    float vbus_filtered_v;
    float voltage_reference_v;
    float current_rms_command_a;
    float current_reference_a;
    float current_integral_v;
    bool rms_valid;
    bool enabled_previous;
} AcdcControl;

typedef struct {
    float low_side_duty;       // TIM8 比较结果对应的低侧占空比 [0~1]
    float current_reference_a; // 当前输入电流幅值指令 [A]
    float current_rms_cmd_a;   // 母线环输出 [A RMS 语义]
    float voltage_reference_v; // 当前软启动参考 [V]
    float vcf_rms_v;           // 一个工频周期 Vcf RMS [V]
    float vbus_filtered_v;     // 100 Hz 陷波后的母线 [V]
    bool gates_enabled;        // 仅 true 时 board runtime 允许开 TIM8 门极
    bool valid;                // 本拍控制结果可用于硬件
    bool current_limited;      // 母线环处于电流指令限幅
    bool duty_limited;         // 电流环撞到最小/最大占空比
} AcdcControlOutput;

// 初始化数据结构并校验频率、窗口和限幅关系
bool AcdcControl_Init(AcdcControl *control, const AcdcControlConfig *cfg);

// 清控制暂态；输入 RMS 会重新积累一个完整工频周期
void AcdcControl_Reset(AcdcControl *control);

/*
 * 执行一拍 20 kHz 控制。
 * enabled=false 时仍积累输入 RMS，但输出 gates_enabled=false；这样进入 RUN
 * 的那一拍不需要再等 20 ms。只有 enabled=true 且测量和软启动均有效时才
 * 可能输出门极许可。
 */
bool AcdcControl_Step(AcdcControl *control, const AcdcMeasurement *measurement, bool enabled, AcdcControlOutput *out);
