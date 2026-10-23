#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "acdc_control.h"
#include "acdc_measurement.h"

typedef enum {
    ACDC_STATE_LOCKED = 0, // 未完成硬件 commissioning，永久禁止门极
    ACDC_STATE_OFF,        // 测量正常，但用户尚未请求启动
    ACDC_STATE_WAIT,       // 已请求启动，等待输入 RMS/预充稳定
    ACDC_STATE_RUN,        // 闭环运行
    ACDC_STATE_FAULT       // 锁存故障，必须人工确认
} AcdcState;

enum {
    ACDC_FAULT_MEASUREMENT = 1UL << 0,
    ACDC_FAULT_INPUT_OVERCURRENT = 1UL << 1,
    ACDC_FAULT_INPUT_OVERVOLTAGE = 1UL << 2,
    ACDC_FAULT_BUS_OVERVOLTAGE = 1UL << 3,
    ACDC_FAULT_BUS_UNDERVOLTAGE = 1UL << 4,
    ACDC_FAULT_CONTROL = 1UL << 5,
    ACDC_FAULT_EXTERNAL_BREAK = 1UL << 6
};

typedef struct {
    AcdcControl control;
    AcdcMeasurement measurement;
    AcdcControlOutput control_output;
    AcdcState state;
    uint32_t faults;
    uint32_t invalid_count;
    uint32_t precharge_count;
    uint32_t undervoltage_count;
    bool start_requested;
} AcdcApplication;

typedef struct {
    AcdcState state;
    uint32_t faults;
    float iin_a;
    float vcf_v;
    float vdc_v;
    float duty_low_side;
    float current_reference_a;
    float vcf_rms_v;
    bool gates_enabled;
} AcdcSnapshot;

// 初始化控制器和应用状态；未 commissioning 时 state=LOCKED
void AcdcApp_Init(AcdcApplication *app);

/*
 * 处理一帧 20 kHz ADC。输出只在 state=RUN、测量有效且无故障时允许门极。
 * 该函数可在中断中调用，不做阻塞等待、动态内存或 HAL 调用。
 */
void AcdcApp_FastStep(AcdcApplication *app, const uint32_t frame[ACDC_ADC_RANK_COUNT], AcdcControlOutput *out);

// 按键请求：故障态先清故障并回到 OFF，正常态切换启动/停止请求
void AcdcApp_ToggleRequest(AcdcApplication *app);

// BKIN/外部故障：立即锁存
void AcdcApp_LatchExternalFault(AcdcApplication *app);

// 测量/ADC/DMA 故障：立即锁存
void AcdcApp_LatchMeasurementFault(AcdcApplication *app);

// 控制配置、PWM 外设初始化等内部错误：立即锁存
void AcdcApp_LatchControlFault(AcdcApplication *app);

// 在主循环中以临界区复制一致快照
void AcdcApp_GetSnapshot(const AcdcApplication *app, AcdcSnapshot *snapshot);
