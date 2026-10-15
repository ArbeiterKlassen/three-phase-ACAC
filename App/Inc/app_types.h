#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PI_F        (3.14159265358979323846f)
#define APP_SQRT3_F     (1.73205080756887729353f)
#define APP_INV_SQRT3_F (0.57735026918962576451f)
#define APP_FAULT_FAST_HISTORY_DEPTH  (16U)
#define APP_FAULT_CYCLE_HISTORY_DEPTH (32U)

typedef struct
{
    uint16_t iin;
    uint16_t vin;
    uint16_t ia;
    uint16_t ib;
    uint16_t vab;
    uint16_t vbc;
    uint16_t vdc;
    uint16_t vin_diag;
    uint32_t sequence;
    bool valid;
} AdcRawFrame;

typedef struct measurements_t
{
    float iin;
    float vin;
    float ia;
    float ib;
    float ic;
    float vab;
    float vbc;
    float vca;
    float va;
    float vb;
    float vc;
    float vdc;
    uint32_t sequence;
    bool valid;
} measurements_t;

typedef measurements_t Measurements;
typedef uint32_t fault_flags_t;

typedef struct
{
    float alpha;
    float beta;
} AlphaBeta;

typedef struct
{
    float a;
    float b;
    float c;
} ThreePhase;

typedef struct
{
    float a;
    float b;
    float c;
    bool enabled;
} InverterPwmCommand;

typedef struct
{
    float leg_a;
    float leg_b;
    bool enabled;
} RectifierPwmCommand;

typedef enum app_mode_t
{
    MODE0_FULL = 0,
    MODE1_INVERTER = 1,
    MODE2_RECTIFIER = 2
} app_mode_t;

typedef enum inverter_run_mode_t
{
    INVERTER_CLOSED_LOOP = 0,
    INVERTER_OPEN_LOOP = 1
} inverter_run_mode_t;

typedef enum app_state_t
{
    OFF = 0,
    SELF_TEST,
    PRECHARGE,
    READY,
    RUN,
    FAULT_LATCH,
    DISCHARGE
} app_state_t;

typedef struct
{
    uint32_t sample_sequence;
    uint16_t pc0_raw;
    uint16_t pc1_raw;
    uint16_t pc2_raw;
    uint16_t pc3_raw;
} AppFaultRawSample;

typedef struct
{
    AlphaBeta voltage_reference;
    AlphaBeta voltage_measured;
    AlphaBeta current_reference;
    AlphaBeta current_measured;
    AlphaBeta voltage_command;
    uint32_t sample_sequence;
    float duty_a;
    float duty_b;
    float duty_c;
    float applied_scale;
    bool saturated;
    bool valid;
} AppFaultControlSample;

typedef struct
{
    inverter_run_mode_t inverter_run_mode;
    fault_flags_t faults;
    uint32_t sample_sequence;
    float dc_voltage_reference_v;
    float vab_v;
    float vbc_v;
    float va_v;
    float vb_v;
    float vc_v;
    float ia_a;
    float ib_a;
    float iin_a;
    float previous_iin_a;
    float vin_v;
    float previous_vin_v;
    float vdc_v;
    uint32_t previous_iin_sequence;
    uint32_t previous_vin_sequence;
    uint16_t iin_raw;
    uint16_t previous_iin_raw;
    uint16_t vin_raw;
    uint16_t previous_vin_raw;
    bool previous_iin_valid;
    bool previous_vin_valid;
    AppFaultRawSample trigger_raw;
    /* Oldest-to-newest samples immediately before the trip frame. */
    AppFaultRawSample fast_history[APP_FAULT_FAST_HISTORY_DEPTH];
    AppFaultControlSample previous_control;
    /* Entry 0 is the most recent completed fundamental-cycle window. */
    float cycle_peak_v[APP_FAULT_CYCLE_HISTORY_DEPTH];
    /* Pre-trip peak in the current incomplete cycle; excludes trigger_raw. */
    float partial_cycle_peak_v;
    uint16_t partial_cycle_samples;
    uint16_t pc2_raw;
    uint16_t pc3_raw;
    uint8_t fast_history_count;
    uint8_t cycle_peak_count;
    bool valid;
} AppFaultSnapshot;

typedef struct
{
    float inverter_a;
    float inverter_b;
    float inverter_c;
    float rectifier_a;
    float rectifier_b;
    bool inverter_enable_request;
    bool rectifier_enable_request;
} AppPwmCommand;

typedef struct
{
    app_mode_t mode;
    inverter_run_mode_t inverter_run_mode;
    app_state_t state;
    float output_frequency_hz;
    float dc_voltage_v;
    float output_line_voltage_reference_v;
    float dc_voltage_reference_v;
    float iin_a;
    float ia_a;
    float ib_a;
    float vab_v;
    float vbc_v;
    uint16_t iin_raw;
    AppFaultSnapshot fault_snapshot;
    fault_flags_t faults;
    uint32_t sample_sequence;
    bool board_config_valid;
    bool pwm_allowed;
    bool rectifier_dc_ready;
} AppStatus;

typedef struct
{
    bool enable;
    bool reset_fault;
    bool precharge_done;
    bool phase_ready;
    bool external_fault;
    bool loop_overrun;
    float inverter_vrms_ref;
    float dc_voltage_ref;
} AppCommand;

typedef struct
{
    Measurements measurement;
    InverterPwmCommand inverter;
    RectifierPwmCommand rectifier;
    app_state_t state;
    fault_flags_t faults;
    bool board_config_valid;
    bool pwm_allowed;
} AppOutput;

#ifdef __cplusplus
}
#endif

#endif
