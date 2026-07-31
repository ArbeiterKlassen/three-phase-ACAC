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
    MODE1_RECTIFIER = 1,
    MODE2_INVERTER = 2
} app_mode_t;

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
    app_state_t state;
    float output_frequency_hz;
    float dc_voltage_v;
    float output_line_voltage_reference_v;
    float dc_voltage_reference_v;
    float ia_a;
    float ib_a;
    float vab_v;
    float vbc_v;
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
