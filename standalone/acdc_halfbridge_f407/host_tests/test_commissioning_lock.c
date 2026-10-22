#include <stdint.h>
#include <stdio.h>

#include "acdc_application.h"

static int g_failures;

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #condition);                                              \
            g_failures++;                                                                                              \
        }                                                                                                              \
    } while (0)

int main(void) {
    AcdcApplication app;
    AcdcControlOutput out;
    uint32_t frame[ACDC_ADC_RANK_COUNT] = {0};

    AcdcApp_Init(&app);
    CHECK(app.state == ACDC_STATE_LOCKED);
    AcdcApp_ToggleRequest(&app);
    CHECK(app.state == ACDC_STATE_LOCKED);
    CHECK(!app.start_requested);

    /* 故障也不能把 LOCKED 降级成可由按键清除的 OFF。 */
    AcdcApp_LatchExternalFault(&app);
    CHECK(app.state == ACDC_STATE_LOCKED);
    AcdcApp_ToggleRequest(&app);
    CHECK(app.state == ACDC_STATE_LOCKED);

    AcdcApp_FastStep(&app, frame, &out);
    CHECK(!out.gates_enabled);
    CHECK(app.state == ACDC_STATE_LOCKED);

    if (g_failures != 0) {
        printf("%d commissioning-lock test(s) failed.\n", g_failures);
        return 1;
    }
    printf("Commissioning lock test passed.\n");
    return 0;
}
