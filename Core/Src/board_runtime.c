#include "board_runtime.h"

#include "adc.h"
#include "app_control.h"
#include "key_board.h"
#include "main.h"
#include "oled.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * 板级运行时（Core 层）：把纯算法 App 层接到真实硬件上（三相逆变工程）。
 *
 * 本文件是"唯一碰寄存器的地方"，三件事全在这里：
 *   1) 20 kHz 快环：ADC DMA 半满/全满中断 → 解包 → App_FastStep20kHz()
 *      → **延迟一拍**装载 CCR/CCER/MOE（与仿真的一周期计算延迟一致）；
 *   2) 安全：任何故障（软件保护、BKIN/Break 回调、ADC 错误、循环超时）
 *      都走 BoardRuntime_LatchTrip —— 硬件关 MOE + 锁存，不许自动恢复；
 *   3) 人机：10 ms 键盘扫描、100 ms OLED 刷新、故障快照分页显示。
 *
 * 中断层次（谁打断谁）：ADC DMA ISR 跑控制快环（最高实时性要求）；
 * TIM1 Break ISR 只做关断锁存；主循环 while(1) 做按键与显示。
 * 共享状态（s_delayed_pwm / s_trip_latched / 显示快照）都用"临界区 +
 * __DMB()"保护，改这些变量前先看对应的 Enter/ExitCritical 用法。
 *
 * 关于 TIM8：本工程只用 TIM1 输出 PWM。TIM8 仍然初始化并运行，但**不接
 * 功率级**（CH1~CH4 的输出使能位一直为 0，引脚不会驱动），只借它的
 * CH4 比较事件当 ADC1 的触发源（TRGO）。这样与整机工程共用同一套
 * CubeMX 配置，不需要改 tim.c / adc.c，采样时刻也保持一致。
 */

#define BOARD_PWM_PERIOD_COUNTS 4200U /* ARR(4199) + 1：占空比→CCR 换算基准 */
#define BOARD_KEY_SCAN_PERIOD_MS 10U  /* 键盘扫描周期（3 次 = 30 ms 消抖） */
#define BOARD_DISPLAY_PERIOD_MS 100U  /* OLED 刷新周期 */
#define BOARD_LONG_PRESS_MS 2000U     /* 长按判定：# 启动 / * 清故障 */
#define BOARD_KEY_DEBOUNCE_SAMPLES 3U /* 连续 3 次读到同一键才算按下 */
#define BOARD_FAULT_PAGE_COUNT 5U     /* 输出过压故障快照的分页数（D 键翻页） */
/*
 * TIM8->CCR4 的 ADC 触发点（CH4 比较事件 → TRGO → ADC1 触发）。
 * 推导：ADC 时钟 21 MHz、TIM8 168 MHz → 1 个 ADC 时钟 = 8 个 TIM 计数。
 * 触发点安排在**载波峰值之前**，使 ADC 转换结束时刻正好落在三角波顶点
 * （电感电流在一个开关周期里的平均值时刻）：
 *   采样要走完 rank1~rank3 → 提前 2×(28+12)+14 = 94 个 ADC 时钟 = 752 计数。
 * 基准是 ARR=4199 → CCR4 = 4199 − 752 = 3447。
 */
#define BOARD_ADC_TRIGGER_COMPARE 3447U

/* TIM1 六个通道使能位（CH1/1N、CH2/2N、CH3/3N）：清它 = 六路门极全关 */
#define BOARD_INV_CCER_MASK                                                                                            \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)
/*
 * OLED 显示用的状态快照：在主循环里先把 App 状态整块拷出来再画，
 * 避免"画到一半被 20 kHz ISR 改写"导致数字前后矛盾（花屏/跳数）。
 * 只放界面真正要显示的字段，不是 AppStatus 的复制品。
 */
typedef struct {
    inverter_run_mode_t inverter_run_mode;
    app_state_t state;
    fault_flags_t faults;
    float output_frequency_hz;
    float output_line_voltage_reference_v;
    float dc_voltage_reference_v;
    float dc_voltage_v;
    float ia_a;
    float ib_a;
    float vab_v;
    float vbc_v;
    bool pwm_allowed;
} BoardDisplayStatus;

/* ADC DMA 目标缓冲（双 ADC 交错，半满/全满各是一帧）——DMA 纯硬件写 */
static volatile uint32_t s_adc_dma[ADC_DMA_WORD_COUNT];
/* 帧序号：每次处理 +1，App 用它的停滞检测采样是否断了 */
static uint32_t s_sample_sequence;
/* 快环耗时峰值（CPU 周期，DWT 计数）。调试用：看控制环还剩多少余量 */
static volatile uint32_t s_max_fast_cycles;
/* 延迟一拍装载的 PWM 命令（本帧算、下帧装，与仿真延迟一致） */
static AppPwmCommand s_delayed_pwm;
static volatile bool s_delayed_pwm_valid;
/* 故障锁存：置位后只有 * 长按（且 BKIN 已释放）才能清 */
static volatile bool s_trip_latched;

/* 键盘消抖状态：候选键 → 稳定键，以及长按计时 */
static uint8_t s_key_candidate;
static uint8_t s_key_stable;
static uint8_t s_key_count;
static bool s_key_long_handled;
static uint32_t s_key_pressed_at;
/* 后台任务的两个节拍（与 HAL_GetTick 毫秒时基比较） */
static uint32_t s_last_key_scan;
static uint32_t s_last_display;
/* 故障快照显示的当前页 + 已冻结的快照副本 */
static uint8_t s_fault_display_page;
static AppFaultSnapshot s_fault_display_snapshot;

/*
 * 占空比（0~1）→ 定时器比较值（0~4199 = ARR）。
 * 0/1 两端直接取极值，中间四舍五入；用 4199 而不是 4200 是因为
 * CCR 的有效上限就是 ARR——写 4200 会永远不等，输出再也翻不回来。
 */
static uint32_t BoardRuntime_DutyToCompare(float duty) {
    if (duty <= 0.0f) return 0U;
    if (duty >= 1.0f) return BOARD_PWM_PERIOD_COUNTS - 1U;
    return (uint32_t)(duty * (float)(BOARD_PWM_PERIOD_COUNTS - 1U) + 0.5f);
}

/*
 * 进临界区：关中断 + 内存屏障，返回进入前的 PRIMASK。
 * __DMB() 是为了让关中断之后、外设寄存器写入之前的内存访问顺序固定，
 * 防止编译器/总线把顺序调换后有人在"半关"状态下读到中间态。
 */
static uint32_t BoardRuntime_EnterCritical(void) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

/* 出临界区：先屏障再恢复中断（进入前是关的就不动，支持嵌套） */
static void BoardRuntime_ExitCritical(uint32_t primask) {
    __DMB();
    if ((primask & 1U) == 0U) __enable_irq();
}

/* 外部故障输入是否有效（PE15 = 逆变 BKIN，PA6 = 预留/备用，高有效） */
static bool BoardRuntime_BreakInputActive(void) {
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET) ||
           (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);
}

/* 两路定时器的硬件 Break 中断标志（BIF）是否已置位（TIM8 只做触发源，一起查无副作用） */
static bool BoardRuntime_BreakFlagSet(void) {
    return ((TIM1->SR & TIM_SR_BIF) != 0U) || ((TIM8->SR & TIM_SR_BIF) != 0U);
}

/* 是否处于"该停机"状态：软件锁存、BKIN 输入、硬件 Break 标志三者之一 */
static bool BoardRuntime_TripPending(void) {
    return s_trip_latched || BoardRuntime_BreakInputActive() || BoardRuntime_BreakFlagSet();
}

/*
 * 关逆变级：先清 MOE 再清通道使能，最后把三个 CCR 放回 50% 中性点。
 * 顺序不能反——先清 CCER 再清 MOE，中间会有一小段"通道开着但电平不定"。
 */
static void BoardRuntime_DisableInverter(void) {
    const uint32_t neutral = BOARD_PWM_PERIOD_COUNTS / 2U;

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    TIM1->CCER &= ~BOARD_INV_CCER_MASK;
    TIM1->CCR1 = neutral;
    TIM1->CCR2 = neutral;
    TIM1->CCR3 = neutral;
}

/*
 * 关断门极。故障路径与"未使能"路径都走它。
 * TIM8 不需要处理：它的通道使能位从头到尾都是 0，只贡献 ADC 触发。
 */
static void BoardRuntime_DisablePwm(void) {
    BoardRuntime_DisableInverter();
}

/*
 * 故障锁存总入口：置锁存 → 硬件全关 → 作废待装载命令 → 通知 App 层
 * （App_TripFromISR 记故障码、拍故障快照、进 FAULT_LATCH）。
 * 所有故障来源（软件保护、BKIN、ADC 错误、循环超时、测量失败）都汇到这。
 */
static void BoardRuntime_LatchTrip(fault_flags_t fault) {
    /* 上电默认"已跳闸"：解锁只能靠 * 长按，且必须 BKIN 已释放 */
    s_trip_latched = true;
    __DMB();
    BoardRuntime_DisablePwm();
    s_delayed_pwm_valid = false;
    App_TripFromISR(fault);
}

/*
 * 把"上一帧算出的"PWM 命令真正写进定时器。
 *
 * 这里是全工程最危险的一段代码，改动前先读懂：
 *   - 最终故障检查、CCR/CCER 写入、MOE 置位必须在同一个极短临界区里：
 *     BKIN 硬件在关中断期间仍会异步清 MOE，若在临界区外开 MOE，
 *     可能与 Break 回调赛跑，让 DMA ISR 用旧命令把功率级重新打开；
 *   - 每次置位后再查一遍 BIF，看到新故障就把命令作废（返回 false）；
 *   - 返回 false 表示"本帧有跳闸"，调用方必须丢弃延迟命令。
 */
static bool BoardRuntime_ApplyDelayedPwm(const AppPwmCommand *command) {
    uint32_t primask;
    bool trip_detected;

    if (command == NULL) {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return false;
    }

    /*
     * BKIN硬件在关中断期间仍可异步清MOE。这里把最终故障检查、CCR/CCER
     * 更新和MOE置位放进极短临界区，并在置位后再次检查BIF，避免Break
     * 回调返回后DMA ISR用旧命令重新开启功率输出。
     */
    primask = BoardRuntime_EnterCritical();
    trip_detected = BoardRuntime_TripPending();
    if (!trip_detected && command->inverter_enable_request) {
        TIM1->CCR1 = BoardRuntime_DutyToCompare(command->inverter_a);
        TIM1->CCR2 = BoardRuntime_DutyToCompare(command->inverter_b);
        TIM1->CCR3 = BoardRuntime_DutyToCompare(command->inverter_c);
        TIM1->CCER |= BOARD_INV_CCER_MASK;
        __HAL_TIM_MOE_ENABLE(&htim1);
    } else {
        BoardRuntime_DisableInverter();
    }

    trip_detected = trip_detected || BoardRuntime_TripPending();
    if (BoardRuntime_TripPending()) {
        s_trip_latched = true;
        BoardRuntime_DisablePwm();
        trip_detected = true;
    }
    BoardRuntime_ExitCritical(primask);

    if (trip_detected) App_TripFromISR(PROTECTION_FAULT_EXTERNAL);
    return !trip_detected;
}

/*
 * 一帧 ADC 数据的完整处理：解包 → 跑一次 20 kHz 快环 → 装载延迟命令。
 *
 * "延迟一拍"是刻意的：本帧算出的占空比要等下一帧才写进定时器，与
 * 仿真里的一周期计算延迟一致（见 README 与 sim_inverter.c）。第一次
 * 进入 RUN 时 s_delayed_pwm_valid 还是 false，所以逆变 MOE 保持关闭。
 *
 * 结尾用 DWT 周期计数器量本帧耗时：
 *   记录峰值供调试；一旦超过 1/20 kHz（SystemCoreClock/20000 个周期）
 *   就判 LOOP_OVERRUN 跳闸——快环超时说明控制已经跟不上载波，
 *   继续跑会让采样与控制错位，宁可停机。
 */
static void BoardRuntime_ProcessAdcFrame(const uint32_t pairs[ADC_DUAL_RANK_COUNT]) {
    const AppPwmCommand *command;
    const AppStatus *status;
    uint32_t started;
    uint32_t elapsed;

    started = DWT->CYCCNT;
    s_sample_sequence++;

    if (!App_ApplyAdcPairs(pairs, s_sample_sequence)) {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return;
    }

    App_FastStep20kHz();
    status = App_GetStatus();
    command = App_GetPwmCommand();

    if ((status == NULL) || (command == NULL) || !status->board_config_valid || !status->pwm_allowed) {
        /* 板级配置没通过校验，或状态机不允许输出 → 全关（这是"上电安全锁"的执行点） */
        BoardRuntime_DisablePwm();
        s_delayed_pwm_valid = false;
    } else {
        /*
         * 当前采样计算出的占空比留到下一帧装载，与仿真中的一周期
         * 计算延迟一致；第一次进入 RUN 仍保持逆变 MOE 关闭。
         */
        if (s_delayed_pwm_valid) {
            if (!BoardRuntime_ApplyDelayedPwm(&s_delayed_pwm)) {
                s_delayed_pwm_valid = false;
                return;
            }
        } else {
            /* 还没有可用的延迟命令（刚进 RUN 的第一帧）→ 先保持全关 */
            BoardRuntime_DisablePwm();
        }
        if (!s_trip_latched) {
            /* 本帧命令存起来，下一帧才装载（延迟一拍，见函数头说明） */
            s_delayed_pwm = *command;
            s_delayed_pwm_valid = true;
        }
    }

    elapsed = DWT->CYCCNT - started;
    if (elapsed > s_max_fast_cycles) s_max_fast_cycles = elapsed;
    if (elapsed > (SystemCoreClock / 20000U)) {
        /* 超过 50 us（一个 20 kHz 周期）→ 控制环已超时，跳闸停机 */
        BoardRuntime_LatchTrip(PROTECTION_FAULT_LOOP_OVERRUN);
    }
}

/*
 * 把 DMA 缓冲里的 4 组（rank）原始双字拷进局部数组再交给控制环。
 * 先拷贝的原因：DMA 缓冲区是硬件持续写入的，直接在里面解包可能
 * 读到"下一帧已经覆盖了一半"的混合数据。offset = 0 表示半满，
 * offset = ADC_DUAL_RANK_COUNT 表示全满（两个回调各走一次）。
 */
static void BoardRuntime_CopyAndProcessAdcFrame(uint32_t offset) {
    uint32_t pairs[ADC_DUAL_RANK_COUNT];
    uint32_t rank;

    for (rank = 0U; rank < ADC_DUAL_RANK_COUNT; rank++) pairs[rank] = s_adc_dma[offset + rank];
    BoardRuntime_ProcessAdcFrame(pairs);
}

/*
 * 尝试清除故障锁存（* 长按调用）。只在 FAULT_LATCH 状态下有效。
 *
 * 顺序即安全策略：先关 PWM、作废待装载命令，再确认两路 FAULT/BKIN
 * 输入都已释放（低电平），才清 BIF 与 NVIC 悬挂位；清完再复查一遍
 * 输入与标志，若复位窗口里又来了新故障就放弃这次复位——宁可多按一次，
 * 也不能在故障还在时把 MOE 挂回去。返回 true 表示确实复位成功。
 */
static bool BoardRuntime_TryResetFault(void) {
    const AppStatus *status = App_GetStatus();
    uint32_t primask;
    bool reset = false;

    if ((status == NULL) || (status->state != FAULT_LATCH)) return false;

    primask = BoardRuntime_EnterCritical();
    BoardRuntime_DisablePwm();
    s_delayed_pwm_valid = false;

    /*
     * 仅在两路FAULT/BKIN均已释放时清除锁存。清BIF后再次检查输入和标志，
     * 若复位窗口内出现新故障，硬件仍会保持MOE关闭并在退出临界区后进ISR。
     */
    if (!BoardRuntime_BreakInputActive()) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
        HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
        HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
        __DMB();
        if (!BoardRuntime_BreakInputActive() && !BoardRuntime_BreakFlagSet()) {
            s_trip_latched = false;
            App_ResetFault();
            s_fault_display_page = 0U;
            s_fault_display_snapshot.valid = false;
            reset = true;
        }
    }
    BoardRuntime_ExitCritical(primask);
    return reset;
}

/*
 * 切换运行模式（F0/1/2 键）。只有 OFF 或"已锁存的 FAULT"且 PWM 已关
 * 时才允许——切模式会丢弃上一帧的 PWM 命令并把两级门极关死。
 * 注意：切模式**不**清除故障锁存（清故障只能 * 长按）。
 */
/*
 * 短按按键处理（码值来自 key_board.c 的 4 位编码）。
 *   A     —— 输出频率在 30/60 Hz 间切换（走 InverterControl_SetFrequency）；
 *   B/C   —— 输出线电压 −0.1 / +0.1 V（赛题 32 V ± 0.1 V 现场微调）；
 *   D     —— 请求停机并立即关门极；若正停在"输出过压"故障页，
 *            顺便翻到快照的下一页（循环 5 页）。
 * 其它键（含 # / *）不在短按里处理，交给长按逻辑。
 */
static void BoardRuntime_HandleShortKey(uint8_t key) {
    const AppStatus *status = App_GetStatus();

    switch (key) {
    case 4U: /* A */
        if (status != NULL) (void)App_SetOutputFrequency((status->output_frequency_hz < 45.0f) ? 60U : 30U);
        break;
    case 8U: /* B: 输出线电压 -0.1 V（赛题 32V±0.1V 现场微调） */
        if (status != NULL) (void)App_SetOutputLineVoltageReference(status->output_line_voltage_reference_v - 0.1f);
        break;
    case 12U: /* C: 输出线电压 +0.1 V */
        if (status != NULL) (void)App_SetOutputLineVoltageReference(status->output_line_voltage_reference_v + 0.1f);
        break;
    case 16U: /* D */
        App_RequestStop();
        BoardRuntime_DisablePwm();
        s_delayed_pwm_valid = false;
        if ((status != NULL) && (status->state == FAULT_LATCH) && status->fault_snapshot.valid &&
            ((status->fault_snapshot.faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL)) {
            s_fault_display_page = (uint8_t)((s_fault_display_page + 1U) % BOARD_FAULT_PAGE_COUNT);
        }
        break;
    default:
        break;
    }
}

/*
 * 键盘扫描（后台 10 ms 调一次）：消抖 → 边沿触发短按 → 长按检测。
 *
 * 消抖：候选键连续 3 次采样不变（≈30 ms）才认作稳定键，避免抖动
 * 造成重复触发。稳定键变化（按下沿）时立刻执行一次短按动作。
 *
 * 长按（≥2 s，只按键 # 和 *，且每次按下只触发一次）：
 *   # —— 请求启动（App_RequestStart，真正能否进 RUN 由状态机与配置校验决定）
 *   * —— 清除故障锁存（BoardRuntime_TryResetFault）
 */
static void BoardRuntime_ScanKeys(uint32_t now) {
    uint8_t raw = (uint8_t)Read_Key();

    if (raw != s_key_candidate) {
        s_key_candidate = raw;
        s_key_count = 1U;
    } else if (s_key_count < BOARD_KEY_DEBOUNCE_SAMPLES) {
        s_key_count++;
    }

    if ((s_key_count >= BOARD_KEY_DEBOUNCE_SAMPLES) && (s_key_stable != s_key_candidate)) {
        s_key_stable = s_key_candidate;
        s_key_pressed_at = now;
        s_key_long_handled = false;
        if (s_key_stable != 0U) BoardRuntime_HandleShortKey(s_key_stable);
    }

    if (!s_key_long_handled && ((s_key_stable == 15U) || (s_key_stable == 13U)) &&
        ((now - s_key_pressed_at) >= BOARD_LONG_PRESS_MS)) {
        if (s_key_stable == 15U) /* # */
            App_RequestStart();
        else /* * */ (void)BoardRuntime_TryResetFault();
        s_key_long_handled = true;
    }
}

/* 状态枚举 → 状态名；越界返回 UNKNOWN，保证画面不会乱 */
static const char *BoardRuntime_StateText(app_state_t state) {
    static const char *const names[] = {"OFF", "SELF_TEST", "PRECHARGE", "READY", "RUN", "FAULT", "DISCHARGE"};

    return ((uint32_t)state < 7U) ? names[(uint32_t)state] : "UNKNOWN";
}

/* 逆变控制方式 → "CTRL:CL"（闭环）/ "CTRL:OL"（开环诊断） */
static const char *BoardRuntime_ControlModeText(inverter_run_mode_t mode) {
    if (mode == INVERTER_CLOSED_LOOP) return "CTRL:CL";
    if (mode == INVERTER_OPEN_LOOP) return "CTRL:OL";
    return "CTRL:??";
}

/*
 * 取最低位有效的故障位，返回**1 基**故障码（与 FaultText 下标、
 * README 故障码表一一对应）。faults = 0 时返回 0 = 无故障。
 */
static uint32_t BoardRuntime_FirstFault(uint32_t faults) {
    uint32_t bit = 0U;

    if (faults == 0U) return 0U;
    while (((faults >> bit) & 1U) == 0U) bit++;
    return bit + 1U;
}

/* 故障码 → OLED 显示的短名（整屏 48 行那格，长度受限） */
static const char *BoardRuntime_FaultText(uint32_t fault_code) {
    /* 下标 = 故障码（1 基），与 protection.h 的位序一一对应 */
    static const char *const names[] = {"NONE",  "ADC INVALID", "SAMPLE STALE",  "PHASE OC",     "DC OV",
                                        "DC UV", "OUTPUT OV",   "EXTERNAL/BKIN", "LOOP OVERRUN", "CONTROL"};

    return (fault_code < (sizeof(names) / sizeof(names[0]))) ? names[fault_code] : "UNKNOWN";
}

/*
 * 浮点 → 放大后的无符号整数（显示用）。三个保险：
 * 非有限值 → 0；取绝对值（配合调用方单独画符号位）；超过上限就钳位——
 * 否则尺寸算错会把 OLED 写花。
 */
static uint32_t BoardRuntime_ScaledMagnitude(float value, float scale, uint32_t maximum) {
    uint32_t scaled;

    if (!isfinite(value)) return 0U;
    value = fabsf(value);
    scaled = (uint32_t)(value * scale + 0.5f);
    return (scaled > maximum) ? maximum : scaled;
}

/* 显示 ±x.xx（电流用：量程小、需要两位小数） */
static void BoardRuntime_ShowSignedHundredths(uint8_t x, uint8_t y, float value) {
    uint32_t scaled = BoardRuntime_ScaledMagnitude(value, 100.0f, 999U);

    OLED_ShowChar(x, y, (value < 0.0f) ? '-' : '+', 8, 1);
    OLED_ShowNum(x + 6U, y, scaled / 100U, 1, 8, 1);
    OLED_ShowChar(x + 12U, y, '.', 8, 1);
    OLED_ShowNum(x + 18U, y, scaled % 100U, 2, 8, 1);
}

/* 显示 ±xx.x（电压用：一位小数够） */
static void BoardRuntime_ShowSignedTenths(uint8_t x, uint8_t y, float value) {
    uint32_t scaled = BoardRuntime_ScaledMagnitude(value, 10.0f, 999U);

    OLED_ShowChar(x, y, (value < 0.0f) ? '-' : '+', 8, 1);
    OLED_ShowNum(x + 6U, y, scaled / 10U, 2, 8, 1);
    OLED_ShowChar(x + 18U, y, '.', 8, 1);
    OLED_ShowNum(x + 24U, y, scaled % 10U, 1, 8, 1);
}

/* 显示 xx.x（无符号：负数/NaN 一律当 0，用于 SET/VDC 等不可能是负的量） */
static void BoardRuntime_ShowUnsignedTenths(uint8_t x, uint8_t y, float value) {
    uint32_t scaled;

    if (!isfinite(value) || (value < 0.0f)) value = 0.0f;
    scaled = (uint32_t)(value * 10.0f + 0.5f);
    if (scaled > 999U) scaled = 999U;
    OLED_ShowNum(x, y, scaled / 10U, 2, 8, 1);
    OLED_ShowChar(x + 12U, y, '.', 8, 1);
    OLED_ShowNum(x + 18U, y, scaled % 10U, 1, 8, 1);
}

/* 显示 x.xxx（显示母线电流比例 K = 0.533 这类小数） */
static void BoardRuntime_ShowUnsignedThousandths(uint8_t x, uint8_t y, float value) {
    uint32_t scaled;

    if (!isfinite(value) || (value < 0.0f)) value = 0.0f;
    scaled = (uint32_t)(value * 1000.0f + 0.5f);
    if (scaled > 9999U) scaled = 9999U;
    OLED_ShowNum(x, y, scaled / 1000U, 1, 8, 1);
    OLED_ShowChar(x + 6U, y, '.', 8, 1);
    OLED_ShowNum(x + 12U, y, scaled % 1000U, 3, 8, 1);
}

/* 显示 3 位无符号整数 */
static void BoardRuntime_ShowUnsignedInteger3(uint8_t x, uint8_t y, float value) {
    uint32_t scaled = BoardRuntime_ScaledMagnitude(value, 1.0f, 999U);

    OLED_ShowNum(x, y, scaled, 3, 8, 1);
}

/*
 * 用通道标定把原始码换算成物理量（物理量 = (码 − 零点) × 增益）。
 * 与 measurement.c 同一公式——这里只是给"没有 Measurements 结构体的
 * 故障快照原始数据"做现场换算，显示用，不参与控制。
 */
static float BoardRuntime_CalibrateRaw(const ChannelCalibration *calibration, uint16_t raw) {
    if ((calibration == NULL) || !isfinite(calibration->offset_count) || !isfinite(calibration->gain)) return 0.0f;
    return ((float)raw - calibration->offset_count) * calibration->gain;
}

/*
 * 从故障快照的快速历史里取"往前 lag 拍"的样本（1 = 触发前一拍）。
 * 越界/参数非法返回 NULL，调用方按 N/A 显示。
 */
static const AppFaultRawSample *BoardRuntime_HistoryAtLag(const AppFaultSnapshot *snapshot, uint8_t lag) {
    if ((snapshot == NULL) || (lag == 0U) || (lag > snapshot->fast_history_count)) return NULL;
    return &snapshot->fast_history[snapshot->fast_history_count - lag];
}

/*
 * 由快照里两路线电压原始码重构三相相电压，返回三相绝对值的最大者。
 * 相电压重构公式与 measurement.c 一致（三线制：va=(2vab+vbc)/3 等）。
 */
static float BoardRuntime_HistoryPhaseMax(const AppFaultRawSample *sample, const MeasurementCalibration *calibration) {
    float vab;
    float vbc;
    float va;
    float vb;
    float vc;
    float maximum;

    if ((sample == NULL) || (calibration == NULL)) return 0.0f;
    vab = BoardRuntime_CalibrateRaw(&calibration->vab, sample->pc2_raw);
    vbc = BoardRuntime_CalibrateRaw(&calibration->vbc, sample->pc3_raw);
    va = (2.0f * vab + vbc) / 3.0f;
    vb = (vbc - vab) / 3.0f;
    vc = -(vab + 2.0f * vbc) / 3.0f;
    maximum = fabsf(va);
    if (fabsf(vb) > maximum) maximum = fabsf(vb);
    return (fabsf(vc) > maximum) ? fabsf(vc) : maximum;
}

/* αβ 矢量幅值 = √(α²+β²)；非有限值按 0 处理（诊断显示用） */
static float BoardRuntime_VectorMagnitude(AlphaBeta vector) {
    float square = vector.alpha * vector.alpha + vector.beta * vector.beta;

    return (isfinite(square) && (square > 0.0f)) ? sqrtf(square) : 0.0f;
}

/* 故障页统一页脚提示：D 翻页、* 长按 2 秒复位 */
static void BoardRuntime_ShowFaultFooter(void) {
    OLED_ShowString(0, 56, (u8 *)"D:NEXT *2S:RST", 8, 1);
}

/*
 * 在 Va/Vb/Vc 里找绝对值最大的一相（真正触发输出过压判据的那相），
 * 通过 *phase 返回相名（'A'/'B'/'C'），函数返回该相的带符号值。
 */
static float BoardRuntime_OutputOvTripValue(const AppFaultSnapshot *snapshot, u8 *phase) {
    float value = snapshot->va_v;

    *phase = 'A';
    if (fabsf(snapshot->vb_v) > fabsf(value)) {
        value = snapshot->vb_v;
        *phase = 'B';
    }
    if (fabsf(snapshot->vc_v) > fabsf(value)) {
        value = snapshot->vc_v;
        *phase = 'C';
    }
    return value;
}

/*
 * 输出过压故障页第 0 页：触发相 + 触发值 + 帧号 + 当前三相/线电压。
 * 第 1~4 页由 BoardRuntime_ShowOutputOvPage 按 s_fault_display_page
 * 分派（D 键翻页）。
 */
static void BoardRuntime_ShowOutputOvSnapshot(const AppFaultSnapshot *snapshot) {
    u8 phase;
    float trip_value = BoardRuntime_OutputOvTripValue(snapshot, &phase);

    OLED_ShowString(0, 0, (u8 *)"FAULT OUTPUT OV", 8, 1);
    OLED_ShowString(0, 8, (u8 *)BoardRuntime_ControlModeText(snapshot->inverter_run_mode), 8, 1);
    OLED_ShowString(66, 8, (u8 *)"SET:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(90, 8, snapshot->dc_voltage_reference_v);

    OLED_ShowString(0, 16, (u8 *)"TRIP:", 8, 1);
    OLED_ShowChar(30, 16, phase, 8, 1);
    BoardRuntime_ShowSignedTenths(42, 16, trip_value);
    OLED_ShowString(78, 16, (u8 *)"S:", 8, 1);
    OLED_ShowNum(90, 16, snapshot->sample_sequence % 1000000UL, 6, 8, 1);

    OLED_ShowString(0, 24, (u8 *)"VA:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 24, snapshot->va_v);
    OLED_ShowString(66, 24, (u8 *)"VB:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 24, snapshot->vb_v);

    OLED_ShowString(0, 32, (u8 *)"VC:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 32, snapshot->vc_v);
    OLED_ShowString(66, 32, (u8 *)"PWM:OFF", 8, 1);

    OLED_ShowString(0, 40, (u8 *)"AB:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 40, snapshot->vab_v);
    OLED_ShowString(66, 40, (u8 *)"BC:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 40, snapshot->vbc_v);

    OLED_ShowString(0, 48, (u8 *)"R2:", 8, 1);
    OLED_ShowNum(18, 48, snapshot->pc2_raw, 4, 8, 1);
    OLED_ShowString(66, 48, (u8 *)"R3:", 8, 1);
    OLED_ShowNum(84, 48, snapshot->pc3_raw, 4, 8, 1);

    OLED_ShowString(0, 56, (u8 *)"M:", 8, 1);
    OLED_ShowNum(12, 56, snapshot->faults, 4, 8, 1);
    OLED_ShowString(42, 56, (u8 *)"D:NEXT *2S:RST", 8, 1);
}

/*
 * 输出过压 · 第 1 页：触发前后 1/2/4/8/16 拍的"线电压重构相电压最大值"
 * 与两路原始码（PC2/PC3）——用来判断故障是"一拍尖峰"还是"持续抬高"。
 */
static void BoardRuntime_ShowOutputOvVoltageHistory(const AppFaultSnapshot *snapshot) {
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U};
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const AppFaultRawSample *sample;
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE N VMAX R2 R3", 8, 1);
    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++) {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowNum(0, y, lags[row], 2, 8, 1);
        sample = BoardRuntime_HistoryAtLag(snapshot, lags[row]);
        if ((sample == NULL) || (cfg == NULL)) {
            OLED_ShowString(18, y, (u8 *)"N/A", 8, 1);
            continue;
        }
        BoardRuntime_ShowUnsignedTenths(18, y, BoardRuntime_HistoryPhaseMax(sample, &cfg->measurement));
        OLED_ShowNum(60, y, sample->pc2_raw, 4, 8, 1);
        OLED_ShowNum(96, y, sample->pc3_raw, 4, 8, 1);
    }
    BoardRuntime_ShowFaultFooter();
}

/*
 * 输出过压 · 第 2 页：触发前后各拍的 Ia/Ib 历史 + 触发拍两路原始码。
 * 过压往往先伴随电流异常（负载断开/滤波谐振），这页是判断依据。
 */
static void BoardRuntime_ShowOutputOvCurrentHistory(const AppFaultSnapshot *snapshot) {
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U};
    const BoardPowerConfig *cfg = BoardPower_GetConfig();
    const AppFaultRawSample *sample;
    float ia;
    float ib;
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE N   IA      IB", 8, 1);
    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++) {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowNum(0, y, lags[row], 2, 8, 1);
        sample = BoardRuntime_HistoryAtLag(snapshot, lags[row]);
        if ((sample == NULL) || (cfg == NULL)) {
            OLED_ShowString(18, y, (u8 *)"N/A", 8, 1);
            continue;
        }
        /* 实物映射：PC0 = Ib、PC1 = Ia（与 README 第 3 节一致）。 */
        ia = BoardRuntime_CalibrateRaw(&cfg->measurement.ia, sample->pc1_raw);
        ib = BoardRuntime_CalibrateRaw(&cfg->measurement.ib, sample->pc0_raw);
        BoardRuntime_ShowSignedHundredths(18, y, ia);
        BoardRuntime_ShowSignedHundredths(72, y, ib);
    }
    OLED_ShowString(0, 48, (u8 *)"R0:", 8, 1);
    OLED_ShowNum(18, 48, snapshot->trigger_raw.pc0_raw, 4, 8, 1);
    OLED_ShowString(66, 48, (u8 *)"R1:", 8, 1);
    OLED_ShowNum(84, 48, snapshot->trigger_raw.pc1_raw, 4, 8, 1);
    BoardRuntime_ShowFaultFooter();
}

/*
 * 输出过压 · 第 3 页：故障前一拍的**控制内部量**（电压参考/实测/误差、
 * 电流参考/实测、桥压指令、三相占空比、过调制缩放）。这一页是"控制环
 * 当时在想什么"的唯一记录，调过压问题时先看它。
 */
static void BoardRuntime_ShowOutputOvControl(const AppFaultSnapshot *snapshot) {
    const AppFaultControlSample *control = &snapshot->previous_control;
    AlphaBeta error;
    float reference;
    float measured;
    float voltage_error;
    float current_reference;
    float current_measured;
    float voltage_command;

    OLED_ShowString(0, 0, (u8 *)"CTRL PREV S:", 8, 1);
    OLED_ShowNum(72, 0, control->sample_sequence % 1000000UL, 6, 8, 1);
    if (!control->valid) {
        OLED_ShowString(0, 16, (u8 *)"NO CONTROL DATA", 8, 1);
        BoardRuntime_ShowFaultFooter();
        return;
    }

    error.alpha = control->voltage_reference.alpha - control->voltage_measured.alpha;
    error.beta = control->voltage_reference.beta - control->voltage_measured.beta;
    reference = BoardRuntime_VectorMagnitude(control->voltage_reference);
    measured = BoardRuntime_VectorMagnitude(control->voltage_measured);
    voltage_error = BoardRuntime_VectorMagnitude(error);
    current_reference = BoardRuntime_VectorMagnitude(control->current_reference);
    current_measured = BoardRuntime_VectorMagnitude(control->current_measured);
    voltage_command = BoardRuntime_VectorMagnitude(control->voltage_command);

    OLED_ShowString(0, 8, (u8 *)"VR:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(18, 8, reference);
    OLED_ShowString(66, 8, (u8 *)"VM:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(84, 8, measured);

    OLED_ShowString(0, 16, (u8 *)"VE:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(18, 16, voltage_error);
    OLED_ShowString(66, 16, (u8 *)"IR:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(84, 16, current_reference);

    OLED_ShowString(0, 24, (u8 *)"IM:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(18, 24, current_measured);
    OLED_ShowString(66, 24, (u8 *)"VC:", 8, 1);
    BoardRuntime_ShowUnsignedInteger3(84, 24, voltage_command);

    OLED_ShowString(0, 32, (u8 *)"DA:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(18, 32, control->duty_a);
    OLED_ShowString(66, 32, (u8 *)"DB:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(84, 32, control->duty_b);

    OLED_ShowString(0, 40, (u8 *)"DC:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(18, 40, control->duty_c);
    OLED_ShowString(66, 40, (u8 *)"SC:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(84, 40, control->applied_scale);

    OLED_ShowString(0, 48, (u8 *)"SAT:", 8, 1);
    OLED_ShowString(24, 48, (u8 *)(control->saturated ? "YES" : "NO"), 8, 1);
    BoardRuntime_ShowFaultFooter();
}

/*
 * 输出过压 · 第 4 页：按工频周期聚合的趋势（每周期最大/均值），
 * 用于判断故障是渐进恶化还是突发——逐拍数据看不出周期级走势。
 */
static void BoardRuntime_ShowOutputOvCycleTrend(const AppFaultSnapshot *snapshot) {
    static const uint8_t lags[] = {1U, 2U, 4U, 8U, 16U, 32U};
    uint8_t row;

    OLED_ShowString(0, 0, (u8 *)"PRE:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(24, 0, snapshot->partial_cycle_peak_v);
    OLED_ShowString(60, 0, (u8 *)"N:", 8, 1);
    OLED_ShowNum(72, 0, snapshot->partial_cycle_samples, 4, 8, 1);

    for (row = 0U; row < (uint8_t)(sizeof(lags)); row++) {
        uint8_t y = (uint8_t)((row + 1U) * 8U);

        OLED_ShowString(0, y, (u8 *)"C-", 8, 1);
        OLED_ShowNum(12, y, lags[row], 2, 8, 1);
        OLED_ShowChar(24, y, ':', 8, 1);
        if (lags[row] <= snapshot->cycle_peak_count)
            BoardRuntime_ShowUnsignedTenths(30, y, snapshot->cycle_peak_v[lags[row] - 1U]);
        else OLED_ShowString(30, y, (u8 *)"N/A", 8, 1);
    }
    BoardRuntime_ShowFaultFooter();
}

/* 输出过压故障页分派：page 0~4 对应上面五个页面（D 键循环翻页） */
static void BoardRuntime_ShowOutputOvPage(const AppFaultSnapshot *snapshot) {
    switch (s_fault_display_page) {
    case 1U:
        BoardRuntime_ShowOutputOvVoltageHistory(snapshot);
        break;
    case 2U:
        BoardRuntime_ShowOutputOvCurrentHistory(snapshot);
        break;
    case 3U:
        BoardRuntime_ShowOutputOvControl(snapshot);
        break;
    case 4U:
        BoardRuntime_ShowOutputOvCycleTrend(snapshot);
        break;
    default:
        BoardRuntime_ShowOutputOvSnapshot(snapshot);
        break;
    }
}

/*
 * OLED 整屏刷新（后台 100 ms 调一次）。两类画面：
 *
 *  ① 故障诊断页：快照有效且故障是"输出过压"时，整屏让给诊断页并提前返回。
 *     快照只在**帧号变化时**重新拷贝（诊断记录一旦生成就冻结），
 *     这样 D 键翻页时看到的是同一个故障现场，不会被后续数据冲掉。
 *  ② 常规状态页：状态/频率、相电流、线电压、控制方式、输出参考、
 *     故障码、PWM 门控状态、按键提示。
 *
 * 开头用一段短临界区把 AppStatus 整块拷进局部变量：之后画屏全程用
 * 快照，20 kHz ISR 再怎么改状态也不会画成"上半屏新值、下半屏旧值"。
 */
static void BoardRuntime_RefreshDisplay(void) {
    BoardDisplayStatus status;
    const AppStatus *shared = App_GetStatus();
    uint32_t primask;
    uint32_t fault_code;
    bool diagnostic_snapshot_available;

    if (shared == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();
    status.inverter_run_mode = shared->inverter_run_mode;
    status.state = shared->state;
    status.faults = shared->faults;
    status.output_frequency_hz = shared->output_frequency_hz;
    status.output_line_voltage_reference_v = shared->output_line_voltage_reference_v;
    status.dc_voltage_reference_v = shared->dc_voltage_reference_v;
    status.dc_voltage_v = shared->dc_voltage_v;
    status.ia_a = shared->ia_a;
    status.ib_a = shared->ib_a;
    status.vab_v = shared->vab_v;
    status.vbc_v = shared->vbc_v;
    status.pwm_allowed = shared->pwm_allowed;
    diagnostic_snapshot_available =
        shared->fault_snapshot.valid && ((shared->fault_snapshot.faults & PROTECTION_FAULT_OUTPUT_OVERVOLTAGE) != 0UL);
    /* 只在"第一次拿到快照"或"帧号变了（新故障）"时重新拷——
     * 否则 D 键翻页期间画面会被新数据顶掉，看不到故障现场。 */
    if (diagnostic_snapshot_available &&
        (!s_fault_display_snapshot.valid ||
         (s_fault_display_snapshot.sample_sequence != shared->fault_snapshot.sample_sequence))) {
        s_fault_display_snapshot = shared->fault_snapshot;
        s_fault_display_page = 0U;
    } else if (!diagnostic_snapshot_available) {
        s_fault_display_snapshot.valid = false;
    }
    if (primask == 0U) __enable_irq();
    fault_code = BoardRuntime_FirstFault(status.faults);

    OLED_Clear();
    if (s_fault_display_snapshot.valid) {
        BoardRuntime_ShowOutputOvPage(&s_fault_display_snapshot);
        OLED_Refresh();
        return;
    }
    s_fault_display_page = 0U;

    OLED_ShowString(0, 0, (u8 *)"DCAC", 8, 1);
    OLED_ShowString(30, 0, (u8 *)BoardRuntime_StateText(status.state), 8, 1);
    OLED_ShowNum(96, 0, (uint32_t)(status.output_frequency_hz + 0.5f), 2, 8, 1);
    OLED_ShowString(108, 0, (u8 *)"Hz", 8, 1);

    OLED_ShowString(0, 8, (u8 *)"IA:", 8, 1);
    BoardRuntime_ShowSignedHundredths(18, 8, status.ia_a);
    OLED_ShowString(66, 8, (u8 *)"IB:", 8, 1);
    BoardRuntime_ShowSignedHundredths(84, 8, status.ib_a);

    OLED_ShowString(0, 16, (u8 *)"AB:", 8, 1);
    BoardRuntime_ShowSignedTenths(18, 16, status.vab_v);
    OLED_ShowString(66, 16, (u8 *)"BC:", 8, 1);
    BoardRuntime_ShowSignedTenths(84, 16, status.vbc_v);

    OLED_ShowString(0, 24, (u8 *)BoardRuntime_ControlModeText(status.inverter_run_mode), 8, 1);
    OLED_ShowString(66, 24, (u8 *)"SET:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(90, 24, status.dc_voltage_reference_v);

    OLED_ShowString(0, 32, (u8 *)"OUT:", 8, 1);
    BoardRuntime_ShowUnsignedTenths(24, 32, status.output_line_voltage_reference_v);
    OLED_ShowString(66, 32, (u8 *)"FLT:", 8, 1);
    OLED_ShowNum(90, 32, fault_code, 2, 8, 1);

    OLED_ShowString(0, 40, (u8 *)"K:", 8, 1);
    BoardRuntime_ShowUnsignedThousandths(12, 40, APP_INVERTER_OUTPUT_LINE_RMS_RATIO);
    OLED_ShowString(66, 40, (u8 *)(status.pwm_allowed ? "PWM:ON" : "PWM:OFF"), 8, 1);
    if (fault_code != 0U) {
        OLED_ShowString(0, 48, (u8 *)"ERR:", 8, 1);
        OLED_ShowString(24, 48, (u8 *)BoardRuntime_FaultText(fault_code), 8, 1);
        OLED_ShowString(0, 56, (u8 *)"*HOLD2S RESET", 8, 1);
    } else {
        OLED_ShowString(0, 48, (u8 *)"A:Hz", 8, 1);
        OLED_ShowString(0, 56, (u8 *)"B:- C:+ #2S D:STOP", 8, 1);
    }
    OLED_Refresh();
}

/*
 * 板级上电初始化（在 main 的 MX_*_Init 之后调用一次）。
 *
 * 顺序与理由：
 *   1) **先置"已跳闸"**：上电默认视为故障锁存，只有 * 长按才解锁——
 *      这样任何上电瞬间的异常都不会让门极乱开；
 *   2) 逆变门极全关 + 三个 CCR 回中性，并把 TIM8 的 ADC 触发点写死；
 *   3) 打开 DWT 周期计数器（快环耗时统计用，见 ProcessAdcFrame）；
 *   4) App_Init / 键盘 / OLED 初始化；
 *   5) 定时器与 App 都就绪后，才使能 TIM BREAK 中断并检查 BKIN——
 *      TIM1 初始化阶段不打开 BIE 就是为了避免 TIM8 句柄未就绪时进回调；
 *   6) 启动 ADC（先从 ADC2，再 ADC1 多模式 DMA），最后先跑 TIM8、
 *      再由 TIM1 的更新事件同步复位 TIM8 —— 采样与 PWM 载波同相靠这一步。
 * 任一步失败即 LatchTrip，绝不带着半初始化状态继续跑。
 */
void BoardRuntime_Init(void) {
    const uint32_t neutral = BOARD_PWM_PERIOD_COUNTS / 2U;
    uint32_t primask;

    s_trip_latched = true;
    s_delayed_pwm_valid = false;
    s_fault_display_page = 0U;
    s_fault_display_snapshot.valid = false;
    BoardRuntime_DisablePwm();
    TIM1->CCR1 = neutral;
    TIM1->CCR2 = neutral;
    TIM1->CCR3 = neutral;
    /*
     * TIM8 的 CH4 只用来产生 ADC1 触发：写死触发点，不随其它状态变化。
     * （tim.c 里 CH4 初始 Pulse 也是 3447，这里再写一次是为了防止
     *   CubeMX 重新生成后把初值改掉而没人发现。）
     */
    TIM8->CCR4 = BOARD_ADC_TRIGGER_COMPARE;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    App_Init();
    Key_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_Refresh();

    /*
     * TIM1初始化阶段不打开BIE，避免TIM8句柄尚未就绪时进入全局关断回调。
     * 到这里两路定时器和App均已初始化，再统一清标志并使能最高优先级Break。
     */
    primask = BoardRuntime_EnterCritical();
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
    s_trip_latched = BoardRuntime_BreakInputActive() || BoardRuntime_BreakFlagSet();
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
    BoardRuntime_ExitCritical(primask);

    if (BoardRuntime_TripPending()) BoardRuntime_LatchTrip(PROTECTION_FAULT_EXTERNAL);

    /*
     * 双ADC中先使能从ADC2，再启动ADC1多模式DMA；TIM8先运行，
     * 随后由TIM1更新事件同步复位。门极输出通道仍保持禁用。
     */
    if ((HAL_ADC_Start(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)s_adc_dma, ADC_DMA_WORD_COUNT) != HAL_OK)) {
        BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
        return;
    }

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE(&htim8);
    __HAL_TIM_ENABLE(&htim1);
}

/*
 * 后台任务（主循环 while(1) 里调用）：
 *   App_BackgroundTask()  —— 应用层杂务
 *   10 ms                 —— 键盘扫描（含消抖与长按判定）
 *   100 ms                —— OLED 刷新
 * 控制算法**不在这里**：它在 ADC DMA 中断里跑，保证与 PWM 同步。
 */
void BoardRuntime_BackgroundTask(void) {
    uint32_t now = HAL_GetTick();

    App_BackgroundTask();
    if ((now - s_last_key_scan) >= BOARD_KEY_SCAN_PERIOD_MS) {
        s_last_key_scan = now;
        BoardRuntime_ScanKeys(now);
    }
    if ((now - s_last_display) >= BOARD_DISPLAY_PERIOD_MS) {
        s_last_display = now;
        BoardRuntime_RefreshDisplay();
    }
}

/* DMA 半满中断：缓冲区前半帧（offset 0）就绪 → 处理一帧控制 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) BoardRuntime_CopyAndProcessAdcFrame(0U);
}

/* DMA 全满中断：缓冲区后半帧（offset = rank 数）就绪 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) BoardRuntime_CopyAndProcessAdcFrame(ADC_DUAL_RANK_COUNT);
}

/* ADC 出错（过载/配置异常等）→ 锁存测量故障，等人工复位 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if ((hadc->Instance == ADC1) || (hadc->Instance == ADC2)) BoardRuntime_LatchTrip(PROTECTION_FAULT_MEASUREMENT);
}

/*
 * 定时器 Break（BKIN）中断回调：硬件已经抢先清了 MOE，这里再走一遍
 * 全局封锁 + 锁存，禁止 AOE 或软件自动恢复。TIM1/TIM8 任一触发都算
 * "外部故障"（PROTECTION_FAULT_EXTERNAL）。
 */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
    if ((htim->Instance == TIM1) || (htim->Instance == TIM8)) {
        /*
         * BKIN已由定时器硬件先清MOE；这里再全局封锁两级并锁存故障，
         * 禁止AOE或软件自动恢复。
         */
        BoardRuntime_LatchTrip(PROTECTION_FAULT_EXTERNAL);
    }
}
