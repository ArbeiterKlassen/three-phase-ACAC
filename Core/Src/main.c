/*
 * 程序入口：初始化时钟与外设，然后把控制权交给板级运行时。
 *
 * 分工：
 *   本文件          —— 时钟树、外设初始化顺序、致命错误处理
 *   board_runtime.c —— 20 kHz 快环、PWM/BKIN/键盘/OLED 的板级适配
 *   App/            —— 纯控制算法，不碰寄存器（host_tests 才能在 PC 上跑）
 *
 * ⚠ 本文件由 CubeMX 生成。下面的中文注释是手写的，**重新生成代码会覆盖掉**。
 *   若改了 .ioc 重新生成，请先提交；生成后用
 *   `git checkout -- Core/Src/main.c` 取回本注释版本（生成内容本身不变时）。
 *
 * 启动顺序不能随意调换，理由见各调用处注释。
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

/* USER CODE BEGIN Includes */
#include "board_runtime.h"
/* USER CODE END Includes */

void SystemClock_Config(void);

int main(void)
{
  /* HAL 底层与节拍初始化。必须在任何外设之前。 */
  HAL_Init();

  /* 先把主频定到 168 MHz——后面所有定时器频率、死区时间、采样时序
   * 都从它派生，所以它必须排在任何外设前面。 */
  SystemClock_Config();

  /* 外设初始化，顺序有依赖：
   *   GPIO       先建立引脚复用与安全初值（此时功率管门极仍为关闭态）
   *   DMA        必须在 ADC 之前——ADC 一启动就开始搬运
   *   TIM1/TIM8  在 ADC 之前——ADC1 由 TIM8 的 TRGO 触发
   *   SPI3       最后——OLED 只做显示，与功率链路无关 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI3_Init();

  /*
   * 交给板级运行时。它会依次：
   *   1) 清 TIM1/TIM8 的 MOE 与通道使能——**上电第一步就把门极关死**
   *   2) 把各路比较值置于 50% 中性值（此时仍不开放门极）
   *   3) 启动 ADC 双模式 DMA，再启动 TIM8、TIM1
   *   4) 之后每收到一帧 ADC 数据就执行一次 20 kHz 快环
   *
   * 应用层在板级参数与采样标定通过校验前会硬锁止 RUN；
   * TIM4、DAC 与旧 icode 控制链不参与本控制链路。
   */
  BoardRuntime_Init();

  /* 后台循环：按键扫描（10 ms）与 OLED 刷新（100 ms）。
   * 控制算法不在这里——它在 ADC DMA 中断里跑，保证与 PWM 同相。 */
  while (1)
  {
    BoardRuntime_BackgroundTask();
  }
}

/**
  * @brief  System Clock Configuration
  *
  * 时钟树（改任何定时器参数前先看这张表）：
  *
  *   HSE 8 MHz
  *     └─ PLL (M=8, N=336, P=2) → VCO 336 MHz → SYSCLK 168 MHz
  *          ├─ AHB  = 168 MHz      (HCLK_DIV1)
  *          ├─ APB1 =  42 MHz      (HCLK_DIV4)
  *          │    └─ TIM2~TIM7 时钟 = 84 MHz  ← APB 分频≠1 时定时器时钟自动 ×2
  *          └─ APB2 =  84 MHz      (HCLK_DIV2)
  *               └─ TIM1/TIM8 时钟 = 168 MHz ← PWM 频率与死区都按它算
  *
  * 由它导出的三个常用数（改参数时对照）：
  *   PWM 频率 = 168 MHz / ((PSC+1) × (ARR+1))，中心对齐再除 2
  *   死区步长 tDTS = 1 / 168 MHz = 5.952 ns，DTG=148 → 1.000 us
  *   ADC 时钟 = PCLK2 / 4 = 21 MHz
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 使能电源时钟并把调压器设到最高档——168 MHz 必须用 Scale1，
   * 否则内核电压不足会随机死机。 */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* 外部 8 MHz 晶振经 PLL 倍频到 168 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 总线分频：APB2 给 84 MHz 是刻意的——定时器时钟会倍频回 168 MHz，
   * TIM1/TIM8 因此拿到最高分辨率（5.952 ns 一格），死区才配得出 1 us。 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  /* FLASH_LATENCY_5：168 MHz 下 Flash 需要 5 个等待周期，写小了会取指错乱 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * 致命错误处理：进入"门极全关 + 停机"状态，不再返回。
 *
 * 这里直接写 BDTR 寄存器清 MOE 而不调 HAL：HAL 可能依赖已经出问题的状态，
 * 而直接写寄存器能在几个时钟周期内关死输出。随后关中断，避免停在这里时
 * 还有中断在改寄存器。
 *
 * 触发来源：HAL 初始化失败、启动阶段的致命错误。
 * 注意：运行中的故障不走这里，走 board_runtime.c 的 BoardRuntime_LatchTrip。
 */
void Error_Handler(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM8->BDTR &= ~TIM_BDTR_MOE;
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/* HAL 参数断言失败的落点。量产固件不打开 USE_FULL_ASSERT；
 * 打开时建议在这里点灯或置一个可供调试器查看的标志。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
