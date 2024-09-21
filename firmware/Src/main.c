#include "main.h"

#include "goldobot/goldobot_main.h"
#include "cmsis_os.h"

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const *argument);

void FPU_IRQHandler(void) {
  while (1) {
  };
}

void NMI_Handler(void) {
  while (1) {
  };
}

void HardFault_Handler(void) __attribute__((naked));
void PrvGetRegistersFromStack(uint32_t *pulFaultStackAddress);

void HardFault_Handler(void) {
  __asm volatile(
      " tst lr, #4                                                \n"
      " ite eq                                                    \n"
      " mrseq r0, msp                                             \n"
      " mrsne r0, psp                                             \n"
      " ldr r1, [r0, #24]                                         \n"
      " ldr r2, handler2_address_const                            \n"
      " bx r2                                                     \n"
      " handler2_address_const: .word PrvGetRegistersFromStack    \n");
}

void MemManage_Handler(void) {
  while (1) {
  };
}

void BusFault_Handler(void) {
  while (1) {
  };
}

void UsageFault_Handler(void) {
  while (1) {
  };
}

void PrvGetRegistersFromStack(uint32_t *pulFaultStackAddress) {
  /* These are volatile to try and prevent the compiler/linker optimising them
  away as the variables never actually get used.  If the debugger won't show the
  values of the variables, make them global my moving their declaration outside
  of this function. */
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;  /* Link register. */
  volatile uint32_t pc;  /* Program counter. */
  volatile uint32_t psr; /* Program status register. */

  r0 = pulFaultStackAddress[0];
  r1 = pulFaultStackAddress[1];
  r2 = pulFaultStackAddress[2];
  r3 = pulFaultStackAddress[3];

  r12 = pulFaultStackAddress[4];
  lr = pulFaultStackAddress[5];
  pc = pulFaultStackAddress[6];
  psr = pulFaultStackAddress[7];

  /* When the following line is hit, the variables contain the register values. */
  for (;;)
    ;
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // this a driver call, pay it no mind
  /* Configure the system clock */
  InitSystemClockConfig(); // init the clock and tell it come external (switch to hsi if failure)

  /* USER CODE BEGIN 2 */

  /* add threads, ... */
  goldobot_main();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  while (1) {
  }
}

/**
 * @brief configure Hal clock, call directly driver function
 * @retval None
 */
void InitSystemClockConfig(void) {
  // Configures oscillators (like HSE, HSI, and PLL).
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // Configures the system and bus clocks (CPU, AHB, APB).
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  //Configures peripheral-specific clocks (USART, ADC, TIM, etc.).
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // High-Speed External (HSE) oscillator. Require external components
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; // Configures the system to use an external crystal oscillator that is bypassed (e.g., the clock is fed directly from an external source, not an oscillator).
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; // The internal 8 MHz oscillator is enabled as a fallback.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // The PLL takes an input clock (e.g., HSI at 8 MHz or HSE at 8 MHz) and multiplies it to generate a higher frequency. This allows the microcontroller to run at much higher speeds than the input clock itself.
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // The PLL source is the HSE (external clock).
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // The PLL multiplies the input clock by 9. If HSE is 8 MHz, the system clock will be 72 MHz.
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;  // No division on the PLL input.
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  /* This line selects which peripherals will have their clocks configured. Each peripheral can have a different clock source:

    USART1/USART2/USART3/UART5: The various UART clocks. For example:
        Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2: USART1 is clocked by APB2 (72 MHz).
        Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK: USART2 uses the system clock (72 MHz).
    I2C1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK: I2C1 uses the system clock.
    ADC12ClockSelection = RCC_ADC12PLLCLK_DIV1: ADC12 uses the PLL clock without any division.
    TIM1/TIM2/TIM16/TIM34: These timers are using the AHB clock (HCLK, 72 MHz).
    */
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART5 |
      RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM16 | RCC_PERIPHCLK_ADC12 |
      RCC_PERIPHCLK_TIM2 | RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
  /** Enables the Clock Security System, which detects clock failures (especially the HSE failure) and automatically switches to a backup clock,
   *  such as the HSI, if a failure occurs.
   */
  HAL_RCC_EnableCSS(); 
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
