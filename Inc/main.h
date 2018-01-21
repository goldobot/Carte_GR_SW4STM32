/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define NUCLEO_GPIO_PC13_Pin GPIO_PIN_13
#define NUCLEO_GPIO_PC13_GPIO_Port GPIOC
#define NUCLEO_GPIO_PC14_Pin GPIO_PIN_14
#define NUCLEO_GPIO_PC14_GPIO_Port GPIOC
#define MAXON_EN_Pin GPIO_PIN_15
#define MAXON_EN_GPIO_Port GPIOC
#define NUCLEO_ADC1_9_GPIO_Pin GPIO_PIN_0
#define NUCLEO_ADC1_9_GPIO_GPIO_Port GPIOC
#define NUCLEO_ADC1_8_GPIO_Pin GPIO_PIN_1
#define NUCLEO_ADC1_8_GPIO_GPIO_Port GPIOC
#define NUCLEO_ADC1_8_AIN_Pin GPIO_PIN_2
#define NUCLEO_ADC1_8_AIN_GPIO_Port GPIOC
#define NUCLEO_ADC1_9_AIN_Pin GPIO_PIN_3
#define NUCLEO_ADC1_9_AIN_GPIO_Port GPIOC
#define MAXON2_DIR_Pin GPIO_PIN_0
#define MAXON2_DIR_GPIO_Port GPIOA
#define MAXON2_PWM_Pin GPIO_PIN_1
#define MAXON2_PWM_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define STM32_PA5_Pin GPIO_PIN_5
#define STM32_PA5_GPIO_Port GPIOA
#define STM32_PA6_Pin GPIO_PIN_6
#define STM32_PA6_GPIO_Port GPIOA
#define STM32_PA7_Pin GPIO_PIN_7
#define STM32_PA7_GPIO_Port GPIOA
#define NUCLEO_DYNA_TX_Pin GPIO_PIN_4
#define NUCLEO_DYNA_TX_GPIO_Port GPIOC
#define NUCLEO_DYNA_DIR_Pin GPIO_PIN_5
#define NUCLEO_DYNA_DIR_GPIO_Port GPIOC
#define NUCLEO_GPIO_PB0_Pin GPIO_PIN_0
#define NUCLEO_GPIO_PB0_GPIO_Port GPIOB
#define NUCLEO_PWM_T3_4_Pin GPIO_PIN_1
#define NUCLEO_PWM_T3_4_GPIO_Port GPIOB
#define MAXON1_DIR_Pin GPIO_PIN_2
#define MAXON1_DIR_GPIO_Port GPIOB
#define NUCLEO_GPIO_PC6_Pin GPIO_PIN_6
#define NUCLEO_GPIO_PC6_GPIO_Port GPIOC
#define MAXON1_PWM_Pin GPIO_PIN_7
#define MAXON1_PWM_GPIO_Port GPIOC
#define NUCLEO_GPIO_PC9_Pin GPIO_PIN_9
#define NUCLEO_GPIO_PC9_GPIO_Port GPIOC
#define QEI1_CH_B_Pin GPIO_PIN_8
#define QEI1_CH_B_GPIO_Port GPIOA
#define QEI1_CH_A_Pin GPIO_PIN_9
#define QEI1_CH_A_GPIO_Port GPIOA
#define NUCLEO_DYNA_RX_Pin GPIO_PIN_10
#define NUCLEO_DYNA_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define FPGA_A7_Pin GPIO_PIN_15
#define FPGA_A7_GPIO_Port GPIOA
#define OLD_UART4_RX_Pin GPIO_PIN_11
#define OLD_UART4_RX_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define QEI2_CH_A_Pin GPIO_PIN_6
#define QEI2_CH_A_GPIO_Port GPIOB
#define QEI2_CH_B_Pin GPIO_PIN_7
#define QEI2_CH_B_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
