/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
#define NUCLEO_DYNA_TX_Pin GPIO_PIN_4
#define NUCLEO_DYNA_TX_GPIO_Port GPIOC
#define NUCLEO_DYNA_DIR_Pin GPIO_PIN_5
#define NUCLEO_DYNA_DIR_GPIO_Port GPIOC
#define NUCLEO_PWM_T3_4_Pin GPIO_PIN_1
#define NUCLEO_PWM_T3_4_GPIO_Port GPIOB
#define MAXON1_DIR_Pin GPIO_PIN_2
#define MAXON1_DIR_GPIO_Port GPIOB
#define NUCLEO_GPIO_PC6_Pin GPIO_PIN_6
#define NUCLEO_GPIO_PC6_GPIO_Port GPIOC
#define MAXON1_PWM_Pin GPIO_PIN_7
#define MAXON1_PWM_GPIO_Port GPIOC
#define NUCLEO_GPIO_PC8_Pin GPIO_PIN_8
#define NUCLEO_GPIO_PC8_GPIO_Port GPIOC
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
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
