/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_it.h"

#include "cmsis_os.h"
#include "main.h"

void goldobot_hal_uart_irq_handler(int uart_index);
void goldobot_hal_dma_irq_handler(int dma_index);
void goldobot_hal_i2c_ev_irq_handler(int ioc_index);
void goldobot_hal_i2c_er_irq_handler(int ioc_index);

void goldobot_hal_spi_irq_handler(int index);
void goldobot_hal_exti_irq_handler();
void goldobot_hal_tim6_irq_handler();

extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

void DMA1_Channel1_IRQHandler(void) { goldobot_hal_dma_irq_handler(0); }

void DMA1_Channel2_IRQHandler(void) { goldobot_hal_dma_irq_handler(1); }

void DMA1_Channel3_IRQHandler(void) { goldobot_hal_dma_irq_handler(2); }

void DMA1_Channel4_IRQHandler(void) { goldobot_hal_dma_irq_handler(3); }

void DMA1_Channel5_IRQHandler(void) { goldobot_hal_dma_irq_handler(4); }

void DMA1_Channel6_IRQHandler(void) { goldobot_hal_dma_irq_handler(5); }

void DMA1_Channel7_IRQHandler(void) { goldobot_hal_dma_irq_handler(6); }

void DMA2_Channel1_IRQHandler(void) { goldobot_hal_dma_irq_handler(7); }

void DMA2_Channel2_IRQHandler(void) { goldobot_hal_dma_irq_handler(8); }

void DMA2_Channel3_IRQHandler(void) { goldobot_hal_dma_irq_handler(9); }

void DMA2_Channel4_IRQHandler(void) { goldobot_hal_dma_irq_handler(10); }

void DMA2_Channel5_IRQHandler(void) { goldobot_hal_dma_irq_handler(11); }

void USART1_IRQHandler(void) { goldobot_hal_uart_irq_handler(0); }

void USART2_IRQHandler(void) { goldobot_hal_uart_irq_handler(1); }

void USART3_IRQHandler(void) { goldobot_hal_uart_irq_handler(2); }

void UART4_IRQHandler(void) { goldobot_hal_uart_irq_handler(3); }

void UART5_IRQHandler(void) { goldobot_hal_uart_irq_handler(4); }

void I2C1_EV_IRQHandler(void) { goldobot_hal_i2c_ev_irq_handler(0); }

void I2C1_ER_IRQHandler(void) { goldobot_hal_i2c_er_irq_handler(0); }

void I2C2_EV_IRQHandler(void) { goldobot_hal_i2c_ev_irq_handler(1); }

void I2C2_ER_IRQHandler(void) { goldobot_hal_i2c_er_irq_handler(1); }

void I2C3_EV_IRQHandler(void) { goldobot_hal_i2c_ev_irq_handler(2); }

void I2C3_ER_IRQHandler(void) { goldobot_hal_i2c_er_irq_handler(2); }

void SPI1_IRQHandler(void) { goldobot_hal_spi_irq_handler(0); }

void SPI2_IRQHandler(void) { goldobot_hal_spi_irq_handler(1); }

void SPI3_IRQHandler(void) { goldobot_hal_spi_irq_handler(2); }

void SPI4_IRQHandler(void) { goldobot_hal_spi_irq_handler(3); }

void EXTI0_IRQHandler(void) { goldobot_hal_exti_irq_handler(); };

/**
 * @brief This function handles TIM6 global interrupt and DAC1 underrun interrupt.
 */
void TIM6_DAC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim6);
	goldobot_hal_tim6_irq_handler();
}
