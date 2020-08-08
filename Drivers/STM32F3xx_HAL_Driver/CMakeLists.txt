add_library(drivers_STM32F3xx_HAL_Driver STATIC)

target_sources(drivers_STM32F3xx_HAL_Driver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_adc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_adc_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_cortex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_dma.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_flash.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_flash_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_gpio.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_i2c.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_i2c_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_pwr.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_rcc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_rcc_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_spi.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_spi_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_tim.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_tim_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_uart.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_uart_ex.c
  )

target_include_directories(drivers_STM32F3xx_HAL_Driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
  )

target_link_libraries(drivers_STM32F3xx_HAL_Driver
  drivers_CMSIS
  )