/**
 * I2C Library
*/
#pragma once

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rcc.h"

namespace SimpleSlam {

#define TIMEOUT_US  1000
#define SINGLE_SIZE 1

/**
 * @brief I2C Timing Register Value
 * @note STM32L475 MCU Reference Manual - 39.4.10 Standard-Mode @ 100kHz
*/
#define SCLL          (0x13 << 0)
#define SCLH          (0x0F << 8)
#define SDADEL        (0x2 << 16)
#define SCLDEL        (0x4 << 20)
#define PRESC         (0x1 << 28)
#define I2C_TIMINGR   (PRESC | SCLDEL | SDADEL | SCLH | SCLL)

/**
 * @brief 
 * @note Reference: UM2153 Appendix A STM32L4 Discovery kit for IoT node I/O assignment
*/
#define SDA_PIN                     GPIO_PIN_10
#define SCL_PIN                     GPIO_PIN_11
#define SCL_SDA_GPIO_PORT           GPIOB
#define SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SCL_SDA_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()

#define INTERNAL_I2C_CLK_ENABLE()     __HAL_RCC_I2C2_CLK_ENABLE()
#define INTERNAL_I2C_CLK_DISABLE()    __HAL_RCC_I2C2_CLK_DISABLE()
#define INTERNAL_I2C_FORCE_RESET()    __HAL_RCC_I2C2_FORCE_RESET()
#define INTERNAL_I2C_RELEASE_RESET()  __HAL_RCC_I2C2_RELEASE_RESET()

void I2C_Init();
void I2C_DeInit();

HAL_StatusTypeDef I2C_Mem_Write(uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef I2C_Mem_Write_Single(uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, uint8_t *value);
HAL_StatusTypeDef I2C_Mem_Read(uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, uint8_t *buffer, uint16_t size);
HAL_StatusTypeDef I2C_Mem_Read_Single(uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, uint8_t* buffer);

}
