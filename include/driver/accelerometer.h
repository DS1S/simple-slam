#include "stm32l4xx_hal.h"

namespace SimpleSlam {

#define ACCEL_ADDRESS 0xD4 // from Discovery Board user manual pg. 29
#define ACCEL_READ_REG 0x28 // from LSM6DSL data sheet pg. 49
#define ACCEL_BUFFER_SIZE 6 // Reads 6 8-bit ints and stores as 3 16-bit ints

// Control registers - LSM6DSL data sheet pg. 49
#define ACCEL_CTRL_1 0x10
#define ACCEL_CTRL_8 0x17

// Control Options - LSM6DSL data sheet pg. 60
#define ACCEL_DATA_RATE 0b1010 // Highest performance data rate
#define ACCEL_FULLSCALE 0b00 // 2G uncertainty
#define ACCEL_SENSITIVITY 0.061f // from LSM6DSL data sheet pg. 21

/**
 * Write configuration settings to Accelerometer control register
*/
void Accel_Init();
void Accel_DeInit();

/**
 * Read raw accelerometer data into buffer
*/
HAL_StatusTypeDef Accel_Read_Raw(int16_t* buffer);
/**
 * Read accelerometer data (in mg) into buffer
 * Includes conversion with sensitivity
*/
HAL_StatusTypeDef Accel_Read(int16_t* buffer);

}
