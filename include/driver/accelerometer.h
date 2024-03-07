#include "stm32l4xx_hal.h"

namespace SimpleSlam::LSM6DSL {

#define ACCEL_I2C_ADDRESS 0xD4 // from Discovery Board user manual pg. 29
#define ACCEL_READ_REG_X_LOW 0x28 // from LSM6DSL data sheet pg. 49
#define ACCEL_READ_REG_X_HIGH 0x29
#define ACCEL_READ_REG_Y_LOW 0x30
#define ACCEL_READ_REG_Y_HIGH 0x31
#define ACCEL_READ_REG_Z_LOW 0x32
#define ACCEL_READ_REG_Z_HIGH 0x33
#define ACCEL_BUFFER_SIZE 6 // Reads 6 8-bit ints and stores as 3 16-bit ints

// Control registers - LSM6DSL data sheet pg. 49
#define ACCEL_CTRL_1_REG 0x10
#define ACCEL_CTRL_8_REG 0x17

// Control Options for Ctrl 1 - LSM6DSL data sheet pg. 60
// Bit[0:3]: ODR (Set to 6.66kHz)
// Bit[4:5]: Sensitivity Scale (Set to 2G uncertainty)
// Bit[6]: Linear bandwidth (Not used)
// Bit[7]: Analog Bandwidth (Set to 1.5kHz)
#define ACCEL_ODR_LOW_POWER 0b00000000
#define ACCEL_ODR_6660HZ 0b10100000 // Highest performance data rate
#define ACCEL_2G_SENSITIVITY_SCALE 0b000000
#define ACCEL_ANALOG_BANDWIDTH 0b1
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
