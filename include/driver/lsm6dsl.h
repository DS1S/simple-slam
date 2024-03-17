#pragma once

#include "stm32l4xx_hal.h"
#include <utility>
#include <string>
#include <optional>

namespace SimpleSlam::LSM6DSL {

// Error handling
enum class ErrorCode {
    I2C_ERROR = 1,
    WHO_AM_I_UNEXPECTED_VALUE = 2,
};

typedef std::pair<ErrorCode, std::string> error_t;

#define RETURN_IF_CONTAINS_ERROR(maybe_error) if (maybe_error.has_value()) { return maybe_error; }
#define RETURN_IF_STATUS_NOT_OK(status, code, message)  \
    if (status != HAL_StatusTypeDef::HAL_OK)            \
    {                                                   \
        return std::make_optional(std::make_pair(       \
            code,                                       \
            message));                                  \
    }                                                   \

#define I2C_ADDRESS 0xD4 // from Discovery Board user manual pg. 29
#define WHO_AM_I_REG 0x0F

#define ACCEL_READ_REG_X_LOW 0x28 // from LSM6DSL data sheet pg. 49
#define ACCEL_READ_REG_X_HIGH 0x29
#define ACCEL_READ_REG_Y_LOW 0x30
#define ACCEL_READ_REG_Y_HIGH 0x31
#define ACCEL_READ_REG_Z_LOW 0x32
#define ACCEL_READ_REG_Z_HIGH 0x33
#define ACCEL_BUFFER_SIZE 6 // Reads 6 8-bit ints and stores as 3 16-bit ints

#define GYRO_READ_REG_X_LOW 0x22
#define GYRO_READ_REG_X_HIGH 0x23
#define GYRO_READ_REG_Y_LOW 0x24
#define GYRO_READ_REG_Y_HIGH 0x25
#define GYRO_READ_REG_Z_LOW 0x26
#define GYRO_READ_REG_Z_HIGH 0x27
#define GYRO_BUFFER_SIZE 6 // Reads 6 8-bit ints and stores as 3 16-bit ints


// Control registers - LSM6DSL data sheet pg. 49
#define CTRL_1_REG 0x10
#define CTRL_2_REG 0x11
#define CTRL_3_REG 0x12
#define CTRL_4_REG 0x13
#define CTRL_6_REG 0x15
#define CTRL_7_REG 0x16
#define CTRL_8_REG 0x17

// Control Options for Ctrl 1 - LSM6DSL data sheet pg. 60
// Bit[0:3]: ODR (Set to 6.66kHz)
// Bit[4:5]: Sensitivity Scale (Set to 2G uncertainty)
// Bit[6]: Low Pass Filter 1's bandwidth (Ignored since LPF2 is used)
// Bit[7]: Analog Bandwidth (Set to 1.5kHz)
#define ACCEL_ODR_LOW_POWER 0x00
#define ACCEL_ODR_6660HZ 0xA0 // Highest performance data rate
#define ACCEL_2G_SENSITIVITY_SCALE 0x00
#define ACCEL_LPF1_BW 0x00
#define ACCEL_ANALOG_BANDWIDTH 0x00
#define ACCEL_SENSITIVITY 0.061f // from LSM6DSL data sheet pg. 21

// Control Options for CTRL2-G Register - LSM6DSL data sheet pg. 61
#define GYRO_ODR_LOW_POWER 0x00
#define GYRO_ODR_6660HZ 0x50
#define GYRO_FS_G 0x00
#define GYRO_FS_125 0x00
#define GYRO_SENSITIVITY 8.75f

// Control Optiosn for CTRL-4-C Register - LSM6DSL data sheet pg. 63
#define GYRO_LPF1_SEL 0x02

// Control Options for CTRL6-G Register - LSM6DSL data sheet pg. 65
#define GYRO_LOW_PASS_BANDWIDTH 0x03

// Control Options for CTRL7-G Registers - LSM6DSL data sheet pg. 66
#define GYRO_HIGH_PASS_EN 0x00
#define GYRO_HIGH_PASS_BANDWIDTH 0x30


// Control Options for Ctrl 3 - LSM6DSL data sheet pg. 62
#define ACCEL_SW_RESET 0x01

// Control Options for Ctrl 8 - LSM6DSL data sheet pg. 66
// Bit[0]: Enable LPF2
// Bit[1:3]: The threshold for low-pass filter
// Bit[5]: Whether filter is low or high-pass
#define ACCEL_LPF2_XL_EN 0x80 // Enable LPF2
#define ACCEL_LPF2_CUTOFF 0x60 // Low-pass cutoff: ODR/400
#define HP_SLOPE_XL_EN 0x00 // Use Low-pass

/**
 * Write configuration settings to Accelerometer control register
*/
std::optional<error_t> Accel_Init();
std::optional<error_t> Accel_DeInit();

/**
 * Write configuration settings to Gyroscope control register
*/
std::optional<error_t> Gyro_Init();
std::optional<error_t> Gyro_DeInit();

/**
 * Read raw accelerometer data into buffer
*/
std::optional<error_t> Accel_Read_Raw(uint8_t* buffer);
/**
 * Read accelerometer data (in mg) into buffer
 * Includes conversion with sensitivity
*/
std::optional<error_t> Accel_Read(int16_t* buffer);

/**
 * Read raw gyroscope data into buffer
*/
std::optional<error_t> Gyro_Read_Raw(uint8_t* buffer);
/**
 * Read gyroscope data (in dps) into buffer
 * Includes conversion with sensitivity
*/
std::optional<error_t> Gyro_Read(float* buffer);

}
