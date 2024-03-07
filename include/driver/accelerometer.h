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

#define ACCEL_I2C_ADDRESS 0xD4 // from Discovery Board user manual pg. 29
#define ACCEL_READ_REG_X_LOW 0x28 // from LSM6DSL data sheet pg. 49
#define ACCEL_READ_REG_X_HIGH 0x29
#define ACCEL_READ_REG_Y_LOW 0x30
#define ACCEL_READ_REG_Y_HIGH 0x31
#define ACCEL_READ_REG_Z_LOW 0x32
#define ACCEL_READ_REG_Z_HIGH 0x33
#define ACCEL_BUFFER_SIZE 6 // Reads 6 8-bit ints and stores as 3 16-bit ints
#define ACCEL_WHO_AM_I_REG 0x0F

// Control registers - LSM6DSL data sheet pg. 49
#define ACCEL_CTRL_1_REG 0x10
#define ACCEL_CTRL_3_REG 0x12
#define ACCEL_CTRL_8_REG 0x17

// Control Options for Ctrl 1 - LSM6DSL data sheet pg. 60
// Bit[0:3]: ODR (Set to 6.66kHz)
// Bit[4:5]: Sensitivity Scale (Set to 2G uncertainty)
// Bit[6]: Low Pass Filter 1's bandwidth (Ignored since LPF2 is used)
// Bit[7]: Analog Bandwidth (Set to 1.5kHz)
#define ACCEL_ODR_LOW_POWER 0b00000000
#define ACCEL_ODR_6660HZ 0b10100000 // Highest performance data rate
#define ACCEL_2G_SENSITIVITY_SCALE 0b000000
#define ACCEL_LPF1_BW 0b00
#define ACCEL_ANALOG_BANDWIDTH 0b0
#define ACCEL_SENSITIVITY 0.061f // from LSM6DSL data sheet pg. 21

// Control Options for Ctrl 3 - LSM6DSL data sheet pg. 62
#define ACCEL_SW_RESET 0b00000001

// Control Options for Ctrl 8 - LSM6DSL data sheet pg. 66
#define ACCEL_LPF2_XL_EN 0b10000000 // Enable LPF2
#define ACCEL_LPF2_CUTOFF 0b1100000 // Low-pass cutoff: ODR/400
#define HP_SLOPE_XL_EN 0b000 // Use Low-pass

/**
 * Write configuration settings to Accelerometer control register
*/
std::optional<error_t> Accel_Init();
std::optional<error_t> Accel_DeInit();

/**
 * Read raw accelerometer data into buffer
*/
std::optional<error_t> Accel_Read_Raw(int16_t* buffer);
/**
 * Read accelerometer data (in mg) into buffer
 * Includes conversion with sensitivity
*/
std::optional<error_t> Accel_Read(int16_t* buffer);

}
