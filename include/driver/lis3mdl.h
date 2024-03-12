/**
 * LIS3MDL Module Driver
 * Author: Makan Dehizadeh
 */

#pragma once

#include <optional>
#include <string>

#include "driver/i2c.h"

namespace SimpleSlam::LIS3MDL {

// General Map
#define LIS3MDL_I2C_DEVICE_ADDRESS 0x3C
#define LIS3MDL_WHO_AM_I 0x0F   // h
#define WHO_AM_I_EXPECTED 0x3D  // 61

// X Y Z Map
#define REG_X_L 0x28
#define REG_X_H 0x29
#define REG_Y_L 0x2A
#define REG_Y_H 0x2B
#define REG_Z_L 0x2C
#define REG_Z_H 0x2D

// Control Map
#define REG_CTRL_1 0x20
#define REG_CTRL_2 0x21
#define REG_CTRL_3 0x22
#define REG_CTRL_4 0x23
#define REG_CTRL_5 0x24

// Sensitivity
#define SENSITIVITY_4G 6842
#define SENSITIVITY_8G 3421
#define SENSITIVITY_12G 2281
#define SENSITIVITY_16G 1711

// Output rate options
#define LOPTS_OUTPUT_RATE_0_625_HZ 0
#define LOPTS_OUTPUT_RATE_1_25_HZ 1
#define LOPTS_OUTPUT_RATE_2_5_HZ 2
#define LOPTS_OUTPUT_RATE_5_HZ 3
#define LOPTS_OUTPUT_RATE_10_HZ 4
#define LOPTS_OUTPUT_RATE_20_HZ 5
#define LOPTS_OUTPUT_RATE_40_HZ 6
#define LOPTS_OUTPUT_RATE_80_HZ 7

// Scale options
#define LOPTS_FULL_SCALE_4_GAUSS 0
#define LOPTS_FULL_SCALE_8_GAUSS 1
#define LOPTS_FULL_SCALE_12_GAUSS 2
#define LOPTS_FULL_SCALE_16_GAUSS 3

enum class ErrorCode {
    I2C_ERROR = 1,
    WHO_AM_I_UNEXPECTED_VALUE = 2,
};
typedef std::pair<ErrorCode, std::string> error_t;

typedef struct {
    // Heads up: These use indexes, not the actual values
    uint8_t
        output_rate;     // 0.625, 1.25, 2.5, 5, 10, 20, 40, 80 Hz (default: 40)
    uint8_t full_scale;  // 4, 8, 12, 16 gauss (default: 4)
    uint8_t bdu;         // Block data update on or off (default: 0)
} LIS3MDL_Config_t;

std::optional<error_t> Init(const LIS3MDL_Config_t& config);
std::optional<error_t> DeInit();
std::optional<error_t> ReadXYZ(int16_t& x, int16_t& y, int16_t& z);

}  // namespace SimpleSlam::LIS3MDL