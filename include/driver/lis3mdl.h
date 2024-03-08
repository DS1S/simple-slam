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

enum class ErrorCode {
    I2C_ERROR = 1,
    WHO_AM_I_UNEXPECTED_VALUE = 2,
};
typedef std::pair<ErrorCode, std::string> error_t;

typedef struct {
    // Heads up: These use indexes, not the actual values
    uint8_t outputRate; // 0.625, 1.25, 2.5, 5, 10, 20, 40, 80 Hz (default: 40)
    uint8_t fullScale;  // 4, 8, 12, 16 gauss (default: 4)
} LIS3MDL_Config_t;

//typedef struct {
//    int16_t minX;
//    int16_t maxX;
//    int16_t minY;
//    int16_t maxY;
//    int16_t minZ;
//    int16_t maxZ;
//} LIS3MDL_Data_t;

std::optional<error_t> Init(const LIS3MDL_Config_t& config);
std::optional<error_t> ReadXYZ(int16_t* x, int16_t* y, int16_t* z);

}  // namespace SimpleSlam::LIS3MDL