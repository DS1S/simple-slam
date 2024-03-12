/**
 * LIS3MDL Module Driver
 * Author: Makan Dehizadeh
 */

#include "driver/lis3mdl.h"

#define RETURN_IF_CONTAINS_ERROR(maybe_error) \
    if (maybe_error.has_value()) {            \
        return maybe_error;                   \
    }
#define RETURN_IF_STATUS_NOT_OK(status, code, message)     \
    if (status != HAL_StatusTypeDef::HAL_OK) {             \
        std::make_optional(std::make_pair(code, message)); \
    }

/**
 * Initialize the LIS3MDL module
 * @param config Configuration for the LIS3MDL module
 */
std::optional<SimpleSlam::LIS3MDL::error_t> SimpleSlam::LIS3MDL::Init(
    const LIS3MDL_Config_t& config) {
    printf("LIS3MDL::Init\n");

    // Validate the who am i register
    uint8_t who_am_i_value = 0;
    HAL_StatusTypeDef status = I2C_Mem_Read_Single(
        LIS3MDL_I2C_DEVICE_ADDRESS, LIS3MDL_WHO_AM_I, &who_am_i_value);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error reading who am i");

    // We have an expected value for the who am i
    if (who_am_i_value != WHO_AM_I_EXPECTED) {
        return std::make_optional(std::make_pair(
            ErrorCode::WHO_AM_I_UNEXPECTED_VALUE, "Unexpected who am i value"));
    }

    // Control Register 1
    // Temp, OM1, OM0, DO2, DO1, DO0, FAST_ODR, ST
    // Note: Not using FAST_ODR yet, not needed and unknown noise in the data
    uint8_t regValue = 0x00;
    uint8_t outputRate = config.outputRate;
    regValue |= (outputRate << 2);
    status =
        I2C_Mem_Write_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_1, regValue);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error writing to control register 1");

    // Control Register 2
    // 0, FS1, FS0, 0, REBOOT, SOFT_RST, 0, 0
    regValue = 0x00;
    uint8_t fullScale = config.fullScale;
    regValue |= (fullScale << 5);
    status =
        I2C_Mem_Write_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_2, regValue);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error writing to control register 2");

    // Control Register 3
    // 0, 0, LP, 0, 0, SIM, MD1, MD0
    // Note: We aren't using the LP (low-power) mode, so we can leave it at 0
    // Note: We aren't using the SIM (SPI mode) so we can leave it at 0
    // Operating mode is always continuous-conversion mode (MD 00)
    regValue = 0x00;
    status =
        I2C_Mem_Write_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_3, regValue);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error writing to control register 3");

    // Control Register 4
    // 0, 0, 0, 0, OMZ1, OMZ0, BLE, 0
    // Note: OMZ is only for Z-axis
    // Note: We want as much precision as possible, so we set OMZ to 11
    // (ultra-performance) Note: We aren't using the BLE (big/little endian)
    // mode, so we can leave it at 0
    regValue = 0x0C;
    status =
        I2C_Mem_Write_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_4, regValue);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error writing to control register 4");

    // Control Register 5
    // FAST_READ, BDU, 0, 0, 0, 0, 0, 0
    // Note: We aren't using the FAST_READ mode, so we can leave it at 0
    // Note: BDU prevents reading while its writing (only writes when read)
    regValue = 0x40;
    status =
        I2C_Mem_Write_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_5, regValue);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error writing to control register 5");

    return {};
}

/**
 * Read the X, Y, and Z values from the LIS3MDL module. Will write the values to
 * the pointers.
 * @param x Pointer to the X value
 * @param y Pointer to the Y value
 * @param z Pointer to the Z value
 */
std::optional<SimpleSlam::LIS3MDL::error_t> SimpleSlam::LIS3MDL::ReadXYZ(
    int16_t& x, int16_t& y, int16_t& z) {
    //    printf("LIS3MDL::ReadXYZ\n");

    // The addresses are consecutive, so we can read 6 bytes in one go
    // This chip uses low and high registers, so we need to do shifting
    // we cast the buffer to a uint8_t so buffer stores high low for each axis
    uint16_t buffer[3];
    HAL_StatusTypeDef status =
        I2C_Mem_Read(LIS3MDL_I2C_DEVICE_ADDRESS, REG_X_L, I2C_MEMADD_SIZE_8BIT,
                     (uint8_t*)&buffer[0], 6);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Error reading XYZ");

    // we use 16 bit buffers, but pass as 8 bit to the I2C functions
    // they automatically grab the low and high registers in the right order
    x = buffer[0];
    y = buffer[1];
    z = buffer[2];

    // Read the control register 2 to get the sensitivity
    uint8_t reg2Value = 0;
    status =
        I2C_Mem_Read_Single(LIS3MDL_I2C_DEVICE_ADDRESS, REG_CTRL_2, &reg2Value);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR,
                            "Error reading control register 2");

    // Now we handle the sensitivity
    // Note: The sensitivity is in milligauss
    int filter;
    switch ((reg2Value >> 5) & 0x03) {
        case 0:
            filter = SENSITIVITY_4G;
            break;
        case 1:
            filter = SENSITIVITY_8G;
            break;
        case 2:
            filter = SENSITIVITY_12G;
            break;
        case 3:
            filter = SENSITIVITY_16G;
            break;
        default:
            return std::make_optional(std::make_pair(
                ErrorCode::I2C_ERROR, "Invalid sensitivity value"));
    }

    // The data sheet uses LSB/gauss, but we want mGauss
    // Divide the raw value by the sensitivity for gauss, then multiply by 1000
    // for mGauss
    // TODO: Someone verify this math
    double sensitivity = 1000.0 / filter;
    x *= sensitivity;
    y *= sensitivity;
    z *= sensitivity;

    return {};
}
