#include "driver/lsm6dsl.h"
#include "driver/i2c.h"
#include "mbed.h"

using namespace SimpleSlam;

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_Init() {
    // Wait for peripheral to turn on
    HAL_StatusTypeDef status;
    uint8_t who_am_i_val = 0;
    while (who_am_i_val == 0x00) {
        status = SimpleSlam::I2C_Mem_Read_Single(ACCEL_I2C_ADDRESS, ACCEL_WHO_AM_I_REG, &who_am_i_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed to read Who Am I"));
    }

    // Ctrl1 options: ODR, Sensitivity, Bandwidth
    uint8_t ctrl_1 = ACCEL_ODR_6660HZ | ACCEL_2G_SENSITIVITY_SCALE | ACCEL_ANALOG_BANDWIDTH;
    status = I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_1_REG,
        ctrl_1
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to write to Ctrl 1");

    // Ctrl8 options: Noise Filtering
    uint8_t ctrl_8 = ACCEL_LPF2_XL_EN | ACCEL_LPF2_CUTOFF | HP_SLOPE_XL_EN;
    status = I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_8_REG,
        ctrl_8
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to write to Ctrl 8");
    return {};
}

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_DeInit() {
    // Reset
    HAL_StatusTypeDef status;
    uint8_t ctrl_3 = ACCEL_SW_RESET;
    status = I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_3_REG,
        ctrl_3
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to write to Ctrl 3");

    // Power Down
    uint8_t ctrl_1 = ACCEL_ODR_LOW_POWER | ACCEL_2G_SENSITIVITY_SCALE | ACCEL_ANALOG_BANDWIDTH;
    status = I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_1_REG,
        ctrl_1
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to write to Ctrl 1");

    return {};
}

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_Read_Raw(int16_t* buffer) {
    HAL_StatusTypeDef status = I2C_Mem_Read(
        ACCEL_I2C_ADDRESS,
        ACCEL_READ_REG_X_LOW,
        1,
        (uint8_t*) buffer,
        ACCEL_BUFFER_SIZE
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to Read Accelerometer Data");
    return {};
}

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_Read(int16_t* buffer) {
    auto maybe_error = Accel_Read_Raw(buffer);
    RETURN_IF_CONTAINS_ERROR(maybe_error);
    for (int i = 0; i < 3; i++) {
        buffer[i] = buffer[i] * ACCEL_SENSITIVITY;
    }
    return {};
}