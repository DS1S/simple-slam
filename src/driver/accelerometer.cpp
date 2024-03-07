#include "driver/accelerometer.h"
#include "driver/i2c.h"
#include "mbed.h"

using namespace SimpleSlam;

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_Init() {
    uint8_t ctrl_1 = ACCEL_ODR_6660HZ + ACCEL_2G_SENSITIVITY_SCALE + ACCEL_ANALOG_BANDWIDTH;
    HAL_StatusTypeDef status = I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_1_REG,
        ctrl_1
    );
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, "Failed to write to Ctrl 1");
    return {};
}

std::optional<SimpleSlam::LSM6DSL::error_t> SimpleSlam::LSM6DSL::Accel_DeInit() {

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
