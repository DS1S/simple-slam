#include "driver/accelerometer.h"
#include "driver/i2c.h"
#include "mbed.h"

using namespace SimpleSlam;

void SimpleSlam::LSM6DSL::Accel_Init() {
    uint8_t ctrl_1 = ACCEL_ODR_6660HZ + ACCEL_2G_SENSITIVITY_SCALE + ACCEL_ANALOG_BANDWIDTH;
    I2C_Mem_Write_Single(
        ACCEL_I2C_ADDRESS,
        ACCEL_CTRL_1_REG,
        ctrl_1
    );
}

void SimpleSlam::LSM6DSL::Accel_DeInit() {

}

HAL_StatusTypeDef SimpleSlam::LSM6DSL::Accel_Read_Raw(int16_t* buffer) {
    HAL_StatusTypeDef status = I2C_Mem_Read(
        ACCEL_I2C_ADDRESS,
        ACCEL_READ_REG_X_LOW,
        1,
        (uint8_t*) buffer,
        ACCEL_BUFFER_SIZE
    );
    return status;
}

HAL_StatusTypeDef SimpleSlam::LSM6DSL::Accel_Read(int16_t* buffer) {
    HAL_StatusTypeDef status = Accel_Read_Raw(buffer);
    for (int i = 0; i < 3; i++) {
        buffer[i] = buffer[i] * ACCEL_SENSITIVITY;
    }
    return status;
}
