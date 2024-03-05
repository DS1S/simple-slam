#include "driver/accelerometer.h"
#include "driver/i2c.h"
#include "mbed.h"

using namespace SimpleSlam;

void SimpleSlam::Accel_Init() {
    uint8_t ctrl_1 = 0;
    ctrl_1 |= (ACCEL_DATA_RATE << 4);
    ctrl_1 |= (ACCEL_FULLSCALE << 2);
    I2C_Mem_Write_Single(
        ACCEL_ADDRESS,
        ACCEL_CTRL_1,
        ctrl_1
    );
}

void SimpleSlam::Accel_DeInit() {

}

HAL_StatusTypeDef SimpleSlam::Accel_Read_Raw(int16_t* buffer) {
    HAL_StatusTypeDef status = I2C_Mem_Read(
        ACCEL_ADDRESS,
        ACCEL_READ_REG,
        1,
        (uint8_t*) buffer,
        ACCEL_BUFFER_SIZE
    );
    return status;
}

HAL_StatusTypeDef SimpleSlam::Accel_Read(int16_t* buffer) {
    HAL_StatusTypeDef status = Accel_Read_Raw(buffer);
    for (int i = 0; i < 3; i++) {
        buffer[i] = buffer[i] * ACCEL_SENSITIVITY;
    }
    return status;
}
