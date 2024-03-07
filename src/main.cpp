#include "mbed.h"

#include "driver/i2c.h"
#include "driver/vl53l0x.h"
#include "driver/accelerometer.h"


int main() {
    printf("Starting Simple-Slam\n");
    SimpleSlam::I2C_Init();

    const SimpleSlam::VL53L0X::VL53L0X_Config_t tof_config{
        .is_voltage_2v8_mode = true,
    };

    auto result = SimpleSlam::VL53L0X::Init(tof_config);
    if (result.has_value()) {
        printf("Result: %s\n", result.value().second.c_str());
    }
    SimpleSlam::LSM6DSL::Accel_Init();
    printf("Finished init\n");
    uint16_t distance = 0;
    int16_t accel_buffer[3];
    while(true) {
        SimpleSlam::VL53L0X::Perform_Single_Shot_Read(distance);
        printf("%u mm\n", distance);
        SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);
        printf("ACCELEROMETER (x, y, z) = (%d mg, %d mg, %d mg)\n",
            accel_buffer[0], accel_buffer[1], accel_buffer[2]
        );
        ThisThread::sleep_for(300ms);
    }
    return 0;
}