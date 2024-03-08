#include <cmath>

#include "driver/i2c.h"
#include "driver/lis3mdl.h"
#include "driver/vl53l0x.h"
#include "mbed.h"

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
        printf("Finished init\n");
        uint16_t distance = 0;
        while(true) {
            SimpleSlam::VL53L0X::Perform_Single_Shot_Read(distance);
            printf("%u mm\n", distance);
            ThisThread::sleep_for(300ms);
        }

    SimpleSlam::LIS3MDL::LIS3MDL_Config_t config{
        .outputRate = 7,  // 80 Hz
        .fullScale = 0,   // 4 gauss
    };

    auto result2 = SimpleSlam::LIS3MDL::Init(config);
    if (result2.has_value()) {
        printf("Result: %s\n", result.value().second.c_str());
    }

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    SimpleSlam::LIS3MDL::LIS3MDL_Data_t data{
        .minX = 999,
        .maxX = -999,
        .minY = 999,
        .maxY = -999,
        .minZ = 999,
        .maxZ = -999,
    };

    while (true) {
        SimpleSlam::LIS3MDL::ReadXYZ(&x, &y, &z);

        int16_t offsetX = (data.maxX + data.minX) / 2;
        int16_t offsetY = (data.maxY + data.minY) / 2;
        int16_t offsetZ = (data.maxZ + data.minZ) / 2;

        double headingRadians = std::atan2(
            y + offsetY, z + offsetZ);  // Calculate heading in radians
        double headingDegrees =
            headingRadians * (180.0 / 3.14);  // Convert to degrees

        // Ensure the heading is between 0-360 degrees
        if (headingDegrees < 0) {
            headingDegrees += 360;
        }

        printf("Heading: %i (X: %d, Y: %d, Z: %d)\n", (int)headingDegrees,
               x + offsetX, y + offsetY, z + offsetZ);
        printf("Min: X: %d, Y: %d, Z: %d\n", data.minX, data.minY, data.minZ);
        printf("Max: X: %d, Y: %d, Z: %d\n", data.maxX, data.maxY, data.maxZ);
        printf("--------------------\n");

        // Calibrate readings
        data.minX = std::min(data.minX, x);
        data.maxX = std::max(data.maxX, x);
        data.minY = std::min(data.minY, y);
        data.maxY = std::max(data.maxY, y);
        data.minZ = std::min(data.minZ, z);
        data.maxZ = std::max(data.maxZ, z);

        ThisThread::sleep_for(300ms);
    }

    return 0;
}