#include "ISM43362Interface.h"
#include "WiFiInterface.h"
#include "car.h"
#include "data/header.h"
#include "data/json.h"
#include "driver/i2c.h"
#include "driver/lis3mdl.h"
#include "driver/lsm6dsl.h"
#include "driver/vl53l0x.h"
#include "http_client/http_client.h"
#include "math/conversion.h"
#include "math/inertial_navigation.h"
#include "mbed.h"

typedef struct {
    int16_t minX;
    int16_t maxX;
    int16_t minY;
    int16_t maxY;
    int16_t minZ;
    int16_t maxZ;
} offset_vars;

void test_http_client() {
    std::unique_ptr<WiFiInterface> wifi = std::make_unique<ISM43362Interface>();
    SimpleSlam::HttpClient http_client(std::move(wifi));
    http_client.init();
    auto status = http_client.get_request("api.restful-api.dev", "/objects/7");
    if (status.has_value()) {
        printf(status.value().second.c_str());
    } else {
        printf("[HttpClient]: GET Succeeded\n");
    }

    SimpleSlam::JSON body_data;
    body_data.add("value", 1);
    SimpleSlam::JSON post_body;
    post_body.add("name", "testobject100").add("data", body_data);

    status =
        http_client.post_request("api.restful-api.dev", "/objects", post_body);
    if (status.has_value()) {
        printf(status.value().second.c_str());
    } else {
        printf("[HttpClient]: POST Succeeded\n");
    }
}

int main() {
    printf("Starting Simple-Slam\n");

    SimpleSlam::JSON my_data;
    my_data.add("age", 1)
        .add("hair", "brown")
        .add_list<float>("points", {3.1, 2, 2.1, 4});

    std::string my_data_str = my_data.build();
    printf("My Data: %s \n", my_data_str.c_str());

    SimpleSlam::LIS3MDL::LIS3MDL_Config_t config{
        .output_rate = LOPTS_OUTPUT_RATE_80_HZ,  // 80 Hz
        .full_scale = LOPTS_FULL_SCALE_4_GAUSS,  // 4 gauss
        .bdu = 0,                                // Block data update off
    };

    SimpleSlam::VL53L0X::VL53L0X_Config_t tof_config{
        .is_voltage_2v8_mode = true,
    };

    SimpleSlam::I2C_Init();
    SimpleSlam::LSM6DSL::Accel_Init();
    SimpleSlam::LSM6DSL::Gyro_Init();
    SimpleSlam::LIS3MDL::Init(config);
    SimpleSlam::VL53L0X::Init(tof_config);
    SimpleSlam::Math::InertialNavigationSystem ins(0.02, SimpleSlam::Math::Vector3(0,0,0), SimpleSlam::Math::Vector3(0,0,0));

    int16_t accel_buffer[3];
    int16_t gyro_buffer[3];
    int16_t magno_buffer[3];
    uint16_t tof_distance = 0;

    CarHardwareInterface car;
    car.init();

    while (false) {
        SimpleSlam::LSM6DSL::Gyro_Read(gyro_buffer);
        SimpleSlam::Math::Vector3 gyro(gyro_buffer[0], gyro_buffer[1],
                                       gyro_buffer[2]);
        printf("GYRO %s\n", gyro.to_string().c_str());
        ThisThread::sleep_for(1s);
    }

    std::vector<SimpleSlam::Math::Vector3> readings;
    for (int i = 0; i < 100; i++) {
        SimpleSlam::LIS3MDL::ReadXYZ(magno_buffer[0], magno_buffer[1],
                                     magno_buffer[2]);
        SimpleSlam::Math::Vector3 temp_magno(magno_buffer[0], magno_buffer[1],
                                             magno_buffer[2]);
        readings.push_back(temp_magno);
        ThisThread::sleep_for(100ms);
    }

    SimpleSlam::Math::magnetometer_calibration_t calibration_data =
        SimpleSlam::Math::Fill_Magnetometer_Calibration_Data(readings);

    Thread t;
    t.start(test_http_client);
    
    while(true){
        // SimpleSlam::Math::Vector3 pos = ins.get_position();
        // printf("POS %s\n", pos.to_string().c_str());

        SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);
        SimpleSlam::LSM6DSL::Gyro_Read(gyro_buffer);

        SimpleSlam::Math::Vector3 temp_accel(accel_buffer[0], accel_buffer[1],
                                             accel_buffer[2]);
        SimpleSlam::Math::Vector3 temp_ang(gyro_buffer[0], gyro_buffer[1],
                                             gyro_buffer[2]);

        SimpleSlam::Math::Vector3 t = temp_accel / 1000;
        SimpleSlam::Math::Vector3 p = temp_ang * (SimpleSlam::Math::pi/180000);
        
        ins.update_position(p, t);
        ThisThread::sleep_for(20ms);
    }

    while (false) {
        SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);
        SimpleSlam::LIS3MDL::ReadXYZ(magno_buffer[0], magno_buffer[1],
                                     magno_buffer[2]);
        SimpleSlam::Math::Vector3 temp_accel(accel_buffer[0], accel_buffer[1],
                                             accel_buffer[2]);

        SimpleSlam::Math::Vector3 temp_magno(magno_buffer[0], magno_buffer[1],
                                             magno_buffer[2]);

        SimpleSlam::Math::Vector3 adjusted_magno(
            SimpleSlam::Math::Adjust_Magnetometer_Vector(temp_magno,
                                                         calibration_data));

        SimpleSlam::VL53L0X::Perform_Single_Shot_Read(tof_distance);
        tof_distance /= 10;

        SimpleSlam::Math::Vector3 north_vector(adjusted_magno.normalize());
        SimpleSlam::Math::Vector3 up_vector(temp_accel.normalize());
        SimpleSlam::Math::Vector3 tof_vector(0, 0, 1);
        SimpleSlam::Math::Vector2 tof_direction_vector =
            SimpleSlam::Math::Convert_Tof_Direction_Vector(
                north_vector, up_vector, tof_vector);
        SimpleSlam::Math::Vector2 mapped_point =
            SimpleSlam::Math::Convert_To_Spatial_Point(
                tof_direction_vector.normalize(), tof_distance);

        printf("TOF SENSOR DISTANCE: %dcm\n", tof_distance);
        printf("ACCELEROMETER (x, y, z) = (%d mg, %d mg, %d mg)\n",
               accel_buffer[0], accel_buffer[1], accel_buffer[2]);
        printf("MAGNETOMETER (x, y, z) = (%d mg, %d mg, %d mg)\n",
               magno_buffer[0], magno_buffer[1], magno_buffer[2]);
        // printf("Direction Vector: %s, %f\n",
        //        tof_direction_vector.normalize().to_string().c_str(),
        //        tof_direction_vector.normalize().to_string().c_str());
        printf("Spatial Vector: %s\n", mapped_point.to_string().c_str());

        if (tof_distance > 25) {
            car.move_forward();
        } else {
            // Stop for a second and smile :D
            car.stop();
            ThisThread::sleep_for(1s);

            // Pick a random direction
            if (rand() % 2 == 0) {
                car.turn_left();
                ThisThread::sleep_for(750ms);
            } else {
                car.turn_right();
                ThisThread::sleep_for(750ms);
            }
        }

        ThisThread::sleep_for(250ms);
    }

    return 0;
}

int test_magetometer() {
    SimpleSlam::LIS3MDL::LIS3MDL_Config_t config{
        .output_rate = LOPTS_OUTPUT_RATE_80_HZ,  // 80 Hz
        .full_scale = LOPTS_FULL_SCALE_4_GAUSS,  // 4 gauss
        .bdu = 0,                                // Block data update off
    };

    auto result = SimpleSlam::LIS3MDL::Init(config);
    if (result.has_value()) {
        printf("Result: %s\n", result.value().second.c_str());
    }

    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    offset_vars data{
        .minX = 999,
        .maxX = -999,
        .minY = 999,
        .maxY = -999,
        .minZ = 999,
        .maxZ = -999,
    };

    while (true) {
        SimpleSlam::LIS3MDL::ReadXYZ(x, y, z);

        int16_t offsetX = (data.maxX - data.minX) / 2;
        int16_t offsetY = (data.maxY - data.minY) / 2;
        int16_t offsetZ = (data.maxZ - data.minZ) / 2;

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
