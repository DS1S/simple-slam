#include "ISM43362Interface.h"
#include "WiFiInterface.h"
#include "calibration.h"
#include "car.h"
#include "data/header.h"
#include "data/json.h"
#include "driver/i2c.h"
#include "driver/lis3mdl.h"
#include "driver/lsm6dsl.h"
#include "driver/vl53l0x.h"
#include "http_client/buffered_http_client.h"
#include "http_client/http_client.h"
#include "math.h"
#include "math/conversion.h"
#include "math/inertial_navigation.h"
#include "math/quaternion.h"
#include "mbed.h"

SimpleSlam::CalibrationStep current_calibration_step(
    SimpleSlam::CalibrationStep::MAGNETOMETER);

SimpleSlam::calibration_data_t calibration_data{
    .magnetometer_calibration_data = {},
    .gyro_offset = {0, 0, 0},
    .accel_offset = {0, 0, 0}};

InterruptIn calibration_button(BUTTON1);
DigitalOut calibration_indicator_led(LED1);

EventQueue calibration_event_queue;
EventQueue sensor_event_queue;

void update_intertial_navigation_system(
    SimpleSlam::Math::InertialNavigationSystem* inertial_navigation_system) {
    int16_t accel_buffer[3];
    int16_t gyro_buffer[3];
    int16_t magno_buffer[3];

    SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);
    SimpleSlam::LSM6DSL::Gyro_Read(gyro_buffer);
    SimpleSlam::LIS3MDL::ReadXYZ(magno_buffer[0], magno_buffer[1],
                                 magno_buffer[2]);

    SimpleSlam::Math::Vector3 temp_accel(accel_buffer[0], accel_buffer[1],
                                         accel_buffer[2]);
    SimpleSlam::Math::Vector3 temp_ang(gyro_buffer[0], gyro_buffer[1],
                                       gyro_buffer[2]);
    SimpleSlam::Math::Vector3 temp_magno(magno_buffer[0], magno_buffer[1],
                                         magno_buffer[2]);

    SimpleSlam::Math::Vector3 calibrated_magnet =
        SimpleSlam::Math::Adjust_Magnetometer_Vector(
            temp_magno, calibration_data.magnetometer_calibration_data)
            .normalize();

    SimpleSlam::Math::Vector3 t = temp_accel / 1000;
    SimpleSlam::Math::Vector3 p = (temp_ang * SimpleSlam::Math::pi / 180000);
    inertial_navigation_system->add_sample((temp_accel / 1000) * 9.8);
    inertial_navigation_system->update_position(p, t, calibrated_magnet);
}

void calculate_spatial_point(
    SimpleSlam::Math::InertialNavigationSystem* inertial_navigation_system,
    SimpleSlam::BufferedHTTPClient* buffered_http_client) {
    int16_t accel_buffer[3];
    int16_t magno_buffer[3];

    SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);
    SimpleSlam::LIS3MDL::ReadXYZ(magno_buffer[0], magno_buffer[1],
                                 magno_buffer[2]);

    SimpleSlam::Math::Vector3 temp_accel(accel_buffer[0], accel_buffer[1],
                                         accel_buffer[2]);

    SimpleSlam::Math::Vector3 temp_magno(magno_buffer[0], magno_buffer[1],
                                         magno_buffer[2]);

    SimpleSlam::Math::Vector3 adjusted_magno(
        SimpleSlam::Math::Adjust_Magnetometer_Vector(
            temp_magno, calibration_data.magnetometer_calibration_data));

    uint16_t tof_distance = 0;
    SimpleSlam::VL53L0X::Perform_Single_Shot_Read(tof_distance);

    // Convert ToF distance to cm.
    tof_distance /= 10;

    // Nothing is infront of it, too far to map point
    if (tof_distance == 0) {
        return;
    }

    SimpleSlam::Math::Vector3 north_vector(adjusted_magno.normalize());
    SimpleSlam::Math::Vector3 up_vector(temp_accel.normalize());
    SimpleSlam::Math::Vector3 tof_vector(0, 0, 1);

    SimpleSlam::Math::Vector2 tof_direction_vector =
        SimpleSlam::Math::Convert_Tof_Direction_Vector(north_vector, up_vector,
                                                       tof_vector);

    SimpleSlam::Math::Vector2 tof_mapped_point =
        SimpleSlam::Math::Convert_To_Spatial_Point(
            tof_direction_vector.normalize(), tof_distance);

    // Convert to cm.
    SimpleSlam::Math::Vector2 position_point =
        inertial_navigation_system->get_position() * 100;

    SimpleSlam::Math::Vector2 spatial_point = tof_mapped_point + position_point;

    buffered_http_client->add_data({spatial_point, position_point});
}

int main() {
    printf("Starting Simple-Slam\n");

    // Turn off LED to show no current calibration step is occuring.
    calibration_indicator_led = 0;

    SimpleSlam::VL53L0X::VL53L0X_Config_t tof_config{
        .is_voltage_2v8_mode = true,
    };

    SimpleSlam::LIS3MDL::LIS3MDL_Config_t magno_config{
        .output_rate = LOPTS_OUTPUT_RATE_80_HZ,
        .full_scale = LOPTS_FULL_SCALE_4_GAUSS,
        .bdu = 0,
    };

    // Initialize I2C communication and all the sensors needed.
    SimpleSlam::I2C_Init();
    SimpleSlam::LIS3MDL::Init(magno_config);
    SimpleSlam::VL53L0X::Init(tof_config);
    SimpleSlam::LSM6DSL::Accel_Init();
    SimpleSlam::LSM6DSL::Gyro_Init();

    // Set timing budget to 19ms for ToF, allows for feasible schedule
    SimpleSlam::VL53L0X::Set_Measurement_Timing_Budget(19000);

    SimpleSlam::calibration_args_t calibration_args{
        .event_queue = &calibration_event_queue,
        .calibration_data = &calibration_data,
        .current_calibration_step = &current_calibration_step,
        .indicator_led = &calibration_indicator_led};

    calibration_button.fall(calibration_event_queue.event(callback(
        SimpleSlam::Handle_Calibration_Step_Change, &calibration_args)));

    // Once all calibration steps are gone through,
    // this event queue will breakout
    calibration_event_queue.dispatch_forever();

    printf("Completed Calibration\n");

    SimpleSlam::Math::InertialNavigationSystem inertial_navigation_system(
        0.022, SimpleSlam::Math::Vector3(0, 0, 0),
        calibration_data.accel_offset, calibration_data.gyro_offset,
        SimpleSlam::Math::Vector3(0, 0, 0), SimpleSlam::Math::Vector3(0, 0, 0));

    // Setup buffered_http_client
    std::unique_ptr<WiFiInterface> wifi(std::make_unique<ISM43362Interface>());
    SimpleSlam::HttpClient http_client(std::move(wifi), 80);
    SimpleSlam::BufferedHTTPClient buffered_http_client(http_client, 10,
                                                        "192.168.2.33");

    // Begin main processing tasks for ToF and Position Calculator (static scheduling)
    // Compute time of calculate spatial point should be around 19ms.
    sensor_event_queue.call_every(22ms,
                                  callback(update_intertial_navigation_system,
                                           &inertial_navigation_system));
    sensor_event_queue.call_every(80ms, callback([&] {
                                      calculate_spatial_point(
                                          &inertial_navigation_system,
                                          &buffered_http_client);
                                  }));

    // Begin buffered http client thread
    Thread buffered_http_client_thread;
    buffered_http_client_thread.start(
        callback([&] { buffered_http_client.begin_processing(); }));

    sensor_event_queue.dispatch_forever();

    return 0;
}

// CarHardwareInterface car;
// car.init();

//     if (tof_distance > 25) {
//         car.move_forward();
//     } else {
//         // Stop for a second and smile :D
//         car.stop();
//         ThisThread::sleep_for(1s);

//         // Pick a random direction
//         if (rand() % 2 == 0) {
//             car.turn_left();
//             ThisThread::sleep_for(750ms);
//         } else {
//             car.turn_right();
//             ThisThread::sleep_for(750ms);
//         }
//     }

//     ThisThread::sleep_for(250ms);
// }