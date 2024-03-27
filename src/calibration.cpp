#include "calibration.h"

#include "driver/lis3mdl.h"
#include "driver/lsm6dsl.h"

void calibrate_accel_gyro(
    DigitalOut* calibration_indicator_led,
    SimpleSlam::calibration_data_t* calibration_data) {
    printf("Calibrating Accelerometer and Gyroscope\n");
    *calibration_indicator_led = 1;

    int16_t accel_buffer[3];
    int16_t gyro_buffer[3];

    SimpleSlam::Math::Vector3 gyro_offset(0, 0, 0);
    SimpleSlam::Math::Vector3 accel_offset(0, 0, 0);

    const int num_samples = 500;
    for (size_t i = 0; i < num_samples; i++) {
        SimpleSlam::LSM6DSL::Gyro_Read(gyro_buffer);
        SimpleSlam::LSM6DSL::Accel_Read(accel_buffer);

        const SimpleSlam::Math::Vector3 temp_accel(
            accel_buffer[0], accel_buffer[1], accel_buffer[2]);
        const SimpleSlam::Math::Vector3 temp_ang(gyro_buffer[0], gyro_buffer[1],
                                                 gyro_buffer[2]);
        gyro_offset = gyro_offset + temp_ang;
        accel_offset = accel_offset + temp_accel;
        ThisThread::sleep_for(20ms);
    }
    // Convert to g's
    calibration_data->accel_offset = (accel_offset / num_samples) / 1000;

    // Convert to radians
    calibration_data->gyro_offset =
        (gyro_offset / num_samples) * (SimpleSlam::Math::pi / 180000);

    printf("Calibrated Gyro Offset: %s\n",
           calibration_data->accel_offset.to_string().c_str());
    printf("Calibrated Accel Offset: %s\n",
           calibration_data->gyro_offset.to_string().c_str());
    *calibration_indicator_led = 0;
}

void calibrate_magnetometer(
    DigitalOut* calibration_indicator_led,
    SimpleSlam::calibration_data_t* calibration_data) {
    printf("Calibrating Magnetometer\n");
    *calibration_indicator_led = 1;

    int16_t magno_buffer[3];
    std::vector<SimpleSlam::Math::Vector3> readings;

    const int num_samples = 500;
    for (int i = 0; i < num_samples; i++) {
        SimpleSlam::LIS3MDL::ReadXYZ(magno_buffer[0], magno_buffer[1],
                                     magno_buffer[2]);
        SimpleSlam::Math::Vector3 temp_magno(magno_buffer[0], magno_buffer[1],
                                             magno_buffer[2]);
        readings.push_back(temp_magno);
        ThisThread::sleep_for(20ms);
    }

    calibration_data->magnetometer_calibration_data =
        SimpleSlam::Math::Fill_Magnetometer_Calibration_Data(readings);

    *calibration_indicator_led = 0;
}

void SimpleSlam::Handle_Calibration_Step_Change(
    calibration_args_t* args) {
    switch (*args->current_calibration_step) {
        case CalibrationStep::MAGNETOMETER:
            calibrate_magnetometer(args->indicator_led, args->calibration_data);
            *args->current_calibration_step = CalibrationStep::ACCEL_GYRO;
            break;
        case CalibrationStep::ACCEL_GYRO:
            calibrate_accel_gyro(args->indicator_led, args->calibration_data);
            *args->current_calibration_step = CalibrationStep::DONE;
            for (int i = 0; i <= 10; i++) {
                *args->indicator_led = !*args->indicator_led;
                ThisThread::sleep_for(200ms);
            }
            break;
        case CalibrationStep::DONE:
            *args->indicator_led = 0;
            args->event_queue->break_dispatch();
            break;
        default:
            break;
    }
}