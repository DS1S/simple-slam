#pragma once

#include "math/conversion.h"
#include "math/vector.h"
#include "mbed.h"

namespace SimpleSlam {

enum class CalibrationStep { MAGNETOMETER, ACCEL_GYRO, DONE };

typedef struct calibration_data {
    SimpleSlam::Math::magnetometer_calibration_t magnetometer_calibration_data;
    SimpleSlam::Math::Vector3 gyro_offset;
    SimpleSlam::Math::Vector3 accel_offset;
} calibration_data_t;

typedef struct calibration_args {
    EventQueue* event_queue;
    calibration_data_t* calibration_data;
    CalibrationStep* current_calibration_step;
    DigitalOut* indicator_led;
} calibration_args_t;

void Handle_Calibration_Step_Change(calibration_args_t* args);

}  // namespace SimpleSlam
