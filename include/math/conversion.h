#pragma once

#include <vector>
#include "math/vector.h"
#include "stdint.h"

namespace SimpleSlam::Math {

inline constexpr double pi = 3.141592653589793238462643383279;

typedef struct magnetometer_calibration {
    double offset_x;
    double offset_y;
    double offset_z;
    double scale_x;
    double scale_y;
    double scale_z;
} magnetometer_calibration_t;

Vector2 Convert_Tof_Direction_Vector(
    SimpleSlam::Math::Vector3 const& north_vector,
    SimpleSlam::Math::Vector3 const& up_vector,
    SimpleSlam::Math::Vector3 const& tof_vector);

Vector2 Convert_To_Spatial_Point(Vector2 const& tof_direction_vector,
                                 uint16_t tof_distance);


Vector3 Adjust_Magnetometer_Vector(Vector3 const& magnetometer_vector, magnetometer_calibration_t const& calibration_data);

magnetometer_calibration_t Fill_Magnetometer_Calibration_Data(std::vector<Vector3> const& readings);

}  // namespace SimpleSlam::Math
