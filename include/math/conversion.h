#pragma once

#include "stdint.h"
#include "math/vector.h"

namespace SimpleSlam::Math {

inline constexpr double pi = 3.145926535;

Vector2 convert_tof_direction_vector(
    const SimpleSlam::Math::Vector3& north_vector, 
    const SimpleSlam::Math::Vector3& up_vector,
    const SimpleSlam::Math::Vector3& tof_vector);

Vector2 convert_to_spatial_point(const Vector2& tof_direction_vector, uint16_t tof_distance);

}
