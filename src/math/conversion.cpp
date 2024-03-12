#include <math.h>
#include "math/conversion.h"

SimpleSlam::Math::Vector2 SimpleSlam::Math::convert_tof_direction_vector(
    const Vector3& north_vector,
    const Vector3& up_vector,
    const Vector3& tof_vector
) {
    const Vector3 east_vector = north_vector.cross(up_vector);

    const double proj_x = tof_vector.dot(east_vector);
    const double proj_y = tof_vector.dot(north_vector);

    return Vector2(proj_x, proj_y);
}

SimpleSlam::Math::Vector2 SimpleSlam::Math::convert_to_spatial_point(const Vector2& tof_direction_vector, uint16_t tof_distance) {
    return tof_direction_vector * (double)tof_distance;
}