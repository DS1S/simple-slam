#include "math/conversion.h"

#include <math.h>

SimpleSlam::Math::Vector2 SimpleSlam::Math::Convert_Tof_Direction_Vector(
    Vector3 const& north_vector, Vector3 const& up_vector,
    Vector3 const& tof_vector) {
    const Vector3 east_vector = up_vector.cross(north_vector);

    const double proj_x = tof_vector.dot(east_vector);
    const double proj_y = tof_vector.dot(north_vector);

    return Vector2(proj_x, proj_y);
}

SimpleSlam::Math::Vector2 SimpleSlam::Math::Convert_To_Spatial_Point(
    Vector2 const& tof_direction_vector, uint16_t tof_distance) {
    return tof_direction_vector * (double)tof_distance;
}

SimpleSlam::Math::magnetometer_calibration_t SimpleSlam::Math::Fill_Magnetometer_Calibration_Data(
    std::vector<SimpleSlam::Math::Vector3> const& readings) {
    double max_x = -32767;
    double max_y = -32767;
    double max_z = -32767;
    double min_x = 32767;
    double min_y = 32767;
    double min_z = 32767;

    for (auto& reading : readings) {
        const double x = reading.get_x();
        const double y = reading.get_y();
        const double z = reading.get_z();

        max_x = std::max(x, max_x);
        min_x = std::min(x, min_x);

        max_y = std::max(y, max_y);
        min_y = std::min(y, min_y);

        max_z = std::max(z, max_z);
        min_z = std::min(z, min_z);
    }

    const double offset_x = (max_x + min_x) / 2;
    const double offset_y = (max_y + min_y) / 2;
    const double offset_z = (max_z + min_z) / 2;

    const double average_delta_x = (max_x - min_x) / 2;
    const double average_delta_y = (max_y - min_y) / 2;
    const double average_delta_z = (max_z - min_z) / 2;

    const double average_delta =
        (average_delta_x + average_delta_y + average_delta_z);

    const double scale_x = average_delta / average_delta_x;
    const double scale_y = average_delta / average_delta_y;
    const double scale_z = average_delta / average_delta_z;

    return magnetometer_calibration_t{
        .offset_x = offset_x,
        .offset_y = offset_y,
        .offset_z = offset_z,
        .scale_x = scale_x,
        .scale_y = scale_y,
        .scale_z = scale_z,
    };
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Adjust_Magnetometer_Vector(
    Vector3 const& magnetometer_vector,
    magnetometer_calibration_t const& calibration_data) {
    return Vector3((magnetometer_vector.get_x() - calibration_data.offset_x) *
                       calibration_data.scale_x,
                   (magnetometer_vector.get_y() - calibration_data.offset_y) *
                       calibration_data.scale_y,
                   (magnetometer_vector.get_z() - calibration_data.offset_z) *
                       calibration_data.scale_z);
}