#include "math/inertial_navigation.h"

#include <math.h>

#include "math/Matrix.h"
#include "math/vector.h"
#include "mbed.h"

SimpleSlam::Math::InertialNavigationSystem::InertialNavigationSystem(
    const double time_delta, const Vector3& accel_offset,
    const Vector3& gyro_offset, const Vector3& velocity,
    const Vector3& position)
    : _time_delta{time_delta},
      _accel_offset{accel_offset},
      _gyro_offset{gyro_offset},
      _velocity{velocity},
      _position{position},
      _rotation_matrix{{Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}} {
}

SimpleSlam::Math::Matrix3
SimpleSlam::Math::InertialNavigationSystem::get_rotation_matrix() const {
    return _rotation_matrix;
}

SimpleSlam::Math::Vector3
SimpleSlam::Math::InertialNavigationSystem::get_velocity() const {
    return _velocity;
}

SimpleSlam::Math::Vector3
SimpleSlam::Math::InertialNavigationSystem::get_position() const {
    return _position;
}

void SimpleSlam::Math::InertialNavigationSystem::update_position(
    const Vector3& angular_velocity, const Vector3& force) {
    const Matrix3 identity_matrix(
        {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)});

    const Matrix3 attitude({
        Vector3(0, -1 * angular_velocity.get_z() * _time_delta,
                angular_velocity.get_y() * _time_delta),
        Vector3(angular_velocity.get_z() * _time_delta, 0,
                -1 * angular_velocity.get_x() * _time_delta),
        Vector3(-1 * angular_velocity.get_y() * _time_delta,
                angular_velocity.get_x() * _time_delta, 0),
    });

    const double sigma = (angular_velocity * _time_delta).magnitude();

    const Matrix3 rotation_matrix =
        _rotation_matrix *
        (identity_matrix + (attitude * (std::sin(sigma) / sigma)) +
         (attitude * attitude) * ((1 - std::cos(sigma)) / std::pow(sigma, 2)));
    // printf("ANG: %s\n%s\n%s\n%s\n\n",
    //     angular_velocity.to_string().c_str(),
    //     rotation_matrix[0].to_string().c_str(),
    //     rotation_matrix[1].to_string().c_str(),
    //     rotation_matrix[2].to_string().c_str());

    const Vector3 local_force = rotation_matrix * force;
    // const Vector3 gravity(0, 0, -1);
    const Vector3 acceleration = local_force + _accel_offset;


    _velocity = _velocity + (acceleration * _time_delta);
    _position = _position + (_velocity * _time_delta);
    _rotation_matrix = rotation_matrix;
    // printf("ACC: %s VEL: %s POS: %s\n", acceleration.to_string().c_str(),
    //        _velocity.to_string().c_str(), _position.to_string().c_str());
}