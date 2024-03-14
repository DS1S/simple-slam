#include "math/inertial_navigation.h"
#include "math/vector.h"
#include "math/Matrix.h"


SimpleSlam::Math::InertialNavigationSystem::InertialNavigationSystem(
    const double& time_delta,
    const Vector3& velocity,
    const Vector3& position) 
: _time_delta{time_delta}, _velocity{velocity}, _position{position}, _rotation_matrix{{Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1)}} {}

SimpleSlam::Math::Matrix3 SimpleSlam::Math::InertialNavigationSystem::get_rotation_matrix() const {
    return _rotation_matrix;
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::InertialNavigationSystem::get_velocity() const {
    return _velocity;
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::InertialNavigationSystem::get_position() const {
    return _position;
}

void SimpleSlam::Math::InertialNavigationSystem::update_position(const Vector3& angular_velocity, const Vector3& acceleration) {
    // const Matrix3 attitude = {
    //     Vector3(0, -1 * angular_velocity.get_z() * _time_delta, angular_velocity.get_y() * _time_delta),
    //     Vector3(angular_velocity.get_z() * _time_delta, 0, -1 * angular_velocity.get_x() * _time_delta),
    //     Vector3(-1 * angular_velocity.get_y() * _time_delta, angular_velocity.get_x() * _time_delta, 0),
    // };


}
