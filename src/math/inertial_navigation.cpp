
#include <math.h>
#include <deque>

#include "mbed.h"
#include "math/inertial_navigation.h"
#include "math/quaternion.h"
#include "math/conversion.h"
#include "math/vector.h"

SimpleSlam::Math::InertialNavigationSystem::InertialNavigationSystem(
    const double time_delta, const Vector3& e_north,
    const Vector3& accel_offset, const Vector3& gyro_offset,
    const Vector3& velocity, const Vector3& position)
    : _time_delta{time_delta},
      _e_north{e_north},
      _q{Quaternion(0, 0, 0, 1)},
      _accel_offset{accel_offset},
      _gyro_offset{gyro_offset},
      _velocity{velocity},
      _position{position} {}

SimpleSlam::Math::Vector3
SimpleSlam::Math::InertialNavigationSystem::get_velocity() const {
    return _velocity;
}

SimpleSlam::Math::Vector3
SimpleSlam::Math::InertialNavigationSystem::get_position() const {
    return _position;
}

void SimpleSlam::Math::InertialNavigationSystem::update_position(
    const Vector3& angular_velocity, const Vector3& force,
    const Vector3& magno) {
    // Gyro Integration
    Quaternion rot = _q;
    Quaternion q_delta(0.5 * angular_velocity[0] * _time_delta,
                       0.5 * angular_velocity[1] * _time_delta,
                       0.5 * angular_velocity[2] * _time_delta, 1);
    q_delta = q_delta / q_delta.norm();
    rot = rot * q_delta;
    rot = rot / rot.norm();

    // Tilt Correction
    Quaternion q_a_body(force[0], force[1], force[2], 0);
    q_a_body = q_a_body / q_a_body.norm();
    const Quaternion q_a_world = rot * q_a_body * rot.conjugate();
    const Vector3 v = q_a_world.complex().normalize();

    const double denom = sqrt(v[0] * v[0] + v[1] * v[1]);
    const Vector3 n(v[1] / denom, -1 * v[0] / denom, 0);
    const double phi = std::atan2(denom, v[2]);
    Quaternion rot_c =
        Quaternion::axis_angle_to_quat(0.05 * phi, n.normalize());

    rot_c = rot_c / rot_c.norm();
    rot_c = rot_c * rot;
    rot_c = rot_c / rot_c.norm();

    // Yaw Correction w/ Magnetometer
    const Quaternion mag_body(magno[0], magno[1], magno[2], 0);
    const Quaternion magno_world = rot_c * mag_body * rot_c.conjugate();
    const Vector3 mag = Vector3(magno_world.x(), magno_world.y(), 0).normalize();

    const int signum = mag[1] == 0 ? 0 : mag[1] > 0 ? 1 : -1;
    const Vector3 north(0,0, -1 * signum);
    const double gamma = std::atan2(abs(mag[1]), mag[0]);
    Quaternion rot_c2 =
        Quaternion::axis_angle_to_quat(0.05 * gamma, north / north.magnitude());

    rot_c2 = rot_c2 / rot_c2.norm();
    rot_c2 = rot_c2 * rot_c;
    rot_c2 = rot_c2 / rot_c2.norm();

    // Rotate force in body frame into local frame
    const Quaternion q_u_body(force[0], force[1], force[2], 0);
    const Quaternion q_u_world = rot_c2 * q_u_body * rot_c2.conjugate();
    const Vector3 world_force = q_u_world.complex() - _accel_offset;

    // Set rotation to our newly corrected quaternion
    _q = rot_c2;

    // Zero velocity update rule
    const double variance = calculate_variance();
    if (variance < _VARIANCE_THRESHOLD) {
        _velocity = Vector3(0, 0, 0);
    } else {
        _velocity = _velocity + (world_force * 9.8 * _time_delta);
    }

    // Position update
    _position = _position + (_velocity * _time_delta);
}

void SimpleSlam::Math::InertialNavigationSystem::add_sample(
    const Vector3& sample) {
    const int size = _samples.size();
    if (size == _NUM_SAMPLES) {
        _samples.pop_front();
    }
    _samples.push_back(sample);
}

double SimpleSlam::Math::InertialNavigationSystem::calculate_variance() const {
    double variance = 0;
    double mean = 0;

    for (auto& sample : _samples) {
        mean += pow(sample.magnitude(), 2);
    }
    mean /= _NUM_SAMPLES;

    for (auto& sample : _samples) {
        const double norm2 = pow(sample.magnitude(), 2);
        variance += pow((norm2 - mean), 2);
    }
    return variance / _NUM_SAMPLES;
}