#include "math/inertial_navigation.h"

#include <math.h>

#include <deque>

#include "math/Matrix.h"
#include "math/quaternion.h"
#include "math/vector.h"
#include "mbed.h"

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
    const Matrix3 identity_matrix(
        {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)});

    const Quaternion mag_body(magno[0], magno[1], magno[2], 0);
    const Quaternion q_u_body(force[0], force[1], force[2], 0);
    const Quaternion q_a_body(force[0], force[1], force[2], 0);

    const Quaternion rot_gyro_offset =
        _q.inverse() *
        Quaternion(_gyro_offset[0], _gyro_offset[1], _gyro_offset[2], 0) * _q;
    const Vector3 omega(angular_velocity[0], angular_velocity[1],
                        angular_velocity[2]);

    Quaternion rot = _q;
    Quaternion q_delta(0.5 * angular_velocity[0] * _time_delta,
                       0.5 * angular_velocity[1] * _time_delta,
                       0.5 * angular_velocity[2] * _time_delta, 1);
    q_delta = q_delta / q_delta.norm();
    rot = rot * q_delta;
    rot = rot / rot.norm();

    const Quaternion q_a_world_unoff = rot * q_a_body * rot.inverse();
    Vector3 v(q_a_world_unoff.x(), q_a_world_unoff.y(), q_a_world_unoff.z());
    v = v / v.magnitude();

    // Tilt Correction
    const double quo = sqrt(v[0] * v[0] + v[1] * v[1]);
    const Vector3 n(v[1] / quo, -1 * v[0] / quo, 0);
    const double phi = std::atan2(quo, v[2]);
    // const double phi = std::acos(v.element(2,0) / v.norm());
    // const Vector3 n = v.cross(Vector3(0,0,1));
    // const Vector3 nc = calculate_cross(Vector3(0,0,1), v);
    // printf("%s %s %s %s\n",
    //     Vector3(v.element(0,0), v.element(1,0),
    //     v.element(2,0)).to_string().c_str(),
    //     Vector3(0,0,1).to_string().c_str(),
    //     Vector3(n.element(0,0), n.element(1,0),
    //     n.element(2,0)).to_string().c_str(), Vector3(nc.element(0,0),
    //     nc.element(1,0), nc.element(2,0)).to_string().c_str()
    // );
    Quaternion rot_c =
        Quaternion::axis_angle_to_quat(0.05 * phi, n / n.magnitude());
    rot_c = rot_c / rot_c.norm();
    rot_c = rot_c * rot;
    rot_c = rot_c / rot_c.norm();

    const Quaternion magno_world = rot_c * mag_body * rot_c.inverse();
    Vector3 mag(magno_world.x(), magno_world.y(), 0);
    Vector3 mag_t(magno_world.x(), magno_world.y(), 0);
    mag = mag.normalize();
    mag_t = mag_t / mag_t.magnitude();
    const int signum = mag[1] == 0 ? 0 : mag[1] > 0 ? 1 : -1;
    // const Vector3 north = Vector3(0, 0, 1);
    // const Vector3 north =
    // mag_t.cross(Vector3(_e_north[0],_e_north[1],_e_north[2]));
    const Vector3 north = mag_t.cross(Vector3(1, 0, 0));
    const double gamma = std::atan2(abs(mag[1]), mag[0]);
    // const double gamma = std::acos(mag.dot(_e_north));
    Quaternion rot_c2 =
        Quaternion::axis_angle_to_quat(0.1 * gamma, north / north.magnitude());
    const Vector3 euler_angle_mag = rot_c2.euler() * 180 / SimpleSlam::Math::pi;

    // printf("ROT: [%f %f %f] [%f %f %f] %s %f\n",
    //     euler_angle_mag.element(0,0), euler_angle_mag.element(1,0),
    //     euler_angle_mag.element(2,0), north.element(0,0), north.element(1,0),
    //     north.element(2,0), mag.to_string().c_str(), gamma * 180 /
    //     SimpleSlam::Math::pi
    // );
    rot_c2 = rot_c2 / rot_c2.norm();
    rot_c2 = rot_c2 * rot_c;
    rot_c2 = rot_c2 / rot_c2.norm();

    Quaternion q_u_world = rot_c2 * q_u_body * rot_c2.inverse();
    const Vector3 world_force_unoff = q_u_world.complex();
    const Vector3 world_force(world_force_unoff[0], world_force_unoff[1],
                              world_force_unoff[2]);

    _q = rot_c2;

    // add_sample(force);
    const double variance = calculate_variance();
    // for (auto& sample: _samples) {
    //     printf("%s ", sample.to_string().c_str());
    // }
    // printf("%f\n", variance);
    if (variance < _VL) {
        _velocity = Vector3(0, 0, 0);
    } else {
        _velocity = _velocity +
                    (Vector3(world_force[0], world_force[1], world_force[2]) *
                     9.8 * _time_delta);
    }

    _position = _position + (_velocity * _time_delta);
    const Vector3 euler_angle = _q.euler() * 180 / SimpleSlam::Math::pi;
    // printf("ROT: [%f %f %f] body_force: %s world_force: [%f, %f, %f]\n",
    //     euler_angle[], euler_angle[],
    //     euler_angle[], force.to_string().c_str(), v[0],
    //     v[1],
    //     v[2]
    // );
    printf(
        "ROT: [%f %f %f] body_force: %s world_force: [%f, %f, %f] VEL %s POS "
        "%s\n",
        euler_angle[0], euler_angle[1], euler_angle[2],
        force.to_string().c_str(), world_force[0], world_force[1],
        world_force[2], _velocity.to_string().c_str(),
        _position.to_string().c_str());
}

void SimpleSlam::Math::InertialNavigationSystem::add_sample(
    const Vector3& sample) {
    const int size = _samples.size();
    if (size == _E) {
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
    mean /= _E;

    for (auto& sample : _samples) {
        const double norm2 = pow(sample.magnitude(), 2);
        variance += pow((norm2 - mean), 2);
    }
    return variance / _E;
}