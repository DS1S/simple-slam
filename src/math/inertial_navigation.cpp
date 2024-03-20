#include "math/inertial_navigation.h"

#include <math.h>

#include "math/Matrix.h"
#include "math/quaternion.h"
#include "math/vector.h"
#include "mbed.h"

SimpleSlam::Math::InertialNavigationSystem::InertialNavigationSystem(
    const double time_delta, const Vector3& temp_acc, const Vector3& accel_offset,
    const Vector3& gyro_offset, const Vector3& velocity,
    const Vector3& position)
    : _time_delta{time_delta},
      _temp_acc{temp_acc},
      _q{Quaternion(0, 0, 0, 1)},
      _accel_offset{accel_offset},
      _gyro_offset{gyro_offset},
      _velocity{velocity},
      _position{position},
      _rotation_matrix{{Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)}} {

        double a1 = atan2(temp_acc[1], temp_acc[2]) * 57.3 * SimpleSlam::Math::pi / 180;
        double a2 = atan2(-1 * temp_acc[0], sqrt(temp_acc[1] * temp_acc[1] + temp_acc[2] * temp_acc[2])) * 57.3 * SimpleSlam::Math::pi / 180;

        TVector3 euler(a1, a2, 0);
        Quaternion q;
        q.euler(euler);
        _q = _q * q;

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

    const Quaternion q_u_body(force[0], force[1], force[2], 0);
    const Quaternion q_a_body(force[0], force[1], force[2], 0);

    const TVector3 omega(
        // (static_cast<double>(static_cast<int>((angular_velocity[0]) * 10))) / 10, 
        // (static_cast<double>(static_cast<int>((angular_velocity[1]) * 10))) / 10, 
        // (static_cast<double>(static_cast<int>((angular_velocity[2]) * 10))) / 10
        angular_velocity[0] - _gyro_offset[0],
        angular_velocity[1] - _gyro_offset[1],
        angular_velocity[2] - _gyro_offset[2]
    );
    // omega = omega * _time_delta;

    Quaternion rot = _q;
    if (omega.norm() != 0) {
        const Quaternion q_delta = Quaternion::axis_angle_to_quat(
                omega.norm() * _time_delta, omega / omega.norm());
        // Quaternion q_delta;
        // q_delta.euler(omega * _time_delta);
        rot = rot * q_delta;
    }

    if (std::pow(rot.norm(), 2) > 1) {
        rot = rot / rot.norm();
    }

    const Quaternion q_a_world_unoff = rot * q_a_body * rot.inverse();
    const Quaternion q_a_world(
        q_a_world_unoff.x() - _accel_offset[0],
        q_a_world_unoff.y() - _accel_offset[1],
        q_a_world_unoff.z() - _accel_offset[2],
        0
    );
    
    const TVector3 v(
        q_a_world.x() / q_a_world.norm(),
        q_a_world.y() / q_a_world.norm(),
        q_a_world.z() / q_a_world.norm()
    );

    const double phi = std::acos(v.element(2,0));
    const TVector3 n = v.cross(TVector3(0,0,1));
    const Quaternion rot_c = Quaternion::axis_angle_to_quat(0.01 * phi, n / n.norm()) * rot;

    Quaternion q_u_world = rot_c * q_u_body * rot_c.inverse();
    const TVector3 world_force_unoff = q_u_world.complex();
    const TVector3 world_force(
        std::abs(world_force_unoff.element(0,0) - _accel_offset[0]) > 0.03 ? world_force_unoff.element(0,0) - _accel_offset[0] : 0,
        std::abs(world_force_unoff.element(1,0) - _accel_offset[0]) > 0.03 ? world_force_unoff.element(1,0) - _accel_offset[1] : 0,
        std::abs(world_force_unoff.element(2,0) - _accel_offset[0]) > 0.03 ? world_force_unoff.element(2,0) - _accel_offset[2] : 0
        // force[0] - _accel_offset[0],
        // force[1] - _accel_offset[1],
        // force[2] - _accel_offset[2]
    );

    _q = rot_c;
    _velocity = _velocity + (Vector3(
        world_force.element(0,0), 
        world_force.element(1,0), 
        world_force.element(2,0) 
    ) * 9.8 * _time_delta);
    _position = _position + (_velocity * _time_delta);
    const TVector3 euler_angle = rot.euler() * 180 / SimpleSlam::Math::pi;
    printf("ROT: [%f %f %f] body_force: %s world_force: [%f, %f, %f]\n",
        euler_angle.element(0,0), euler_angle.element(1,0), euler_angle.element(2,0),
        force.to_string().c_str(),
        world_force.element(0, 0),
        world_force.element(1, 0),
        world_force.element(2, 0)
    );
    // printf("body_force: %s world_force: [%f, %f, %f] VEL %s POS %s\n",
    //     force.to_string().c_str(),
    //     world_force.element(0, 0),
    //     world_force.element(1, 0),
    //     world_force.element(2, 0),
    //     _velocity.to_string().c_str(),
    //     _position.to_string().c_str()
    // );

    // const double alpha =
    // static_cast<double>(static_cast<int>(angular_velocity.get_x() *
    // _time_delta * 10)) / 10; const double beta =
    // static_cast<double>(static_cast<int>(angular_velocity.get_y() *
    // _time_delta * 10)) / 10; const double gamma =
    // static_cast<double>(static_cast<int>(angular_velocity.get_z() *
    // _time_delta * 10)) / 10; const Matrix3 roll({Vector3(1, 0, 0),
    //                     Vector3(0, std::cos(alpha), -1 * std::sin(alpha)),
    //                     Vector3(0, std::sin(alpha), std::cos(alpha))});
    // const Matrix3 pitch({Vector3(std::cos(beta), 0, std::sin(beta)),
    //                      Vector3(0, 1, 0),
    //                      Vector3(-1 * std::sin(beta), 0, std::cos(beta))});
    // const Matrix3 yaw({Vector3(std::cos(gamma), -1 * std::sin(gamma), 0),
    //                    Vector3(std::sin(gamma), std::cos(gamma), 0),
    //                    Vector3(0, 0, 1)});

    // const Matrix3 attitude = (yaw * pitch * roll);

    // const Matrix3 attitude({
    //     Vector3(0, -1 * angular_velocity.get_z() * _time_delta,
    //             angular_velocity.get_y() * _time_delta),
    //     Vector3(angular_velocity.get_z() * _time_delta, 0,
    //             -1 * angular_velocity.get_x() * _time_delta),
    //     Vector3(-1 * angular_velocity.get_y() * _time_delta,
    //             angular_velocity.get_x() * _time_delta, 0),
    // });
    // const Matrix3 attitude({
    //     Vector3(0, -1 * gamma, beta),
    //     Vector3(gamma, 0, -1 * alpha),
    //     Vector3(-1 * beta, alpha, 0),
    // });

    // const double sigma = (angular_velocity * _time_delta).magnitude();

    // const Matrix3 rotation_matrix =
    //     _rotation_matrix *
    //     (identity_matrix + (attitude * (std::sin(sigma) / sigma)) +
    //      (attitude * attitude) * ((1 - std::cos(sigma)) / std::pow(sigma,
    //      2)));
    // const Matrix3 rotation_matrix = _rotation_matrix * attitude;
    // printf("ANG: %s\n%s\n%s\n%s\n",
    //     angular_velocity.to_string().c_str(),
    //     rotation_matrix[0].to_string().c_str(),
    //     rotation_matrix[1].to_string().c_str(),
    //     rotation_matrix[2].to_string().c_str());

    // const Vector3 local_force = rotation_matrix.transpose() * force;
    // const Vector3 gravity(0, 0, -1);
    // const Vector3 acceleration = local_force + gravity;
    // printf("FORCE: %s LOCAL: %s ANG: %s\n",
    //     force.to_string().c_str(),
    //     local_force.to_string().c_str(),
    //     angular_velocity.to_string().c_str());
    // const Vector3 acceleration = local_force - _accel_offset;

    // _velocity = _velocity + ((acceleration * _time_delta) * 9.8);
    // _position = _position + (_velocity * _time_delta);
    // _rotation_matrix = rotation_matrix;
    // printf("%s\n%s\n%s\n\n",
    //     _rotation_matrix[0].to_string().c_str(),
    //     _rotation_matrix[1].to_string().c_str(),
    //     _rotation_matrix[2].to_string().c_str()
    // );
    // printf("ACC: %s VEL: %s POS: %s\n", .to_string().c_str(),
    //        _velocity.to_string().c_str(), _position.to_string().c_str());
}