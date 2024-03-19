#pragma once

#include <array>

#include "math/vector.h"
#include "math/matrix.h"
#include "math/quaternion.h"


namespace SimpleSlam::Math {

    class InertialNavigationSystem {
        public:
            InertialNavigationSystem(
                const double time_delta,
                const Vector3& temp_accel,
                const Vector3& accel_offset,
                const Vector3& gyro_offset,
                const Vector3& velocity,
                const Vector3& position
            );
            Matrix3 get_rotation_matrix() const;
            Vector3 get_velocity() const;
            Vector3 get_position() const;
            void update_position(const Vector3& angular_velocity, const Vector3& force);

        private:
            const double _time_delta;
            Quaternion _q;
            Vector3 _temp_acc;
            Vector3 _accel_offset;
            Vector3 _gyro_offset;
            Vector3 _velocity;
            Vector3 _position;
            Matrix3 _rotation_matrix;
            
    };
}