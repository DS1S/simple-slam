#pragma once

#include <array>

#include "math/vector.h"
#include "math/matrix.h"


namespace SimpleSlam::Math {

    class InertialNavigationSystem {
        public:
            InertialNavigationSystem(
                const double& time_delta,
                const Vector3& velocity,
                const Vector3& position
            );
            Matrix3 get_rotation_matrix() const;
            Vector3 get_velocity() const;
            Vector3 get_position() const;
            void update_position(const Vector3& angular_velocity, const Vector3& acceleration);

        private:
            const double _time_delta;
            Vector3 _velocity;
            Vector3 _position;
            Matrix3 _rotation_matrix;
            
    };
}