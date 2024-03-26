#pragma once

#include <deque>

#include "math/quaternion.h"
#include "math/vector.h"

namespace SimpleSlam::Math {

class InertialNavigationSystem {
   public:
    InertialNavigationSystem(const double time_delta, const Vector3& e_north,
                             const Vector3& accel_offset,
                             const Vector3& gyro_offset,
                             const Vector3& velocity, const Vector3& position);
    Vector3 get_velocity() const;
    Vector3 get_position() const;
    void update_position(const Vector3& angular_velocity, const Vector3& force,
                         const Vector3& magno);
    void add_sample(const Vector3& sample);
    double calculate_variance() const;

   private:
    static const int _NUM_SAMPLES = 8;
    static const int _VARIANCE_THRESHOLD = 300;
    const double _time_delta;
    Vector3 _e_north;
    Quaternion _q;
    Vector3 _accel_offset;
    Vector3 _gyro_offset;
    Vector3 _velocity;
    Vector3 _position;
    std::deque<Vector3> _samples;
};
}  // namespace SimpleSlam::Math