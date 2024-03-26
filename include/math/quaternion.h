/**
 * Disclaimer: The code written in this file has been heavily inspired from https://cs.stanford.edu/~acoates/quaternion.h. 
 * Certain adjustments were made to make it cohesive with our Vector3 implementation.
*/

#pragma once

#include "vector.h"

namespace SimpleSlam::Math {
class Quaternion {
   private:
    double _mData[4];

   public:
    static Quaternion axis_angle_to_quat(const double& theta, const Vector3& v);
    Quaternion();
    Quaternion(const Vector3& v, double w);
    Quaternion(double x, double y, double z, double w);
    double x() const;
    double y() const;
    double z() const;
    double w() const;
    Vector3 complex() const;
    void complex(const Vector3& c);
    double real() const;
    void real(double r);
    Quaternion conjugate(void) const;
    Quaternion inverse(void) const;
    Quaternion product(const Quaternion& rhs) const;
    Quaternion operator*(const Quaternion& rhs) const;
    Quaternion operator*(double s) const;
    Quaternion operator+(const Quaternion& rhs) const;
    Quaternion operator-(const Quaternion& rhs) const;
    Quaternion operator-() const;
    Quaternion operator/(double s) const;
    double norm() const;
    Vector3 euler(void) const;
};
}  // namespace SimpleSlam::Math
