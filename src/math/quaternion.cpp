/**
 * Disclaimer: The code written in this file has been heavily inspired from https://cs.stanford.edu/~acoates/quaternion.h. 
 * Certain adjustments were made to make it cohesive with our Vector3 implementation.
*/

#include <math.h>

#include "math/conversion.h"
#include "math/vector.h"
#include "math/quaternion.h"


SimpleSlam::Math::Quaternion::Quaternion() {
    _mData[0] = _mData[1] = _mData[2] = 0;
    _mData[3] = 1;
}

SimpleSlam::Math::Quaternion::Quaternion(const SimpleSlam::Math::Vector3& v, double w) {
    _mData[0] = v[0];
    _mData[1] = v[1];
    _mData[2] = v[2];
    _mData[3] = w;
}

SimpleSlam::Math::Quaternion::Quaternion(double x, double y, double z, double w) {
    _mData[0] = x;
    _mData[1] = y;
    _mData[2] = z;
    _mData[3] = w;
}

double SimpleSlam::Math::Quaternion::x() const { return _mData[0]; }
double SimpleSlam::Math::Quaternion::y() const { return _mData[1]; }
double SimpleSlam::Math::Quaternion::z() const { return _mData[2]; }
double SimpleSlam::Math::Quaternion::w() const { return real(); }

SimpleSlam::Math::Vector3 SimpleSlam::Math::Quaternion::complex() const { return Vector3(_mData[0], _mData[1], _mData[2]); }
void SimpleSlam::Math::Quaternion::complex(const Vector3& c) {
    _mData[0] = c[0];
    _mData[1] = c[1];
    _mData[2] = c[2];
}

/**
 * @brief Retrieve the scalar part of the quaternion.
 *
 * @return The scalar part of the quaternion.
 */
double SimpleSlam::Math::Quaternion::real() const { return _mData[3]; }
void SimpleSlam::Math::Quaternion::real(double r) { _mData[3] = r; }

/**
 * @brief Computes the axis-angle representation of the quaternion.
 * 
 * @return The quaternion formulated by the angle and axis of rotation.
*/
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::axis_angle_to_quat(const double& theta,
                                        const Vector3& v) {
    return Quaternion(v[0] * sin(theta / 2), v[1] * sin(theta / 2),
                        v[2] * sin(theta / 2), cos(theta / 2));
}

SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::conjugate(void) const {
    return Quaternion(complex() * -1, real());
}

/**
 * @brief Computes the inverse of this quaternion.
 *
 * @note This is a general inverse.  If you know a priori
 * that you're using a unit quaternion (i.e., norm() == 1),
 * it will be significantly faster to use conjugate() instead.
 *
 * @return The quaternion q such that q * (*this) == (*this) * q
 * == [ 0 0 0 1 ]<sup>T</sup>.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::inverse(void) const { return conjugate() / pow(norm(), 2); }

/**
 * @brief Computes the product of this quaternion with the
 * quaternion 'rhs'.
 *
 * @param rhs The right-hand-side of the product operation.
 *
 * @return The quaternion product (*this) x @p rhs.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::product(const Quaternion& rhs) const {
    return Quaternion(
        y() * rhs.z() - z() * rhs.y() + x() * rhs.w() + w() * rhs.x(),
        z() * rhs.x() - x() * rhs.z() + y() * rhs.w() + w() * rhs.y(),
        x() * rhs.y() - y() * rhs.x() + z() * rhs.w() + w() * rhs.z(),
        w() * rhs.w() - x() * rhs.x() - y() * rhs.y() - z() * rhs.z());
}

/**
 * @brief Quaternion product operator.
 *
 * The result is a quaternion such that:
 *
 * result.real() = (*this).real() * rhs.real() -
 * (*this).complex().dot(rhs.complex());
 *
 * and:
 *
 * result.complex() = rhs.complex() * (*this).real
 * + (*this).complex() * rhs.real()
 * - (*this).complex().cross(rhs.complex());
 *
 * @return The quaternion product (*this) x rhs.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator*(const Quaternion& rhs) const { return product(rhs); }

/**
 * @brief Quaternion scalar product operator.
 * @param s A scalar by which to multiply all components
 * of this quaternion.
 * @return The quaternion (*this) * s.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator*(double s) const {
    return Quaternion(complex() * s, real() * s);
}

/**
 * @brief Produces the sum of this quaternion and rhs.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator+(const Quaternion& rhs) const {
    return Quaternion(x() + rhs.x(), y() + rhs.y(), z() + rhs.z(),
                        w() + rhs.w());
}

/**
 * @brief Produces the difference of this quaternion and rhs.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator-(const Quaternion& rhs) const {
    return Quaternion(x() - rhs.x(), y() - rhs.y(), z() - rhs.z(),
                        w() - rhs.w());
}

/**
 * @brief Unary negation.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator-() const { return Quaternion(-x(), -y(), -z(), -w()); }

/**
 * @brief Quaternion scalar division operator.
 * @param s A scalar by which to divide all components
 * of this quaternion.
 * @return The quaternion (*this) / s.
 */
SimpleSlam::Math::Quaternion SimpleSlam::Math::Quaternion::operator/(double s) const {
    return Quaternion(complex() / s, real() / s);
}

/**
 * @brief Returns the norm ("magnitude") of the quaternion.
 * @return The 2-norm of [ w(), x(), y(), z() ]<sup>T</sup>.
 */
double SimpleSlam::Math::Quaternion::norm() const {
    return sqrt(_mData[0] * _mData[0] + _mData[1] * _mData[1] +
                _mData[2] * _mData[2] + _mData[3] * _mData[3]);
}

/** @brief Returns an equivalent euler angle representation of
 * this quaternion.
 * @return Euler angles in roll-pitch-yaw order.
 */
SimpleSlam::Math::Vector3 SimpleSlam::Math::Quaternion::euler(void) const {
    const static double PI_OVER_2 = SimpleSlam::Math::pi * 0.5;
    double sqw, sqx, sqy, sqz;

    // quick conversion to Euler angles to give tilt to user
    sqw = _mData[3] * _mData[3];
    sqx = _mData[0] * _mData[0];
    sqy = _mData[1] * _mData[1];
    sqz = _mData[2] * _mData[2];

    const double roll =
        atan2(2.0 * (_mData[3] * _mData[0] + _mData[1] * _mData[2]),
                1 - 2 * (sqx + sqy));
    const double pitch =
        asin(2.0 * (_mData[3] * _mData[1] - _mData[0] * _mData[2]));
    const double yaw =
        atan2(2.0 * (_mData[0] * _mData[1] + _mData[3] * _mData[2]),
                1 - 2 * (sqy + sqz));

    Vector3 euler(roll, pitch, yaw);
    return euler;
}

