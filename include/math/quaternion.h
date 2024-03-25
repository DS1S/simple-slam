#pragma once

#include <math.h>

#include <iostream>

#include "conversion.h"
#include "tmatrix.h"
#include "vector.h"

namespace SimpleSlam::Math {
class Quaternion {
   private:
    double _mData[4];

   public:
    Quaternion() {
        _mData[0] = _mData[1] = _mData[2] = 0;
        _mData[3] = 1;
    }

    Quaternion(const Vector3& v, double w) {
        _mData[0] = v[0];
        _mData[1] = v[1];
        _mData[2] = v[2];
        _mData[3] = w;
    }

    Quaternion(const double* array) {
        if (!array) {
            std::cerr << "Constructing quaternion from 0 array." << std::endl;
        }
        for (uint32_t i = 0; i < 4; i++) {
            _mData[i] = array[i];
        }
    }

    Quaternion(double x, double y, double z, double w) {
        _mData[0] = x;
        _mData[1] = y;
        _mData[2] = z;
        _mData[3] = w;
    }

    double x() const { return _mData[0]; }
    double y() const { return _mData[1]; }
    double z() const { return _mData[2]; }
    double w() const { return real(); }

    Vector3 complex() const { return Vector3(_mData[0], _mData[1], _mData[2]); }
    void complex(const Vector3& c) {
        _mData[0] = c[0];
        _mData[1] = c[1];
        _mData[2] = c[2];
    }

    /**
     * @brief Retrieve the scalar part of the quaternion.
     *
     * @return The scalar part of the quaternion.
     */
    double real() const { return _mData[3]; }
    void real(double r) { _mData[3] = r; }

    /**
     * @brief Computes the axis-angle representation of the quaternion.
     * 
     * @return The quaternion formulated by the angle and axis of rotation.
    */
    static Quaternion axis_angle_to_quat(const double& theta,
                                         const Vector3& v) {
        return Quaternion(v[0] * sin(theta / 2), v[1] * sin(theta / 2),
                          v[2] * sin(theta / 2), cos(theta / 2));
    }

    Quaternion conjugate(void) const {
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
    Quaternion inverse(void) const { return conjugate() / pow(norm(), 2); }

    /**
     * @brief Computes the product of this quaternion with the
     * quaternion 'rhs'.
     *
     * @param rhs The right-hand-side of the product operation.
     *
     * @return The quaternion product (*this) x @p rhs.
     */
    Quaternion product(const Quaternion& rhs) const {
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
    Quaternion operator*(const Quaternion& rhs) const { return product(rhs); }

    /**
     * @brief Quaternion scalar product operator.
     * @param s A scalar by which to multiply all components
     * of this quaternion.
     * @return The quaternion (*this) * s.
     */
    Quaternion operator*(double s) const {
        return Quaternion(complex() * s, real() * s);
    }

    /**
     * @brief Produces the sum of this quaternion and rhs.
     */
    Quaternion operator+(const Quaternion& rhs) const {
        return Quaternion(x() + rhs.x(), y() + rhs.y(), z() + rhs.z(),
                          w() + rhs.w());
    }

    /**
     * @brief Produces the difference of this quaternion and rhs.
     */
    Quaternion operator-(const Quaternion& rhs) const {
        return Quaternion(x() - rhs.x(), y() - rhs.y(), z() - rhs.z(),
                          w() - rhs.w());
    }

    /**
     * @brief Unary negation.
     */
    Quaternion operator-() const { return Quaternion(-x(), -y(), -z(), -w()); }

    /**
     * @brief Quaternion scalar division operator.
     * @param s A scalar by which to divide all components
     * of this quaternion.
     * @return The quaternion (*this) / s.
     */
    Quaternion operator/(double s) const {
        if (s == 0) std::clog << "Dividing quaternion by 0." << std::endl;
        return Quaternion(complex() / s, real() / s);
    }

    /**
     * @brief Returns the norm ("magnitude") of the quaternion.
     * @return The 2-norm of [ w(), x(), y(), z() ]<sup>T</sup>.
     */
    double norm() const {
        return sqrt(_mData[0] * _mData[0] + _mData[1] * _mData[1] +
                    _mData[2] * _mData[2] + _mData[3] * _mData[3]);
    }

    /**
     * @brief Computes the rotation matrix represented by a unit
     * quaternion.
     *
     * @note This does not check that this quaternion is normalized.
     * It formulaically returns the matrix, which will not be a
     * rotation if the quaternion is non-unit.
     */
    TMatrix3 rotationMatrix() const {
        double m[9] = {
            1 - 2 * y() * y() - 2 * z() * z(), 2 * x() * y() - 2 * z() * w(),
            2 * x() * z() + 2 * y() * w(),     2 * x() * y() + 2 * z() * w(),
            1 - 2 * x() * x() - 2 * z() * z(), 2 * y() * z() - 2 * x() * w(),
            2 * x() * z() - 2 * y() * w(),     2 * y() * z() + 2 * x() * w(),
            1 - 2 * x() * x() - 2 * y() * y()};
        return TMatrix3(m);
    }

    /**
     * @brief Computes the quaternion that is equivalent to a given
     * euler angle rotation.
     * @param euler A 3-vector in order:  roll-pitch-yaw.
     */
    void euler(const Vector3& euler) {
        double c1 = cos(euler[2] * 0.5);
        double c2 = cos(euler[1] * 0.5);
        double c3 = cos(euler[0] * 0.5);
        double s1 = sin(euler[2] * 0.5);
        double s2 = sin(euler[1] * 0.5);
        double s3 = sin(euler[0] * 0.5);

        _mData[0] = c1 * c2 * s3 - s1 * s2 * c3;
        _mData[1] = c1 * s2 * c3 + s1 * c2 * s3;
        _mData[2] = s1 * c2 * c3 - c1 * s2 * s3;
        _mData[3] = c1 * c2 * c3 + s1 * s2 * s3;
    }

    /** @brief Returns an equivalent euler angle representation of
     * this quaternion.
     * @return Euler angles in roll-pitch-yaw order.
     */
    Vector3 euler(void) const {
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
};
}  // namespace SimpleSlam::Math
