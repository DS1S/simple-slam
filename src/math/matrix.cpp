#include "math/matrix.h"

#include "math/vector.h"

SimpleSlam::Math::Matrix3::Matrix3(
    const std::array<SimpleSlam::Math::Vector3, 3> matrix)
    : _matrix{matrix} {}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Matrix3::operator[] (
    const size_t& index) const {
    return _matrix[index];
}

SimpleSlam::Math::Matrix3 SimpleSlam::Math::Matrix3::operator*(
    const SimpleSlam::Math::Matrix3& other) const{
    const double a = _matrix[0][0];
    const double b = _matrix[0][1];
    const double c = _matrix[0][2];
    const double d = _matrix[1][0];
    const double e = _matrix[1][1];
    const double f = _matrix[1][2];
    const double g = _matrix[2][0];
    const double h = _matrix[2][1];
    const double i = _matrix[2][2];

    const double j = other[0][0];
    const double k = other[0][1];
    const double l = other[0][2];
    const double m = other[1][0];
    const double n = other[1][1];
    const double o = other[1][2];
    const double p = other[2][0];
    const double q = other[2][1];
    const double r = other[2][2];

    return SimpleSlam::Math::Matrix3(
        {Vector3(a * j + b * m + c * p, a * k + b * n + c * q,
                 a * l + b * o + c * r),
         Vector3(d * j + e * m + f * p, d * k + e * n + f * q,
                 d * l + e * o + f * r),
         Vector3(g * j + h * m + i * p, g * k + h * n + i * q,
                 g * l + h * o + i * r)});
}

SimpleSlam::Math::Matrix3 SimpleSlam::Math::Matrix3::operator*(
    const double& scalar) const {
    return SimpleSlam::Math::Matrix3(
        {_matrix[0] * scalar, _matrix[1] * scalar, _matrix[2] * scalar});
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Matrix3::operator*(
    const Vector3& other) const {
    return Vector3(_matrix[0].dot(other), _matrix[1].dot(other),
                   _matrix[2].dot(other));
}

SimpleSlam::Math::Matrix3 SimpleSlam::Math::Matrix3::operator+(
    const SimpleSlam::Math::Matrix3& other) const {
    const double a = _matrix[0][0];
    const double b = _matrix[0][1];
    const double c = _matrix[0][2];
    const double d = _matrix[1][0];
    const double e = _matrix[1][1];
    const double f = _matrix[1][2];
    const double g = _matrix[2][0];
    const double h = _matrix[2][1];
    const double i = _matrix[2][2];

    const double j = other[0][0];
    const double k = other[0][1];
    const double l = other[0][2];
    const double m = other[1][0];
    const double n = other[1][1];
    const double o = other[1][2];
    const double p = other[2][0];
    const double q = other[2][1];
    const double r = other[2][2];

    return SimpleSlam::Math::Matrix3({Vector3(a + j, b + k, c + l),
                                      Vector3(d + m, e + n, f + o),
                                      Vector3(g + p, h + q, i + r)});
}
