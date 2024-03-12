#include "math/vector.h"

#include <math.h>

/**
 * Vector 3 Implementation
 */
SimpleSlam::Math::Vector3::Vector3(double x, double y, double z)
    : _x{x}, _y{y}, _z{z} {}

double SimpleSlam::Math::Vector3::dot(const Vector3& other) const {
    return _x * other._x + _y * other._y + _z * other._z;
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Vector3::normalize() const {
    return Vector3(_x, _y, _z) / magnitude();
}

double SimpleSlam::Math::Vector3::magnitude() const {
    return std::sqrt(_x * _x + _y * _y + _z * _z);
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Vector3::cross(
    const Vector3& other) const {
    const double c_x = _y * other._z - _z * other._y;
    const double c_y = _z * other._x - _x * other._z;
    const double c_z = _x * other._y - _y * other._x;
    return Vector3(c_x, c_y, c_z);
}

double SimpleSlam::Math::Vector3::get_x() const { return _x; }

double SimpleSlam::Math::Vector3::get_y() const { return _y; }

double SimpleSlam::Math::Vector3::get_z() const { return _z; }

SimpleSlam::Math::Vector3 SimpleSlam::Math::Vector3::operator*(
    double scalar) const {
    return Vector3(scalar * _x, scalar * _y, scalar * _z);
}

SimpleSlam::Math::Vector3 SimpleSlam::Math::Vector3::operator/(
    double scalar) const {
    return Vector3(_x / scalar, _y / scalar, _z / scalar);
}

std::string SimpleSlam::Math::Vector3::to_string() const {
    return std::string()
        .append("[")
        .append(std::to_string(_x))
        .append(", ")
        .append(std::to_string(_y))
        .append(", ")
        .append(std::to_string(_z))
        .append("]");
}

/**
 * Vector 2 Implementation
 */
SimpleSlam::Math::Vector2::Vector2(double x, double y) : _x{x}, _y{y} {}

SimpleSlam::Math::Vector2 SimpleSlam::Math::Vector2::operator+(
    const Vector2& other) const {
    return Vector2(_x + other._x, _y + other._y);
}

SimpleSlam::Math::Vector2 SimpleSlam::Math::Vector2::operator*(
    double scalar) const {
    return Vector2(scalar * _x, scalar * _y);
}

SimpleSlam::Math::Vector2 SimpleSlam::Math::Vector2::operator/(
    double scalar) const {
    return Vector2(_x / scalar, _y / scalar);
}

double SimpleSlam::Math::Vector2::get_x() const { return _x; }

double SimpleSlam::Math::Vector2::get_y() const { return _y; }

SimpleSlam::Math::Vector2 SimpleSlam::Math::Vector2::normalize() const {
    return Vector2(_x, _y) / magnitude();
}

double SimpleSlam::Math::Vector2::magnitude() const {
    return std::sqrt(_x * _x + _y * _y);
}

std::string SimpleSlam::Math::Vector2::to_string() const {
    return std::string()
        .append("[")
        .append(std::to_string(_x))
        .append(", ")
        .append(std::to_string(_y))
        .append("]");
}
