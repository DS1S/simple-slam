#pragma once
#include <string>

namespace SimpleSlam::Math {

class Vector3 {
    public:
        Vector3(double x, double y, double z);
        double dot(const Vector3& other) const;
        Vector3 cross(const Vector3& other) const;
        Vector3 normalize() const;
        Vector3 operator*(double scalar) const;
        Vector3 operator/(double scalar) const;
        double magnitude() const;
        double get_x() const;
        double get_y() const;
        double get_z() const;
        std::string to_string() const;

    private:
        const double _x{0};
        const double _y{0};
        const double _z{0};
};

class Vector2 {
    public:
        Vector2(double x, double y);
        Vector2 operator+(const Vector2& other) const;
        Vector2 operator*(double scalar) const;
        Vector2 operator/(double scalar) const;
        Vector2 normalize() const;
        static void normalize(Vector2& vector);
        double magnitude() const;
        double get_x() const;
        double get_y() const;
        std::string to_string() const;

    private:
        const double _x{0};
        const double _y{0};
};

}
