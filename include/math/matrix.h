#pragma once

#include <array>
#include "vector.h"

namespace SimpleSlam::Math {
    /**
     * 3x3 Matrix Representation
    */
    class Matrix3 {
        public:
        Matrix3(const std::array<Vector3, 3> matrix);
        Vector3 operator[](const size_t& index) const;
        Matrix3 operator*(const Matrix3& other) const;
        Vector3 operator*(const Vector3& other) const;
        Matrix3 operator*(const double& scalar) const;
        Matrix3 operator+(const Matrix3& other) const;

        private:
        std::array<Vector3, 3> _matrix;
    };

}