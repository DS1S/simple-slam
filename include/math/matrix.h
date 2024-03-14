#pragma once

#include <array>
#include "vector.h"

namespace SimpleSlam::Math {
    /**
     * 3x3 Matrix Representation
    */
    class Matrix3 {
        public:
        Matrix3(const std::array<SimpleSlam::Math::Vector3, 3> matrix);
        Vector3 operator[](const size_t index);
        Matrix3 operator*(const Matrix3& other);

        private:
        std::array<SimpleSlam::Math::Vector3, 3> _matrix;
    };

    const SimpleSlam::Math::Matrix3 IDENTITY_MATRIX({Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1)});
}