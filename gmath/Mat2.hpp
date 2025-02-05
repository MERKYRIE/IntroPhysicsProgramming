//
// Created by gaetz on 02/02/2025.
//

#ifndef GMATH_MAT2_HPP
#define GMATH_MAT2_HPP

#include <Defines.hpp>

namespace gmath {

    class Mat2 {
    public:
        Mat2() = default;
        Mat2( const Mat2 & rhs );
        Mat2( f32 m0_, f32 m2_, f32 m1_, f32 m3_ );
        f32& operator()(int x, int y);

        const Mat2 & operator *= ( float rhs );
        const Mat2 & operator += ( const Mat2 & rhs );

        [[nodiscard]] f32 Determinant() const { return m0 * m3 - m2 * m1; }

    public:
        f32 m0 { 0 }, m2 { 0 }; // Matrix first row (2 components)
        f32 m1 { 0 }, m3 { 0 }; // Matrix second row (2 components)
    };

} // gmath

#endif //GMATH_MAT2_HPP
