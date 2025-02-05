//
// Created by gaetz on 02/02/2025.
//

#include "Mat2.hpp"

namespace gmath {
    Mat2::Mat2( const Mat2 & rhs ) {
        m0 = rhs.m0;
        m1 = rhs.m1;
        m2 = rhs.m2;
        m3 = rhs.m3;
    }

    Mat2::Mat2( f32 m0_, f32 m2_, f32 m1_, f32 m3_  ) {
        m0 = m0_;
        m1 = m1_;
        m2 = m2_;
        m3 = m3_;
    }

    const Mat2 & Mat2::operator *= ( const float rhs ) {
        m0 *= rhs;
        m1 *= rhs;
        m2 *= rhs;
        m3 *= rhs;
        return *this;
    }

    const Mat2 & Mat2::operator += ( const Mat2 & rhs ) {
        m0 += rhs.m0;
        m1 += rhs.m1;
        m2 += rhs.m2;
        m3 += rhs.m3;
        return *this;
    }

    f32 &Mat2::operator()(int x, int y) {
        switch (y * 2 + x) {
            case 0:
                return m0;
            case 1:
                return m1;
            case 2:
                return m2;
            case 3:
                return m3;
            default:
                return m0;
        }
    }
} // gmath