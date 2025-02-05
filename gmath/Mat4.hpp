#pragma once

#ifndef GMATH_MAT4_HPP
#define GMATH_MAT4_HPP

#include <Defines.hpp>


namespace gmath {
    class Vec;
    class Mat3;
    class Quat;

    class Mat4 {
    public:
        Mat4() = default;

        Mat4(f32 m0_, f32 m4_, f32 m8_, f32 m12_,
             f32 m1_, f32 m5_, f32 m9_, f32 m13_,
             f32 m2_, f32 m6_, f32 m10_, f32 m14_,
             f32 m3_, f32 m7_, f32 m11_, f32 m15_)
                :
                m0{m0_}, m4{m4_}, m8{m8_}, m12{m12_},
                m1{m1_}, m5{m5_}, m9{m9_}, m13{m13_},
                m2{m2_}, m6{m6_}, m10{m10_}, m14{m14_},
                m3{m3_}, m7{m7_}, m11{m11_}, m15{m15_} {}

        f32 m0 {0}, m4 {0}, m8 {0}, m12 {0}; // Matrix first row (4 components)
        f32 m1 {0}, m5 {0}, m9 {0}, m13 {0}; // Matrix second row (4 components)
        f32 m2 {0}, m6 {0}, m10 {0}, m14 {0}; // Matrix third row (4 components)
        f32 m3 {0}, m7 {0}, m11 {0}, m15 {0}; // Matrix fourth row (4 components)

        static const Mat4 identity;
        static const Mat4 zero;

        f32& operator()(int x, int y);
        f32 operator()(int x, int y) const;
        Mat4& operator*=(f32 rhs);
        Mat4 operator * ( float rhs ) const;
        Mat4 operator * ( const Mat4 & rhs ) const;

        static Mat4 CreateRotationZ(f32 angle);

        static Mat4 CreateRotationX(f32 angle);

        static Mat4 CreateRotationY(f32 angle);

        static Mat4 CreateTranslation(f32 x, f32 y, f32 z);

        static Mat4 CreateScale(float xScale, float yScale, float zScale);

        static Mat4 CreateScale(const Vec &scaleVector);

        static Mat4 CreateScale(float scale);

        static Mat4 CreateFromQuaternion(const class Quat &q);


        [[nodiscard]] Vec GetScale() const;

        [[nodiscard]] const f32 *ToArrayConst() const;
        [[nodiscard]] f32 *ToArray();

        static Mat4 CreateLookAt(const Vec &eye, const Vec &target, const Vec &up);

        static Mat4
        CreateOrthographicOffCenter(f32 left, f32 right, f32 bottom, f32 top, f32 zNearPlane, f32 zFarPlane);

        static Mat4
        CreatePerspectiveFieldOfView(f32 fieldOfView, f32 aspectRatio, f32 nearPlaneDistance, f32 farPlaneDistance);

        f32 Trace() const;
        f32 Determinant() const;
        Mat4 Transpose() const;
        Mat4 Inverse() const;
        Mat3 Minor( i32 i, i32 j ) const;
        f32 Cofactor( i32 i, i32 j ) const;
        void Orient(Vec pos, Vec fwd, Vec up );
    };
}

#endif //GMATH_MAT4_HPP