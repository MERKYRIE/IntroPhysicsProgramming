//
// Created by gaetz on 02/02/2025.
//

#include "Mat3.hpp"
#include <Mat2.hpp>
#include <Vec.hpp>

namespace gmath {
    Mat3 Mat3::zero{0, 0, 0, 0, 0, 0, 0, 0, 0};
    Mat3 Mat3::identity{1, 0, 0, 0, 1, 0, 0, 0, 1};;

    Mat3::Mat3(const Mat3 &rhs) {
        m0 = rhs.m0;
        m3 = rhs.m3;
        m6 = rhs.m6;
        m1 = rhs.m1;
        m4 = rhs.m4;
        m7 = rhs.m7;
        m2 = rhs.m2;
        m5 = rhs.m5;
        m8 = rhs.m8;
    }

    Mat3::Mat3(const Vec &row0_, const Vec &row1_, const Vec &row2_) {
        m0 = row0_.x;
        m3 = row0_.y;
        m6 = row0_.z;
        m1 = row1_.x;
        m4 = row1_.y;
        m7 = row1_.z;
        m2 = row2_.x;
        m5 = row2_.y;
        m8 = row2_.z;
    }


    Mat3::Mat3(f32 m0_, f32 m3_, f32 m6_,
               f32 m1_, f32 m4_, f32 m7_,
               f32 m2_, f32 m5_, f32 m8_) {
        m0 = m0_;
        m1 = m1_;
        m2 = m2_;
        m3 = m3_;
        m4 = m4_;
        m5 = m5_;
        m6 = m6_;
        m7 = m7_;
        m8 = m8_;
    }

    const f32 *Mat3::ToArrayConst() const {
        return reinterpret_cast<const f32 *>(this);
    }

    f32 *Mat3::ToArray() {
        return reinterpret_cast<f32 *>(this);
    }

    const Mat3 &Mat3::operator*=(const f32 rhs) {
        m0 *= rhs;
        m1 *= rhs;
        m2 *= rhs;
        m3 *= rhs;
        m4 *= rhs;
        m5 *= rhs;
        m6 *= rhs;
        m7 *= rhs;
        m8 *= rhs;
        return *this;
    }

    const Mat3 &Mat3::operator+=(const Mat3 &rhs) {
        m0 += rhs.m0;
        m3 += rhs.m3;
        m6 += rhs.m6;
        m1 += rhs.m1;
        m4 += rhs.m4;
        m7 += rhs.m7;
        m2 += rhs.m2;
        m5 += rhs.m5;
        m8 += rhs.m8;
        return *this;
    }

    void Mat3::Zero() {
        m0 = 0;
        m3 = 0;
        m6 = 0;
        m1 = 0;
        m4 = 0;
        m7 = 0;
        m2 = 0;
        m5 = 0;
        m8 = 0;
    }

    void Mat3::Identity() {
        m0 = 1;
        m3 = 0;
        m6 = 0;
        m1 = 0;
        m4 = 1;
        m7 = 0;
        m2 = 0;
        m5 = 0;
        m8 = 1;
    }

    f32 Mat3::Trace() const {
        const f32 xx = m0 * m0;
        const f32 yy = m4 * m4;
        const f32 zz = m8 * m8;
        return (xx + yy + zz);
    }

    f32 Mat3::Determinant() const {
        const f32 i = m0 * (m4 * m8 - m7 * m5);
        const f32 j = m3 * (m1 * m8 - m7 * m2);
        const f32 k = m6 * (m1 * m5 - m4 * m2);
        return (i - j + k);
    }

    Mat3 Mat3::Transpose() const {
        return {m0, m1, m2, m3, m4, m5, m6, m7, m8};
    }

    Mat3 Mat3::Inverse() const {
        Mat3 inv;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                inv(j, i) = Cofactor(i, j);    // Perform the transpose while calculating the cofactors
            }
        }
        f32 det = Determinant();
        f32 invDet = 1.0f / det;
        inv *= invDet;
        return inv;
    }

    Mat2 Mat3::Minor(const int i, const int j) const {
        Mat2 minor;

        int yy = 0;
        for (int y = 0; y < 3; y++) {
            if (y == j) {
                continue;
            }

            int xx = 0;
            for (int x = 0; x < 3; x++) {
                if (x == i) {
                    continue;
                }
                f32 data = (*this)(x, y);
                minor(xx, yy) = data;
                xx++;
            }

            yy++;
        }
        return minor;
    }

    f32 Mat3::Cofactor(const int i, const int j) const {
        const Mat2 minor = Minor(i, j);
        const f32 C = f32(pow(-1, i + 1 + j + 1)) * minor.Determinant();
        return C;
    }

    Vec Mat3::operator*(const Vec &rhs) const {
        Vec tmp{};
        Vec row0{m0, m3, m6};
        Vec row1{m1, m4, m7};
        Vec row2{m2, m5, m8};
        tmp.x = row0.Dot(rhs);
        tmp.y = row1.Dot(rhs);
        tmp.z = row2.Dot(rhs);
        return tmp;
    }

    Mat3 Mat3::operator*(const f32 rhs) const {
        Mat3 tmp{
                Vec{m0, m3, m6} * rhs,
                Vec{m1, m4, m7} * rhs,
                Vec{m2, m5, m8} * rhs
        };
        return tmp;
    }

    Mat3 Mat3::operator*(const Mat3 &rhs) const {
        Mat3 tmp;
        tmp.m0 = m0 * rhs.m0 + m3 * rhs.m1 + m6 * rhs.m2;
        tmp.m3 = m0 * rhs.m3 + m3 * rhs.m4 + m6 * rhs.m5;
        tmp.m6 = m0 * rhs.m6 + m3 * rhs.m7 + m6 * rhs.m8;

        tmp.m1 = m1 * rhs.m0 + m4 * rhs.m1 + m7 * rhs.m2;
        tmp.m4 = m1 * rhs.m3 + m4 * rhs.m4 + m7 * rhs.m5;
        tmp.m7 = m1 * rhs.m6 + m4 * rhs.m7 + m7 * rhs.m8;

        tmp.m2 = m2 * rhs.m0 + m5 * rhs.m1 + m8 * rhs.m2;
        tmp.m5 = m2 * rhs.m3 + m5 * rhs.m4 + m8 * rhs.m5;
        tmp.m8 = m2 * rhs.m6 + m5 * rhs.m7 + m8 * rhs.m8;

        return tmp;
    }

    Mat3 Mat3::operator+(const Mat3 &rhs) const {
        Mat3 tmp;
        tmp.m0 = m0 + rhs.m0;
        tmp.m3 = m3 + rhs.m3;
        tmp.m6 = m6 + rhs.m6;
        tmp.m1 = m1 + rhs.m1;
        tmp.m4 = m4 + rhs.m4;
        tmp.m7 = m7 + rhs.m7;
        tmp.m2 = m2 + rhs.m2;
        tmp.m5 = m5 + rhs.m5;
        tmp.m8 = m8 + rhs.m8;
        return tmp;
    }

    f32& Mat3::operator()(int x, int y) {
        switch (y * 3 + x) {
            case 0: return m0;
            case 1: return m1;
            case 2: return m2;
            case 3: return m3;
            case 4: return m4;
            case 5: return m5;
            case 6: return m6;
            case 7: return m7;
            case 8: return m8;
            default: return m0;
        }
    }

    f32 Mat3::operator()(int x, int y) const {
        switch (y * 3 + x) {
            case 0: return m0;
            case 1: return m1;
            case 2: return m2;
            case 3: return m3;
            case 4: return m4;
            case 5: return m5;
            case 6: return m6;
            case 7: return m7;
            case 8: return m8;
            default: return m0;
        }
    }
}