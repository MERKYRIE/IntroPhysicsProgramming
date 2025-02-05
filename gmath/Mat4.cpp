#include "Mat4.hpp"
#include "Mat3.hpp"
#include <cstdlib>

#include <Vec.hpp>
#include <Quat.hpp>

namespace gmath {
    const Mat4 Mat4::identity
            {
                    1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f
            };

    const Mat4 Mat4::zero
            {
                    0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 0.0f
            };

    Mat4 Mat4::CreateRotationZ(float radians) {
        return Mat4
                {
                        cosf(radians), sinf(radians), 0, 0,
                        -sinf(radians), cosf(radians), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1
                };
    }

    Mat4 Mat4::CreateRotationX(float radians) {
        return Mat4
                {
                        1, 0, 0, 0,
                        0, cosf(radians), 0, sinf(radians),
                        0, -sinf(radians), 0, cosf(radians),
                        0, 0, 0, 1
                };
    }

    Mat4 Mat4::CreateRotationY(float radians) {
        return Mat4
                {
                        cosf(radians), sinf(radians), 0, 0,
                        0, 1, 0, 0,
                        -sinf(radians), cosf(radians), 0, 0,
                        0, 0, 0, 1
                };
    }

    Mat4 Mat4::CreateTranslation(float x, float y, float z) {
        return Mat4
                {
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        x, y, z, 1
                };
    }

    // Extract the scale component from the matrix
    Vec Mat4::GetScale() const {
        Vec retVal;
        retVal.x = Vec(m0, m1, m2).Magnitude();
        retVal.y = Vec(m4, m5, m3).Magnitude();
        retVal.z = Vec(m8, m9, m10).Magnitude();
        return retVal;
    }

    // Create a scale matrix with x, y, and z scales
    Mat4 Mat4::CreateScale(float xScale, float yScale, float zScale) {
        return
                {
                        xScale, 0.0f, 0.0f, 0.0f,
                        0.0f, yScale, 0.0f, 0.0f,
                        0.0f, 0.0f, zScale, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f
                };
    }

    Mat4 Mat4::CreateScale(const Vec &scaleVector) {
        return CreateScale(scaleVector.x, scaleVector.y, scaleVector.z);
    }

    // Create a scale matrix with a uniform factor
    Mat4 Mat4::CreateScale(float scale) {
        return CreateScale(scale, scale, scale);
    }

    Mat4 Mat4::CreateOrthographicOffCenter(float left, float right, float bottom, float top, float zNearPlane,
                                           float zFarPlane) {
        return Mat4
                {
                        2.0f / (right - left), 0, 0, 0,
                        0, 2.0f / (top - bottom), 0, 0,
                        0, 0, 1.0f / (zNearPlane - zFarPlane), 0,
                        (left + right) / (left - right), (top + bottom) / (bottom - top),
                        zNearPlane / (zNearPlane - zFarPlane), 1
                };
    }

    Mat4 Mat4::CreatePerspectiveFieldOfView(float fieldOfView, float aspectRatio, float nearPlaneDistance,
                                            float farPlaneDistance) {
        float num = 1.0f / ((float) tanf(fieldOfView * 0.5f));

        return Mat4
                {
                        num / aspectRatio, 0, 0, 0,
                        0, num, 0, 0,
                        0, 0, farPlaneDistance / (nearPlaneDistance - farPlaneDistance), -1,
                        0, 0, (nearPlaneDistance * farPlaneDistance) / (nearPlaneDistance - farPlaneDistance), 0
                };
    }

    Mat4 Mat4::operator*(const Mat4 &other) const {
        Mat4 result;

        result.m0 = (
                (m0 * other.m0) +
                (m4 * other.m1) +
                (m8 * other.m2) +
                (m12 * other.m3)
        );

        result.m4 = (
                (m0 * other.m4) +
                (m4 * other.m5) +
                (m8 * other.m6) +
                (m12 * other.m7)
        );

        result.m8 = (
                (m0 * other.m8) +
                (m4 * other.m9) +
                (m8 * other.m10) +
                (m12 * other.m11)
        );

        result.m12 = (
                (m0 * other.m12) +
                (m4 * other.m13) +
                (m8 * other.m14) +
                (m12 * other.m15)
        );

        result.m1 = (
                (m1 * other.m0) +
                (m5 * other.m1) +
                (m9 * other.m2) +
                (m13 * other.m3)
        );

        result.m5 = (
                (m1 * other.m4) +
                (m5 * other.m5) +
                (m9 * other.m6) +
                (m13 * other.m7)
        );

        result.m9 = (
                (m1 * other.m8) +
                (m5 * other.m9) +
                (m9 * other.m10) +
                (m13 * other.m11)
        );

        result.m13 = (
                (m1 * other.m12) +
                (m5 * other.m13) +
                (m9 * other.m14) +
                (m13 * other.m15)
        );

        result.m2 = (
                (m2 * other.m0) +
                (m6 * other.m1) +
                (m10 * other.m2) +
                (m14 * other.m3)
        );

        result.m6 = (
                (m2 * other.m4) +
                (m6 * other.m5) +
                (m10 * other.m6) +
                (m14 * other.m7)
        );

        result.m10 = (
                (m2 * other.m8) +
                (m6 * other.m9) +
                (m10 * other.m10) +
                (m14 * other.m11)
        );

        result.m14 = (
                (m2 * other.m12) +
                (m6 * other.m13) +
                (m10 * other.m14) +
                (m14 * other.m15)
        );

        result.m3 = (
                (m3 * other.m0) +
                (m7 * other.m1) +
                (m11 * other.m2) +
                (m15 * other.m3)
        );

        result.m7 = (
                (m3 * other.m4) +
                (m7 * other.m5) +
                (m11 * other.m6) +
                (m15 * other.m7)
        );

        result.m11 = (
                (m3 * other.m8) +
                (m7 * other.m9) +
                (m11 * other.m10) +
                (m15 * other.m11)
        );

        result.m15 = (
                (m3 * other.m12) +
                (m7 * other.m13) +
                (m11 * other.m14) +
                (m15 * other.m15)
        );

        return result;
    }

    const f32 *Mat4::ToArrayConst() const {
        return reinterpret_cast<const f32 *>(this);
    }

    f32 *Mat4::ToArray()  {
        return reinterpret_cast<f32 *>(this);
    }

    Mat4 Mat4::CreateLookAt(const Vec &eye, const Vec &target, const Vec &up) {
        Vec zAxis = (eye - target).Normalize();
        Vec xAxis = up.Cross(zAxis).Normalize();
        Vec yAxis = zAxis.Cross(xAxis);

        return Mat4{
                xAxis.x, yAxis.x, zAxis.x, 0,
                xAxis.y, yAxis.y, zAxis.y, 0,
                xAxis.z, yAxis.z, zAxis.z, 0,
                -xAxis.Dot(eye), -yAxis.Dot(eye), -zAxis.Dot(eye), 1
        };
    }

    Mat4 Mat4::CreateFromQuaternion(const Quat &q) {
        Mat4 mat;

        mat.m0 = 1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z;
        mat.m4 = 2.0f * q.x * q.y + 2.0f * q.w * q.z;
        mat.m8 = 2.0f * q.x * q.z - 2.0f * q.w * q.y;
        mat.m12 = 0.0f;

        mat.m1 = 2.0f * q.x * q.y - 2.0f * q.w * q.z;
        mat.m5 = 1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z;
        mat.m9 = 2.0f * q.y * q.z + 2.0f * q.w * q.x;
        mat.m13 = 0.0f;

        mat.m2 = 2.0f * q.x * q.z + 2.0f * q.w * q.y;
        mat.m6 = 2.0f * q.y * q.z - 2.0f * q.w * q.x;
        mat.m10 = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;
        mat.m14 = 0.0f;

        mat.m3 = 0.0f;
        mat.m7 = 0.0f;
        mat.m11 = 0.0f;
        mat.m15 = 1.0f;

        return mat;
    }


    float Mat4::Trace() const {
        const float xx = m0 * m0;
        const float yy = m5 * m5;
        const float zz = m10 * m10;
        const float ww = m15 * m15;
        return xx + yy + zz + ww;
    }

    inline float Mat4::Determinant() const {
        float det = 0.0f;
        float sign = 1.0f;
        for (int j = 0; j < 4; j++) {
            Mat3 minor = Minor(0, j);

            det += (*this)(0, j) * minor.Determinant() * sign;
            sign = sign * -1.0f;
        }
        return det;
    }

    inline Mat4 Mat4::Transpose() const {
        Mat4 transpose;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transpose(i, j) = (*this)(j, i);
            }
        }
        return transpose;
    }

    inline Mat4 Mat4::Inverse() const {
        Mat4 inv;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                inv(j, i) = Cofactor(i, j);    // Perform the transpose while calculating the cofactors
            }
        }
        float det = Determinant();
        float invDet = 1.0f / det;
        inv *= invDet;
        return inv;
    }

    Mat3 Mat4::Minor(const int i, const int j) const {
        Mat3 minor;

        int yy = 0;
        for (int y = 0; y < 4; y++) {
            if (y == j) {
                continue;
            }

            int xx = 0;
            for (int x = 0; x < 4; x++) {
                if (x == i) {
                    continue;
                }

                minor(xx, yy) = (*this)(x, y);
                xx++;
            }

            yy++;
        }
        return minor;
    }

    inline float Mat4::Cofactor(const int i, const int j) const {
        const Mat3 minor = Minor(i, j);
        const float C = float(pow(-1, i + 1 + j + 1)) * minor.Determinant();
        return C;
    }

    inline void Mat4::Orient(Vec pos, Vec fwd, Vec up) {
        Vec zAxis = fwd.Normalize();
        Vec xAxis = up.Cross(zAxis).Normalize();
        Vec yAxis = zAxis.Cross(xAxis);

        m0 = xAxis.x;
        m1 = xAxis.y;
        m2 = xAxis.z;
        m3 = 0;

        m4 = yAxis.x;
        m5 = yAxis.y;
        m6 = yAxis.z;
        m7 = 0;

        m8 = zAxis.x;
        m9 = zAxis.y;
        m10 = zAxis.z;
        m11 = 0;

        m12 = pos.x;
        m13 = pos.y;
        m14 = pos.z;
        m15 = 1;
    }

    f32 Mat4::operator()(int x, int y) const {
        switch (y * 3 + x) {
            case 0:
                return m0;
            case 1:
                return m1;
            case 2:
                return m2;
            case 3:
                return m3;
            case 4:
                return m4;
            case 5:
                return m5;
            case 6:
                return m6;
            case 7:
                return m7;
            case 8:
                return m8;
            case 9:
                return m9;
            case 10:
                return m10;
            case 11:
                return m11;
            case 12:
                return m12;
            case 13:
                return m13;
            case 14:
                return m14;
            case 15:
                return m15;
            default:
                return m0;
        }
    }

    f32 &Mat4::operator()(int x, int y) {
        switch (y * 3 + x) {
            case 0:
                return m0;
            case 1:
                return m1;
            case 2:
                return m2;
            case 3:
                return m3;
            case 4:
                return m4;
            case 5:
                return m5;
            case 6:
                return m6;
            case 7:
                return m7;
            case 8:
                return m8;
            case 9:
                return m9;
            case 10:
                return m10;
            case 11:
                return m11;
            case 12:
                return m12;
            case 13:
                return m13;
            case 14:
                return m14;
            case 15:
                return m15;
            default:
                return m0;
        }
    }

    Mat4 &Mat4::operator*=(const f32 rhs) {
        m0 *= rhs;
        m1 *= rhs;
        m2 *= rhs;
        m3 *= rhs;
        m4 *= rhs;
        m5 *= rhs;
        m6 *= rhs;
        m7 *= rhs;
        m8 *= rhs;
        m9 *= rhs;
        m10 *= rhs;
        m11 *= rhs;
        m12 *= rhs;
        m13 *= rhs;
        m14 *= rhs;
        m15 *= rhs;
        return *this;
    }

    Mat4 Mat4::operator*(const float rhs) const {
        Mat4 tmp;
        tmp.m0 = m0 * rhs;
        tmp.m1 = m1 * rhs;
        tmp.m2 = m2 * rhs;
        tmp.m3 = m3 * rhs;
        tmp.m4 = m4 * rhs;
        tmp.m5 = m5 * rhs;
        tmp.m6 = m6 * rhs;
        tmp.m7 = m7 * rhs;
        tmp.m8 = m8 * rhs;
        tmp.m9 = m9 * rhs;
        tmp.m10 = m10 * rhs;
        tmp.m11 = m11 * rhs;
        tmp.m12 = m12 * rhs;
        tmp.m13 = m13 * rhs;
        tmp.m14 = m14 * rhs;
        tmp.m15 = m15 * rhs;
        return tmp;
    }
}