//
// Created by gaetz on 02/02/2025.
//

#include "Quat.hpp"
#include <Mat3.hpp>

namespace gmath {
    const Quat Quat::identity(0.0f, 0.0f, 0.0f, 1.0f);

    Quat Quat::Slerp(const Quat &a, const Quat &b, real f) {
        real rawCosm = Quat::Dot(a, b);

        real cosom = -rawCosm;
        if (rawCosm >= 0.0f) {
            cosom = rawCosm;
        }

        real scale0, scale1;

        if (cosom < 0.9999f) {
            const real omega = acos(cosom);
            const real invSin = 1.f / RealSin(omega);
            scale0 = RealSin((1.f - f) * omega) * invSin;
            scale1 = RealSin(f * omega) * invSin;
        } else {
            // Use linear interpolation if the quaternions
            // are collinear
            scale0 = 1.0f - f;
            scale1 = f;
        }

        if (rawCosm < 0.0f) {
            scale1 = -scale1;
        }

        Quat retVal;
        retVal.x = scale0 * a.x + scale1 * b.x;
        retVal.y = scale0 * a.y + scale1 * b.y;
        retVal.z = scale0 * a.z + scale1 * b.z;
        retVal.w = scale0 * a.w + scale1 * b.w;
        retVal.Normalize();
        return retVal;
    }

    Quat::Quat(const Vec &axis, real angle) {
        real scalar = RealSin(angle / 2.0f);
        x = axis.x * scalar;
        y = axis.y * scalar;
        z = axis.z * scalar;
        w = RealCos(angle / 2.0f);
    }

    void Quat::Set(real inX, real inY, real inZ, real inW) {
        x = inX;
        y = inY;
        z = inZ;
        w = inW;
    }

    void Quat::Conjugate() {
        x *= -1.0f;
        y *= -1.0f;
        z *= -1.0f;
    }

    real Quat::MagnitudeSq() const {
        return (x * x + y * y + z * z + w * w);
    }

    real Quat::Magnitude() const {
        return RealSqrt(MagnitudeSq());
    }

    void Quat::Normalize() {
        real len = Magnitude();
        x /= len;
        y /= len;
        z /= len;
        w /= len;
    }

    Quat Quat::Normalize(const Quat &q) {
        Quat retVal = q;
        retVal.Normalize();
        return retVal;
    }

    Quat Quat::Lerp(const Quat &a, const Quat &b, real f) {
        Quat retVal;
        retVal.x = gmath::Lerp(a.x, b.x, f);
        retVal.y = gmath::Lerp(a.y, b.y, f);
        retVal.z = gmath::Lerp(a.z, b.z, f);
        retVal.w = gmath::Lerp(a.w, b.w, f);
        retVal.Normalize();
        return retVal;
    }

    Quat Quat::Concatenate(const Quat &q, const Quat &p) {
        Quat retVal;

        // Vector component is:
        // ps * qv + qs * pv + pv x qv
        Vec qv(q.x, q.y, q.z);
        Vec pv(p.x, p.y, p.z);
        Vec newVec = p.w * qv + q.w * pv + pv.Cross(qv);
        retVal.x = newVec.x;
        retVal.y = newVec.y;
        retVal.z = newVec.z;

        // Scalar component is:
        // ps * qs - pv . qv
        retVal.w = p.w * q.w - pv.Dot(qv);

        return retVal;
    }


    Quat &Quat::operator*=(const float &rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        w *= rhs;
        return *this;
    }

    Quat &Quat::operator*=(const Quat &rhs) {
        Quat temp = *this * rhs;
        w = temp.w;
        x = temp.x;
        y = temp.y;
        z = temp.z;
        return *this;
    }

    Quat Quat::operator*(const Quat &rhs) const {
        Quat temp;
        temp.w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
        temp.x = (x * rhs.w) + (w * rhs.x) + (y * rhs.z) - (z * rhs.y);
        temp.y = (y * rhs.w) + (w * rhs.y) + (z * rhs.x) - (x * rhs.z);
        temp.z = (z * rhs.w) + (w * rhs.z) + (x * rhs.y) - (y * rhs.x);
        return temp;
    }

    void Quat::Invert() {
        *this *= 1.0f / MagnitudeSq();
        x = -x;
        y = -y;
        z = -z;
    }

    Quat Quat::Inverse() const {
        Quat val( *this );
        val.Invert();
        return val;
    }

    Vec Quat::RotatePoint(const Vec &rhs) const {
        Quat vector( rhs.x, rhs.y, rhs.z, 0.0f );
        Quat final = *this * vector * Inverse();
        return { final.x, final.y, final.z };
    }

    Mat3 Quat::RotateMatrix(const Mat3 &rhs) const {
        Vec row0 = RotatePoint( { rhs.m0, rhs.m3, rhs.m6 } );
        Vec row1 = RotatePoint( { rhs.m1, rhs.m4, rhs.m7 } );
        Vec row2 = RotatePoint( { rhs.m2, rhs.m5, rhs.m8 } );
        return {row0, row1, row2};
    }

    bool Quat::IsValid() const {
        if ( x * 0 != x * 0 ) {
            return false;
        }
        if ( y * 0 != y * 0 ) {
            return false;
        }
        if ( z * 0 != z * 0 ) {
            return false;
        }
        if ( w * 0 != w * 0 ) {
            return false;
        }
        return true;
    }

    Mat3 Quat::ToMat3() const {
        Mat3 mat = Mat3::identity;
        Vec row0 = RotatePoint( {mat.m0, mat.m3, mat.m6 } );
        Vec row1 = RotatePoint( {mat.m1, mat.m4, mat.m7 } );
        Vec row2 = RotatePoint( {mat.m2, mat.m5, mat.m8 } );
        return {row0, row1, row2};
    }


}