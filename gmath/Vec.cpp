//
// Created by Gaëtan Blaise-Cazalet on 13/01/2025.
//

#include "Vec.hpp"

namespace gmath {

    const Vec Vec::zero {0, 0, 0 };
    const Vec Vec::one {1, 1, 1 };
    const Vec Vec::up {0, 1, 0 };
    const Vec Vec::right {1, 0, 0 };
    const Vec Vec::forward {0, 0, -1 };
    const Vec Vec::down {0, -1, 0 };
    const Vec Vec::left {-1, 0, 0 };
    const Vec Vec::backward {0, 0, 1 };
    
    void Vec::Invert() {
        x = -x;
        y = -y;
        z = -z;
    }

    real Vec::Magnitude() const {
        return RealSqrt(x * x + y * y + z * z);
    }

    real Vec::SquareMagnitude() const {
        return x * x + y * y + z * z;
    }

    Vec& Vec::Normalize() {
        real length = Magnitude();
        if (length > 0) {
            (*this) *= ((real)1)/length;
        }
        return *this;
    }

    void Vec::Clear() {
        x = y = z = 0;
    }

    void Vec::operator*=(const real scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
    }

    Vec Vec::operator*(const real scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    void Vec::operator+=(const Vec& v) {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    Vec Vec::operator+(const Vec& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    void Vec::operator-=(const Vec& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    Vec Vec::operator-(const Vec& v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vec Vec::operator/(i32 scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    void Vec::AddScaledVector(const Vec& v, const real scale) {
        x += v.x * scale;
        y += v.y * scale;
        z += v.z * scale;
    }

    void Vec::MakeOrthonormalBasis(Vec& a, Vec& b, Vec& c) {
        a.Normalize();
        c = a % b;
        if (c.SquareMagnitude() == 0) {
            return;
        }
        c.Normalize();
        b = c % a;
    }

    Vec Vec::ComponentProduct(const Vec& v) const {
        return {x * v.x, y * v.y, z * v.z};
    }

    void Vec::ComponentProductUpdate(const Vec& v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }

    void Vec::operator%=(const Vec& v) {
        *this = Cross(v);
    }
}
