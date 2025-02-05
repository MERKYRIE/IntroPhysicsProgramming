//
// Created by Gaëtan Blaise-Cazalet on 13/01/2025.
//

#include "Precision.hpp"

#ifndef GMATH_VEC3_HPP
#define GMATH_VEC3_HPP

namespace gmath {
    class Vec {
    public:
        real x { 0 };
        real y { 0 };
        real z { 0 };
        real w { 1 };

    public:
        Vec() = default;
        Vec(real x, real y, real z, real w = 1) : x(x), y(y), z(z), w(w) {}

        static const Vec zero;
        static const Vec one;
        static const Vec up;
        static const Vec right;
        static const Vec forward;
        static const Vec down;
        static const Vec left;
        static const Vec backward;

        void Invert();

        [[nodiscard]] real Magnitude() const;

        [[nodiscard]] real SquareMagnitude() const;

        Vec &Normalize();

        void Clear();

        void operator*=(real scalar);

        Vec operator*(real scalar) const;

        void operator+=(const Vec &v);

        Vec operator+(const Vec &v) const;

        void operator-=(const Vec &v);

        Vec operator-(const Vec &v) const;

        Vec operator/(i32 scalar) const;

        void AddScaledVector(const Vec &v, real scale);

        friend Vec operator*(float scalar, const Vec &vec) {
            return {vec.x * scalar, vec.y * scalar, vec.z * scalar};
        }

        /** Makes an orthonormal basis from vector, that won't change direction.
         * Vector b and c will be oriented so that the base is orthonormal.
         * All vectors will be normalized.
         * This function is working for a right-handed coordinate system.
         */
        static void MakeOrthonormalBasis(Vec &a, Vec &b, Vec &c);

        /** Calculate and return a component-wise product of this vector with the given vector. */
        Vec ComponentProduct(const Vec &v) const;

        /** Perform a component-wise product with the given vector and set this vector to its result. */
        void ComponentProductUpdate(const Vec &v);

        /** Calculate and return the scalar product of this vector with the given vector. */
        real ScalarProduct(const Vec &v) const { return x * v.x + y * v.y + z * v.z; }

        /** Calculate and return the scalar product of this vector with the given vector. */
        real Dot(const Vec &v) { return x * v.x + y * v.y + z * v.z; }

        /** Calculate and return the scalar product of this vector with the given vector. */
        real operator*(const Vec &v) const { return x * v.x + y * v.y + z * v.z; }

        /** Calculate and return the vector product of this vector with the given vector. */
        Vec VectorProduct(const Vec &v) const {
            return Vec(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
        }

        /** Calculate and return the vector product of this vector with the given vector. */
        Vec Cross(const Vec &v) const { return Vec(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }

        /** Calculate and return the vector product of this vector with the given vector. */
        Vec operator%(const Vec &v) const { return Vec(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }

        /** Updates this vector to be the vector product of its current value and the given vector. */
        void operator%=(const Vec &v);
    };
}


#endif //GMATH_VEC3_HPP
