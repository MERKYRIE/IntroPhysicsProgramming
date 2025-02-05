#pragma once
#include "Body.hpp"
#include "Shape.hpp"

namespace gphysics {
    class Contact;
    class ShapeSphere;
    
    class Intersections
    {
    public:
        static bool Intersect(Body& a, Body& b,
            const float dt, Contact& contact);
        static bool RaySphere(const Vec& rayStart, const Vec& rayDir,
            const Vec& sphereCenter, const float sphereRadius, float& t0, float& t1);
        static bool SphereSphereDynamic(
            const ShapeSphere& shapeA, const ShapeSphere& shapeB,
            const Vec& posA, const Vec& posB, const Vec& velA, const Vec& velB,
            const float dt, Vec& ptOnA, Vec& ptOnB, float& timeOfImpact);
    };
}