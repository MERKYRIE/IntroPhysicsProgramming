#include "Intersections.hpp"
#include "ShapeSphere.hpp"
#include "Contact.hpp"

namespace gphysics {
    bool Intersections::Intersect(Body& a, Body& b,
        const float dt, Contact& contact)
    {
        contact.a = &a;
        contact.b = &b;
        const Vec ab = b.position - a.position;
        contact.normal = ab;
        contact.normal.Normalize();

        if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE
        && b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) {
            ShapeSphere* sphereA = static_cast<ShapeSphere*>(a.shape);
            ShapeSphere* sphereB = static_cast<ShapeSphere*>(b.shape);

            Vec posA = a.position;
            Vec posB = b.position;
            Vec valA = a.linearVelocity;
            Vec velB = b.linearVelocity;

            if (Intersections::SphereSphereDynamic(*sphereA, *sphereB,
                posA, posB, valA, velB, dt,
                contact.ptOnAWorldSpace, contact.ptOnBWorldSpace,
                contact.timeOfImpact))
            {
                // Step bodies forward to get local space collision points
                a.Update(contact.timeOfImpact);
                b.Update(contact.timeOfImpact);

                // Convert world space contacts to local space
                contact.ptOnALocalSpace =
                    a.WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
                contact.ptOnBLocalSpace =
                    b.WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

                Vec ab = a.position - b.position;
                contact.normal = ab;
                contact.normal.Normalize();

                // Unwind time step
                a.Update(-contact.timeOfImpact);
                b.Update(-contact.timeOfImpact);

                // Calculate separation distance
                float r = ab.Magnitude()
                          - (sphereA->radius + sphereB->radius);
                contact.separationDistance = r;
                return true;
            }
        }

        return false;
    }

    bool Intersections::RaySphere(const Vec& rayStart, const Vec& rayDir,
            const Vec& sphereCenter, const float sphereRadius, float& t0, float& t1)
    {
        const Vec& s = sphereCenter - rayStart;
        const float a = Vec(rayDir).Dot(rayDir);
        const float b = Vec(s).Dot(rayDir);
        const float c = Vec(s).Dot(s) - sphereRadius * sphereRadius;

        const float delta = b * b - a * c;
        const float inverseA = 1.0f / a;

        if (delta < 0) {
            // No solution
            return false;
        }

        const float deltaRoot = sqrtf(delta);
        t0 = (b - deltaRoot) * inverseA;
        t1 = (b + deltaRoot) * inverseA;

        return true;
    }

    bool Intersections::SphereSphereDynamic(
            const ShapeSphere& shapeA, const ShapeSphere& shapeB,
            const Vec& posA, const Vec& posB, const Vec& velA, const Vec& velB,
            const float dt, Vec& ptOnA, Vec& ptOnB, float& timeOfImpact)
    {
        const Vec relativeVelocity = velA - velB;

        const Vec startPtA = posA;
        const Vec endPtA = startPtA + relativeVelocity * dt;
        const Vec rayDir = endPtA - startPtA;

        float t0 = 0;
        float t1 = 0;
        if (Vec(rayDir).Dot(rayDir) < 0.001f * 0.001f)
        {
            // Ray is too short, just check if already intersecting
            Vec ab = posB - posA;
            float radius = shapeA.radius + shapeB.radius + 0.001f;
            if (Vec(ab).Dot(ab) > radius * radius)
            {
                return false;
            }
        }
        else if (!RaySphere(startPtA, rayDir, posB,
            shapeA.radius + shapeB.radius, t0, t1))
        {
            return false;
        }

        // Change from [0, 1] to [0, dt];
        t0 *= dt;
        t1 *= dt;

        // If the collision in only in the past, there will be
        // no future collision for this frame
        if (t1 < 0) return false;

        // Get earliest positive time of impact
        timeOfImpact = t0 < 0.0f ? 0.0f : t0;

        // If the earliest collision is too far in the future,
        // then there's no collision this frame
        if (timeOfImpact > dt) {
            return false;
        }

        // Get the points on the respective points of collision
        // and return true
        Vec newPosA = posA + velA * timeOfImpact;
        Vec newPosB = posB + velB * timeOfImpact;
        Vec ab = newPosB - newPosA;
        ab.Normalize();

        ptOnA = newPosA + ab * shapeA.radius;
        ptOnB = newPosB - ab * shapeB.radius;
        return true;
    }
}