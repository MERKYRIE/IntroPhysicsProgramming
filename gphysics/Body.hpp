//
// Created by gaetz on 03/02/2025.
//

#ifndef GPHYSICS_BODY_HPP
#define GPHYSICS_BODY_HPP

#include "Vec.hpp"
#include "Quat.hpp"

namespace gdraw {
    class Shape;
    class Drawable;
}

namespace gphysics {
    class Shape;
}

namespace gmath {
    class Mat3;
}

using gmath::Vec;
using gmath::Quat;
using gphysics::Shape;
using gdraw::Drawable;
using gmath::Mat3;

namespace gphysics {
    class Body
    {
    public:
        Body(const Vec &position_, const Quat &orientation_, Shape* shape_, Drawable* drawable_);

        void Update(f32 dt);

        Vec GetCenterOfMassWorldSpace() const;
        Vec GetCenterOfMassBodySpace() const;

        Vec WorldSpaceToBodySpace(const Vec& worldPoint);
        Vec BodySpaceToWorldSpace(const Vec& bddyPoint);

        void ApplyImpulseLinear(const Vec& impulse);

        Mat3 GetInverseInertiaTensorBodySpace() const;
        Mat3 GetInverseInertiaTensorWorldSpace() const;

        void ApplyImpulseAngular(const Vec& impulse);

        /// <summary>
        /// Apply impulse on a specific world space
        /// </summary>
        /// <param name="impulsePoint">
        /// The world space location of the application of the impulse
        /// </param>
        /// <param name="impulse">
        /// The world space direction and magnitude of the impulse
        ///</param>
        void ApplyImpulse(const Vec& impulsePoint, const Vec& impulse);

        Vec position;
        Quat orientation;
        Shape* shape { nullptr };
        Drawable* drawable { nullptr };
        Vec linearVelocity;
        float inverseMass;
        float elasticity;
        Vec angularVelocity;
        float friction;

    };
}



#endif //GPHYSICS_BODY_HPP
