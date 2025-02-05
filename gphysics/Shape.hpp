//
// Created by gaetz on 02/02/2025.
//

#ifndef GPHYSICS_SHAPE_HPP
#define GPHYSICS_SHAPE_HPP

#include "Vec.hpp"

namespace gmath {
    class Mat3;
    class Quat;
}

namespace gphysics {
    class Bounds;
}

using gmath::Vec;
using gmath::Mat3;
using gmath::Quat;
using gphysics::Bounds;

namespace gphysics {
    class Shape {
    public:
        enum class ShapeType
        {
            SHAPE_SPHERE,
        };

        [[nodiscard]] virtual ShapeType GetType() const = 0;
        virtual Vec GetCenterOfMass() const { return centerOfMass; }
        virtual Mat3 InertiaTensor() const = 0;
        virtual Bounds GetBounds(const Vec& pos, const Quat& orient) const = 0;
        virtual Bounds GetBounds() const = 0;

    protected:
        Vec centerOfMass;
    };
}




#endif //GPHYSICS_SHAPE_HPP
