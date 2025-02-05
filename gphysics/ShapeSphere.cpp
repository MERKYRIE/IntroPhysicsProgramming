//
// Created by gaetz on 03/02/2025.
//

#include "ShapeSphere.hpp"
#include "Mat3.hpp"
#include "Bounds.hpp"

namespace gphysics {
    Mat3 ShapeSphere::InertiaTensor() const
    {
        Mat3 tensor;
        tensor.Zero();
        tensor.m0 = 2.0f * radius * radius / 5.0f;
        tensor.m4 = 2.0f * radius * radius / 5.0f;
        tensor.m8 = 2.0f * radius * radius / 5.0f;
        return tensor;
    }

    Bounds ShapeSphere::GetBounds(const Vec& pos, const Quat& orient) const
    {
        Bounds tmp;
        tmp.mins = Vec(-radius, -radius, -radius) + pos;
        tmp.maxs = Vec(radius, radius, radius) + pos;
        return tmp;
    }

    Bounds ShapeSphere::GetBounds() const
    {
        Bounds tmp;
        tmp.mins = Vec(-radius, -radius, -radius);
        tmp.maxs = Vec(radius, radius, radius);
        return tmp;
    }
}