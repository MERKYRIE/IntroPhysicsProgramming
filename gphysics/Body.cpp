//
// Created by gaetz on 03/02/2025.
//

#include "Body.hpp"
#include "Shape.hpp"
#include "Drawable.hpp"
#include "Mat3.hpp"
#include "SphereCube.hpp"
#include "ShapeSphere.hpp"

using gdraw::SphereCube;

namespace gphysics {

    Body::Body(const Vec &position_, const Quat &orientation_, Shape *shape_, Drawable *drawable_) :
            position{position_},
            orientation{orientation_},
            shape{shape_},
            drawable{drawable_} {

    }

    void Body::Update(f32 dt) {
        position += linearVelocity * dt;

        // We have an angular velocity around the center of mass,
        // this needs to be converted to relative to model position.
        // This way we can properly update the orientation
        // of the model
        Vec positionCM = GetCenterOfMassWorldSpace();
        Vec CMToPositon = position - positionCM;

        // Total torques is equal to external applied
        // torques + internal torque (precession)
        // T = Texternal + w x I * w
        // Texternal = 0 because it was applied in the collision
        // response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        Mat3 orientationMat = orientation.ToMat3();
        Mat3 inertiaTensor = orientationMat
                             * shape->InertiaTensor() * orientationMat.Transpose();
        Vec alpha = inertiaTensor.Inverse()
                    * (angularVelocity.Cross(inertiaTensor * angularVelocity));
        angularVelocity += alpha * dt;

        // Update orientation
        Vec dAngle = angularVelocity * dt;
        Quat dq = Quat(dAngle, dAngle.Magnitude());
        orientation = dq * orientation;
        orientation.Normalize();

        // Get the new model position
        position = positionCM + dq.RotatePoint(CMToPositon);

        drawable->Update(dt);

        static_cast<SphereCube *>(drawable)->transform
        =
        Mat4::CreateTranslation(position.x , position.y , position.z) * Mat4::CreateFromQuaternion(orientation) * Mat4::CreateScale(static_cast<ShapeSphere *>(shape)->radius);
    }

    Vec Body::GetCenterOfMassWorldSpace() const
    {
        const Vec centerOfMass = shape->GetCenterOfMass();
        const Vec pos = position + orientation.RotatePoint(centerOfMass);
        return pos;
    }

    Vec Body::GetCenterOfMassBodySpace() const
    {
        return shape->GetCenterOfMass();
    }

    Vec Body::WorldSpaceToBodySpace(const Vec& worldPoint)
    {
        const Vec tmp = worldPoint - GetCenterOfMassWorldSpace();
        const Quat invertOrient = orientation.Inverse();
        Vec bodySpace = invertOrient.RotatePoint(tmp);
        return bodySpace;
    }

    Vec Body::BodySpaceToWorldSpace(const Vec& bodyPoint)
    {
        Vec worldSpace = GetCenterOfMassWorldSpace()
                         + orientation.RotatePoint(bodyPoint);
        return worldSpace;
    }

    void Body::ApplyImpulseLinear(const Vec& impulse)
    {
        if (inverseMass == 0.0f) return;
        // dv = J / m
        linearVelocity += impulse * inverseMass;
    }

    Mat3 Body::GetInverseInertiaTensorBodySpace() const
    {
        Mat3 inertiaTensor = shape->InertiaTensor();
        Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;
        return inverseInertiaTensor;
    }

    Mat3 Body::GetInverseInertiaTensorWorldSpace() const
    {
        Mat3 inertiaTensor = shape->InertiaTensor();
        Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;
        Mat3 orient = orientation.ToMat3();
        inverseInertiaTensor = orient * inverseInertiaTensor
                                      * orient.Transpose();
        return inverseInertiaTensor;
    }

    void Body::ApplyImpulseAngular(const Vec& impulse)
    {
        if (inverseMass == 0.0f) return;

        // L = I w = r x p
        // dL = I dw = r x J
        // dw = I^-1 * ( r x J )
        angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

        // Clamp angular velocity
        // -- 30 rad per seconds, sufficient for now
        const float maxAngularSpeed = 30.0f;
        if (Vec(angularVelocity).Dot(angularVelocity) > maxAngularSpeed * maxAngularSpeed)
        {
            angularVelocity.Normalize();
            angularVelocity *= maxAngularSpeed;
        }
    }

    void Body::ApplyImpulse(const Vec& impulsePoint, const Vec& impulse)
    {
        if (inverseMass == 0.0f) return;
        ApplyImpulseLinear(impulse);

        // Applying impulse must produce torques through the center of mass
        Vec position = GetCenterOfMassWorldSpace();
        Vec r = impulsePoint - position;
        Vec dL = r.Cross(impulse); // World space
        ApplyImpulseAngular(dL);
    }
}