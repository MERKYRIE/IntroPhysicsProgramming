//
// Created by Gaëtan Blaise-Cazalet on 14/01/2025.
//

#include "Particle.hpp"
#include "Asserts.hpp"

namespace gphysics
{
    void Particle::ClearAccumulator() { forceAccum.Clear(); }

    void Particle::Integrate(const real duration) {
        // We don't integrate things with infinite mass
        if (inverseMass <= 0.0f) { return; }
        GASSERT_DEBUG(duration > 0.0f);

        // Update linear position
        position.AddScaledVector(velocity, duration);
        // Update linear velocity
        Vec resultingAcceleration = acceleration;
        velocity.AddScaledVector(resultingAcceleration, inverseMass);
        velocity *= RealPow(damping, duration); // Damping for numerical stability

        // Clear forces
        ClearAccumulator();
    }
} // gphysics
