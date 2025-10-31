#pragma once

#include "core.h"

namespace AccelEngine
{
    class Particle
    {
    public:
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;

        Vector3 forceAccum;

        real damping = 0.99f;
        real inverseMass = 1.0f;

        void integrate(real duration);
        void clearAccumulator();
        void addForce(const Vector3 & force);
    };
} // namespace AccelEngine
