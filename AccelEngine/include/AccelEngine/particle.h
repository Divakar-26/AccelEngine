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


        real damping = 0.99f;
        real inverseMass;

        void integrate(real duration);
    };
} // namespace AccelEngine
