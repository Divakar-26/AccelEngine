#include "AccelEngine/particle.h"
#include <cassert>

namespace AccelEngine
{
    void Particle::integrate(real duration)
    {
        if (duration <= 0.0f) return;

        assert(inverseMass > 0.0f);

        position.addScaledVector(velocity, duration);

        Vector3 resultingAcc = acceleration;

        velocity.addScaledVector(resultingAcc, duration);

        velocity *= pow(damping, duration);
    }
}
