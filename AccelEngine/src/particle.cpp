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
        resultingAcc.addScaledVector(forceAccum, 1 / inverseMass);

        velocity.addScaledVector(resultingAcc, duration);

        velocity *= pow(damping, duration);

        clearAccumulator();
    }

    void Particle::clearAccumulator(){
        forceAccum.clear();
    }

    void Particle::addForce(const Vector3 & force){
        forceAccum += force;
    }
}
