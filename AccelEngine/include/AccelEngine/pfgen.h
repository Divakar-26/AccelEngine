#pragma once

#include "particle.h"
#include "core.h"
#include <vector>

namespace AccelEngine
{
    class ParticleForceGenerator
    {
    public:
        virtual void updateForce(Particle *particle, real duration) = 0;
    };

    class ParticleForceRegistry
    {

    protected:
        struct ParticleForceRegistration
        {
            Particle *p;
            ParticleForceGenerator *fg;
        };

        typedef std::vector<ParticleForceRegistration> Registry;
        Registry registration;

    public:
        void add(Particle *p, ParticleForceGenerator *fg);
        void remove(Particle *p, ParticleForceGenerator *fg);
        void clear();

        void updateForces(real duration);
    };

    class ParticleGravity : public ParticleForceGenerator
    {
        Vector3 gravity;

    public:
        ParticleGravity(const Vector3 & gravity) : gravity(gravity) {}

        virtual void updateForce(Particle * particle, real duration);
    };
}