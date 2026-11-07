#pragma once

#include "particle.h"
#include "core.h"
#include <vector>

namespace AccelEngine
{
    // generic class for particleForces like, gravity, friction etc etc
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
        Vector2 gravity;

    public:
        ParticleGravity(const Vector2 &gravity) : gravity(gravity) {}

        virtual void updateForce(Particle *particle, real duration);
    };

    class ParticleDrag : public ParticleForceGenerator
    {
        // k1 is for linear drag like, table or road
        real k1;

        // k2 is for quadratic drag used when drag changes with speed like air or water
        real k2;

    public:
        ParticleDrag(real k1, real k2) : k1(k1), k2(k2) {}
        virtual void updateForce(Particle *particle, real duration);
    };

    // spring force generator, takes other particle sprint constant and rest lenght as argument
    class ParticleSpring : public ParticleForceGenerator
    {
        Particle *other;
        real springConstant;
        real restLenght;

    public:
        ParticleSpring(Particle *other, real sC, real rL) : other(other), springConstant(sC), restLenght(rL) {}

        virtual void updateForce(Particle *p, real duration);
    };

    class ParticleAnchoredSpring : public ParticleForceGenerator
    {
        Vector2 *anchor;
        real springConstant;
        real restLenght;

    public:
        ParticleAnchoredSpring(Vector2 *anchor, real sC, real rL) : anchor(anchor), springConstant(sC), restLenght(rL) {}

        virtual void updateForce(Particle *p, real duration);
    };

    class ParticleBungee : public ParticleForceGenerator
    {
        Particle *other;
        real springConstant;
        real restLenght;

    public:
        ParticleBungee(Particle *other, real sC, real rL) : other(other), springConstant(sC), restLenght(rL) {}

        virtual void updateForce(Particle *p, real duration);
    };
}