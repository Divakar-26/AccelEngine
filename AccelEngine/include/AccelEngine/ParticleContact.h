#pragma once

#include "particle.h"
#include "core.h"

namespace AccelEngine
{
    class ParticleContact
    {
    public:
        /* hold the particle which are involved in collision*/
        Particle *particle[2];
        Vector2 contactNormal;
        real restitution;

        real penetration;
        void resolve(real duration);
        real calculateSeperatingVelocity() const;

    protected:

    private:
        void resolveVelocity(real duration);
        void resolveInterpenetration(real duration);
    };

    class ParticleContactResolver
    {
        protected:
            unsigned iterations;
            unsigned itertaionsUsed;

        public:
            ParticleContactResolver(unsigned iterations) : iterations(iterations) {}

            void setIterations(unsigned iterations){
                this->iterations = iterations;
            }
            void resolveContact(ParticleContact * contactArray, unsigned numContact, real duration);
    };
}
