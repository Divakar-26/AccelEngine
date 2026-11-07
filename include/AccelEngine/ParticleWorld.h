#pragma once

#include <AccelEngine/particle.h>
#include <AccelEngine/ParticleContact.h>
#include <AccelEngine/core.h>
#include <AccelEngine/pfgen.h>

namespace AccelEngine
{
    class ParticleWorld
    {
        struct ParticleRegistration
        {
            Particle *particle;
            ParticleRegistration *next;
        };

        struct ContactGenRegistration
        {
            ParticleContactGenerator *gen;
            ContactGenRegistration *next;
        };

        //holds particle
        ParticleRegistration *firstParticle;
        //holds contact genration
        ContactGenRegistration *firstContactGen;

        //holds force registry
        ParticleForceRegistry registry;

        //holds resolver for contact
        ParticleContactResolver resolver;

        //holds the contacts between particle
        ParticleContact *contacts;

        unsigned maxContacts;

    public:
        ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

        void startFrame();
        void integrate(real duration);
        unsigned generateContacts();
        void runPhysics(real duration);

        // helpers
        void addParticle(Particle *p);
        void addContactGenerator(ParticleContactGenerator *gen);
        ParticleForceRegistry &getForceRegistry() { return registry; }
    };
}
