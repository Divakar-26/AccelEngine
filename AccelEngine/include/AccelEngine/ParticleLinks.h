#pragma once

#include <AccelEngine/particle.h>
#include <AccelEngine/ParticleContact.h>

namespace AccelEngine
{
    class ParticleLink
    {
        public:
            Particle * particle[2];
        
        protected:
            real currentLength() const;
    };
}