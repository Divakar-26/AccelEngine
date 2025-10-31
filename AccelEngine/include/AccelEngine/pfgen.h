#pragma once

#include "particle.h"
#include "core.h"

namespace AccelEngine{
    class ParticleForceGenerator{

        public:
            virtual void updateForce(Particle * particle, real duration) = 0;
    };
}