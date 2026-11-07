#pragma once

#include "core.h"

namespace AccelEngine
{
    class Particle
    {
    public:
        Vector2 position;
        Vector2 velocity;
        Vector2 acceleration;

        Vector2 forceAccum;

        real damping = 0.99f;
        real inverseMass = 1.0f;

        void integrate(real duration);
        void clearAccumulator();
        void addForce(const Vector2 & force);

        //helper functions

        bool hasFiniteMass(){
            return inverseMass > 0.0f;  
        }

        Vector2 getVelocity() {return velocity;}
        Vector2 getPosition() {return position;}
        real getMass() {return 1.0f / inverseMass;}
      
        void setVelocity(Vector2 vel){
          this->velocity = vel;
        }
        void setPosition(Vector2 pos){
          this->position = pos;
        }
    };
} // namespace AccelEngine
