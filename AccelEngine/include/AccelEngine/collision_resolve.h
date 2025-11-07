#pragma once
#include <AccelEngine/body.h>
#include <AccelEngine/narrow_collision.h>

namespace AccelEngine
{
    class CollisionResolve
    {
    public:
        // Position correction only (separates objects)
        static void SolvePosition(Contact& contact, float correctionFactor = 0.8f, float slop = 0.01f);
        
        // Velocity response (bounce, friction)
        static void SolveVelocity(Contact& contact, float restitution = 0.2f, float friction = 0.4f);
        
        // Complete resolution (position + velocity)
        static void Solve(Contact& contact, float dt);
    };
}