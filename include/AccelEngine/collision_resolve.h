#pragma once
#include <AccelEngine/body.h>
#include <AccelEngine/collision_narrow.h>

namespace AccelEngine
{
    class CollisionResolve
    {
    public:
        static void SolvePosition(Contact& contact, float correctionFactor = 0.8f, float slop = 0.01f);
        static void SolvePositionWithRotation(Contact &contact, float baumgarte, float slop);

        static void SolveVelocity(Contact& contact, float friction = 0.4f);
        static void SolveVelocityWithRoatation(Contact & contact);
        static void SolveVelocityWithRoatationAndFriction(Contact & contact);
        // Complete resolution (position + velocity)
        static void Solve(Contact& contact, float dt);
    };
}