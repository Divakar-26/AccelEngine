#pragma once

#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <vector>
#include <utility>

namespace AccelEngine
{
    class CoarseCollision
    {
        public:
        static void FindPotentialPairs(World *world, std::vector<std::pair<RigidBody *, RigidBody *>> &pairs);
    };
} // namespace AccelEngine
