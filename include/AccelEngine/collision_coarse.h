#pragma once

#include <AccelEngine/body.h>
#include <vector>
#include <utility>

namespace AccelEngine
{

    class CoarseCollision
    {
    public:
        static void FindPotentialPairs(const std::vector<RigidBody *> &bodies, std::vector<std::pair<RigidBody *, RigidBody *>> &pairs);
    };
} // namespace AccelEngine
