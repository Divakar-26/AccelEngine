#pragma once
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <vector>
#include <utility>

namespace AccelEngine
{
    struct Contact
    {
        RigidBody *a;
        RigidBody *b;
        Vector2 normal;
        float penetration;

        Vector2 contactPoints[2];
        int contactCount;
    };

    class NarrowCollision
    {
    public:
        static bool SATCollision(const RigidBody *A, const RigidBody *B, Contact &contact);
        static void FindContacts(World *world,
                                 const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
                                 std::vector<Contact> &contacts);
    };
};
