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
        static bool IntersectRectangles(std::vector<Vector2> verticesA, std::vector<Vector2> verticesB, Contact &contacts);
        static bool IntersectCircles(Vector2 center1, real radius1,  Vector2 center2, real radius2, Contact &contact);


        static std::pair<real, real> projectOnAxis(std::vector<Vector2> vertices, Vector2 axis);
        static void FindContacts(World *world,
                                 const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
                                 std::vector<Contact> &contacts);
    };
};
