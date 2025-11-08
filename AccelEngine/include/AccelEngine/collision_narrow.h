#pragma once
#include <AccelEngine/body.h>
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


        static bool IntersectRectangles(std::vector<Vector2> verticesA, Vector2 centera,  std::vector<Vector2> verticesB, Vector2 centerB, Contact &contacts);
        static bool IntersectCircles(Vector2 center1, real radius1, Vector2 center2, real radius2, Contact &contact);
        static bool IntersectCircleRectangle(Vector2 center1, real radius, std::vector<Vector2> vertices, Contact &contact);

        static std::pair<real, real> projectOnAxis(std::vector<Vector2> vertices, Vector2 axis);
        static std::pair<real, real> projectOnCircle(Vector2 center, real radius, std::vector<Vector2> vertices, Vector2 axis);
        static int FindClosestPointOnRectangle(Vector2 center, std::vector<Vector2> vertices);
        static void FindPointSegmentDistance(Vector2 center, Vector2 edge1, Vector2 edge2, real & distanceSquared, Vector2 & conatct);

        static void FindContacts(
            const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
            std::vector<Contact> &contacts);


        //contacts
        static void FindCircleVsRectangleContact(Vector2 center, real radius, Vector2 rectCenter, std::vector<Vector2> verticesA, Contact & contact);

    };
};
