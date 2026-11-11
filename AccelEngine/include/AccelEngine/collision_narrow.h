#pragma once
#include <AccelEngine/body.h>
#include <vector>
#include <utility>

namespace AccelEngine
{
    struct Contact
    {
        RigidBody *a = nullptr;
        RigidBody *b = nullptr;
        Vector2 normal;          // unit length, points from a to b
        float penetration = 0;   // deepest penetration of the manifold

        Vector2 contactPoints[2];
        float depths[2] = {0, 0}; // per-point penetration (negative = slightly separated, speculative)
        int contactCount = 0;
        int indexA = -1, indexB = -1; // a/b->worldIndex, filled by World::step (saves pointer chasing)
    };
    

    class NarrowCollision
    {
    public:
        // Single-manifold collision (circles and boxes use the dedicated fast paths; any other shape returns its
        // first manifold).
        static bool SATCollision(const RigidBody *A, const RigidBody *B, Contact &contact, real speculativeDistance = 0.0f);

        // Every manifold between two bodies, appended to `out` (a compound body can produce several). Returns the
        // number appended. This is what World uses.
        static int Collide(const RigidBody *A, const RigidBody *B, real speculativeDistance, std::vector<Contact> &out);

        // Rounded convex polygon vs rounded convex polygon (circle = 1 vertex, capsule = 2). Normal points A -> B.
        static bool CollideConvex(const WorldPoly &A, const WorldPoly &B, real speculativeDistance, Contact &contact);

        // Box vs box: SAT + reference-face clipping. Keeps points up to speculativeDistance apart.
        static bool IntersectRectangles(const Vector2 * verticesA, Vector2 centera, const Vector2 * verticesB, Vector2 centerB, Contact &contacts, real speculativeDistance = 0.0f);
        static bool IntersectCircles(Vector2 center1, real radius1, Vector2 center2, real radius2, Contact &contact, real speculativeDistance = 0.0f);
        static bool IntersectCircleRectangle(Vector2 center1, real radius, Vector2 * vertices, Contact &contact, real speculativeDistance = 0.0f);

        static std::pair<real, real> projectOnAxis(const Vector2 * vertices, Vector2 axis);
        static std::pair<real, real> projectOnCircle(Vector2 center, real radius, const Vector2 * vertices, Vector2 axis);
        static int FindClosestPointOnRectangle(Vector2 center, const Vector2 * vertices);
        static void FindPointSegmentDistance(Vector2 center, Vector2 edge1, Vector2 edge2, real &distanceSquared, Vector2 &conatct);

        static void FindContacts(
            const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
            std::vector<Contact> &contacts,
            real speculativeDistance);
        // (kept as a separate overload: SIMPEngine's PhysicsSystem was compiled against this two-argument symbol)
        static void FindContacts(
            const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
            std::vector<Contact> &contacts);

        // contacts
        static void FindCircleVsRectangleContact(Vector2 center, real radius, Vector2 rectCenter, const Vector2 * verticesA, Contact &contact);
        static void FindRectVsRectContact(const Vector2 * verticesA, const Vector2 * verticesB, Contact &contacts);
    };
};
