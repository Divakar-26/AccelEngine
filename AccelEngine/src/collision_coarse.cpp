#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/core.h>
#include <cmath>

using namespace AccelEngine;

static inline bool AABBOverlap(const Vector2 &minA, const Vector2 &maxA,
                               const Vector2 &minB, const Vector2 &maxB)
{
    if (maxA.x < minB.x || minA.x > maxB.x)
        return false;
    if (maxA.y < minB.y || minA.y > maxB.y)
        return false;
    return true;
}

inline void computeAABB(const RigidBody *body, Vector2 &outMin, Vector2 &outMax)
{
    if (body->shapeType == ShapeType::CIRCLE)
    {
        float r = body->circle.radius;

        outMin = Vector2(body->position.x - r, body->position.y - r);
        outMax = Vector2(body->position.x + r, body->position.y + r);

        return;
    }

    // shapeType == AABB (or OBB using transformMatrix)
    const Vector2 &half = body->aabb.halfSize;

    Vector2 corners[4] = {
        Vector2(-half.x, -half.y),
        Vector2(half.x, -half.y),
        Vector2(half.x, half.y),
        Vector2(-half.x, half.y)};

    outMin = Vector2(1e9f, 1e9f);
    outMax = Vector2(-1e9f, -1e9f);

    for (int i = 0; i < 4; ++i)
    {
        Vector2 worldCorner = body->transformMatrix * corners[i];
        worldCorner += body->position;

        if (worldCorner.x < outMin.x)
            outMin.x = worldCorner.x;
        if (worldCorner.y < outMin.y)
            outMin.y = worldCorner.y;
        if (worldCorner.x > outMax.x)
            outMax.x = worldCorner.x;
        if (worldCorner.y > outMax.y)
            outMax.y = worldCorner.y;
    }
}


void CoarseCollision::FindPotentialPairs(std::vector<RigidBody*> bodies, std::vector<std::pair<RigidBody *, RigidBody *>> &pairs)
{
    // clear pairs from previous frame
    pairs.clear();
    Vector2 minA, maxA, minB, maxB;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        computeAABB(bodies[i], minA, maxA);
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            computeAABB(bodies[j], minB, maxB);
            if (AABBOverlap(minA, maxA, minB, maxB))
            {
                pairs.emplace_back(bodies[i], bodies[j]);
            }
        }
    }
}
