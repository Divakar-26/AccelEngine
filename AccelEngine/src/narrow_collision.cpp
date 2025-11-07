#include <AccelEngine/narrow_collision.h>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace AccelEngine;

bool NarrowCollision::IntersectRectangles(std::vector<Vector2> verticesA, std::vector<Vector2> verticesB, Contact &contacts)
{
    Vector2 normal(0, 0);
    real depth = std::numeric_limits<real>::max();

    for (int i = 0; i < verticesA.size(); i++)
    {
        Vector2 va = verticesA[i];
        Vector2 vb = verticesA[(i + 1) % verticesA.size()];

        Vector2 edge = vb - va;
        Vector2 axis(-edge.y, edge.x);
        axis.normalize();

        std::pair<real, real> minMaxA = projectOnAxis(verticesA, axis);
        std::pair<real, real> minMaxB = projectOnAxis(verticesB, axis);

        if (minMaxA.second < minMaxB.first || minMaxB.second < minMaxA.first)
        {
            return false;
        }

        real axisDepth = std::min(minMaxB.second - minMaxA.first, minMaxA.second - minMaxB.first);

        if (axisDepth < depth)
        {
            depth = axisDepth;
            normal = axis;
        }
    }

    for (int i = 0; i < verticesB.size(); i++)
    {
        Vector2 va = verticesB[i];
        Vector2 vb = verticesB[(i + 1) % verticesB.size()];

        Vector2 edge = vb - va;
        Vector2 axis(-edge.y, edge.x);
        axis.normalize();

        std::pair<real, real> minMaxA = projectOnAxis(verticesA, axis);
        std::pair<real, real> minMaxB = projectOnAxis(verticesB, axis);

        if (minMaxA.second < minMaxB.first || minMaxB.second < minMaxA.first)
        {
            return false;
        }

        real axisDepth = std::min(minMaxB.second - minMaxA.first, minMaxA.second - minMaxB.first);

        if (axisDepth < depth)
        {
            depth = axisDepth;
            normal = axis;
        }
    }

    Vector2 centerA = (verticesA[0] + verticesA[1] + verticesA[2] + verticesA[3]) * 0.25f;
    Vector2 centerB = (verticesB[0] + verticesB[1] + verticesB[2] + verticesB[3]) * 0.25f;
    Vector2 direction = centerB - centerA;

    if (direction.scalarProduct(normal) < 0.0f)
    {
        normal = normal * -1.0f;
    }

    contacts.normal = normal;
    contacts.penetration = depth;

    return true;
}

std::pair<real, real> NarrowCollision::projectOnAxis(std::vector<Vector2> vertices, Vector2 axis)
{
    real min = std::numeric_limits<real>::max();
    real max = std::numeric_limits<real>::lowest();

    for (int i = 0; i < vertices.size(); i++)
    {
        Vector2 v = vertices[i];
        real proj = v.scalarProduct(axis);

        if (proj < min)
        {
            min = proj;
        }
        if (proj > max)
        {
            max = proj;
        }
    }

    return {min, max};
}

bool NarrowCollision::SATCollision(const RigidBody *A, const RigidBody *B, Contact &contact)
{
    // Get transformed vertices for both rectangles
    Vector2 verticesA[4];
    Vector2 verticesB[4];

    RigidBody::getTransformedVertices(A, verticesA);
    RigidBody::getTransformedVertices(B, verticesB);

    // Convert to vectors for the existing function
    std::vector<Vector2> vecA(verticesA, verticesA + 4);
    std::vector<Vector2> vecB(verticesB, verticesB + 4);

    // Check for intersection using SAT
    if (!IntersectRectangles(vecA, vecB, contact))
    {
        return false;
    }

    contact.a = const_cast<RigidBody *>(A);
    contact.b = const_cast<RigidBody *>(B);
    contact.contactCount = 0;
    return true;
}

void NarrowCollision::FindContacts(World *world,
                                   const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
                                   std::vector<Contact> &contacts)
{
    contacts.clear();

    for (auto &pair : potentialPairs)
    {
        Contact contact;
        if (SATCollision(pair.first, pair.second, contact))
        {

            contacts.push_back(contact);
        }
    }
}