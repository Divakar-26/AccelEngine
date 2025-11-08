#include <AccelEngine/collision_narrow.h>
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

    depth /= normal.magnitude();

    contacts.normal = normal;
    contacts.penetration = depth;

    return true;
}

bool NarrowCollision::IntersectCircles(Vector2 center1, real radius1, Vector2 center2, real radius2, Contact &contact)
{
    Vector2 diff = center2 - center1;
    real dist = (center1 - center2).magnitude();
    real radii = radius1 + radius2;

    if (dist > radii)
    {
        return false;
    }

    Vector2 normal = (center2 - center1).normalized();
    real depth = radii - dist;

    contact.normal = normal;
    contact.penetration = depth;

    return true;
}

bool NarrowCollision::IntersectCircleRectangle(Vector2 center1, real radius, std::vector<Vector2> vertices, Contact &contact)
{
    Vector2 normal(0, 0);
    real depth = std::numeric_limits<real>::max();

    for (int i = 0; i < vertices.size(); i++)
    {
        Vector2 va = vertices[i];
        Vector2 vb = vertices[(i + 1) % vertices.size()];

        Vector2 edge = vb - va;
        Vector2 axis(-edge.y, edge.x);
        axis.normalize();

        std::pair<real, real> minMaxA = projectOnAxis(vertices, axis);
        std::pair<real, real> minMaxB = projectOnCircle(center1, radius, vertices, axis);

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

    int cpIndex = FindClosestPointOnRectangle(center1, vertices);
    Vector2 cp = vertices[cpIndex];
    Vector2 axis = center1 - cp;
    axis.normalize();

    std::pair<real, real> minMaxA = projectOnAxis(vertices, axis);
    std::pair<real, real> minMaxB = projectOnCircle(center1, radius, vertices, axis);

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

    depth /= normal.magnitude();
    normal.normalize();

    Vector2 centerA = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;
    Vector2 direction = centerA - center1;

    if (direction.scalarProduct(normal) < 0.0f)
    {
        normal = normal * -1.0f;
    }

    contact.normal = normal;
    contact.penetration = depth;

    return true;
}

int NarrowCollision::FindClosestPointOnRectangle(Vector2 center, std::vector<Vector2> vertices)
{
    int result = -1;
    real minDisatance = std::numeric_limits<real>::max();

    for (int i = 0; i < vertices.size(); i++)
    {
        Vector2 v = vertices[i];
        real distance = Vector2::distance(v, center);

        if (distance < minDisatance)
        {
            minDisatance = distance;
            result = i;
        }
    }

    return result;
}

std::pair<real, real> NarrowCollision::projectOnCircle(Vector2 center, real radius, std::vector<Vector2> vertices, Vector2 axis)
{    
    Vector2 direction = axis.normalized();
    Vector2 directionAndRadiud = direction * radius;

    Vector2 p1 = center + directionAndRadiud;
    Vector2 p2 = center - directionAndRadiud;

    real min = p1.scalarProduct(axis.normalized());
    real max = p2.scalarProduct(axis.normalized());

    if (min > max)
    {
        // swap the min and max
        std::swap(min, max);
    }

    return {min, max};
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
    if (A->shapeType == ShapeType::AABB && B->shapeType == ShapeType::AABB)
    {
        Vector2 verticesA[4];
        Vector2 verticesB[4];

        RigidBody::getTransformedVertices(A, verticesA);
        RigidBody::getTransformedVertices(B, verticesB);

        std::vector<Vector2> vecA(verticesA, verticesA + 4);
        std::vector<Vector2> vecB(verticesB, verticesB + 4);

        if (!IntersectRectangles(vecA, vecB, contact))
        {
            return false;
        }
    }
    else if (A->shapeType == ShapeType::CIRCLE && B->shapeType == ShapeType::CIRCLE)
    {
        if (!IntersectCircles(A->position, A->circle.radius, B->position, B->circle.radius, contact))
        {
            return false;
        }
    }
    else if (A->shapeType == ShapeType::CIRCLE && B->shapeType == ShapeType::AABB)
    {
        Vector2 verts[4];
        RigidBody::getTransformedVertices(B, verts);

        std::vector<Vector2> rect(verts, verts + 4);

        if (!IntersectCircleRectangle(A->position, A->circle.radius, rect, contact))
            return false;
    }

    else if (A->shapeType == ShapeType::AABB && B->shapeType == ShapeType::CIRCLE)
    {
        Vector2 verts[4];
        RigidBody::getTransformedVertices(A, verts);

        std::vector<Vector2> rect(verts, verts + 4);

        if (!IntersectCircleRectangle(B->position, B->circle.radius, rect, contact))
            return false;

        contact.normal = contact.normal * -1;
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