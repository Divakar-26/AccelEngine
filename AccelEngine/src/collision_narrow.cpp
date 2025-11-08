#include <AccelEngine/collision_narrow.h>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace AccelEngine;

bool NarrowCollision::IntersectRectangles(std::vector<Vector2> verticesA, Vector2 centera, std::vector<Vector2> verticesB, Vector2 centerb, Contact &contacts)
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

    // Vector2 centerA = (verticesA[0] + verticesA[1] + verticesA[2] + verticesA[3]) * 0.25f;
    // Vector2 centerB = (verticesB[0] + verticesB[1] + verticesB[2] + verticesB[3]) * 0.25f;
    Vector2 direction = centerb - centera;

    if (direction.scalarProduct(normal) < 0.0f)
    {
        normal = normal * -1.0f;
    }

    depth /= normal.magnitude();

    FindRectVsRectContact(verticesA, verticesB, contacts);

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

    Vector2 contactPoint = center1 + normal * radius1;

    contact.contactPoints[0] = contactPoint;
    contact.contactCount = 1;
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

    FindCircleVsRectangleContact(center1, radius, centerA, vertices, contact);

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

std::pair<real, real> NarrowCollision::projectOnAxis(const std::vector<Vector2> vertices, Vector2 axis)
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

void NarrowCollision::FindPointSegmentDistance(Vector2 center, Vector2 edge1, Vector2 edge2, real &distanceSquared, Vector2 &conatct)
{
    Vector2 ab = edge2 - edge1;
    Vector2 ap = center - edge1;

    real proj = ab.scalarProduct(ap);
    real abLenSqr = ab.squareMagnitude();
    float d = proj / abLenSqr;

    if (d <= 0.0f)
    {
        conatct = edge1;
    }
    else if (d >= 1.0f)
    {
        conatct = edge2;
    }
    else
    {
        conatct = edge1 + ab * d;
    }

    real r = Vector2::distance(center, conatct);
    distanceSquared = r * r;
}

void NarrowCollision::FindRectVsRectContact(std::vector<Vector2> verticesA, std::vector<Vector2> verticesB, Contact &contacts)
{
    int contactCount = 0;
    Vector2 contact1(0, 0);
    Vector2 contact2(0, 0);
    real minDistSq = std::numeric_limits<real>::max();

    for (int i = 0; i < verticesA.size(); i++)
    {
        Vector2 p = verticesA[i];
        for (int j = 0; j < verticesB.size(); j++)
        {
            Vector2 va = verticesB[j];
            Vector2 vb = verticesB[(j + 1) % verticesB.size()];

            real distSqrd;
            Vector2 cp;

            FindPointSegmentDistance(p, va, vb, distSqrd, cp);

            if (Vector2::nearlyEqual(distSqrd, minDistSq))
            {
                if (!Vector2::nearlyEqual(cp, contact1))
                {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSqrd < minDistSq)
            {
                minDistSq = distSqrd;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }

    for (int i = 0; i < verticesB.size(); i++)
    {
        Vector2 p = verticesB[i];
        for (int j = 0; j < verticesA.size(); j++)
        {
            Vector2 va = verticesA[j];
            Vector2 vb = verticesA[(j + 1) % verticesA.size()];

            real distSqrd;
            Vector2 cp;

            FindPointSegmentDistance(p, va, vb, distSqrd, cp);

            if (Vector2::nearlyEqual(distSqrd, minDistSq))
            {
                if (!Vector2::nearlyEqual(cp, contact1))
                {
                    contact2 = cp;
                    contactCount = 2;
                }
            }
            else if (distSqrd < minDistSq)
            {
                minDistSq = distSqrd;
                contactCount = 1;
                contact1 = cp;
            }
        }
    }

    contacts.contactPoints[0] = contact1;
    contacts.contactPoints[1] = contact2;
    contacts.contactCount = contactCount;
}

void NarrowCollision::FindCircleVsRectangleContact(Vector2 center, real radius, Vector2 rectCenter, std::vector<Vector2> verticesA, Contact &contact)
{
    real minDistanceSqr = std::numeric_limits<real>::max();
    Vector2 actualContact;
    for (int i = 0; i < verticesA.size(); i++)
    {
        Vector2 va = verticesA[i];
        Vector2 vb = verticesA[(i + 1) % verticesA.size()];

        real distanceSquared;
        Vector2 contact2(0, 0);

        FindPointSegmentDistance(center, va, vb, distanceSquared, contact2);

        if (distanceSquared < minDistanceSqr)
        {
            minDistanceSqr = distanceSquared;
            actualContact = contact2;
        }
    }

    contact.contactPoints[0] = actualContact;
    contact.contactCount = 1;
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

        if (!IntersectRectangles(vecA, A->position, vecB, B->position, contact))
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
    // contact.contactCount = 0;

    return true;
}

void NarrowCollision::FindContacts(
    const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
    std::vector<Contact> &contacts)
{
    contacts.clear();

    for (auto &pair : potentialPairs)
    {
        Contact c;
        if (SATCollision(pair.first, pair.second, c))
            contacts.push_back(c);
    }
}
