#include <AccelEngine/collision_narrow.h>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace AccelEngine;

static constexpr int verticesSize = 4;

namespace
{
    // Outward normal of edge i of a counter-clockwise quad (RigidBody::getTransformedVertices order).
    inline Vector2 edgeNormal(const Vector2 *v, int i)
    {
        Vector2 e = v[(i + 1) % verticesSize] - v[i];
        return Vector2(e.y, -e.x).normalized();
    }

    // Largest separation of `other` from any face of `poly` (> 0 means a separating axis exists).
    real maxFaceSeparation(const Vector2 *poly, const Vector2 *other, int &bestFace)
    {
        real best = std::numeric_limits<real>::lowest();
        bestFace = 0;
        for (int i = 0; i < verticesSize; i++)
        {
            Vector2 n = edgeNormal(poly, i);
            real minSep = std::numeric_limits<real>::max();
            for (int j = 0; j < verticesSize; j++)
                minSep = std::min(minSep, n.scalarProduct(other[j] - poly[i]));
            if (minSep > best)
            {
                best = minSep;
                bestFace = i;
            }
        }
        return best;
    }

    // Keep the part of segment in[0..1] where dot(n, p) <= offset.
    int clipSegment(const Vector2 in[2], Vector2 out[2], const Vector2 &n, real offset)
    {
        int count = 0;
        real d0 = n.scalarProduct(in[0]) - offset;
        real d1 = n.scalarProduct(in[1]) - offset;
        if (d0 <= 0.0f) out[count++] = in[0];
        if (d1 <= 0.0f) out[count++] = in[1];
        if (d0 * d1 < 0.0f && count < 2)
        {
            real t = d0 / (d0 - d1);
            out[count++] = in[0] + (in[1] - in[0]) * t;
        }
        return count;
    }
}

bool NarrowCollision::IntersectRectangles(const Vector2 *verticesA, Vector2 centera, const Vector2 *verticesB, Vector2 centerb, Contact &contacts, real speculativeDistance)
{
    // ---- SAT over the face normals of both boxes ----
    int faceA, faceB;
    real separationA = maxFaceSeparation(verticesA, verticesB, faceA);
    if (separationA > speculativeDistance)
        return false;
    real separationB = maxFaceSeparation(verticesB, verticesA, faceB);
    if (separationB > speculativeDistance)
        return false;

    // ---- pick the reference face; prefer A unless B is clearly better, so the choice
    //      does not flip-flop between substeps (a major source of resting jitter) ----
    const Vector2 *ref, *inc;
    int refFace;
    bool flip;
    const real tolerance = 0.1f * std::max(speculativeDistance, (real)0.05f);
    if (separationB > separationA + tolerance)
    {
        ref = verticesB; inc = verticesA; refFace = faceB; flip = true;
    }
    else
    {
        ref = verticesA; inc = verticesB; refFace = faceA; flip = false;
    }

    const Vector2 r1 = ref[refFace];
    const Vector2 r2 = ref[(refFace + 1) % verticesSize];
    const Vector2 tangent = (r2 - r1).normalized();
    const Vector2 refNormal(tangent.y, -tangent.x); // outward from the reference box

    // ---- incident face: the face of the other box most anti-parallel to refNormal ----
    int incFace = 0;
    real minDot = std::numeric_limits<real>::max();
    for (int i = 0; i < verticesSize; i++)
    {
        real d = edgeNormal(inc, i).scalarProduct(refNormal);
        if (d < minDot)
        {
            minDot = d;
            incFace = i;
        }
    }
    Vector2 incident[2] = {inc[incFace], inc[(incFace + 1) % verticesSize]};

    // ---- clip the incident face to the side planes of the reference face ----
    Vector2 clip1[2], clip2[2];
    if (clipSegment(incident, clip1, tangent * -1.0f, -tangent.scalarProduct(r1)) < 2 ||
        clipSegment(clip1, clip2, tangent, tangent.scalarProduct(r2)) < 2)
    {
        // degenerate (numerically): fall back to the old closest-feature points
        FindRectVsRectContact(verticesA, verticesB, contacts);
        real depth = -std::max(separationA, separationB);
        contacts.depths[0] = contacts.depths[1] = depth;
        contacts.penetration = depth;
        contacts.normal = flip ? edgeNormal(verticesB, faceB) * -1.0f : edgeNormal(verticesA, faceA);
        return contacts.contactCount > 0;
    }

    // ---- keep points behind (or within the speculative distance of) the reference face ----
    const real refOffset = refNormal.scalarProduct(r1);
    int count = 0;
    real deepest = std::numeric_limits<real>::lowest();
    for (int i = 0; i < 2; i++)
    {
        real separation = refNormal.scalarProduct(clip2[i]) - refOffset;
        if (separation <= speculativeDistance)
        {
            // midpoint between the incident point and the reference face
            contacts.contactPoints[count] = clip2[i] - refNormal * (separation * 0.5f);
            contacts.depths[count] = -separation;
            deepest = std::max(deepest, -separation);
            count++;
        }
    }

    if (count == 0)
        return false;

    contacts.contactCount = count;
    contacts.penetration = deepest;
    contacts.normal = flip ? refNormal * -1.0f : refNormal; // always from A to B
    (void)centera;
    (void)centerb;
    return true;
}

bool NarrowCollision::IntersectCircles(Vector2 center1, real radius1, Vector2 center2, real radius2, Contact &contact, real speculativeDistance)
{
    Vector2 diff = center2 - center1;
    real dist = (center1 - center2).magnitude();
    real radii = radius1 + radius2;

    if (dist > radii + speculativeDistance)
    {
        return false;
    }

    Vector2 normal = dist > 1e-6f ? diff / dist : Vector2(0, 1);
    real depth = radii - dist;

    Vector2 contactPoint = center1 + normal * radius1;

    contact.contactPoints[0] = contactPoint;
    contact.depths[0] = depth;
    contact.contactCount = 1;
    contact.normal = normal;
    contact.penetration = depth;

    return true;
}

bool NarrowCollision::IntersectCircleRectangle(Vector2 center1, real radius, Vector2 *vertices, Contact &contact, real speculativeDistance)
{
    // test with an inflated radius so near-touching pairs keep a (speculative) contact
    radius += speculativeDistance;

    Vector2 normal(0, 0);
    real depth = std::numeric_limits<real>::max();

    int verticesSize = 4;

    for (int i = 0; i < verticesSize; i++)
    {
        Vector2 va = vertices[i];
        Vector2 vb = vertices[(i + 1) % verticesSize];

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
    depth -= speculativeDistance; // back to the real radius (negative = small gap)

    Vector2 centerA = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;
    Vector2 direction = centerA - center1;

    if (direction.scalarProduct(normal) < 0.0f)
    {
        normal = normal * -1.0f;
    }

    FindCircleVsRectangleContact(center1, radius, centerA, vertices, contact);

    contact.normal = normal;
    contact.penetration = depth;
    contact.depths[0] = depth;

    return true;
}

int NarrowCollision::FindClosestPointOnRectangle(Vector2 center, const Vector2 *vertices)
{
    int result = -1;
    real minDisatance = std::numeric_limits<real>::max();

    int verticesSize = 4;

    for (int i = 0; i < verticesSize; i++)
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

std::pair<real, real> NarrowCollision::projectOnCircle(Vector2 center, real radius, const Vector2 *vertices, Vector2 axis)
{
    Vector2 direction = axis;
    Vector2 directionAndRadiud = direction * radius;

    Vector2 p1 = center + directionAndRadiud;
    Vector2 p2 = center - directionAndRadiud;

    real min = p1.scalarProduct(axis);
    real max = p2.scalarProduct(axis);

    if (min > max)
    {
        std::swap(min, max);
    }

    return {min, max};
}

std::pair<real, real> NarrowCollision::projectOnAxis(const Vector2 *vertices, Vector2 axis)
{
    real min = std::numeric_limits<real>::max();
    real max = std::numeric_limits<real>::lowest();

    for (int i = 0; i < verticesSize; i++)
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

void NarrowCollision::FindRectVsRectContact(const Vector2 *verticesA, const Vector2 *verticesB, Contact &contacts)
{
    int contactCount = 0;
    Vector2 contact1(0, 0);
    Vector2 contact2(0, 0);
    real minDistSq = std::numeric_limits<real>::max();

    for (int i = 0; i < verticesSize; i++)
    {
        Vector2 p = verticesA[i];
        for (int j = 0; j < verticesSize; j++)
        {
            Vector2 va = verticesB[j];
            Vector2 vb = verticesB[(j + 1) % verticesSize];

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

    for (int i = 0; i < verticesSize; i++)
    {
        Vector2 p = verticesB[i];
        for (int j = 0; j < verticesSize; j++)
        {
            Vector2 va = verticesA[j];
            Vector2 vb = verticesA[(j + 1) % verticesSize];

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

void NarrowCollision::FindCircleVsRectangleContact(Vector2 center, real radius, Vector2 rectCenter, const Vector2 *verticesA, Contact &contact)
{
    real minDistanceSqr = std::numeric_limits<real>::max();
    Vector2 actualContact;
    for (int i = 0; i < verticesSize; i++)
    {
        Vector2 va = verticesA[i];
        Vector2 vb = verticesA[(i + 1) % verticesSize];

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

bool NarrowCollision::SATCollision(const RigidBody *A, const RigidBody *B, Contact &contact, real speculativeDistance)
{
    if (A->usesParts() || B->usesParts())
    {
        std::vector<Contact> tmp;
        if (Collide(A, B, speculativeDistance, tmp) == 0)
            return false;
        contact = tmp[0];
        return true;
    }
    if (A->shapeType == ShapeType::AABB && B->shapeType == ShapeType::AABB)
    {
        Vector2 verticesA[4];
        Vector2 verticesB[4];

        RigidBody::getTransformedVertices(A, verticesA);
        RigidBody::getTransformedVertices(B, verticesB);

        if (!IntersectRectangles(verticesA, A->position, verticesB, B->position, contact, speculativeDistance))
        {
            return false;
        }
    }
    else if (A->shapeType == ShapeType::CIRCLE && B->shapeType == ShapeType::CIRCLE)
    {
        if (!IntersectCircles(A->position, A->circle.radius, B->position, B->circle.radius, contact, speculativeDistance))
        {
            return false;
        }
    }
    else if (A->shapeType == ShapeType::CIRCLE && B->shapeType == ShapeType::AABB)
    {
        Vector2 verts[4];
        RigidBody::getTransformedVertices(B, verts);

        if (!IntersectCircleRectangle(A->position, A->circle.radius, verts, contact, speculativeDistance))
            return false;
    }

    else if (A->shapeType == ShapeType::AABB && B->shapeType == ShapeType::CIRCLE)
    {
        Vector2 verts[4];
        RigidBody::getTransformedVertices(A, verts);

        if (!IntersectCircleRectangle(B->position, B->circle.radius, verts, contact, speculativeDistance))
            return false;

        contact.normal = contact.normal * -1;
    }

    contact.a = const_cast<RigidBody *>(A);
    contact.b = const_cast<RigidBody *>(B);
    // contact.contactCount = 0;

    return true;
}

void NarrowCollision::FindContacts(const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
                                   std::vector<Contact> &contacts)
{
    FindContacts(potentialPairs, contacts, 0.0f);
}

void NarrowCollision::FindContacts(
    const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
    std::vector<Contact> &contacts,
    real speculativeDistance)
{
    contacts.clear();

    for (auto &pair : potentialPairs)
    {
        RigidBody *A = pair.first;
        RigidBody *B = pair.second;

        Vector2 diff = B->position - A->position;
        float rsum = A->boundingRadius + B->boundingRadius + speculativeDistance;

        if (diff.scalarProduct(diff) > rsum * rsum)
            continue;

        if (!RigidBody::shouldCollide(*A, *B))
            continue;
        Collide(A, B, speculativeDistance, contacts);
    }
}


// ============================================================================
// Generic rounded convex polygons (polygon / capsule / compound parts)
// ============================================================================

namespace
{
    // Closest points between segments p1-q1 and p2-q2; returns the parameters along each.
    void closestSegmentPoints(const Vector2 &p1, const Vector2 &q1, const Vector2 &p2, const Vector2 &q2,
                              real &s, real &t)
    {
        const Vector2 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        const real a = d1.squareMagnitude(), e = d2.squareMagnitude(), f = d2.scalarProduct(r);
        const real eps = 1e-9f;
        if (a <= eps && e <= eps) { s = t = 0.0f; return; }
        if (a <= eps)
        {
            s = 0.0f;
            t = std::clamp(f / e, 0.0f, 1.0f);
            return;
        }
        const real c = d1.scalarProduct(r);
        if (e <= eps)
        {
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f);
            return;
        }
        const real b = d1.scalarProduct(d2);
        const real denom = a * e - b * b;
        s = denom > eps ? std::clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
        t = (b * s + f) / e;
        if (t < 0.0f)
        {
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f);
        }
        else if (t > 1.0f)
        {
            t = 1.0f;
            s = std::clamp((b - c) / a, 0.0f, 1.0f);
        }
    }

    // Max over edges of `poly` of the separation of `other`'s vertices (core shapes, radii ignored).
    real polyFaceSeparation(const WorldPoly &poly, const WorldPoly &other, int &bestEdge)
    {
        real best = std::numeric_limits<real>::lowest();
        bestEdge = 0;
        for (int i = 0; i < poly.count; i++)
        {
            real minSep = std::numeric_limits<real>::max();
            for (int j = 0; j < other.count; j++)
                minSep = std::min(minSep, poly.n[i].scalarProduct(other.v[j] - poly.v[i]));
            if (minSep > best)
            {
                best = minSep;
                bestEdge = i;
            }
        }
        return best;
    }

    // circle (point p, radius rc) against a polygon / capsule with >= 2 vertices. Normal points polygon -> circle.
    bool circleVsPoly(const Vector2 &p, real rc, const WorldPoly &P, real spec, Vector2 &normalOut, real &separation, Vector2 &point)
    {
        const real R = rc + P.radius;
        real best = std::numeric_limits<real>::lowest();
        int edge = 0;
        for (int i = 0; i < P.count; i++)
        {
            const real s = P.n[i].scalarProduct(p - P.v[i]);
            if (s > best)
            {
                best = s;
                edge = i;
            }
        }
        if (best > R + spec)
            return false;

        const Vector2 a = P.v[edge], b = P.v[(edge + 1) % P.count];
        const Vector2 ab = b - a;
        const real len2 = ab.squareMagnitude();
        const real f = len2 > 1e-12f ? std::clamp((p - a).scalarProduct(ab) / len2, 0.0f, 1.0f) : 0.0f;
        const Vector2 q = a + ab * f;

        if (best > 0.0f)
        {
            const Vector2 d = p - q;
            const real dist = d.magnitude();
            if (dist > R + spec)
                return false;
            normalOut = dist > 1e-6f ? d / dist : P.n[edge];
            separation = dist - R;
        }
        else
        {
            normalOut = P.n[edge];
            separation = best - R;
        }
        // midway between the two surfaces
        point = ((q + normalOut * P.radius) + (p - normalOut * rc)) * 0.5f;
        return true;
    }

    // clip segment in[0..1] to dot(n, p) <= offset
    int clipSeg(const Vector2 in[2], Vector2 out[2], const Vector2 &n, real offset)
    {
        int count = 0;
        const real d0 = n.scalarProduct(in[0]) - offset;
        const real d1 = n.scalarProduct(in[1]) - offset;
        if (d0 <= 0.0f) out[count++] = in[0];
        if (d1 <= 0.0f) out[count++] = in[1];
        if (d0 * d1 < 0.0f && count < 2)
        {
            const real t = d0 / (d0 - d1);
            out[count++] = in[0] + (in[1] - in[0]) * t;
        }
        return count;
    }
}

bool NarrowCollision::CollideConvex(const WorldPoly &A, const WorldPoly &B, real spec, Contact &contact)
{
    // ---- circle cases ----
    if (A.count == 1 && B.count == 1)
    {
        const Vector2 d = B.v[0] - A.v[0];
        const real dist = d.magnitude();
        const real R = A.radius + B.radius;
        if (dist > R + spec)
            return false;
        const Vector2 n = dist > 1e-6f ? d / dist : Vector2(0, 1);
        contact.normal = n;
        contact.contactCount = 1;
        contact.contactPoints[0] = ((A.v[0] + n * A.radius) + (B.v[0] - n * B.radius)) * 0.5f;
        contact.depths[0] = contact.penetration = R - dist;
        return true;
    }
    if (A.count == 1 || B.count == 1)
    {
        const bool aIsCircle = A.count == 1;
        const WorldPoly &C = aIsCircle ? A : B;
        const WorldPoly &P = aIsCircle ? B : A;
        Vector2 nOut, pt;
        real sep;
        if (!circleVsPoly(C.v[0], C.radius, P, spec, nOut, sep, pt))
            return false;
        contact.normal = aIsCircle ? nOut * -1.0f : nOut;
        contact.contactCount = 1;
        contact.contactPoints[0] = pt;
        contact.depths[0] = contact.penetration = -sep;
        return true;
    }

    // ---- capsule vs capsule: closest points of the core segments ----
    if (A.count == 2 && B.count == 2)
    {
        const real rT = A.radius + B.radius;
        real s, t;
        closestSegmentPoints(A.v[0], A.v[1], B.v[0], B.v[1], s, t);
        const Vector2 c1 = A.v[0] + (A.v[1] - A.v[0]) * s;
        const Vector2 c2 = B.v[0] + (B.v[1] - B.v[0]) * t;
        const Vector2 d = c2 - c1;
        const real dist = d.magnitude();
        if (dist > rT + spec)
            return false;

        const Vector2 dirA = (A.v[1] - A.v[0]).normalized();
        const Vector2 dirB = (B.v[1] - B.v[0]).normalized();
        Vector2 n;
        if (dist > 1e-4f)
            n = d / dist;
        else
        {
            // segments cross: push apart along A's normal, towards B's centre
            n = dirA.perpendicular();
            if (n.scalarProduct(B.v[0] + B.v[1] - A.v[0] - A.v[1]) < 0.0f)
                n = n * -1.0f;
        }
        contact.normal = n;

        // side by side (parallel, normal across the axes): two points along the overlap
        const bool parallel = std::fabs(dirA.cross(dirB)) < 0.12f;
        if (parallel && std::fabs(n.scalarProduct(dirA)) < 0.3f)
        {
            Vector2 seg[2] = {B.v[0], B.v[1]}, c1s[2], c2s[2];
            const real lo = dirA.scalarProduct(A.v[0]), hi = dirA.scalarProduct(A.v[1]);
            const real mn = std::min(lo, hi), mx = std::max(lo, hi);
            if (clipSeg(seg, c1s, dirA * -1.0f, -mn) == 2 && clipSeg(c1s, c2s, dirA, mx) == 2)
            {
                int count = 0;
                for (int i = 0; i < 2; i++)
                {
                    const real core = n.scalarProduct(c2s[i] - A.v[0]);
                    const real depth = rT - core;
                    if (depth > -spec)
                    {
                        contact.contactPoints[count] = c2s[i] + n * ((A.radius - B.radius - core) * 0.5f);
                        contact.depths[count] = depth;
                        count++;
                    }
                }
                if (count > 0)
                {
                    contact.contactCount = count;
                    contact.penetration = std::max(contact.depths[0], count > 1 ? contact.depths[1] : contact.depths[0]);
                    return true;
                }
            }
        }
        contact.contactCount = 1;
        contact.contactPoints[0] = ((c1 + n * A.radius) + (c2 - n * B.radius)) * 0.5f;
        contact.depths[0] = contact.penetration = rT - dist;
        return true;
    }

    // ---- polygon / capsule vs polygon / capsule ----
    const real rTotal = A.radius + B.radius;
    int edgeA, edgeB;
    const real sepA = polyFaceSeparation(A, B, edgeA);
    if (sepA > rTotal + spec)
        return false;
    const real sepB = polyFaceSeparation(B, A, edgeB);
    if (sepB > rTotal + spec)
        return false;

    // prefer A as reference unless B is clearly better (stops the choice flip-flopping between substeps)
    const real tolerance = 0.1f * std::max(spec, 0.05f);
    const bool flip = sepB > sepA + tolerance;
    const WorldPoly &ref = flip ? B : A;
    const WorldPoly &inc = flip ? A : B;
    const int refEdge = flip ? edgeB : edgeA;
    const real maxSep = std::max(sepA, sepB);

    const Vector2 r1 = ref.v[refEdge];
    const Vector2 r2 = ref.v[(refEdge + 1) % ref.count];
    const Vector2 refNormal = ref.n[refEdge];

    // incident edge: the one most anti-parallel to the reference normal
    int incEdge = 0;
    real minDot = std::numeric_limits<real>::max();
    for (int i = 0; i < inc.count; i++)
    {
        const real d = inc.n[i].scalarProduct(refNormal);
        if (d < minDot)
        {
            minDot = d;
            incEdge = i;
        }
    }
    const Vector2 i1 = inc.v[incEdge];
    const Vector2 i2 = inc.v[(incEdge + 1) % inc.count];

    // cores apart (only rounded corners can touch): vertex-vertex contacts need the true closest points
    if (maxSep > 0.0f)
    {
        real s, t;
        closestSegmentPoints(r1, r2, i1, i2, s, t);
        const bool refVertex = s <= 0.0f || s >= 1.0f;
        const bool incVertex = t <= 0.0f || t >= 1.0f;
        if (refVertex && incVertex)
        {
            const Vector2 pr = r1 + (r2 - r1) * s;
            const Vector2 pi = i1 + (i2 - i1) * t;
            const Vector2 d = pi - pr; // ref -> incident
            const real dist = d.magnitude();
            if (dist > rTotal + spec)
                return false;
            const Vector2 nRef = dist > 1e-6f ? d / dist : refNormal;
            contact.normal = flip ? nRef * -1.0f : nRef;
            contact.contactCount = 1;
            contact.contactPoints[0] = ((pr + nRef * ref.radius) + (pi - nRef * inc.radius)) * 0.5f;
            contact.depths[0] = contact.penetration = rTotal - dist;
            return true;
        }
    }

    // ---- face contact: clip the incident edge against the reference edge's side planes ----
    const Vector2 tangent = (r2 - r1).normalized();
    Vector2 inc2[2] = {i1, i2}, c1[2], c2[2];
    int n1 = clipSeg(inc2, c1, tangent * -1.0f, -tangent.scalarProduct(r1));
    int n2 = n1 == 2 ? clipSeg(c1, c2, tangent, tangent.scalarProduct(r2)) : 0;
    if (n2 < 2)
    {
        // numerically degenerate: use the incident vertex closest to the reference face
        const real s1 = refNormal.scalarProduct(i1 - r1), s2 = refNormal.scalarProduct(i2 - r1);
        c2[0] = s1 < s2 ? i1 : i2;
        n2 = 1;
    }

    const real refOffset = refNormal.scalarProduct(r1);
    int count = 0;
    real deepest = std::numeric_limits<real>::lowest();
    for (int i = 0; i < n2; i++)
    {
        const real core = refNormal.scalarProduct(c2[i]) - refOffset;
        const real separation = core - rTotal;
        if (separation <= spec)
        {
            contact.contactPoints[count] = c2[i] + refNormal * ((ref.radius - inc.radius - core) * 0.5f);
            contact.depths[count] = -separation;
            deepest = std::max(deepest, -separation);
            count++;
        }
    }
    if (count == 0)
        return false;
    contact.contactCount = count;
    contact.penetration = deepest;
    contact.normal = flip ? refNormal * -1.0f : refNormal;
    return true;
}

int NarrowCollision::Collide(const RigidBody *A, const RigidBody *B, real spec, std::vector<Contact> &out)
{
    // circles and boxes keep their dedicated (tuned) paths
    if (!A->usesParts() && !B->usesParts())
    {
        Contact c;
        if (SATCollision(A, B, c, spec))
        {
            out.push_back(c);
            return 1;
        }
        return 0;
    }

    const int na = A->worldPartCount(), nb = B->worldPartCount();
    const bool singleA = na == 1 && !A->usesParts(), singleB = nb == 1 && !B->usesParts();
    int added = 0;
    WorldPoly pa, pb;
    // a body that is a single primitive is converted once; compound parts are converted per pair
    if (singleA) A->getWorldPart(0, pa);
    if (singleB) B->getWorldPart(0, pb);
    for (int i = 0; i < na; i++)
    {
        if (!singleA) A->getWorldPart(i, pa);
        for (int j = 0; j < nb; j++)
        {
            if (!singleB) B->getWorldPart(j, pb);
            const Vector2 d = pb.center - pa.center;
            const real rs = pa.boundRadius + pb.boundRadius + spec;
            if (d.squareMagnitude() > rs * rs)
                continue;
            Contact c;
            if (CollideConvex(pa, pb, spec, c))
            {
                c.a = const_cast<RigidBody *>(A);
                c.b = const_cast<RigidBody *>(B);
                out.push_back(c);
                added++;
            }
        }
    }
    return added;
}
