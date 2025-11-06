#include <AccelEngine/narrow_collision.h>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace AccelEngine;

static void getTransformedCorners(const RigidBody *body, Vector2 corners[4])
{
    const Vector2 &half = body->aabb.halfSize;
    Vector2 localCorners[4] = {
        Vector2(-half.x, -half.y),
        Vector2(half.x, -half.y),
        Vector2(half.x, half.y),
        Vector2(-half.x, half.y)};

    for (int i = 0; i < 4; ++i)
    {
        corners[i] = body->transformMatrix * localCorners[i] + body->position;
    }
}

static void projectOntoAxis(const Vector2 corners[4], const Vector2 &axis, float &min, float &max)
{
    min = max = corners[0].scalarProduct(axis);
    for (int i = 1; i < 4; ++i)
    {
        float proj = corners[i].scalarProduct(axis);
        if (proj < min)
            min = proj;
        if (proj > max)
            max = proj;
    }
}

// Find the contact points between two OBBs
static void findContactPoints(const Vector2 cornersA[4], const Vector2 cornersB[4], 
                              const Vector2 &normal, Contact &contact)
{
    // Find the incident face (face from B that's most anti-parallel to normal)
    int incidentFace = 0;
    float minDot = std::numeric_limits<float>::max();
    
    Vector2 faceNormalsB[4];
    for (int i = 0; i < 4; ++i)
    {
        Vector2 edge = cornersB[(i + 1) % 4] - cornersB[i];
        faceNormalsB[i] = edge.perpendicular().normalized();
        
        float dot = faceNormalsB[i].scalarProduct(normal);
        if (dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }
    
    // Get the incident face vertices
    Vector2 incidentFaceVertices[2] = {
        cornersB[incidentFace],
        cornersB[(incidentFace + 1) % 4]
    };
    
    // Clip the incident face against the reference face (from A)
    Vector2 referenceFaceNormal = normal;
    
    // Find the reference face (face from A that's most parallel to normal)
    int referenceFace = 0;
    float maxDot = -std::numeric_limits<float>::max();
    
    Vector2 faceNormalsA[4];
    for (int i = 0; i < 4; ++i)
    {
        Vector2 edge = cornersA[(i + 1) % 4] - cornersA[i];
        faceNormalsA[i] = edge.perpendicular().normalized();
        
        float dot = faceNormalsA[i].scalarProduct(normal);
        if (dot > maxDot)
        {
            maxDot = dot;
            referenceFace = i;
        }
    }
    
    // Get the reference face vertices and normal
    Vector2 refFaceVertices[2] = {
        cornersA[referenceFace],
        cornersA[(referenceFace + 1) % 4]
    };
    Vector2 refFaceNormal = faceNormalsA[referenceFace];
    
    // Simple contact point calculation: use the deepest points
    // This is a simplified approach - for a more robust solution, implement Sutherland-Hodgman clipping
    
    contact.contactCount = 0;
    
    // Check which vertices from B are inside A
    for (int i = 0; i < 4 && contact.contactCount < 2; ++i)
    {
        Vector2 vertex = cornersB[i];
        Vector2 toVertex = vertex - cornersA[0];
        
        // Simple inside test (not perfect but works for convex shapes)
        bool inside = true;
        for (int j = 0; j < 4; ++j)
        {
            Vector2 edge = cornersA[(j + 1) % 4] - cornersA[j];
            Vector2 faceNormal = edge.perpendicular().normalized();
            
            if (faceNormal.scalarProduct(vertex - cornersA[j]) > 0)
            {
                inside = false;
                break;
            }
        }
        
        if (inside)
        {
            contact.contactPoints[contact.contactCount++] = vertex;
        }
    }
    
    // Check which vertices from A are inside B
    for (int i = 0; i < 4 && contact.contactCount < 2; ++i)
    {
        Vector2 vertex = cornersA[i];
        Vector2 toVertex = vertex - cornersB[0];
        
        bool inside = true;
        for (int j = 0; j < 4; ++j)
        {
            Vector2 edge = cornersB[(j + 1) % 4] - cornersB[j];
            Vector2 faceNormal = edge.perpendicular().normalized();
            
            if (faceNormal.scalarProduct(vertex - cornersB[j]) > 0)
            {
                inside = false;
                break;
            }
        }
        
        if (inside)
        {
            contact.contactPoints[contact.contactCount++] = vertex;
        }
    }
    
    // If no vertices found, use the center points as fallback
    if (contact.contactCount == 0)
    {
        // Simple midpoint on the penetration vector
        Vector2 centerA = (cornersA[0] + cornersA[1] + cornersA[2] + cornersA[3]) / 4.0f;
        Vector2 centerB = (cornersB[0] + cornersB[1] + cornersB[2] + cornersB[3]) / 4.0f;
        Vector2 direction = centerB - centerA;
        
        contact.contactPoints[0] = centerA + direction * 0.5f;
        contact.contactCount = 1;
    }
}

bool NarrowCollision::SATCollision(const RigidBody *A, const RigidBody *B, Contact &contact)
{
    Vector2 cornersA[4], cornersB[4];
    getTransformedCorners(A, cornersA);
    getTransformedCorners(B, cornersB);

    // Test axes from both shapes
    Vector2 axes[4] = {
        (cornersA[1] - cornersA[0]).normalized(),
        (cornersA[3] - cornersA[0]).normalized(),
        (cornersB[1] - cornersB[0]).normalized(),
        (cornersB[3] - cornersB[0]).normalized()};

    float penetration = std::numeric_limits<float>::max();
    Vector2 normal;

    for (int i = 0; i < 4; ++i)
    {
        Vector2 axis = axes[i].perpendicular().normalized();

        float minA, maxA, minB, maxB;
        projectOntoAxis(cornersA, axis, minA, maxA);
        projectOntoAxis(cornersB, axis, minB, maxB);

        if (maxA < minB || maxB < minA)
            return false; // Separating axis found

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap < penetration)
        {
            penetration = overlap;
            normal = axis;
        }
    }

    // Ensure normal points from A → B
    Vector2 direction = B->position - A->position;
    if (direction.scalarProduct(normal) < 0)
        normal.invert();

    contact.a = const_cast<RigidBody*>(A);
    contact.b = const_cast<RigidBody*>(B);
    contact.normal = normal;
    contact.penetration = penetration;
    
    // Find the actual contact points
    findContactPoints(cornersA, cornersB, normal, contact);

    return true;
}

void NarrowCollision::FindContacts(World *world,
                                   const std::vector<std::pair<RigidBody *, RigidBody *>> &potentialPairs,
                                   std::vector<Contact> &contacts)
{
    contacts.clear();

    for (auto &pair : potentialPairs)
    {
        RigidBody *A = pair.first;
        RigidBody *B = pair.second;

        Contact contact;
        if (SATCollision(A, B, contact))
        {
            contacts.push_back(contact);

            // Optional: color for debug
            A->c = {0, 255, 0, 255};
            B->c = {0, 255, 0, 255};
        }
    }
}