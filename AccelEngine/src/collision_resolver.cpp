#include <AccelEngine/collision_resolver.h>
#include <algorithm>

using namespace AccelEngine;

void ContactResolver::ResolveContacts(std::vector<Contact> &contacts, float dt)
{
    const float restitution = 0.3f;
    const float percent = 0.8f;
    const float slop = 0.01f;

    for (auto &c : contacts)
    {
        RigidBody *A = c.a;
        RigidBody *B = c.b;

        if (!A || !B)
            continue;

        // Process each contact point individually
        for (int pointIdx = 0; pointIdx < c.contactCount; ++pointIdx)
        {
            Vector2 contactPoint = c.contactPoints[pointIdx];
            
            // --- 1. Positional correction ---
            Vector2 correction = c.normal * (std::max(c.penetration - slop, 0.0f) / (A->inverseMass + B->inverseMass)) * percent;
            A->position -= correction * A->inverseMass;
            B->position += correction * B->inverseMass;

            // --- 2. Relative velocity at contact point ---
            // Calculate vectors from center of mass to contact point
            Vector2 ra = contactPoint - A->position;
            Vector2 rb = contactPoint - B->position;

            // Calculate velocity at contact point (including rotational component)
            Vector2 va = A->velocity + Vector2(-A->rotation * ra.y, A->rotation * ra.x);
            Vector2 vb = B->velocity + Vector2(-B->rotation * rb.y, B->rotation * rb.x);
            Vector2 relativeVel = vb - va;

            float velAlongNormal = relativeVel * c.normal;
            if (velAlongNormal > 0)
                continue;

            // --- 3. Compute denominator including angular inertia ---
            float raCrossN = ra.cross(c.normal);
            float rbCrossN = rb.cross(c.normal);
            float invMassSum = A->inverseMass + B->inverseMass +
                              (raCrossN * raCrossN) * A->inverseInertia +
                              (rbCrossN * rbCrossN) * B->inverseInertia;

            if (invMassSum == 0.0f)
                continue;

            // --- 4. Impulse magnitude ---
            float j = -(1.0f + restitution) * velAlongNormal;
            j /= invMassSum;

            Vector2 impulse = c.normal * j;

            // --- 5. Apply linear impulse ---
            A->velocity -= impulse * A->inverseMass;
            B->velocity += impulse * B->inverseMass;

            // --- 6. Apply angular impulse ---
            A->rotation -= ra.cross(impulse) * A->inverseInertia;
            B->rotation += rb.cross(impulse) * B->inverseInertia;
        }
    }
}