#include <AccelEngine/collision_resolve.h>
#include <cmath>
#include <iostream>
using namespace AccelEngine;

void CollisionResolve::SolvePosition(Contact &contact, float correctionFactor, float slop)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    if (contact.penetration <= slop)
        return;

    Vector2 correction = contact.normal * (contact.penetration - slop) * correctionFactor;

    // Apply correction based on inverse mass (heavier objects move less)
    float totalInverseMass = A->getInverseMass() + B->getInverseMass();

    if (totalInverseMass > 0.0f)
    {
        // Move each body proportionally to their mass
        if (A->getInverseMass() > 0.0f)
        {
            A->position -= correction * (A->getInverseMass() / totalInverseMass);
            A->calculateDerivativeData();
        }

        if (B->getInverseMass() > 0.0f)
        {
            B->position += correction * (B->getInverseMass() / totalInverseMass);
            B->calculateDerivativeData();
        }
    }
}

void CollisionResolve::SolvePositionWithRotation(Contact &contact, float baumgarte, float slop)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    if (contact.penetration <= slop)
        return;

    int count = contact.contactCount;
    if (count <= 0)
        return;

    float totalMass = A->inverseMass + B->inverseMass;

    for (int i = 0; i < count; i++)
    {
        Vector2 point = contact.contactPoints[i];

        Vector2 ra = point - A->position;
        Vector2 rb = point - B->position;

        Vector2 n = contact.normal;

        float baum = baumgarte * (contact.penetration - slop);

        float raCrossN = ra.cross(n);
        float rbCrossN = rb.cross(n);

        float denom =
            A->inverseMass +
            B->inverseMass +
            (raCrossN * raCrossN) * A->inverseInertia +
            (rbCrossN * rbCrossN) * B->inverseInertia;

        if (denom <= 0.0f)
            continue;

        float impulseMag = baum / denom;
        impulseMag /= (float)count;

        Vector2 impulse = n * impulseMag;

        if (A->inverseMass > 0.0f)
        {
            A->position -= impulse * A->inverseMass;
            A->orientation -= ra.cross(impulse) * A->inverseInertia;
        }

        if (B->inverseMass > 0.0f)
        {
            B->position += impulse * B->inverseMass;
            B->orientation += rb.cross(impulse) * B->inverseInertia;
        }
    }

    A->calculateDerivativeData();
    B->calculateDerivativeData();
}

void CollisionResolve::SolveVelocity(Contact &contact, float friction)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    Vector2 relativeVelocity = B->velocity - A->velocity;
    float velocityAlongNormal = relativeVelocity.scalarProduct(contact.normal);

    if (velocityAlongNormal > 0.0f)
        return;

    float totalInverseMass = A->getInverseMass() + B->getInverseMass();
    if (totalInverseMass <= 0.0f)
        return;

    real restituion = std::min(A->restitution, B->restitution);
    float j = -(1.0f + restituion) * velocityAlongNormal;
    j /= totalInverseMass;

    // Apply impulse
    Vector2 impulse = contact.normal * j;

    if (A->getInverseMass() > 0.0f)
    {
        A->velocity -= impulse * A->getInverseMass();
    }

    if (B->getInverseMass() > 0.0f)
    {
        B->velocity += impulse * B->getInverseMass();
    }
}

void CollisionResolve::SolveVelocityWithRoatation(Contact &contact)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;
    Vector2 normal = contact.normal;
    Vector2 contact1 = contact.contactPoints[0];
    Vector2 contact2 = contact.contactPoints[1];
    int contactCount = contact.contactCount;

    Vector2 contactList[2];
    contactList[0] = contact1;
    contactList[1] = contact2;

    Vector2 impulseList[2];
    Vector2 raList[2];
    Vector2 rbList[2];

    real restituion = std::min(A->restitution, B->restitution);

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        real contactVelocityMag = relativeVelocity.scalarProduct(normal);

        if (contactVelocityMag > 0.0f)
        {
            continue;
        }

        real raPerpDotN = raPerp.scalarProduct(normal);
        real rbPerpDotN = rbPerp.scalarProduct(normal);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotN * raPerpDotN) * A->inverseInertia +
                     (rbPerpDotN * rbPerpDotN) * B->inverseInertia;

        real j = -(1.0f + restituion) * contactVelocityMag;
        j /= denom;
        j /= (real)contactCount;

        Vector2 impulse = normal * j;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 impulse = impulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (impulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(impulse) * -1) * A->inverseInertia;
        B->velocity += impulse * B->inverseMass;
        B->rotation += (rb.cross(impulse)) * B->inverseInertia;
    }
}

void CollisionResolve::SolveVelocityWithRoatationAndFriction(Contact &contact)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;
    Vector2 normal = contact.normal;
    Vector2 contact1 = contact.contactPoints[0];
    Vector2 contact2 = contact.contactPoints[1];
    int contactCount = contact.contactCount;

    Vector2 contactList[2];
    contactList[0] = contact1;
    contactList[1] = contact2;

    Vector2 impulseList[2];
    Vector2 raList[2];
    Vector2 rbList[2];

    real jList[2] = {0.0f, 0.0f};

    Vector2 frictionImpulseList[2] = {Vector2(0, 0), Vector2(0, 0)};

    real sf = (A->staticFriction + B->staticFriction) * 0.5f;
    real df = (A->dynamicFriction + B->dynamicFriction) * 0.5f;

    real restituion = std::min(A->restitution, B->restitution);

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        real contactVelocityMag = relativeVelocity.scalarProduct(normal);

        if (contactVelocityMag > 0.0f)
        {
            continue;
        }

        real raPerpDotN = raPerp.scalarProduct(normal);
        real rbPerpDotN = rbPerp.scalarProduct(normal);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotN * raPerpDotN) * A->inverseInertia +
                     (rbPerpDotN * rbPerpDotN) * B->inverseInertia;

        real j = -(1.0f + restituion) * contactVelocityMag;
        j /= denom;
        j /= (real)contactCount;

        jList[i] = j;

        Vector2 impulse = normal * j;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 impulse = impulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (impulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(impulse) * -1) * A->inverseInertia;
        B->velocity += impulse * B->inverseMass;
        B->rotation += (rb.cross(impulse)) * B->inverseInertia;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 ra = contactList[i] - A->position;
        Vector2 rb = contactList[i] - B->position;

        raList[i] = ra;
        rbList[i] = rb;

        Vector2 raPerp(-ra.y, ra.x);
        Vector2 rbPerp(-rb.y, rb.x);

        Vector2 angularLinearVelocityA = raPerp * A->rotation;
        Vector2 angularLinearVelocityB = rbPerp * B->rotation;

        Vector2 relativeVelocity = (B->velocity + angularLinearVelocityB) - (A->velocity + angularLinearVelocityA);

        Vector2 tangent = relativeVelocity - normal * relativeVelocity.scalarProduct(normal);
        if (Vector2::nearlyEqual(tangent, Vector2(0.0f, 0.0f)))
        {
            continue;
        }
        else
        {
            tangent.normalize();
        }

        real raPerpDotT = raPerp.scalarProduct(tangent);
        real rbPerpDotT = rbPerp.scalarProduct(tangent);

        real denom = A->inverseMass + B->inverseMass +
                     (raPerpDotT * raPerpDotT) * A->inverseInertia +
                     (rbPerpDotT * rbPerpDotT) * B->inverseInertia;

        real jt = -1 * relativeVelocity.scalarProduct(tangent);
        ;
        jt /= denom;
        jt /= (real)contactCount;

        real j = jList[i];

        Vector2 frictionImpulse;
        if (Vector2::abs(jt) <= j * sf)
        {
            frictionImpulse = tangent * jt;
        }
        else
        {
            frictionImpulse = tangent * -j * df;
        }

        frictionImpulseList[i] = frictionImpulse;
    }

    for (int i = 0; i < contactCount; i++)
    {
        Vector2 frictionImpulse = frictionImpulseList[i];

        Vector2 ra = raList[i];
        Vector2 rb = rbList[i];

        A->velocity += (frictionImpulse * -1) * A->inverseMass;
        A->rotation += (ra.cross(frictionImpulse) * -1) * A->inverseInertia;
        B->velocity += frictionImpulse * B->inverseMass;
        B->rotation += (rb.cross(frictionImpulse)) * B->inverseInertia;
    }
}

void CollisionResolve::Solve(Contact &contact, float dt)
{
    RigidBody *A = contact.a;
    RigidBody *B = contact.b;

    SolvePosition(contact);
    SolveVelocityWithRoatationAndFriction(contact);
}
