#include <AccelEngine/collision_resolve.h>
#include <cmath>

using namespace AccelEngine;

void CollisionResolve::SolvePosition(Contact& contact, float correctionFactor, float slop)
{
    RigidBody* A = contact.a;
    RigidBody* B = contact.b;
    
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

void CollisionResolve::SolveVelocity(Contact& contact, float friction)
{
    RigidBody* A = contact.a;
    RigidBody* B = contact.b;
    
    Vector2 relativeVelocity = B->velocity - A->velocity;
    float velocityAlongNormal = relativeVelocity.scalarProduct(contact.normal);
    
    if (velocityAlongNormal > 0.0f)
        return;
    
    float totalInverseMass = A->getInverseMass() + B->getInverseMass();
    if (totalInverseMass <= 0.0f)
        return;
    
    // restitution = 0.0f -> completely inelastic (no bounce)
    // restitution = 1.0f -> completely elastic (perfect bounce)
    real restituion = std::min(A->restitution , B->restitution);
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

void CollisionResolve::Solve(Contact& contact, float dt)
{
    // Solve position first (separate objects)
    SolvePosition(contact);
    
    // Then solve velocity (bounce response)
    SolveVelocity(contact);
}