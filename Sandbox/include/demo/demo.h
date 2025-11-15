#pragma once
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cstdlib> 
#include <cmath>
#include <SDL3/SDL.h>


using namespace AccelEngine;


class Demo
{
public:
    virtual ~Demo() {}

    virtual void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity
    ) = 0;

    virtual const char *getName() const = 0;

    virtual void drawImGui() {}
    virtual void update() {}

protected:

    RigidBody* makeAABB(
        World& world,
        std::vector<RigidBody*>& bodies,
        ForceRegistry& registry,
        ForceGenerator* gravity,
        Vector2 pos,
        Vector2 size,
        float invMass = 1.0f,
        float orientation = 0.0f,
        SDL_Color color = {255, 255, 255, 255},
        float restitution = 0.3f
    )
    {
        RigidBody* b = new RigidBody();
        b->shapeType = ShapeType::AABB;
        b->aabb.halfSize = size;
        b->position = pos;
        b->inverseMass = invMass;
        b->restitution = restitution;
        b->orientation = orientation;
        b->angularDamping = 0.98f;

        // Compute inertia correctly
        if (invMass > 0.0f)
        {
            float mass = 1.0f / invMass;
            float w = size.x;
            float h = size.y;
            float inertia = (1.0f / 12.0f) * mass * (w * w + h * h);
            b->inverseInertia = 1.0f / inertia;
        }
        else
        {
            b->inverseInertia = 0.0f;
        }

        b->c = randomColor();
        b->calculateDerivativeData();
        b->calculateInertia();
        world.addBody(b);
        bodies.push_back(b);
        if (gravity && invMass > 0.0f)
            registry.add(b, gravity);

        return b;
    }


    RigidBody* makeCircle(
        World& world,
        std::vector<RigidBody*>& bodies,
        ForceRegistry& registry,
        ForceGenerator* gravity,
        Vector2 pos,
        float radius,
        float invMass = 1.0f,
        SDL_Color color = {255, 255, 255, 255},
        float restitution = 0.3f
    )
    {
        RigidBody* b = new RigidBody();
        b->shapeType = ShapeType::CIRCLE;
        b->circle.radius = radius;
        b->position = pos;
        b->inverseMass = invMass;
        b->restitution = restitution;
        b->angularDamping = 0.98f;

        if (invMass > 0.0f)
        {
            float mass = 1.0f / invMass;
            float inertia = 0.5f * mass * radius * radius;
            b->inverseInertia = 1.0f / inertia;
        }
        else
        {
            b->inverseInertia = 0.0f;
        }

        b->c = {(float)color.r, (float)color.g, (float)color.b, (float)color.a};
        b->calculateDerivativeData();

        b->c = randomColor();

        world.addBody(b);
        bodies.push_back(b);
        if (gravity && invMass > 0.0f)
            registry.add(b, gravity);

        return b;
    }

    Color randomColor()
    {
        return {
            (float)(rand() % 200 + 55),
            (float)(rand() % 200 + 55),
            (float)(rand() % 200 + 55),
            255
        };
    }
};
