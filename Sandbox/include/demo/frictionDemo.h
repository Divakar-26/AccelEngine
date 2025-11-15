#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cmath>
#include <vector>
#include "UI.h"

class FrictionDemo : public Demo
{
public:
    const char *getName() const override { return "Friction Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        world.joints.clear();
        RigidBody *ground = new RigidBody();
        ground->shapeType = ShapeType::AABB;
        ground->aabb.halfSize = {800, 40};
        ground->position = {600, 80};
        ground->inverseMass = 0.0f;
        ground->inverseInertia = 0.0f;
        ground->orientation = 0;
        ground->c = {100, 100, 100, 255};
        ground->calculateDerivativeData();
        world.addBody(ground);
        bodies.push_back(ground);

        RigidBody *plank1 = new RigidBody();
        plank1->shapeType = ShapeType::AABB;
        plank1->aabb.halfSize = {343, 25};
        plank1->position = {289, 747};
        plank1->inverseMass = 0.0f;
        plank1->orientation = -19.0f;
        plank1->c = {150, 170, 90, 255};
        plank1->staticFriction = 0.0f;
        plank1->dynamicFriction = 0.0f;
        plank1->calculateDerivativeData();
        plank1->calculateInertia();
        world.addBody(plank1);
        bodies.push_back(plank1);

        RigidBody *plank2 = new RigidBody();
        plank2->shapeType = ShapeType::AABB;
        plank2->aabb.halfSize = {343, 25};
        plank2->position = {809, 567};
        plank2->inverseMass = 0.0f;
        plank2->orientation = 19.0f;
        plank2->staticFriction = 0.0f;
        plank2->dynamicFriction = 0.0f;
        plank2->c = {150, 170, 90, 255};
        plank2->calculateDerivativeData();
        plank2->calculateInertia();
        world.addBody(plank2);
        bodies.push_back(plank2);

        RigidBody *plank3 = new RigidBody();
        plank3->shapeType = ShapeType::AABB;
        plank3->aabb.halfSize = {343, 25};
        plank3->position = {289, 300};
        plank3->inverseMass = 0.0f;
        plank3->orientation = -19.0f;
        plank3->c = {150, 170, 90, 255};
        plank3->staticFriction = 0.0f;
        plank3->dynamicFriction = 0.0f;
        plank3->calculateDerivativeData();
        plank3->calculateInertia();
        world.addBody(plank3);
        bodies.push_back(plank3);

        for (int i = 0; i < 5; i++)
        {
            // Now 0.9, 0.7, 0.5, 0.3, 0.1 → last one is slipperiest
            float friction = 0.9f - i * 0.2f;

            RigidBody *a = makeAABB(
                world, bodies, registry, gravity,
                Vector2(80 + 100 * i, 920),
                Vector2(25, 25),
                4.0f, 0.0f);

            a->staticFriction = friction;
            a->dynamicFriction = friction * 0.8f;
        }
    }
};
