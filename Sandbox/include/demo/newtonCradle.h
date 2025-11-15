#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <vector>
#include "UI.h"

class NewtonsCradle : public Demo
{
public:
    const char *getName() const override { return "Newton's Cradle"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        RigidBody * plank = makeAABB(world, bodies, registry, gravity, {500,500},{200,25}, 0.0, 0.0f);

        int numberOfCircles = 5;
        int radius = 25;
        int totalRadius = numberOfCircles * radius;
        int startX = plank->position.y - totalRadius / 2; 
        int startPointX = -totalRadius;
        
        for(int i = 0; i < numberOfCircles; i++){
            RigidBody * circle1 = makeCircle(world, bodies, registry, gravity, {(startX + radius * i ) + 20, 300}, 25.0, 1.0f);
            

            circle1->restitution = 1.0f;
            circle1->linearDamping = 1.0f;

            Vector2 pointOnPlank = {startPointX + (radius * i * 2), 0};
            DistanceJoint * j = new DistanceJoint(plank, circle1, pointOnPlank, {0,0});
            world.addJoint(j);
        }
    }

    void drawImGui() override
    {

    }

    void update() override {}
};
