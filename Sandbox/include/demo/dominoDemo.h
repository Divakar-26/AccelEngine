#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cmath>
#include <vector>
#include "UI.h"

class DominoDemo : public Demo
{
public:
    const char *getName() const override { return "Domino Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        world.joints.clear();
        RigidBody *ground = makeAABB(world, bodies, registry, gravity, {600, 40}, {600, 30}, 0.0, 0.0);
        ground->staticFriction  = 1.0f;
        ground->dynamicFriction = 1.0f;

        int numberOfBoxes = 10;
        int startX = 200;
        float boxhW = 10;
        float boxhH = 30;
        int startY = 100;
        float gap = boxhW * 2 + boxhH * 2 - 20;

        RigidBody *platform = makeAABB(world, bodies, registry, gravity, {70, 325}, {70, 25}, 0.0, 0.0);
        RigidBody *circle = makeCircle(world, bodies, registry, gravity, {74, 117}, 25, 1.0);

        DistanceJoint *j = new DistanceJoint(platform, circle, {0, 0}, {0, 0});
        world.addJoint(j);

        for (int i = 0; i < numberOfBoxes; i++)
        {
            RigidBody *a = makeAABB(world, bodies, registry, gravity, {startX + (gap * i), startY}, {boxhW, boxhH}, 1.0, 0.0);
            a->staticFriction = 0.2f;
            a->dynamicFriction = 0.2f;
            a->linearDamping = 1.0;
        }
    }

    void drawImGui() override
    {
    }

    void update() override
    {
    }

private:
    void buildDominos()
    {
    }
};
