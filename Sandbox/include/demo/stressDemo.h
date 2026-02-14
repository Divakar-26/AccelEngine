#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cmath>
#include <vector>
#include "UI.h"

class StressDemo : public Demo
{
public:
    const char *getName() const override { return "Stress Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        world.joints.clear();

        // SAME ground as DominoDemo
        RigidBody *ground =
            makeAABB(world, bodies, registry, gravity,
                     {600, 40},
                     {1400, 30},
                     0.0f,
                     0.0f);

        ground->staticFriction = 1.0f;
        ground->dynamicFriction = 1.0f;

        // Your walls (optional)
        makeAABB(world, bodies, registry, gravity, {1946, 2099}, {50, 2000}, 0.0f, 0.0f);
        makeAABB(world, bodies, registry, gravity, {40, 2099}, {50, 2000}, 0.0f, 0.0f);
    }

    void drawImGui() override
    {
    }

    void update() override
    {
    }
};
