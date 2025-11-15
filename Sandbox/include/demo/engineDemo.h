#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cmath>
#include <vector>
#include "UI.h"

class EngineDemo : public Demo
{
public:
    RigidBody *piston;
    RigidBody *wheel;
    const char *getName() const override { return "Engine Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        RigidBody *shaft1 = makeAABB(world, bodies, registry, gravity, {500, 500}, {25, 200}, 0.0, 0.0);
        RigidBody *shaft2 = makeAABB(world, bodies, registry, gravity, {790, 500}, {25, 200}, 0.0, 0.0);

        piston = makeAABB(world, bodies, registry, gravity, {650, 400}, {120, 100}, 1.0, 0.0);
        piston->inverseMass = 1.0;
        piston->lockRotation = true;
        piston->ignoreGravity = true;

        wheel = makeCircle(world, bodies, registry, gravity, {650, 70}, 100, 1.0);
        wheel->orientation = 0.0f;
        wheel->rotation = 0.0;
        wheel->lockPosition = true;
        wheel->lockRotation = false;
        wheel->angularDamping = 1.0;

        DistanceJoint *j = new DistanceJoint(piston, wheel, {0, -piston->getHeigt() / 2}, {100, 0});

        world.addJoint(j);

        piston->staticFriction = 0.0f;
        piston->dynamicFriction = 0.0f;

        shaft1->staticFriction = 0.0f;
        shaft1->dynamicFriction = 0.0f;

        shaft2->staticFriction = 0.0f;
        shaft2->dynamicFriction = 0.0f;
    }

    void drawImGui() override
    {
    }

    void update() override
    {
        real angle = wheel->orientation; // radians
        if (piston->getPosition().y >= 447.0 &&
            angle > 0.0f &&
            angle < (60.0f * 3.14159265f / 180.0f))
        {
            // std::cout<<"hello"<<std::endl;
            piston->addForceAtBodyPoint({0, -8000}, {0, piston->getHeigt() / 2});
        }
    }
};
