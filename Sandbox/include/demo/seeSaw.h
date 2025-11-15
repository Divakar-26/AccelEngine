#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <AccelEngine/joint.h>
#include <vector>
#include <cmath>
#include "UI.h"

class SeeSawDemo : public Demo
{
private:
    RigidBody *plank = nullptr;
    RigidBody *pivot = nullptr;
    DistanceJoint *hinge = nullptr;

public:
    const char *getName() const override { return "See-Saw"; }

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

        plank = new RigidBody();
        plank->shapeType = ShapeType::AABB;
        plank->aabb.halfSize = {200, 10}; // Long thin board
        plank->position = {600, 150};
        plank->inverseMass = 0.3f; // fairly heavy
        plank->restitution = 0.2f;
        plank->orientation = 0;
        plank->c = {220, 120, 60, 255};
        plank->calculateInertia();
        plank->angularDamping = 0.99f;
        plank->calculateDerivativeData();
        
        RigidBody * c = new RigidBody();
        c = new RigidBody();
        c->shapeType = ShapeType::CIRCLE;
        c->circle.radius = 20.0f; // Long thin board
        c->position = {600, 80 + 40};
        c->inverseMass = 0.3f; // fairly heavy
        c->restitution = 0.2f;
        c->orientation = 0;
        c->c = {220, 120, 60, 255};
        c->calculateInertia();
        c->angularDamping = 0.99f;
        c->calculateDerivativeData();

        DistanceJoint * j1 = new DistanceJoint(c, ground, Vector2(0,0), Vector2(0,0), 20+40);
        DistanceJoint * j2 = new DistanceJoint(plank, c, Vector2(0,0), Vector2(0,0), 20+10);


        world.addJoint(j1);
        world.addJoint(j2);
        world.addBody(c);
        world.addBody(plank);
        bodies.push_back(plank);
        bodies.push_back(c);
        registry.add(plank, gravity);
    }

    void drawImGui() override
    {
        ImGui::Begin("See-Saw Controls");
        ImGui::Text("This demo shows a teeter-totter with pivot joint");
        ImGui::Separator();
        ImGui::Text("Left Ball: heavier = tilt left");
        ImGui::Text("Right Ball: heavier = tilt right");
        ImGui::Text("You can drag balls to see torque effect!");
        ImGui::End();
    }
};
