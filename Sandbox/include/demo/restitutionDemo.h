#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <vector>
#include "UI.h"

class RestitutionDemo : public Demo
{
public:
    const char *getName() const override { return "Restitution Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        // -----------------------
        // Ground (static)
        // -----------------------
        RigidBody *ground = new RigidBody();
        ground->shapeType = ShapeType::AABB;
        ground->aabb.halfSize = {600, 40};
        ground->position = {600, 100};
        ground->inverseMass = 0.0f;   // static body
        ground->inverseInertia = 0.0f;
        ground->orientation = 0;
        ground->restitution = 1.0f;
        ground->c = {120, 120, 120, 255};
        ground->calculateDerivativeData();

        world.addBody(ground);
        bodies.push_back(ground);

        // -----------------------
        // Restitution Test Balls
        // -----------------------
        float startY = 450.0f;
        float startX = 350.0f;

        std::vector<float> restitutions = {0.0f, 0.2f, 0.5f, 0.8f, 1.0f};

        for (int i = 0; i < restitutions.size(); i++)
        {
            float x = startX + i * 120.0f;
            float r = restitutions[i];

            RigidBody *ball = makeCircle(world, bodies, registry, gravity, {x, startY}, 25, 1.0f);
            ball->ignoreGravity = false;
            ball->restitution = r;

            // give each ball a different color
            ball->c = { 
                (float)(50 + i * 40),
                (float)(200 - i * 30),
                (float)(100 + i * 30),
                255 
            };
        }
    }

    void drawImGui() override
    {
        ImGui::Begin("Restitution Demo Info", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Balls have different bounciness:");
        ImGui::BulletText("0.0  - No bounce");
        ImGui::BulletText("0.2  - Slight bounce");
        ImGui::BulletText("0.5  - Medium");
        ImGui::BulletText("0.8  - Very bouncy");
        ImGui::BulletText("1.0  - Perfectly elastic");
        ImGui::End();
    }

    void update() override {}
};
