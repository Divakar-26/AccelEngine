#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <cmath>
#include <vector>
#include "UI.h"

class SoftBodyDemo : public Demo
{
public:
    const char* getName() const override { return "Soft Body Simulation (with Ground + ImGui)"; }

    void init(
        World& world,
        std::vector<RigidBody*>& bodies,
        ForceRegistry& registry,
        ForceGenerator* gravity
    ) override
    {
        worldRef = &world;
        bodiesRef = &bodies;
        registryRef = &registry;
        gravityRef = gravity;

        buildSoftBody();
    }

    void drawImGui() override
    {
        ImGui::Begin("Soft Body Controls");

        ImGui::Text("Soft Body Settings");
        ImGui::Separator();

        ImGui::SliderInt("Rows", &rows, 3, 12);
        ImGui::SliderInt("Columns", &cols, 3, 12);
        ImGui::SliderFloat("Spacing", &spacing, 20.0f, 80.0f);
        ImGui::SliderFloat("Spring Stiffness (K)", &springK, 1000.0f, 15000.0f);
        ImGui::SliderFloat("Spring Damping", &springD, 1.0f, 80.0f);

        ImGui::Spacing();
        if (ImGui::Button("Rebuild Soft Body"))
        {
            rebuildRequested = true;
        }

        ImGui::End();
    }

    void update()
    {
        if (rebuildRequested)
        {
            rebuildRequested = false;
            rebuildSoftBody();
        }
    }

private:
    void buildSoftBody()
    {
        // =====================================================
        // GROUND
        // =====================================================
        RigidBody* ground = new RigidBody();
        ground->shapeType = ShapeType::AABB;
        ground->aabb.halfSize = {600, 40};
        ground->position = {600, 100};
        ground->inverseMass = 0.0f;
        ground->inverseInertia = 0.0f;
        ground->orientation = 0;
        ground->c = {120, 120, 120, 255};
        ground->calculateDerivativeData();

        worldRef->addBody(ground);
        bodiesRef->push_back(ground);

        // =====================================================
        // SOFT BODY GRID
        // =====================================================
        std::vector<std::vector<RigidBody*>> grid(rows, std::vector<RigidBody*>(cols));
        const float startX = 600.0f - (cols * spacing) / 2.0f;
        const float startY = 700.0f;

        const float diagSpacing = std::sqrt(spacing * spacing * 2.0f);

        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                RigidBody* b = new RigidBody();

                b->shapeType = ShapeType::CIRCLE;
                b->circle.radius = 10.0f;
                b->position = { startX + c * spacing, startY - r * spacing };
                b->inverseMass = 1.0f;
                b->restitution = 0.2f;
                b->angularDamping = 0.98f;

                real mass = 1.0f / b->inverseMass;
                real inertia = 0.5f * mass * b->circle.radius * b->circle.radius;
                b->inverseInertia = 1.0f / inertia;

                b->calculateDerivativeData();
                b->c = {(float)180 + rand() % 60, (float)100 + rand() % 70, (float)200 + rand() % 55, (float)255};

                worldRef->addBody(b);
                bodiesRef->push_back(b);
                registryRef->add(b, gravityRef);

                grid[r][c] = b;
            }
        }

        // =====================================================
        // SPRINGS
        // =====================================================
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols - 1; ++c)
            {
                Spring* s = new Spring(Vector2(0,0), grid[r][c+1], Vector2(0,0), springK, spacing);
                s->damping = springD;
                registryRef->add(grid[r][c], s);
            }
        }

        for (int r = 0; r < rows - 1; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                Spring* s = new Spring(Vector2(0,0), grid[r+1][c], Vector2(0,0), springK, spacing);
                s->damping = springD;
                registryRef->add(grid[r][c], s);
            }
        }

        const float diagSpacing2 = std::sqrt(spacing * spacing * 2.0f);
        for (int r = 0; r < rows - 1; ++r)
        {
            for (int c = 0; c < cols - 1; ++c)
            {
                Spring* s1 = new Spring(Vector2(0,0), grid[r+1][c+1], Vector2(0,0), springK, diagSpacing2);
                s1->damping = springD;
                registryRef->add(grid[r][c], s1);

                Spring* s2 = new Spring(Vector2(0,0), grid[r+1][c], Vector2(0,0), springK, diagSpacing2);
                s2->damping = springD;
                registryRef->add(grid[r][c+1], s2);
            }
        }
    }

    void rebuildSoftBody()
    {
        worldRef->clear();    // depends on your engine’s clear API
        bodiesRef->clear();
        registryRef->clear();

        buildSoftBody();
    }

private:
    // references
    World* worldRef = nullptr;
    std::vector<RigidBody*>* bodiesRef = nullptr;
    ForceRegistry* registryRef = nullptr;
    ForceGenerator* gravityRef = nullptr;

    // tweakable params
    int rows = 6;
    int cols = 8;
    float spacing = 40.0f;
    float springK = 9000.0f;
    float springD = 35.0f;

    bool rebuildRequested = false;
};
