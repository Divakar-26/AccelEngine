#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "UI.h"

// Container for large-scale tests: spawn thousands of bodies at once from the "Stress Test" panel.
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
        worldRef = &world;
        bodiesRef = &bodies;
        registryRef = &registry;
        gravityRef = gravity;

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
        ImGui::Begin("Stress Test", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::SliderInt("Count", &spawnCount, 100, 50000);
        ImGui::Combo("Shape", &shape, "Mixed\0Boxes\0Circles\0");
        ImGui::SliderFloat("Size (half / radius)", &size, 3.0f, 30.0f);
        if (ImGui::Button("Spawn grid"))
            spawnGrid();
        ImGui::Text("Zoom out with E, in with Q. The grid is placed above everything already here.");
        ImGui::End();
    }

    void update() override
    {
    }

private:
    void spawnGrid()
    {
        if (!worldRef)
            return;

        // inside the walls, above the highest body already in the container
        const float left = 90.0f, right = 1896.0f;
        float floorY = 70.0f;
        for (RigidBody *b : *bodiesRef)
            if (b->inverseMass > 0.0f)
                floorY = std::max(floorY, b->worldAABBMax.y);

        const float cell = size * 2.0f + 2.0f;
        const int cols = std::max(1, (int)((right - left) / cell));
        for (int i = 0; i < spawnCount; i++)
        {
            const int row = i / cols, col = i % cols;
            // small per-row offset so columns do not stack perfectly
            Vector2 pos(left + cell * 0.5f + col * cell + (row % 2) * 0.25f * size, floorY + cell * 0.5f + 1.0f + row * cell);
            const bool box = shape == 1 || (shape == 0 && (i % 2) == 1);
            if (box)
                makeAABB(*worldRef, *bodiesRef, *registryRef, gravityRef, pos, {size, size}, 1.0f, 0.0f);
            else
                makeCircle(*worldRef, *bodiesRef, *registryRef, gravityRef, pos, size, 1.0f);
        }
    }

    World *worldRef = nullptr;
    std::vector<RigidBody *> *bodiesRef = nullptr;
    ForceRegistry *registryRef = nullptr;
    ForceGenerator *gravityRef = nullptr;

    int spawnCount = 10000;
    int shape = 0;
    float size = 6.0f;
};
