#pragma once
#include "showcase.h"

// A reservoir of thousands of low-friction particles held back by a dam. Remove the dam and the
// wave floods a little village and gets thrown up a ramp at the far end.
class DamBreakDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Dam Break"; }

    void drawImGui() override
    {
        ImGui::Begin("Dam Break", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Water particles: %d", waterCount);
        if (ImGui::Button(damIntact ? "Break the dam!" : "Dam is gone", ImVec2(220, 40)) && damIntact)
            breakDam();
        ImGui::SliderInt("Particles", &targetCount, 1000, 9000);
        ImGui::SliderFloat("Particle radius", &radius, 4.0f, 9.0f);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("The water settles and falls asleep (drawn dimmed) until the dam goes. "
            "Untick Physics > 'Dim sleeping bodies' for full colour.");
        ImGui::End();
    }

    void breakDam()
    {
        if (!damIntact)
            return;
        damIntact = false;
        // the Sandbox has no per-body removal: move the dam far away and wake everything
        dam->position = Vector2(-100000.0f, -100000.0f);
        dam->calculateDerivativeData();
        worldRef->wakeAll();
    }

protected:
    void build() override
    {
        damIntact = true;
        const Color concrete = rgb(120, 120, 128);

        // basin
        RigidBody *ground = box({960, 40}, {930, 30}, 0.0f);
        ground->c = rgb(70, 70, 78);
        box({30, 620}, {30, 580}, 0.0f)->c = concrete;
        box({1890, 620}, {30, 580}, 0.0f)->c = concrete;

        // the dam
        dam = box({900, 470}, {14, 400}, 0.0f);
        dam->c = rgb(150, 110, 70);

        // ramp at the far end throws the wave upwards
        segment({1560, 70}, {1858, 260}, 10)->c = concrete;

        // village: little houses (box walls + roof) between the dam and the ramp
        for (int h = 0; h < 4; h++)
        {
            const float x = 1050.0f + h * 125.0f;
            const Color wall = hsv(0.08f + 0.03f * h, 0.45f, 0.95f);
            // each storey: two walls (32 tall) with a floor slab (10 thick) on top, 0.5-1 px gaps
            float floorTop = 70.0f;
            for (int level = 0; level < 3; level++)
            {
                const float wallY = floorTop + 16.5f;
                RigidBody *l = box({x - 22, wallY}, {8, 16}, 1.0f);
                RigidBody *r = box({x + 22, wallY}, {8, 16}, 1.0f);
                const float slabY = wallY + 16 + 5.5f;
                RigidBody *slab = box({x, slabY}, {32, 5}, 1.0f);
                l->c = r->c = wall;
                slab->c = rgb(110, 75, 50);
                floorTop = slabY + 5;
            }
            RigidBody *roof = box({x, floorTop + 9}, {38, 8}, 1.0f);
            roof->c = rgb(180, 50, 45);
        }

        // water: fill the reservoir from the floor up, colour by depth
        const float cell = radius * 2.0f + 0.4f;
        const float left = 60 + radius + 1, right = 886 - radius;
        const int cols = std::max(1, (int)((right - left) / cell));
        const int rows = (targetCount + cols - 1) / cols;
        waterCount = 0;
        for (int i = 0; i < targetCount; i++)
        {
            const int row = i / cols, col = i % cols;
            Vector2 p(left + col * cell + (row % 2) * cell * 0.5f, 70 + radius + 1 + row * cell * 0.9f);
            if (p.x > right)
                continue;
            RigidBody *w = ball(p, radius, 1.0f);
            w->restitution = 0.0f;
            w->staticFriction = 0.02f;
            w->dynamicFriction = 0.01f;
            w->angularDamping = 0.9f;
            const float t = (float)row / std::max(1, rows - 1);
            w->c = hsv(0.58f - 0.03f * t, 0.85f - 0.45f * t, 0.55f + 0.45f * t);
            waterCount++;
        }
    }

private:
    RigidBody *dam = nullptr;
    bool damIntact = true;
    int targetCount = 4000;
    int waterCount = 0;
    float radius = 6.0f;
};
