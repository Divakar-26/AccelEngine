#pragma once
#include "showcase.h"

// A closed box full of coloured shapes. Rotate the gravity direction with a slider (or let it spin) and the
// whole pile slides, tumbles and re-settles against a different wall.
class GravityTiltDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Gravity Tilt"; }

    void drawImGui() override
    {
        ImGui::Begin("Gravity Tilt", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Shapes: %d   awake: %d", shapeCount, worldRef->getAwakeBodyCount());
        ImGui::SliderFloat("Gravity direction (deg)", &angleDeg, -180.0f, 180.0f);
        ImGui::SliderFloat("Strength", &strength, 100.0f, 2500.0f);
        ImGui::Checkbox("Spin gravity", &spinning);
        ImGui::SliderFloat("Spin speed (deg/s)", &spinSpeed, -180.0f, 180.0f);
        ImGui::SliderInt("Shapes (rebuild)", &count, 100, 1500);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("-90 is normal gravity (down). Changing the direction wakes the sleeping pile; "
            "gravity itself never wakes bodies, so the demo does it explicitly.");
        ImGui::End();

        // arrow showing gravity
        const Vector2 d = direction();
        ImDrawList *dl = ImGui::GetBackgroundDrawList();
        SDL_FPoint a = Renderer2D::worldToScreen(center.x, center.y);
        dl->AddLine(ImVec2(a.x, a.y), ImVec2(a.x + d.x * 80, a.y - d.y * 80), IM_COL32(255, 80, 80, 220), 5.0f);
        dl->AddCircleFilled(ImVec2(a.x + d.x * 80, a.y - d.y * 80), 8.0f, IM_COL32(255, 80, 80, 220));
    }

    float getAngle() const { return angleDeg; }
    void setAngle(float deg) { angleDeg = deg; }
    void setSpin(bool on, float degPerSec) { spinning = on; spinSpeed = degPerSec; }

protected:
    void build() override
    {
        angleDeg = -90.0f;
        lastApplied = 1e9f;
        const float half = 430.0f, t = 15.0f;
        const Color wall = rgb(80, 85, 100);
        box(center + Vector2(0, -half - t), {half + 2 * t, t}, 0.0f)->c = wall;
        box(center + Vector2(0, half + t), {half + 2 * t, t}, 0.0f)->c = wall;
        box(center + Vector2(-half - t, 0), {t, half + 2 * t}, 0.0f)->c = wall;
        box(center + Vector2(half + t, 0), {t, half + 2 * t}, 0.0f)->c = wall;

        shapeCount = 0;
        const float cell = 36.0f;
        const int cols = (int)((2 * half - 20) / cell);
        for (int i = 0; i < count; i++)
        {
            const int row = i / cols, col = i % cols;
            const Vector2 p(center.x - half + 18 + col * cell, center.y - half + 20 + row * cell);
            if (p.y > center.y + half - 18)
                break;
            RigidBody *b = (i % 3 == 0) ? box(p, {frand(9, 14), frand(9, 14)}, 1.0f, frand(0, 3.14f))
                                        : ball(p, frand(8, 15), 1.0f);
            b->restitution = 0.15f;
            b->staticFriction = 0.5f;
            b->dynamicFriction = 0.4f;
            b->c = hsv((float)i / count, 0.75f, 1.0f);
            shapeCount++;
        }
    }

    void step() override
    {
        if (spinning)
            angleDeg += spinSpeed / 360.0f; // step() runs once per substep (6 per 1/60 s)
        if (angleDeg > 180.0f) angleDeg -= 360.0f;
        if (angleDeg < -180.0f) angleDeg += 360.0f;

        if (std::fabs(angleDeg - lastApplied) > 0.05f)
        {
            if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
                g->gravity = direction() * strength;
            worldRef->wakeAll(); // forces never wake sleeping bodies
            lastApplied = angleDeg;
        }
        else if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
        {
            const Vector2 want = direction() * strength;
            if ((g->gravity - want).squareMagnitude() > 1.0f) // strength slider
            {
                g->gravity = want;
                worldRef->wakeAll();
            }
        }
    }

private:
    Vector2 direction() const
    {
        const float a = angleDeg * 3.14159265f / 180.0f;
        return {std::cos(a), std::sin(a)};
    }

    Vector2 center{960.0f, 520.0f};
    float angleDeg = -90.0f, strength = 980.0f, spinSpeed = 40.0f, lastApplied = 1e9f;
    bool spinning = false;
    int count = 700, shapeCount = 0;
};
