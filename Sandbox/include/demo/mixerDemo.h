#pragma once
#include "showcase.h"

// A round drum with two motor-driven paddles stirring four colour-sorted batches of balls.
// The paddles are "kinematic": position locked, very heavy, angular velocity set every substep.
class MixerDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Color Mixer"; }

    void drawImGui() override
    {
        ImGui::Begin("Color Mixer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Balls: %d", ballCount);
        ImGui::SliderFloat("Paddle speed (rad/s)", &speed, -6.0f, 6.0f);
        if (ImGui::Button("Reverse"))
            speed = -speed;
        ImGui::SameLine();
        if (ImGui::Button("Stop"))
            speed = 0.0f;
        ImGui::SliderInt("Balls (rebuild)", &count, 300, 3000);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("Four sorted colours get stirred into a mix; the paddles are position-locked bodies "
            "whose angular velocity is set every substep.");
        ImGui::End();
    }

protected:
    void build() override
    {
        paddles.clear();
        const Vector2 c = center;
        const float R = 440.0f;

        // drum: a ring of static planks
        const int segments = 60;
        for (int i = 0; i < segments; i++)
        {
            const float a = 6.2831853f * i / segments;
            const Vector2 p = c + Vector2(std::cos(a), std::sin(a)) * (R + 10);
            RigidBody *s = box(p, {R * 3.14159f / segments + 3.0f, 10.0f}, 0.0f, a + 1.5707963f);
            s->c = rgb(70, 70, 80);
            s->staticFriction = 0.5f;
            s->dynamicFriction = 0.4f;
        }

        // two paddles, driven in the same direction (see step())
        for (int k = 0; k < 2; k++)
        {
            // set low so they dig into the pile (gravity keeps most balls in the lower half)
            RigidBody *p = box(c + Vector2(k == 0 ? -195.0f : 195.0f, -120.0f), {165.0f, 9.0f}, 0.05f);
            p->lockPosition = true;
            p->canSleep = false;
            p->angularDamping = 1.0f;
            p->c = rgb(235, 235, 240);
            paddles.push_back(p);
        }

        // balls: four colour batches, one per quadrant of the drum
        const Color batch[4] = {hsv(0.53f, 0.85f, 1.0f), hsv(0.90f, 0.8f, 1.0f), hsv(0.14f, 0.9f, 1.0f), hsv(0.33f, 0.8f, 0.9f)};
        const float r = 6.0f, cell = 2 * r + 1.5f;
        ballCount = 0;
        for (float y = c.y - R + r + 6; y < c.y + R && ballCount < count; y += cell)
        {
            for (float x = c.x - R + r + 6; x < c.x + R && ballCount < count; x += cell)
            {
                const Vector2 p(x, y);
                if ((p - c).magnitude() > R - r - 8)
                    continue;
                if (std::fabs(y - (c.y - 120)) < 22 && std::fabs(std::fabs(x - c.x) - 195) < 175)
                    continue; // keep clear of the paddles
                RigidBody *b = ball(p, r, 1.0f);
                b->restitution = 0.1f;
                b->staticFriction = 0.3f;
                b->dynamicFriction = 0.2f;
                const int q = (x < c.x ? 0 : 1) + (y < c.y ? 0 : 2);
                b->c = batch[q];
                ballCount++;
            }
        }
    }

    void step() override
    {
        // motor: drive the paddles' angular velocity every substep. Same direction for both, so the
        // material is carried across the middle (opposite directions just push it into the centre).
        for (RigidBody *p : paddles)
            p->rotation = speed;
    }

private:
    std::vector<RigidBody *> paddles;
    Vector2 center{960, 520};
    float speed = 3.0f;
    int count = 1800;
    int ballCount = 0;
};
