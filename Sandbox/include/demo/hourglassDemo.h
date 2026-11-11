#pragma once
#include "showcase.h"

// Sand running through a 20 px neck. "Flip" reverses gravity (and wakes the sleeping pile) so the sand
// runs back the other way. Grains are small, slightly sticky circles; the settled pile falls asleep.
class HourglassDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Hourglass"; }

    void drawImGui() override
    {
        ImGui::Begin("Hourglass", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Grains: %zu   in upper half: %d   in lower half: %d", grains.size(), countHalf(true), countHalf(false));
        if (ImGui::Button("Flip the hourglass", ImVec2(220, 40)))
            flip();
        ImGui::SliderInt("Grains (rebuild)", &count, 300, 1600);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("Flip reverses gravity and wakes the pile. The neck is ~7 grains wide, so the flow is a steady stream.");
        ImGui::End();
    }

    void flip()
    {
        if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
            g->gravity = g->gravity * -1.0f;
        worldRef->wakeAll();
    }

    int countHalf(bool upper) const
    {
        int n = 0;
        for (const RigidBody *b : grains)
            if ((b->position.y > centerY) == upper)
                n++;
        return n;
    }

protected:
    void build() override
    {
        grains.clear();
        if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
            g->gravity = Vector2(0.0f, -std::fabs(g->gravity.y)); // always start with sand on top

        const Color glass = rgb(150, 200, 220);
        const float cx = 960.0f, neck = 18.0f; // half width of the neck (~7 grains across)
        // left half outline; the right half is mirrored about x = 960
        const Vector2 left[] = {{790, 900}, {790, 760}, {cx - neck, centerY + 20}, {cx - neck, centerY - 20}, {790, 280}, {790, 140}};
        for (int side = -1; side <= 1; side += 2)
            for (size_t i = 0; i + 1 < sizeof(left) / sizeof(left[0]); i++)
            {
                Vector2 a = left[i], b = left[i + 1];
                if (side == 1) { a.x = 2 * cx - a.x; b.x = 2 * cx - b.x; }
                RigidBody *w = segment(a, b, 5.0f);
                w->c = glass;
                w->staticFriction = 0.3f;
                w->dynamicFriction = 0.2f;
            }
        segment({790, 900}, {2 * cx - 790, 900}, 6)->c = rgb(120, 85, 55); // caps
        segment({790, 140}, {2 * cx - 790, 140}, 6)->c = rgb(120, 85, 55);

        // sand: packed rows in the upper bulb
        const float r = 2.6f, cell = 2 * r + 0.5f;
        const int cols = (int)((2 * cx - 800.0f - 800.0f - 2 * r) / cell);
        for (int i = 0; i < count; i++)
        {
            const int row = i / cols, col = i % cols;
            const Vector2 p(805.0f + r + col * cell + (row % 2) * cell * 0.5f, 775.0f + row * cell * 0.88f);
            if (p.y > 892.0f)
                break;
            RigidBody *s = ball(p, r, 1.0f);
            s->restitution = 0.0f;
            s->staticFriction = 0.35f;
            s->dynamicFriction = 0.25f;
            s->angularDamping = 0.95f;
            s->c = hsv(0.10f + 0.03f * frand(0, 1), 0.55f, 0.85f + 0.15f * frand(0, 1));
            grains.push_back(s);
        }
    }

private:
    std::vector<RigidBody *> grains;
    const float centerY = 520.0f;
    int count = 1100;
};
