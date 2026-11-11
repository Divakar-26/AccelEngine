#pragma once
#include "showcase.h"

// Rube Goldberg machine: a row of dominoes tips a ball down a ramp, the ball starts a second row, and the
// last domino brings down a brick tower. Press Start (or nudge the first domino yourself).
class ChainReactionDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Chain Reaction"; }

    void drawImGui() override
    {
        ImGui::Begin("Chain Reaction", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Stage 1 dominoes down: %d / %d", fallen(row1), (int)row1.size());
        ImGui::Text("Stage 2 dominoes down: %d / %d", fallen(row2), (int)row2.size());
        ImGui::Text("Tower bricks knocked:  %d / %d", knocked(), (int)tower.size());
        if (ImGui::Button("Start!", ImVec2(220, 40)))
            start();
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Everything is asleep until the first push: contacts wake the chain one island at a time.");
        ImGui::End();
    }

    void start()
    {
        if (row1.empty())
            return;
        // a slowly tipping domino must not doze off mid-fall: keep the whole chain awake from now on
        for (RigidBody *b : *bodiesRef)
            if (!b->isStatic())
                b->canSleep = false;
        RigidBody *first = row1.front();
        first->applyImpulseAtPoint({90.0f, 0.0f}, first->position + Vector2(0, 20));
    }

    int fallen(const std::vector<RigidBody *> &row) const
    {
        int n = 0;
        for (const RigidBody *b : row)
            if (std::fabs(std::sin(b->orientation)) > 0.7f) // tilted more than ~45 degrees
                n++;
        return n;
    }

    int knocked() const
    {
        int n = 0;
        for (size_t i = 0; i < tower.size(); i++)
            if ((tower[i]->position - towerStart[i]).magnitude() > 8.0f)
                n++;
        return n;
    }

    int towerSize() const { return (int)tower.size(); }
    int stage1Down() const { return fallen(row1); }
    int stage2Down() const { return fallen(row2); }

protected:
    void build() override
    {
        row1.clear(); row2.clear(); tower.clear(); towerStart.clear();
        const Color wood = rgb(110, 80, 55);

        box({960, 40}, {930, 30}, 0.0f)->c = rgb(70, 75, 70);

        // stage 1: long platform on the left
        box({440, 790}, {340, 10}, 0.0f)->c = wood; // top at y = 800, x 100..780
        for (int i = 0; i < 31; i++)
        {
            RigidBody *d = domino({140.0f + i * 20.0f, 800.0f + 24.5f}, i % 2 ? 0.0f : 0.0f);
            d->c = hsv(0.55f + i * 0.01f, 0.7f, 0.95f);
            row1.push_back(d);
        }
        // the ball sits at the end of the platform, in reach of the last domino (which is 48 px tall)
        RigidBody *b = ball({766, 800 + 16.5f}, 16, 2.5f); // light, and right at the lip of the ramp
        b->c = rgb(230, 70, 60);
        b->restitution = 0.2f;

        // ramp down to stage 2 (ends slightly inside the platform so the ball rolls over the joint)
        segment({780, 789}, {1160, 552}, 5)->c = wood; // top surface starts ~4 px below platform A and ends ~2 px below platform B

        // stage 2: lower platform with a second row, then the tower
        box({1425, 550}, {275, 10}, 0.0f)->c = wood; // top at y = 560, x 1150..1700
        for (int i = 0; i < 14; i++)
        {
            RigidBody *d = domino({1215.0f + i * 20.0f, 560.0f + 24.5f}, 0.0f);
            d->c = hsv(0.08f + i * 0.012f, 0.8f, 0.95f);
            row2.push_back(d);
        }
        // brick tower right behind the last domino (it is 48 px tall, so the tower starts within its reach).
        // Running bond with half bricks at the ends, like the Wrecking Ball building: every brick's centre
        // of mass sits well inside the brick(s) below it.
        const float bw = 14.0f, bh = 9.0f, left = 1500.0f;
        for (int row = 0; row < 5; row++)
        {
            const float y = 560.0f + bh + 0.5f + row * (2 * bh + 0.4f);
            float x = left;
            const int count = (row % 2) ? 4 : 3;
            for (int i = 0; i < count; i++)
            {
                const bool half = (row % 2) && (i == 0 || i == count - 1);
                const float w = half ? bw * 0.5f : bw;
                RigidBody *br = box({x + w, y}, {w - 0.3f, bh}, half ? 2.0f : 4.0f); // light bricks (mass 0.25)
                br->staticFriction = 0.2f;
                br->dynamicFriction = 0.15f;
                br->restitution = 0.0f;
                br->c = hsv(0.0f + 0.02f * (row % 3), 0.65f, 0.7f + 0.03f * row);
                tower.push_back(br);
                towerStart.push_back(br->position);
                x += 2 * w;
            }
        }
    }

private:
    RigidBody *domino(Vector2 pos, float angle)
    {
        RigidBody *d = box(pos, {4.0f, 24.0f}, 1.0f, angle);
        d->staticFriction = 0.6f;
        d->dynamicFriction = 0.4f;
        d->restitution = 0.0f;
        return d;
    }

    std::vector<RigidBody *> row1, row2, tower;
    std::vector<Vector2> towerStart;
};
