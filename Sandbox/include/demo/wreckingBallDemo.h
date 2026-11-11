#pragma once
#include "showcase.h"

// A brick building (stable stacks that sleep until hit) and a 50x-heavier wrecking ball on a jointed chain.
// Release the ball, or fire cannonballs, and watch the wake-up cascade.
class WreckingBallDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Wrecking Ball"; }

    void drawImGui() override
    {
        ImGui::Begin("Wrecking Ball", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Bricks: %d", brickCount);
        if (ImGui::Button(released ? "Ball released" : "Release the ball!", ImVec2(220, 40)) && !released)
            releaseBall();
        if (ImGui::Button("Fire cannonball", ImVec2(220, 30)))
            fireCannonball();
        ImGui::SliderInt("Building floors", &floors, 10, 44);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("Bricks rest asleep (dimmed) until something hits them; the impact wakes only the island it touches.");
        ImGui::End();
    }

    void releaseBall()
    {
        if (released)
            return;
        released = true;
        wreckingBall->lockPosition = false;
        wreckingBall->lockRotation = false;
        wreckingBall->wakeUp();
        for (RigidBody *l : chain)
            l->wakeUp();
    }

    void fireCannonball()
    {
        RigidBody *c = ball({60, 260}, 16, 0.2f); // mass 5
        c->velocity = Vector2(1900, 380);
        c->restitution = 0.1f;
        c->c = rgb(40, 40, 45);
    }

    RigidBody *getBall() const { return wreckingBall; }

protected:
    void build() override
    {
        released = false;
        chain.clear();

        box({960, 40}, {930, 30}, 0.0f)->c = rgb(85, 90, 80);

        // ---- building: running-bond brick walls, two towers ----
        brickCount = 0;
        buildTower(1060, 12, floors);                     // x 1060..1540
        buildTower(1600, 7, std::max(6, floors * 2 / 3)); // x 1600..1880

        // ---- wrecking ball on a chain, pulled back to the left ----
        const Vector2 anchorPos(980, 990);
        RigidBody *anchor = ball(anchorPos, 8, 0.0f);
        anchor->enableCollision = false;
        anchor->c = rgb(60, 60, 60);

        const int links = 12;
        const float seg = 34.0f;
        const float angle = 72.0f * 3.14159265f / 180.0f; // from vertical
        const Vector2 dir(-std::sin(angle), -std::cos(angle));

        RigidBody *prev = anchor;
        for (int i = 1; i <= links; i++)
        {
            RigidBody *l = ball(anchorPos + dir * (seg * i), 5, 1.0f);
            l->enableCollision = false; // the chain passes through bricks; only the ball hits
            l->linearDamping = 0.85f;    // light damping so the pendulum dies down and the scene can sleep again
            l->c = rgb(150, 150, 155);
            link(prev, l, {0, 0}, {0, 0}, seg);
            chain.push_back(l);
            prev = l;
        }
        const float ballR = 55.0f;
        wreckingBall = ball(anchorPos + dir * (seg * links + ballR), ballR, 0.02f); // mass 50
        wreckingBall->restitution = 0.05f;
        wreckingBall->linearDamping = 0.85f;
        wreckingBall->c = rgb(35, 35, 40);
        wreckingBall->lockPosition = true; // held until released
        wreckingBall->lockRotation = true;
        link(prev, wreckingBall, {0, 0}, {0, 0}, ballR + 2.0f);
    }

private:
    void buildTower(float left, int bricksWide, int rows)
    {
        const float bw = 20.0f, bh = 10.0f; // half sizes
        const float gap = 0.4f;
        for (int row = 0; row < rows; row++)
        {
            const float y = 70 + bh + 0.5f + row * (bh * 2 + gap);
            const bool offset = row % 2;
            float x = left;
            // offset rows start and end with a half brick
            const int count = offset ? bricksWide + 1 : bricksWide;
            for (int i = 0; i < count; i++)
            {
                const bool half = offset && (i == 0 || i == count - 1);
                const float w = half ? bw * 0.5f : bw;
                RigidBody *b = box({x + w, y}, {w - gap * 0.5f, bh}, half ? 2.0f : 1.0f);
                b->staticFriction = 0.8f;
                b->dynamicFriction = 0.6f;
                b->restitution = 0.0f;
                const float shade = frand(0.0f, 1.0f);
                b->c = hsv(0.02f + 0.03f * shade, 0.65f + 0.15f * shade, 0.62f + 0.25f * shade);
                x += 2 * w;
                brickCount++;
            }
        }
    }

    RigidBody *wreckingBall = nullptr;
    std::vector<RigidBody *> chain;
    bool released = false;
    int floors = 34;
    int brickCount = 0;
};
