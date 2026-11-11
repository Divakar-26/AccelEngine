#pragma once
#include "showcase.h"

// A wooden fort with green "guards" inside. Aim with the angle/power sliders (the dotted line shows the
// exact trajectory: it is computed from the same gravity the engine uses) and fire boulders at it.
class SiegeDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Siege"; }

    void drawImGui() override
    {
        ImGui::Begin("Siege", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Guards defeated: %d / %d     Shots: %d", defeated(), (int)guards.size(), shots);
        ImGui::SliderFloat("Angle (deg)", &angleDeg, 5.0f, 80.0f);
        ImGui::SliderFloat("Power (px/s)", &power, 400.0f, 2200.0f);
        ImGui::SliderFloat("Boulder radius", &boulderRadius, 10.0f, 35.0f);
        if (ImGui::Button("FIRE!", ImVec2(220, 40)))
            fire();
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("A guard is defeated when it is knocked more than 60 px from its post or falls to the ground. "
            "Boulders are heavy: bigger ones hit harder.");
        ImGui::End();

        drawTrajectory();
    }

    void fire()
    {
        const float a = angleDeg * 3.14159265f / 180.0f;
        RigidBody *b = ball(cannon, boulderRadius, 1.0f / (boulderRadius * boulderRadius * 0.05f)); // mass ~ area
        b->velocity = Vector2(std::cos(a), std::sin(a)) * power;
        b->restitution = 0.2f;
        b->c = rgb(70, 70, 75);
        shots++;
        // forget old boulders so the scene stays small
        boulders.push_back(b);
        if (boulders.size() > 12)
        {
            RigidBody *old = boulders.front();
            boulders.erase(boulders.begin());
            worldRef->removeBody(old); // unlinks it (it is not World-owned: made via makeCircle)
            registryRef->remove(old);  // Sandbox-side bookkeeping
            bodiesRef->erase(std::remove(bodiesRef->begin(), bodiesRef->end(), old), bodiesRef->end());
            delete old;
        }
    }

    int defeated() const
    {
        int n = 0;
        for (size_t i = 0; i < guards.size(); i++)
            if ((guards[i]->position - guardStart[i]).magnitude() > 60.0f || guards[i]->position.y < 75.0f)
                n++;
        return n;
    }

    int guardCount() const { return (int)guards.size(); }
    void setAim(float deg, float p) { angleDeg = deg; power = p; }

protected:
    void build() override
    {
        guards.clear(); guardStart.clear(); boulders.clear();
        shots = 0;
        box({960, 40}, {930, 30}, 0.0f)->c = rgb(80, 120, 70);
        // little catapult base
        box({cannon.x, 80}, {40, 10}, 0.0f)->c = rgb(110, 80, 55);

        const Color wood = rgb(160, 115, 70), plank = rgb(135, 95, 55);
        // fort: three towers joined by beams; guards sit on the floors
        buildTower(1250, 4, wood, plank);
        buildTower(1450, 6, wood, plank);
        buildTower(1650, 3, wood, plank);
    }

private:
    void buildTower(float x, int floorsCount, Color wood, Color plank)
    {
        float top = 70.0f;
        for (int f = 0; f < floorsCount; f++)
        {
            const float postH = 40.0f;
            RigidBody *l = box({x - 45, top + postH + 0.5f}, {7, postH}, 1.0f);
            RigidBody *r = box({x + 45, top + postH + 0.5f}, {7, postH}, 1.0f);
            const float beamY = top + 2 * postH + 1.0f + 6.0f;
            RigidBody *beam = box({x, beamY}, {60, 6}, 1.0f);
            l->c = r->c = wood;
            beam->c = plank;
            for (RigidBody *p : {l, r, beam})
            {
                p->staticFriction = 0.7f;
                p->dynamicFriction = 0.5f;
                p->restitution = 0.0f;
            }
            if (f % 2 == 0 || f == floorsCount - 1)
            {
                RigidBody *g = ball({x + frand(-20, 20), beamY + 6.0f + 11.0f}, 11, 1.2f);
                g->c = rgb(90, 200, 90);
                g->restitution = 0.1f;
                g->staticFriction = 0.9f;
                g->dynamicFriction = 0.8f;
                guards.push_back(g);
                guardStart.push_back(g->position);
            }
            top = beamY + 6.0f;
        }
    }

    void drawTrajectory()
    {
        const float a = angleDeg * 3.14159265f / 180.0f;
        Vector2 v = Vector2(std::cos(a), std::sin(a)) * power;
        float gy = -980.0f;
        if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
            gy = g->gravity.y;
        ImDrawList *dl = ImGui::GetBackgroundDrawList();
        for (float t = 0.0f; t < 4.0f; t += 0.05f)
        {
            const Vector2 p = cannon + v * t + Vector2(0, 0.5f * gy * t * t);
            if (p.y < 70.0f)
                break;
            SDL_FPoint s = Renderer2D::worldToScreen(p.x, p.y);
            dl->AddCircleFilled(ImVec2(s.x, s.y), 3.0f, IM_COL32(255, 255, 255, 200));
        }
    }

    std::vector<RigidBody *> guards, boulders;
    std::vector<Vector2> guardStart;
    Vector2 cannon{140.0f, 130.0f};
    float angleDeg = 38.0f, power = 1350.0f, boulderRadius = 20.0f;
    int shots = 0;
};
