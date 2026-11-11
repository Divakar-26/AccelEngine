#pragma once
#include "showcase.h"

// Inverse-square pull towards a point (softened so it stays finite near the centre).
class PointGravity : public ForceGenerator
{
public:
    Vector2 center;
    real strength = 1.2e7f; // G*M in px^3/s^2
    real softening = 30.0f;
    bool repel = false;

    void updateForce(RigidBody *body, real) override
    {
        if (body->inverseMass <= 0.0f)
            return;
        const Vector2 d = center - body->position;
        const real r2 = d.squareMagnitude() + softening * softening;
        const real invR = 1.0f / std::sqrt(r2);
        const real mass = 1.0f / body->inverseMass;
        const real f = strength * mass / r2 * (repel ? -1.0f : 1.0f);
        body->forceAccum += d * (f * invR);
    }
};

// Thousands of asteroids in orbit around a planet under a custom force generator (no world gravity).
// They collide, clump and rain onto the planet; colour shows speed. Let the well follow the mouse,
// or flip it to repel.
class GravityWellDemo : public ShowcaseDemo
{
public:
    const char *getName() const override { return "Gravity Well"; }

    void drawImGui() override
    {
        ImGui::Begin("Gravity Well", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Asteroids: %d", (int)asteroids.size());
        ImGui::Checkbox("Black hole follows the mouse", &followMouse);
        ImGui::Checkbox("Repel", &well.repel);
        float s = well.strength / 1e6f;
        if (ImGui::SliderFloat("Strength (x1e6)", &s, 1.0f, 40.0f))
            well.strength = s * 1e6f;
        ImGui::SliderInt("Asteroids (rebuild)", &count, 500, 6000);
        if (ImGui::Button("Rebuild"))
            requestReset();
        tip("No world gravity here: a custom ForceGenerator pulls every body towards one point. "
            "Colour = speed (blue slow, red fast).");
        ImGui::End();

        if (followMouse)
            target = mouseWorld();

        // colour by speed (once per frame)
        for (RigidBody *a : asteroids)
        {
            const float t = std::min(a->velocity.magnitude() / 450.0f, 1.0f);
            a->c = hsv(0.62f - 0.62f * t, 0.85f, 0.55f + 0.45f * t);
        }
    }

    PointGravity &getWell() { return well; }
    const std::vector<RigidBody *> &getAsteroids() const { return asteroids; }

protected:
    void build() override
    {
        asteroids.clear();
        followMouse = false;
        well.center = home;
        target = home;

        planet = ball(home, 45, 0.0f, false); // static
        planet->c = rgb(60, 110, 200);
        planet->staticFriction = 0.9f;
        planet->dynamicFriction = 0.7f;

        // asteroids in a disc with near-circular orbits (v = sqrt(GM / r)), all going the same way
        for (int i = 0; i < count; i++)
        {
            const float r = std::sqrt(frand(0.0f, 1.0f)) * 380.0f + 110.0f;
            const float a = frand(0.0f, 6.2831853f);
            const Vector2 dir(std::cos(a), std::sin(a));
            const Vector2 pos = home + dir * r;
            bool overlapping = false;
            for (size_t k = asteroids.size() > 200 ? asteroids.size() - 200 : 0; k < asteroids.size() && !overlapping; k++)
                overlapping = (asteroids[k]->position - pos).squareMagnitude() < 14.0f * 14.0f;
            if (overlapping)
                continue;

            RigidBody *b = ball(pos, frand(2.5f, 6.0f), 1.0f, false);
            const float v = std::sqrt(well.strength / r) * frand(0.93f, 1.05f);
            b->velocity = dir.perpendicular() * v;
            b->restitution = 0.15f;
            b->staticFriction = 0.4f;
            b->dynamicFriction = 0.3f;
            b->angularDamping = 1.0f;
            b->canSleep = false; // the well can move at any time, and forces do not wake bodies
            registryRef->add(b, &well);
            asteroids.push_back(b);
        }
    }

    void step() override
    {
        // move the well (and the planet with it) smoothly towards the target
        const Vector2 to = followMouse ? target : home;
        well.center += (to - well.center) * 0.08f;
        if (followMouse)
        {
            planet->position = Vector2(-100000, -100000); // the black hole has no surface
        }
        else
        {
            planet->position = well.center;
        }
        planet->calculateDerivativeData();
    }

private:
    PointGravity well;
    RigidBody *planet = nullptr;
    std::vector<RigidBody *> asteroids;
    Vector2 home{960, 520};
    Vector2 target{960, 520};
    bool followMouse = false;
    int count = 2500;
};
