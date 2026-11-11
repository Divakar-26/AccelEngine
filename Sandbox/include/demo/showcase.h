#pragma once
// Shared helpers for the large "showcase" demos (Dam Break, Galton Board, Wrecking Ball, ...).
#include "demo.h"
#include "renderer2D.h"
#include "imgui.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <vector>

class ShowcaseDemo : public Demo
{
public:
    void init(World &world, std::vector<RigidBody *> &bodies, ForceRegistry &registry, ForceGenerator *gravity) override
    {
        worldRef = &world;
        bodiesRef = &bodies;
        registryRef = &registry;
        gravityRef = gravity;
        world.joints.clear();
        ownedJoints.clear();
        if (Gravity *g = dynamic_cast<Gravity *>(gravity))
            g->gravity = Vector2(0.0f, -980.0f); // a previous demo may have changed it
        build();
    }

    // Called once per physics substep by Game::update (before world.step).
    void update() override
    {
        if (resetRequested)
        {
            resetRequested = false;
            clearScene();
            if (Gravity *g = dynamic_cast<Gravity *>(gravityRef))
                g->gravity = Vector2(0.0f, -980.0f);
            build();
        }
        step();
    }

protected:
    virtual void build() = 0; // create the scene
    virtual void step() {}    // per-substep logic (motors, emitters, ...)

    // Rebuild on the next substep (safe to call from drawImGui).
    void requestReset() { resetRequested = true; }

    // Removes and frees everything in the scene (the Sandbox has no per-body remove).
    void clearScene()
    {
        worldRef->clear();
        worldRef->joints.clear();
        for (Joint *j : ownedJoints)
            delete j;
        ownedJoints.clear();
        registryRef->clear();
        for (RigidBody *b : *bodiesRef)
            delete b;
        bodiesRef->clear();
    }

    // ---- scene building helpers ----
    RigidBody *box(Vector2 pos, Vector2 half, float invMass, float angle = 0.0f, bool gravity = true)
    {
        return makeAABB(*worldRef, *bodiesRef, *registryRef, gravity ? gravityRef : nullptr, pos, half, invMass, angle);
    }

    RigidBody *ball(Vector2 pos, float radius, float invMass, bool gravity = true)
    {
        return makeCircle(*worldRef, *bodiesRef, *registryRef, gravity ? gravityRef : nullptr, pos, radius, invMass);
    }

    // Static plank between two points.
    RigidBody *segment(Vector2 a, Vector2 b, float halfThickness)
    {
        Vector2 d = b - a;
        float len = d.magnitude();
        return box((a + b) * 0.5f, {len * 0.5f, halfThickness}, 0.0f, std::atan2(d.y, d.x));
    }

    DistanceJoint *link(RigidBody *a, RigidBody *b, Vector2 localA, Vector2 localB, float restLength)
    {
        auto *j = new DistanceJoint(a, b, localA, localB, restLength);
        worldRef->addJoint(j);
        ownedJoints.push_back(j);
        return j;
    }

    static Color rgb(float r, float g, float b) { return {r, g, b, 255.0f}; }

    // h, s, v in [0, 1]
    static Color hsv(float h, float s, float v)
    {
        h = h - std::floor(h);
        float i = std::floor(h * 6.0f), f = h * 6.0f - i;
        float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
        float r, g, b;
        switch ((int)i % 6)
        {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        default: r = v, g = p, b = q; break;
        }
        return {r * 255.0f, g * 255.0f, b * 255.0f, 255.0f};
    }

    static float frand(float lo, float hi) { return lo + (hi - lo) * (float)rand() / (float)RAND_MAX; }

    static Vector2 mouseWorld()
    {
        float sx, sy;
        SDL_GetMouseState(&sx, &sy);
        SDL_FPoint w = Renderer2D::screenToWorld(sx, sy);
        return {w.x, w.y};
    }

    static void tip(const char *text)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.65f, 0.65f, 1.0f));
        ImGui::TextWrapped("%s", text);
        ImGui::PopStyleColor();
    }

    World *worldRef = nullptr;
    std::vector<RigidBody *> *bodiesRef = nullptr;
    ForceRegistry *registryRef = nullptr;
    ForceGenerator *gravityRef = nullptr;
    std::vector<Joint *> ownedJoints;
    bool resetRequested = false;
};
