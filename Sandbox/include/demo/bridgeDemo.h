#pragma once
#include "demo.h"
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <AccelEngine/ForceRegistry.h>
#include <AccelEngine/ForceGenerator.h>
#include <vector>
#include <cmath>
#include "UI.h"

class BridgeDemo : public Demo
{
public:
    const char *getName() const override { return "Bridge Demo"; }

    void init(
        World &world,
        std::vector<RigidBody *> &bodies,
        ForceRegistry &registry,
        ForceGenerator *gravity) override
    {
        world.joints.clear();
        world.joints.clear();
        worldRef = &world;
        bodiesRef = &bodies;
        registryRef = &registry;
        gravityRef = gravity;

        buildBridge();
    }

    void drawImGui() override
    {
        ImGui::Begin("Bridge Controls");

        ImGui::SliderInt("Planks", &plankCount, 2, 60);
        ImGui::SliderFloat("Plank Width", &plankWidth, 20.0f, 120.0f);
        ImGui::SliderFloat("Plank Height", &plankHeight, 10.0f, 60.0f);
        ImGui::SliderFloat("Spacing", &spacing, 20.0f, 150.0f);

        ImGui::Separator();
        ImGui::Text("Spring");

        ImGui::SliderFloat("Stiffness", &springK, 10.0f, 9000.0f);
        ImGui::SliderFloat("Damping", &springDamping, 0.0f, 500.0f);
        ImGui::SliderFloat("Rest Multiplier", &restMultiplier, 0.2f, 2.0f);

        ImGui::Separator();
        ImGui::SliderFloat("Anchor Offset", &anchorOffset, 0.0f, 150.0f);

        if (ImGui::Button("Rebuild Bridge"))
            rebuildRequested = true;

        ImGui::End();

        if (rebuildRequested)
        {
            rebuildRequested = false;
            buildBridge();
        }
    }

private:
    void buildBridge()
    {
        // clear previous
        worldRef->clear();
        bodiesRef->clear();
        registryRef->clear();

        // ground Y and create left ground
        const float groundY = 100.0f;
        RigidBody *ground1 = makeAABB(
            *worldRef, *bodiesRef, *registryRef, gravityRef,
            {0.0f, groundY}, {300.0f, 30.0f}, 0.0f, 0.0f,
            SDL_Color{96, 132, 171, 255});

        // compute left edge world-x
        const float leftEdgeX = ground1->position.x + ground1->aabb.halfSize.x;

        // total span between inner edges should be spacing * (plankCount + 1)
        const float totalInnerSpan = spacing * (plankCount + 1);

        // create right ground and place it so the inner span between edges equals totalInnerSpan
        RigidBody *ground2 = makeAABB(
            *worldRef, *bodiesRef, *registryRef, gravityRef,
            {leftEdgeX + totalInnerSpan + 300.0f, groundY}, // temporary x, we'll correct next
            {300.0f, 30.0f}, 0.0f, 0.0f,
            SDL_Color{96, 132, 171, 255});
        // reposition ground2 so its left edge is exactly leftEdgeX + totalInnerSpan
        ground2->position.x = leftEdgeX + totalInnerSpan + ground2->aabb.halfSize.x;

        // first plank center x = leftEdge + spacing
        const float firstPlankCenterX = leftEdgeX + spacing;
        const float bridgeY = 250.0f;

        std::vector<RigidBody *> planks;
        planks.reserve(plankCount);

        // create planks (AABB) centered using uniform spacing
        for (int i = 0; i < plankCount; ++i)
        {
            RigidBody *plank = new RigidBody();
            plank->shapeType = ShapeType::AABB;
            plank->aabb.halfSize = {plankWidth * 0.5f, plankHeight * 0.5f};
            plank->position = {firstPlankCenterX + i * spacing, bridgeY};
            plank->inverseMass = 1.0f;
            plank->linearDamping = 0.96f;
            plank->angularDamping = 0.99f;
            plank->inverseInertia = 1.0f / 50.0f;
            plank->c = {180, 120, 80, 255};
            plank->calculateDerivativeData();

            worldRef->addBody(plank);
            bodiesRef->push_back(plank);
            registryRef->add(plank, gravityRef);

            planks.push_back(plank);
        }

        // helper to connect two bodies using local attachment points (aLocal on A, bLocal on B)
        auto connectEdgeSpring = [&](RigidBody *A, const Vector2 &aLocal,
                                     RigidBody *B, const Vector2 &bLocal)
        {
            // world points of the attachment:
            Vector2 pA = A->position + aLocal;
            Vector2 pB = B->position + bLocal;

            // rest length should be the current world distance (so spring starts relaxed)
            float rest = 30.0f;

            Spring *s = new Spring(aLocal, B, bLocal, springK, rest);
            s->damping = springDamping;

            registryRef->add(A, s);
            registryRef->add(B, s);
        };

        // connect plank-to-plank by their adjacent edges (right edge of A to left edge of B)
        for (int i = 0; i < plankCount - 1; ++i)
        {
            RigidBody *A = planks[i];
            RigidBody *B = planks[i + 1];
            Vector2 aLocal = {A->aabb.halfSize.x, 0.0f};  // right edge of A
            Vector2 bLocal = {-B->aabb.halfSize.x, 0.0f}; // left edge of B
            connectEdgeSpring(A, aLocal, B, bLocal);
        }

        // left-most plank to ground1: connect plank left-edge → ground1 inner edge minus anchorOffset
        {
            RigidBody *p = planks.front();
            Vector2 aLocal = {-p->aabb.halfSize.x, 0.0f};
            Vector2 bLocal = {ground1->aabb.halfSize.x - anchorOffset, 0.0f};
            connectEdgeSpring(p, aLocal, ground1, bLocal);
        }

        // right-most plank to ground2: plank right-edge → ground2 inner edge plus anchorOffset
        {
            RigidBody *p = planks.back();
            Vector2 aLocal = {p->aabb.halfSize.x, 0.0f};
            Vector2 bLocal = {-ground2->aabb.halfSize.x + anchorOffset, 0.0f};
            connectEdgeSpring(p, aLocal, ground2, bLocal);
        }
    }

private:
    // Parameters
    int plankCount = 20;
    float plankWidth = 50.0f;
    float plankHeight = 20.0f;
    float spacing = 60.0f;

    float springK = 3000.0f;
    float springDamping = 20.0f;
    float restMultiplier = 1.0f;
    float anchorOffset = 10.0f;

    bool rebuildRequested = false;

    World *worldRef = nullptr;
    std::vector<RigidBody *> *bodiesRef = nullptr;
    ForceRegistry *registryRef = nullptr;
    ForceGenerator *gravityRef = nullptr;
};
