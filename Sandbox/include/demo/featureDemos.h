#pragma once
// Demos for the newer engine features: polygons / capsules / compound shapes, hinge / motor / wheel joints,
// collision layers, sensors and one-way platforms. Bodies and joints are created through World::createX, so the
// World owns them (World::clear() frees everything); the `bodies` list is only used for drawing and picking.
#include "demo.h"
#include "renderer2D.h"
#include <imgui.h>
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class FeatureDemo : public Demo
{
public:
    void init(World &world, std::vector<RigidBody *> &bodies, ForceRegistry &registry, ForceGenerator *gravity) override
    {
        worldRef = &world;
        bodiesRef = &bodies;
        registryRef = &registry;
        gravityRef = gravity;
        if (Gravity *g = dynamic_cast<Gravity *>(gravity))
            g->gravity = Vector2(0.0f, -980.0f);
        world.joints.clear();
        build();
    }

    void update() override
    {
        if (resetRequested)
        {
            resetRequested = false;
            worldRef->clear(); // frees the owned bodies and joints
            bodiesRef->clear();
            registryRef->clear();
            build();
        }
        step();
    }

protected:
    virtual void build() = 0;
    virtual void step() {}
    void requestReset() { resetRequested = true; }

    static Color rgb(float r, float g, float b) { return {r, g, b, 255.0f}; }
    static Color hsv(float h, float s, float v)
    {
        h = h - std::floor(h);
        const float i = std::floor(h * 6.0f), f = h * 6.0f - i;
        const float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
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

    static void tip(const char *text)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.65f, 0.65f, 1.0f));
        ImGui::TextWrapped("%s", text);
        ImGui::PopStyleColor();
    }

    BodyDef def(Vector2 pos, float mass, Color c, float angle = 0.0f)
    {
        BodyDef d;
        d.position = pos;
        d.mass = mass;
        d.color = c;
        d.angle = angle;
        d.restitution = 0.15f;
        return d;
    }

    // Registers a body for drawing, and for gravity if it is dynamic.
    RigidBody *track(RigidBody *b)
    {
        if (!b)
            return nullptr;
        bodiesRef->push_back(b);
        if (b->inverseMass > 0.0f && gravityRef)
            registryRef->add(b, gravityRef);
        return b;
    }

    RigidBody *box(Vector2 pos, Vector2 half, float mass, Color c, float angle = 0.0f)
    {
        return track(worldRef->createBox(half, def(pos, mass, c, angle)));
    }
    RigidBody *ball(Vector2 pos, float r, float mass, Color c)
    {
        return track(worldRef->createCircle(r, def(pos, mass, c)));
    }
    RigidBody *capsule(Vector2 pos, float halfLength, float r, float mass, Color c, float angle = 0.0f)
    {
        return track(worldRef->createCapsule(halfLength, r, def(pos, mass, c, angle)));
    }
    RigidBody *polygon(Vector2 pos, const std::vector<Vector2> &pts, float mass, Color c, float angle = 0.0f)
    {
        return track(worldRef->createPolygon(pts, def(pos, mass, c, angle)));
    }
    RigidBody *ngon(Vector2 pos, int sides, float r, float mass, Color c, float angle = 0.0f)
    {
        return track(worldRef->createRegularPolygon(sides, r, def(pos, mass, c, angle)));
    }
    RigidBody *compound(Vector2 pos, const std::vector<ConvexPart> &parts, float mass, Color c, float angle = 0.0f)
    {
        return track(worldRef->createCompound(parts, def(pos, mass, c, angle)));
    }

    // 1920x1000 room: floor top at y = 60, walls at both sides.
    void room(float left = 0.0f, float right = 1920.0f)
    {
        const Color stone = rgb(88, 94, 104);
        box({(left + right) * 0.5f, 30}, {(right - left) * 0.5f + 40, 30}, 0.0f, stone);
        box({left - 20, 500}, {20, 600}, 0.0f, stone);
        box({right + 20, 500}, {20, 600}, 0.0f, stone);
    }

    World *worldRef = nullptr;
    std::vector<RigidBody *> *bodiesRef = nullptr;
    ForceRegistry *registryRef = nullptr;
    ForceGenerator *gravityRef = nullptr;
    bool resetRequested = false;
};

// ================================================================================================================
// Shape gallery: polygons, capsules and compound shapes
// ================================================================================================================
class ShapesDemo : public FeatureDemo
{
public:
    const char *getName() const override { return "Shapes: polygons, capsules, compounds"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Shapes", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Bodies: %zu", bodiesRef->size());
        ImGui::Checkbox("Rain", &rain);
        const char *names[] = {"Triangle", "Pentagon", "Hexagon", "Capsule", "L shape", "T shape", "Dumbbell", "Plus", "Wheel"};
        for (int i = 0; i < 9; i++)
        {
            if (ImGui::Button(names[i], ImVec2(100, 0)))
                spawn(i, {frand(500, 1400), 880});
            if (i % 3 != 2)
                ImGui::SameLine();
        }
        if (ImGui::Button("Drop 30 random"))
            for (int i = 0; i < 30; i++)
                spawn(rand() % 9, {frand(250, 1650), frand(700, 1500)});
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Everything is built from convex parts. Compound shapes (L, T, dumbbell, plus, wheel) are several parts on one rigid body.");
        ImGui::End();
    }

protected:
    void build() override
    {
        room();
        // a ramp and a pit to roll around in
        box({420, 150}, {170, 12}, 0.0f, rgb(88, 94, 104), -0.35f);
        box({1500, 150}, {170, 12}, 0.0f, rgb(88, 94, 104), 0.35f);
        for (int i = 0; i < 18; i++)
            spawn(i % 9, {300.0f + 100.0f * (i % 9), 300.0f + 150.0f * (i / 9)});
        rainTimer = 0.0f;
    }

    void step() override
    {
        if (!rain)
            return;
        rainTimer += 1.0f / 360.0f;
        if (rainTimer > 0.25f && bodiesRef->size() < 600)
        {
            rainTimer = 0.0f;
            spawn(rand() % 9, {frand(300, 1600), 1000});
        }
    }

private:
    void spawn(int kind, Vector2 p)
    {
        const Color c = hsv(frand(0, 1), 0.55f, 0.95f);
        const float a = frand(0, 6.28f);
        switch (kind)
        {
        case 0: ngon(p, 3, 34, 2.0f, c, a); break;
        case 1: ngon(p, 5, 32, 2.5f, c, a); break;
        case 2: ngon(p, 6, 30, 2.5f, c, a); break;
        case 3: capsule(p, 36, 12, 2.0f, c, a); break;
        case 4: compound(p, {ConvexPart::box({45, 12}, {0, 0}), ConvexPart::box({12, 33}, {-33, 33})}, 3.0f, c, a); break;
        case 5: compound(p, {ConvexPart::box({50, 11}, {0, 0}), ConvexPart::box({11, 35}, {0, -35})}, 3.0f, c, a); break;
        case 6: compound(p, {ConvexPart::circle({-38, 0}, 17), ConvexPart::circle({38, 0}, 17), ConvexPart::box({38, 5})}, 3.0f, c, a); break;
        case 7: compound(p, {ConvexPart::box({44, 12}), ConvexPart::box({12, 44})}, 3.0f, c, a); break;
        default:
        {
            std::vector<ConvexPart> parts{ConvexPart::circle({0, 0}, 14)};
            for (int i = 0; i < 4; i++)
            {
                const float t = i * 0.7853982f;
                parts.push_back(ConvexPart::box({34, 5}, {0, 0}, t));
            }
            compound(p, parts, 3.5f, c, a);
            break;
        }
        }
    }

    bool rain = false;
    float rainTimer = 0.0f;
};

// ================================================================================================================
// Car: wheel joints (suspension + motor) on a compound chassis
// ================================================================================================================
class CarDemo : public FeatureDemo
{
public:
    const char *getName() const override { return "Car: wheel joints"; }

    void drawImGui() override
    {
        ImGui::Begin("Car", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Hold L / J to drive forward / back, K to brake");
        ImGui::SliderFloat("Throttle (slider)", &throttle, -1.0f, 1.0f);
        ImGui::SliderFloat("Top speed (rad/s)", &topSpeed, 2.0f, 40.0f);
        ImGui::SliderFloat("Motor torque", &torque, 5000.0f, 200000.0f, "%.0f");
        bool changed = ImGui::SliderFloat("Suspension (Hz)", &hertz, 0.5f, 10.0f);
        changed |= ImGui::SliderFloat("Damping ratio", &damping, 0.0f, 2.0f);
        if (changed)
            for (WheelJoint *w : wheels)
                w->setSpring(hertz, damping);
        if (!wheels.empty())
            ImGui::Text("Wheel speed %.1f rad/s, suspension %.1f px", wheels[0]->getWheelSpeed(), wheels[0]->getTranslation());
        if (ImGui::Button("Reset"))
            requestReset();
        tip("The wheels are circles bolted to the chassis with WheelJoints: a spring along the suspension axis, a free spin and a motor.");
        ImGui::End();
    }

protected:
    void build() override
    {
        wheels.clear();
        room();
        const Color rock = rgb(98, 88, 78);
        // ramp up, flat top, ramp down, then a field of bumps
        box({330, 120}, {200, 12}, 0.0f, rock, 0.28f);
        box({650, 175}, {150, 12}, 0.0f, rock);
        box({960, 120}, {200, 12}, 0.0f, rock, -0.28f);
        for (int i = 0; i < 5; i++)
            ball({1230.0f + i * 120.0f, 52.0f}, 22.0f + 4.0f * (i % 2), 0.0f, rock);
        // a loose crate pile to smash
        for (int i = 0; i < 4; i++)
            for (int k = 0; k <= i; k++)
                box({1760.0f - 28.0f * i + 56.0f * k, 78.0f + 56.0f * (3 - i)}, {26, 26}, 0.4f, hsv(0.08f + 0.02f * i, 0.6f, 0.9f));

        // chassis: floor plate + cabin
        const Vector2 at(120, 130);
        BodyDef cd = def(at, 7.0f, rgb(220, 70, 60));
        cd.groupIndex = -1; // wheels (same group) never collide with the chassis
        chassis = track(worldRef->createCompound(
            {ConvexPart::box({62, 9}, {0, 0}),
             ConvexPart::polygon(std::vector<Vector2>{{-34, 9}, {44, 9}, {26, 34}, {-22, 34}}.data(), 4)},
            cd));
        chassis->canSleep = false;
        for (int i = 0; i < 2; i++)
        {
            BodyDef wd = def(at + Vector2(i == 0 ? -42.0f : 42.0f, -22.0f), 1.2f, rgb(40, 42, 50));
            wd.staticFriction = 1.6f;
            wd.dynamicFriction = 1.3f;
            wd.restitution = 0.0f;
            wd.groupIndex = -1;
            RigidBody *w = track(worldRef->createCircle(19, wd));
            w->canSleep = false;
            WheelJoint *wj = worldRef->createWheelJoint(chassis, w, w->position, {0, 1});
            wj->setSpring(hertz, damping);
            wj->setLimits(-14, 14);
            wheels.push_back(wj);
        }
    }

    void step() override
    {
        float drive = throttle;
        const bool *keys = SDL_GetKeyboardState(nullptr);
        if (!ImGui::GetIO().WantTextInput)
        {
            if (keys[SDL_SCANCODE_L]) drive = 1.0f;
            if (keys[SDL_SCANCODE_J]) drive = -1.0f;
        }
        const bool brake = !ImGui::GetIO().WantTextInput && keys[SDL_SCANCODE_K];
        for (WheelJoint *w : wheels)
        {
            // positive wheel spin is counter-clockwise, which drives the car left; negate for "forward = right"
            w->setMotor(-drive * topSpeed, brake ? torque * 2.0f : (std::fabs(drive) > 0.01f ? torque : torque * 0.05f));
            if (brake)
                w->setMotor(0.0f, torque * 2.0f);
        }
        if (std::fabs(drive) > 0.01f || brake)
            chassis->wakeUp();
    }

private:
    RigidBody *chassis = nullptr;
    std::vector<WheelJoint *> wheels;
    float throttle = 0.0f, topSpeed = 14.0f, torque = 60000.0f, hertz = 4.0f, damping = 0.7f;
};

// ================================================================================================================
// Ragdolls, hinges and a motorised windmill
// ================================================================================================================
class RagdollDemo : public FeatureDemo
{
public:
    const char *getName() const override { return "Hinges: ragdolls and motors"; }

    void drawImGui() override
    {
        ImGui::Begin("Hinges", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        if (ImGui::Button("Drop a ragdoll"))
            ragdoll({frand(300, 1500), 900});
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        if (ImGui::SliderFloat("Windmill speed (rad/s)", &windSpeed, -4.0f, 4.0f) && windmill)
        {
            windmillJoint->setMotor(windSpeed, 4e7f);
            windmill->wakeUp();
        }
        tip("Ragdoll joints are RevoluteJoints with angle limits (shown as the red wedges). The windmill hub is a hinge driven by a motor.");
        ImGui::End();
    }

protected:
    void build() override
    {
        room();
        const Color stone = rgb(88, 94, 104);
        box({560, 330}, {220, 12}, 0.0f, stone, -0.12f);
        box({1420, 250}, {260, 12}, 0.0f, stone, 0.10f);

        // windmill: a plus-shaped body on a hinge fixed to the world
        RigidBody *hub = box({1000, 330}, {8, 8}, 0.0f, stone);
        windmill = compound({1000, 330}, {ConvexPart::box({130, 9}), ConvexPart::box({9, 130})}, 8.0f, rgb(230, 170, 60));
        windmill->canSleep = false;
        windmillJoint = worldRef->createRevoluteJoint(hub, windmill, {1000, 330});
        windmillJoint->setMotor(windSpeed, 4e7f);
        // pendulum
        RigidBody *pin = box({1700, 760}, {8, 8}, 0.0f, stone);
        RigidBody *weight = ball({1700, 560}, 34, 12.0f, rgb(120, 130, 220));
        RigidBody *rod = box({1700, 660}, {5, 100}, 1.0f, rgb(200, 200, 205));
        rod->groupIndex = -2;
        weight->groupIndex = -2;
        worldRef->createRevoluteJoint(pin, rod, {1700, 760});
        worldRef->createRevoluteJoint(rod, weight, {1700, 560});
        weight->velocity = {-500, 0};

        for (int i = 0; i < 4; i++)
            ragdoll({400.0f + 280.0f * i, 700.0f + 90.0f * (i % 2)});
    }

private:
    RigidBody *limb(Vector2 from, float angle, float length, float radius, float mass, Color c, int group)
    {
        const Vector2 dir(std::cos(angle), std::sin(angle));
        BodyDef d = def(from + dir * (length * 0.5f), mass, c, angle);
        d.groupIndex = (int16_t)group;
        d.restitution = 0.0f;
        d.staticFriction = 0.8f;
        d.dynamicFriction = 0.6f;
        return track(worldRef->createCapsule(std::max(length * 0.5f - radius, 1.0f), radius, d));
    }

    RevoluteJoint *hinge(RigidBody *a, RigidBody *b, Vector2 p, float lo, float hi)
    {
        RevoluteJoint *j = worldRef->createRevoluteJoint(a, b, p);
        j->setLimits(lo, hi);
        return j;
    }

    void ragdoll(Vector2 p)
    {
        const int group = -(++ragdollCount) - 10; // negative: the limbs of one ragdoll never collide
        const Color skin = hsv(frand(0, 1), 0.35f, 0.95f), cloth = hsv(frand(0, 1), 0.6f, 0.85f);
        const float down = -1.5707963f;

        BodyDef td = def(p, 4.0f, cloth);
        td.groupIndex = (int16_t)group;
        td.restitution = 0.0f;
        RigidBody *torso = track(worldRef->createBox({13, 27}, td));
        BodyDef hd = def(p + Vector2(0, 45), 1.0f, skin);
        hd.groupIndex = (int16_t)group;
        hd.restitution = 0.0f;
        RigidBody *head = track(worldRef->createCircle(14, hd));
        hinge(torso, head, p + Vector2(0, 28), -0.6f, 0.6f);

        for (int side = -1; side <= 1; side += 2)
        {
            // arms hang down and out a little
            const Vector2 shoulder = p + Vector2(side * 14.0f, 22.0f);
            const float armAngle = down + side * 0.25f;
            RigidBody *upperArm = limb(shoulder, armAngle, 30, 5.5f, 0.8f, skin, group);
            hinge(torso, upperArm, shoulder, -2.4f, 2.4f);
            const Vector2 elbow = shoulder + Vector2(std::cos(armAngle), std::sin(armAngle)) * 30.0f;
            RigidBody *foreArm = limb(elbow, armAngle, 28, 5.0f, 0.6f, skin, group);
            hinge(upperArm, foreArm, elbow, 0.0f, 2.3f);

            const Vector2 hip = p + Vector2(side * 7.0f, -27.0f);
            RigidBody *thigh = limb(hip, down, 36, 7.0f, 1.4f, cloth, group);
            hinge(torso, thigh, hip, -1.6f, 1.6f);
            const Vector2 knee = hip + Vector2(0, -36);
            RigidBody *shin = limb(knee, down, 36, 6.0f, 1.0f, skin, group);
            hinge(thigh, shin, knee, -2.3f, 0.0f);
        }
    }

    RigidBody *windmill = nullptr;
    RevoluteJoint *windmillJoint = nullptr;
    float windSpeed = 1.6f;
    int ragdollCount = 0;
};

// ================================================================================================================
// Collision layers, sensors and one-way platforms
// ================================================================================================================
class FilterDemo : public FeatureDemo
{
public:
    const char *getName() const override { return "Filters: layers, sensors, one-way"; }

    void drawImGui() override
    {
        ImGui::Begin("Filters", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Checkbox("Spawn red + blue balls", &spawning);
        if (ImGui::Button("Jump"))
            for (RigidBody *j : jumpers)
                j->applyImpulse({0, 900.0f / j->inverseMass});
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        ImGui::Text("In the updraft sensor: %d", inUpdraft);
        ImGui::Text("In the score bucket: %d   (total %d)", inBucket, scored);
        tip("Each colour passes through its own gate and rests on the other (collision categories and masks). "
            "The yellow-edged platforms are one-way: capsules jump up through them and land on top. "
            "The green zone is a sensor that blows bodies upward; the bucket counts what falls in without stopping it.");
        ImGui::End();
    }

    void drawWorld() override
    {
        Renderer2D::DrawText("RED GATE", {330, 560}, 16, {255, 120, 120, 230});
        Renderer2D::DrawText("BLUE GATE", {330, 440}, 16, {130, 170, 255, 230});
        Renderer2D::DrawText("UPDRAFT", {1010, 70}, 16, {150, 255, 170, 230});
        Renderer2D::DrawText("BUCKET: " + std::to_string(inBucket), {1560, 100}, 18, {255, 255, 255, 230});
    }

protected:
    enum : uint16_t { GROUND = 1, RED = 2, BLUE = 4, REDGATE = 8, BLUEGATE = 16, CAPS = 32 };

    void build() override
    {
        jumpers.clear();
        inUpdraft = inBucket = 0;
        room();
        const Color stone = rgb(88, 94, 104);

        // gates: a slanted floor with a gap-filling pane. REDGATE blocks only red, BLUEGATE only blue.
        gate({330, 560}, {130, 7}, rgb(220, 70, 70), REDGATE, 0.12f);
        gate({330, 440}, {130, 7}, rgb(80, 120, 230), BLUEGATE, -0.12f);

        // one-way platforms
        for (int i = 0; i < 3; i++)
        {
            BodyDef d = def({900.0f + 150.0f * i, 240.0f + 130.0f * i}, 0.0f, rgb(150, 120, 70));
            d.oneWay = true;
            d.categoryBits = GROUND;
            track(worldRef->createBox({100, 8}, d));
        }
        // jumpers: capsules that hop up through the platforms
        for (int i = 0; i < 3; i++)
        {
            BodyDef d = def({900.0f + 150.0f * i, 300.0f + 130.0f * i}, 1.5f, hsv(0.1f + 0.15f * i, 0.5f, 0.95f));
            d.categoryBits = CAPS;
            d.restitution = 0.0f;
            RigidBody *c = track(worldRef->createCapsule(14, 14, d));
            c->inverseInertia = 0.0f; // stays upright
            jumpers.push_back(c);
        }

        // updraft sensor
        {
            BodyDef d = def({1060, 200}, 0.0f, rgb(60, 220, 110));
            d.isSensor = true;
            d.categoryBits = GROUND;
            updraft = track(worldRef->createBox({55, 140}, d));
        }
        // score bucket: sensor between two low walls
        box({1480, 70}, {8, 40}, 0.0f, stone);
        box({1680, 70}, {8, 40}, 0.0f, stone);
        {
            BodyDef d = def({1580, 70}, 0.0f, rgb(250, 210, 60));
            d.isSensor = true;
            d.categoryBits = GROUND;
            bucket = track(worldRef->createBox({92, 36}, d));
        }
        // slide into the bucket
        box({1500, 480}, {150, 8}, 0.0f, stone, -0.25f);
        timer = 0.0f;
    }

    void step() override
    {
        // sensor events are those of the previous substep, which is exactly "who overlaps right now"
        int up = 0, buck = 0;
        for (const CollisionEvent &e : worldRef->getSensorEvents())
        {
            RigidBody *s = e.a->isSensor ? e.a : e.b;
            RigidBody *other = e.a->isSensor ? e.b : e.a;
            if (s == updraft)
            {
                up++;
                if (!other->isStatic())
                    other->addForce({0, 1.0f / other->inverseMass * 1600.0f}); // lifts a ~1.6 g wind
            }
            else if (s == bucket)
                buck++;
        }
        inUpdraft = up;
        if (buck > inBucket)
            scored += buck - inBucket;
        inBucket = buck;

        if (spawning)
        {
            timer += 1.0f / 360.0f;
            if (timer > 0.35f && bodiesRef->size() < 260)
            {
                timer = 0.0f;
                const bool red = rand() & 1;
                BodyDef d = def({frand(220, 440), 980}, 1.0f, red ? rgb(230, 70, 70) : rgb(70, 110, 235));
                d.categoryBits = red ? RED : BLUE;
                // red balls hit ground + the blue gate... (gate categories are what they pass / hit)
                d.maskBits = GROUND | (red ? BLUEGATE | RED : REDGATE | BLUE);
                d.restitution = 0.35f;
                track(worldRef->createCircle(frand(9, 15), d));
            }
        }
    }

private:
    void gate(Vector2 p, Vector2 half, Color c, uint16_t category, float angle)
    {
        BodyDef d = def(p, 0.0f, c, angle);
        d.categoryBits = category;
        track(worldRef->createBox(half, d));
    }

    RigidBody *updraft = nullptr, *bucket = nullptr;
    std::vector<RigidBody *> jumpers;
    bool spawning = true;
    float timer = 0.0f;
    int inUpdraft = 0, inBucket = 0, scored = 0;
};
