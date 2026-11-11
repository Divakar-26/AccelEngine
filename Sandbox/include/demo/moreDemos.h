#pragma once
// More demos built on FeatureDemo: Pinball, Catapult, Hill Climb, Hinged Bridge, Marble Run, Portals, Tower Smash.
#include "featureDemos.h"
#include <map>

class MoreDemo : public FeatureDemo
{
protected:
    // static plank between two points
    RigidBody *segment(Vector2 a, Vector2 b, float halfThickness, Color c)
    {
        const Vector2 d = b - a;
        return box((a + b) * 0.5f, {d.magnitude() * 0.5f, halfThickness}, 0.0f, c, std::atan2(d.y, d.x));
    }
    static bool key(SDL_Scancode k)
    {
        return !ImGui::GetIO().WantTextInput && SDL_GetKeyboardState(nullptr)[k];
    }
    static bool keyEdge(SDL_Scancode k, bool &was)
    {
        const bool now = key(k);
        const bool edge = now && !was;
        was = now;
        return edge;
    }
};

// ================================================================================================================
// Pinball: flippers are hinges with limits and a motor
// ================================================================================================================
class PinballDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Pinball"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Pinball", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Score: %d   Balls: %zu", score, balls.size());
        ImGui::Text("Flippers: hold Z (left) and M (right)");
        if (ImGui::Button("Launch ball"))
            launch();
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Flippers are boxes on RevoluteJoints with angle limits [0, 0.9 rad]; the motor lifts them when the key is held and pushes them back onto the lower limit otherwise.");
        ImGui::End();
    }

    void drawWorld() override { Renderer2D::DrawText("SCORE " + std::to_string(score), {960, 930}, 22, {255, 240, 150, 240}); }

protected:
    void build() override
    {
        balls.clear();
        bumpers.clear();
        score = 0;
        room();
        const Color wall = rgb(70, 82, 120);
        box({560, 520}, {12, 480}, 0.0f, wall);
        box({1360, 520}, {12, 480}, 0.0f, wall);
        box({960, 995}, {420, 12}, 0.0f, wall);
        segment({560, 880}, {720, 990}, 12, wall);
        segment({1360, 880}, {1200, 990}, 12, wall);
        segment({572, 400}, {800, 215}, 9, wall);
        segment({1348, 400}, {1120, 215}, 9, wall);

        const Vector2 pos[3] = {{820, 720}, {1100, 720}, {960, 560}};
        for (int i = 0; i < 3; i++)
        {
            RigidBody *b = ball(pos[i], 36, 0.0f, hsv(0.02f + 0.3f * i, 0.7f, 0.95f));
            b->restitution = 1.0f;
            bumpers.push_back({pos[i], 36.0f});
        }
        for (int i = 0; i < 4; i++)
            ball({700.0f + 170.0f * i, 880.0f - 90.0f * (i % 2)}, 10, 0.0f, rgb(200, 200, 215))->restitution = 0.8f;

        flipper(true);
        flipper(false);
        launch();
    }

    void step() override
    {
        const float torque = 2e7f;
        left->setMotor(key(SDL_SCANCODE_Z) ? 28.0f : -28.0f, torque);
        right->setMotor(key(SDL_SCANCODE_M) ? -28.0f : 28.0f, torque);
        if (key(SDL_SCANCODE_Z)) leftBody->wakeUp();
        if (key(SDL_SCANCODE_M)) rightBody->wakeUp();

        for (RigidBody *b : balls)
        {
            for (auto &bp : bumpers)
            {
                const Vector2 d = b->position - bp.first;
                const float dist = d.magnitude();
                if (dist < bp.second + 12.0f + 2.0f && dist > 1e-3f && b->velocity.scalarProduct(d) < 150.0f)
                {
                    b->velocity += d / dist * 420.0f; // pop bumper
                    score += 100;
                }
            }
            if (b->position.y < 120.0f)
            {
                b->setPose({frand(700, 1220), 930}, 0);
                b->setVelocity({frand(-150, 150), 0});
            }
        }
    }

private:
    void flipper(bool isLeft)
    {
        const float L = 72.0f;
        const Vector2 pivot = isLeft ? Vector2(800, 190) : Vector2(1120, 190);
        const float a = isLeft ? -0.45f : 3.14159265f + 0.45f;
        RigidBody *pin = box(pivot, {4, 4}, 0.0f, rgb(60, 60, 70));
        BodyDef d = def(pivot + Vector2(std::cos(a), std::sin(a)) * L, 3.0f, rgb(245, 205, 70), a);
        d.restitution = 0.2f;
        RigidBody *f = track(worldRef->createBox({L, 12}, d));
        f->canSleep = false;
        RevoluteJoint *j = worldRef->createRevoluteJoint(pin, f, pivot);
        if (isLeft) { j->setLimits(0.0f, 0.9f); left = j; leftBody = f; }
        else { j->setLimits(-0.9f, 0.0f); right = j; rightBody = f; }
    }

    void launch()
    {
        BodyDef d = def({frand(800, 1100), 900}, 1.0f, rgb(240, 240, 250));
        d.restitution = 0.8f;
        RigidBody *b = track(worldRef->createCircle(12, d));
        b->canSleep = false;
        b->velocity = {frand(-300, 300), frand(-100, 100)};
        balls.push_back(b);
    }

    std::vector<RigidBody *> balls;
    std::vector<std::pair<Vector2, float>> bumpers;
    RevoluteJoint *left = nullptr, *right = nullptr;
    RigidBody *leftBody = nullptr, *rightBody = nullptr;
    int score = 0;
};

// ================================================================================================================
// Catapult: a counterweighted lever on a hinge, held by a motor brake until you fire
// ================================================================================================================
class CatapultDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Catapult"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Catapult", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        if (ImGui::Button("FIRE!", ImVec2(120, 36)) || keyEdge(SDL_SCANCODE_F, fWas))
            fire();
        ImGui::SameLine();
        if (ImGui::Button("Reset"))
            requestReset();
        ImGui::SliderFloat("Power (rad/s)", &power, 1.5f, 6.0f);
        if (ImGui::SliderFloat("Arm mass", &cwMass, 5.0f, 80.0f))
            requestReset();
        ImGui::Text("Targets left standing: %d", standing());
        tip("The arm is a compound body (long arm + counterweight + tray) on a RevoluteJoint with an upper limit. The motor holds it (speed 0) until FIRE, then drives it at the chosen power until the upper limit stops it and the ball flies. Press F too.");
        ImGui::End();
    }

protected:
    void build() override
    {
        room();
        const Color wood = rgb(160, 120, 70);
        const Vector2 P(520, 340);
        box({P.x, 170}, {18, 110}, 0.0f, wood);
        RigidBody *hub = box(P, {6, 6}, 0.0f, rgb(60, 60, 60));
        BodyDef d = def(P, cwMass, rgb(190, 140, 80), -1.309f);
        d.restitution = 0.0f;
        arm = track(worldRef->createCompound({ConvexPart::box({150, 6}, {100, 0}),          // long arm
                                              ConvexPart::box({60, 60}, {-110, 0}),           // counterweight
                                              ConvexPart::box({20, 3}, {235, 9}),           // tray
                                              ConvexPart::box({3, 14}, {216, 20}),         // back lip
                                              ConvexPart::box({3, 14}, {256, 20})},        // front lip
                                             d));
        arm->canSleep = false;
        joint = worldRef->createRevoluteJoint(hub, arm, P);
        joint->setLimits(0.0f, 0.87f);
        joint->setMotor(0.0f, 1e9f); // brake
        fired = false;

        const float c = std::cos(-1.309f), sn = std::sin(-1.309f);
        const Vector2 tip = P + Vector2(c * 236.0f - sn * 26.0f, sn * 236.0f + c * 26.0f);
        BodyDef bd = def(tip, 1.0f, rgb(40, 40, 46));
        bd.restitution = 0.2f;
        projectile = track(worldRef->createCircle(11, bd));
        projectile->canSleep = false;

        // target castle
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4 - i; k++)
                targets.push_back(box({1200.0f + 62.0f * k + 31.0f * i, 90.0f + 52.0f * i}, {30, 25}, 0.6f, hsv(0.55f + 0.05f * i, 0.5f, 0.95f)));
    }

    void step() override
    {
        if (!fired)
        {
            arm->wakeUp();
            projectile->wakeUp();
        }
    }

private:
    void fire()
    {
        if (fired)
            return;
        fired = true;
        joint->setMotor(power, 5e8f); // the torsion spring: swing until the upper limit stops the arm
        arm->wakeUp();
    }

    int standing() const
    {
        int n = 0;
        for (RigidBody *t : targets)
            if (t->position.y > 70.0f && t->position.x > 1100.0f)
                n++;
        return n;
    }

    RigidBody *arm = nullptr, *projectile = nullptr;
    RevoluteJoint *joint = nullptr;
    std::vector<RigidBody *> targets;
    bool fired = false, fWas = false;
    float cwMass = 40.0f, power = 3.9f;
};

// ================================================================================================================
// Hill climb: wheel joints over polygon terrain, coins are sensors
// ================================================================================================================
class HillClimbDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Hill Climb"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Hill Climb", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Hold L (gas), J (reverse), K (brake)");
        ImGui::Text("Coins: %d / %d   %s", coins, totalCoins, finished ? "FINISHED!" : "");
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Terrain is a row of static convex polygons. Coins and the flag are sensors: the car passes through them and the demo reads getSensorEvents().");
        ImGui::End();
    }

    void drawWorld() override { Renderer2D::DrawText("FINISH", {1840, 330}, 18, {255, 255, 255, 230}); }

protected:
    static float height(float x) { return 150.0f + 80.0f * std::sin(x / 190.0f) + 35.0f * std::sin(x / 67.0f + 1.0f); }

    void build() override
    {
        wheels.clear();
        coinBodies.clear();
        coins = 0;
        finished = false;
        const Color dirt = rgb(112, 84, 60);
        for (float x = -60; x < 1980; x += 40)
        {
            const std::vector<Vector2> pts{{x, 0}, {x + 40, 0}, {x + 40, height(x + 40)}, {x, height(x)}};
            BodyDef d = def({0, 0}, 0.0f, dirt);
            d.staticFriction = 1.0f;
            d.dynamicFriction = 0.9f;
            track(worldRef->createPolygon(pts, d));
        }
        box({-20, 500}, {20, 600}, 0.0f, dirt);
        box({1940, 500}, {20, 600}, 0.0f, dirt);

        totalCoins = 0;
        for (float x = 260; x < 1750; x += 130)
        {
            BodyDef d = def({x, height(x) + 70}, 0.0f, rgb(255, 210, 40));
            d.isSensor = true;
            coinBodies.push_back(track(worldRef->createCircle(13, d)));
            totalCoins++;
        }
        {
            BodyDef d = def({1860, height(1860) + 90}, 0.0f, rgb(80, 220, 120));
            d.isSensor = true;
            flag = track(worldRef->createBox({10, 80}, d));
        }

        const Vector2 at(120, height(120) + 70);
        BodyDef cd = def(at, 7.0f, rgb(70, 140, 235));
        cd.groupIndex = -1;
        const std::vector<Vector2> cab{{-34, 9}, {44, 9}, {26, 34}, {-22, 34}};
        chassis = track(worldRef->createCompound({ConvexPart::box({62, 9}), ConvexPart::polygon(cab.data(), 4)}, cd));
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
            wj->setSpring(4.0f, 0.7f);
            wj->setLimits(-14, 14);
            wheels.push_back(wj);
        }
    }

    void step() override
    {
        float drive = 0.0f;
        if (key(SDL_SCANCODE_L)) drive = 1.0f;
        if (key(SDL_SCANCODE_J)) drive = -1.0f;
        const bool brake = key(SDL_SCANCODE_K);
        for (WheelJoint *w : wheels)
        {
            if (brake) w->setMotor(0.0f, 1.2e5f);
            else w->setMotor(-drive * 14.0f, std::fabs(drive) > 0.01f ? 7e4f : 3e3f);
        }
        if (drive != 0.0f || brake)
            chassis->wakeUp();

        for (const CollisionEvent &e : worldRef->getSensorEvents())
        {
            RigidBody *s = e.a->isSensor ? e.a : e.b;
            RigidBody *o = e.a->isSensor ? e.b : e.a;
            if (o != chassis && o->shapeType != ShapeType::CIRCLE)
                continue;
            if (s == flag)
                finished = true;
            else if (std::find(coinBodies.begin(), coinBodies.end(), s) != coinBodies.end())
            {
                coinBodies.erase(std::find(coinBodies.begin(), coinBodies.end(), s));
                bodiesRef->erase(std::find(bodiesRef->begin(), bodiesRef->end(), s));
                worldRef->removeBody(s);
                coins++;
                break; // the event list refers to bodies we may just have removed
            }
        }
    }

private:
    RigidBody *chassis = nullptr, *flag = nullptr;
    std::vector<WheelJoint *> wheels;
    std::vector<RigidBody *> coinBodies;
    int coins = 0, totalCoins = 0;
    bool finished = false;
};

// ================================================================================================================
// Hinged bridge: planks on revolute joints; cut it
// ================================================================================================================
class BridgeHingeDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Hinged Bridge"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Hinged Bridge", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        if (ImGui::Button("Drop crates"))
            for (int i = 0; i < 5; i++)
                box({frand(800, 1100), 700.0f + 60.0f * i}, {frand(20, 34), frand(15, 28)}, 1.5f, hsv(frand(0, 1), 0.5f, 0.95f), frand(0, 3));
        ImGui::SameLine();
        if (ImGui::Button("Drop balls"))
            for (int i = 0; i < 6; i++)
                ball({frand(700, 1250), 700.0f + 40.0f * i}, frand(14, 26), 4.0f, hsv(frand(0, 1), 0.6f, 0.95f));
        if (ImGui::Button("Drop wheel-cart"))
            cart({420, 400});
        ImGui::SameLine();
        if (ImGui::Button("Cut the middle") && !joints.empty())
        {
            worldRef->removeJoint(joints[joints.size() / 2]);
            joints.erase(joints.begin() + joints.size() / 2);
            worldRef->wakeAll();
        }
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Each plank is a box; neighbours share a RevoluteJoint with a small angle limit, the ends are pinned to static cliffs.");
        ImGui::End();
    }

protected:
    void build() override
    {
        joints.clear();
        room();
        const Color rock = rgb(96, 92, 100);
        RigidBody *L = box({150, 150}, {150, 150}, 0.0f, rock);
        RigidBody *R = box({1770, 150}, {150, 150}, 0.0f, rock);
        const int n = 18;
        const float x0 = 300, x1 = 1620, y0 = 293, sag = 150;
        std::vector<Vector2> p;
        for (int i = 0; i <= n; i++)
        {
            const float t = (float)i / n;
            p.push_back({x0 + (x1 - x0) * t, y0 - 4.0f * sag * t * (1 - t)});
        }
        RigidBody *prev = L;
        for (int i = 0; i < n; i++)
        {
            const Vector2 d = p[i + 1] - p[i];
            RigidBody *plank = box((p[i] + p[i + 1]) * 0.5f, {d.magnitude() * 0.5f - 1.0f, 7}, 2.0f, rgb(176, 130, 80), std::atan2(d.y, d.x));
            plank->staticFriction = 1.0f;
            plank->dynamicFriction = 0.8f;
            RevoluteJoint *j = worldRef->createRevoluteJoint(prev, plank, p[i]);
            j->setLimits(-0.5f, 0.5f);
            joints.push_back(j);
            prev = plank;
        }
        RevoluteJoint *j = worldRef->createRevoluteJoint(prev, R, p[n]);
        j->setLimits(-0.5f, 0.5f);
        joints.push_back(j);
    }

private:
    void cart(Vector2 at)
    {
        BodyDef cd = def(at, 4.0f, rgb(220, 80, 70));
        cd.groupIndex = -(int)(rand() % 3000) - 5;
        RigidBody *c = track(worldRef->createBox({40, 8}, cd));
        for (int i = -1; i <= 1; i += 2)
        {
            BodyDef wd = def(at + Vector2(i * 28.0f, -16.0f), 1.0f, rgb(40, 40, 46));
            wd.groupIndex = cd.groupIndex;
            wd.staticFriction = 1.0f;
            RigidBody *w = track(worldRef->createCircle(14, wd));
            WheelJoint *wj = worldRef->createWheelJoint(c, w, w->position, {0, 1});
            wj->setSpring(5.0f, 0.7f);
            wj->setLimits(-8, 8);
        }
        c->velocity = {120, 0};
    }

    std::vector<RevoluteJoint *> joints;
};

// ================================================================================================================
// Marble run: pegs, zig-zag ramps, a motorised spinner, one-way flaps and a counter
// ================================================================================================================
class MarbleRunDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Marble Run"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Marble Run", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Checkbox("Emit marbles", &emit);
        ImGui::Text("Marbles: %zu   Collected: %d", marbles.size(), collected);
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Zig-zag capsule rails, hexagon pegs, a motor-driven cross spinner, one-way flaps the marbles drop through, and a sensor counting what reaches the bottom.");
        ImGui::End();
    }

protected:
    void build() override
    {
        marbles.clear();
        collected = 0;
        timer = 0.0f;
        room();
        const Color rail = rgb(90, 100, 130);
        // zig-zag rails (capsules from static polygons would also work; planks keep it simple)
        segment({260, 900}, {900, 800}, 8, rail);
        segment({1000, 740}, {420, 640}, 8, rail);
        segment({320, 580}, {900, 480}, 8, rail);
        box({1050, 790}, {8, 60}, 0.0f, rail);
        // hexagon peg field
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 7; c++)
            {
                RigidBody *peg = track(worldRef->createRegularPolygon(6, 13, def({1150.0f + 90.0f * c + (r % 2) * 45.0f, 880.0f - 85.0f * r}, 0.0f, rgb(120, 180, 150))));
                peg->restitution = 0.5f;
            }
        // spinner
        RigidBody *hub = box({700, 360}, {5, 5}, 0.0f, rail);
        spinner = compound({700, 360}, {ConvexPart::box({100, 8}), ConvexPart::box({8, 100})}, 6.0f, rgb(235, 170, 60));
        spinner->canSleep = false;
        worldRef->createRevoluteJoint(hub, spinner, {700, 360})->setMotor(1.5f, 5e6f);
        // collector lanes: one-way flaps and a counting sensor
        for (int i = 0; i < 3; i++)
        {
            BodyDef d = def({1180.0f + 220.0f * i, 330.0f - 0.0f}, 0.0f, rgb(230, 200, 100));
            d.oneWay = true;
            track(worldRef->createBox({90, 7}, d));
        }
        segment({1100, 430}, {1700, 470}, 8, rail);
        box({1750, 150}, {10, 120}, 0.0f, rail);
        BodyDef sd = def({960, 90}, 0.0f, rgb(250, 210, 60));
        sd.isSensor = true;
        sensor = track(worldRef->createBox({700, 18}, sd));
    }

    void step() override
    {
        if (emit)
        {
            timer += 1.0f / 360.0f;
            if (timer > 0.4f && marbles.size() < 160)
            {
                timer = 0;
                BodyDef d = def({frand(240, 300), 960}, 0.4f, hsv(frand(0, 1), 0.7f, 0.95f));
                d.restitution = 0.35f;
                d.angularDamping = 0.995f;
                RigidBody *m = track(worldRef->createCircle(frand(8, 12), d));
                marbles.push_back(m);
            }
        }
        std::vector<RigidBody *> inside;
        for (const CollisionEvent &e : worldRef->getSensorEvents())
        {
            RigidBody *o = e.a == sensor ? e.b : (e.b == sensor ? e.a : nullptr);
            if (o && !counted[o])
            {
                counted[o] = true;
                collected++;
            }
        }
    }

private:
    std::vector<RigidBody *> marbles;
    std::map<RigidBody *, bool> counted;
    RigidBody *spinner = nullptr, *sensor = nullptr;
    bool emit = true;
    float timer = 0.0f;
    int collected = 0;
};

// ================================================================================================================
// Portals: sensors that teleport bodies, plus a booster pad
// ================================================================================================================
class PortalDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Portals and boosters"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Portals", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Checkbox("Rain shapes", &rain);
        ImGui::Text("Teleports: %d", teleports);
        if (ImGui::Button("Reset"))
            requestReset();
        tip("Portals and the booster pad are sensors. When a body overlaps a portal the demo moves it to the other one, keeping its velocity; the pad adds an impulse.");
        ImGui::End();
    }

    void drawWorld() override
    {
        Renderer2D::DrawText("IN", {610, 270}, 20, {140, 190, 255, 240});
        Renderer2D::DrawText("OUT", {1620, 620}, 20, {255, 190, 120, 240});
        Renderer2D::DrawText("BOOST", {1000, 80}, 16, {150, 255, 170, 240});
    }

protected:
    void build() override
    {
        cooldown.clear();
        teleports = 0;
        room();
        const Color stone = rgb(88, 94, 104);
        segment({120, 420}, {520, 330}, 9, stone);
        box({100, 520}, {10, 120}, 0.0f, stone);
        segment({1450, 540}, {1800, 600}, 9, stone);
        box({1830, 480}, {10, 200}, 0.0f, stone);
        segment({1450, 540}, {1450, 420}, 9, stone);
        box({1000, 120}, {110, 14}, 0.0f, stone, 0.0f);

        BodyDef a = def({610, 170}, 0.0f, rgb(80, 140, 255));
        a.isSensor = true;
        portalA = track(worldRef->createBox({26, 70}, a));
        BodyDef b = def({1660, 660}, 0.0f, rgb(255, 150, 70));
        b.isSensor = true;
        portalB = track(worldRef->createBox({26, 70}, b));
        BodyDef p = def({1000, 160}, 0.0f, rgb(70, 220, 120));
        p.isSensor = true;
        pad = track(worldRef->createBox({100, 20}, p));
        for (int i = 0; i < 5; i++)
            spawn();
    }

    void step() override
    {
        for (auto &c : cooldown)
            c.second -= 1.0f / 360.0f;
        for (const CollisionEvent &e : worldRef->getSensorEvents())
        {
            RigidBody *s = e.a->isSensor ? e.a : e.b;
            RigidBody *o = e.a->isSensor ? e.b : e.a;
            if (o->isStatic() || o->isSensor)
                continue;
            if (s == pad)
            {
                o->applyImpulse({0, 1.0f / o->inverseMass * 40.0f});
                continue;
            }
            float &cd = cooldown[o];
            if (cd > 0.0f)
                continue;
            RigidBody *dest = s == portalA ? portalB : (s == portalB ? portalA : nullptr);
            if (!dest)
                continue;
            o->setPose(dest->position, o->orientation);
            o->velocity = s == portalA ? Vector2(o->velocity.x * 0.2f + 380.0f, 250.0f) : Vector2(-300.0f, 100.0f);
            o->calculateDerivativeData();
            cd = 0.6f;
            teleports++;
        }
        if (rain)
        {
            timer += 1.0f / 360.0f;
            if (timer > 0.5f && bodiesRef->size() < 160)
            {
                timer = 0;
                spawn();
            }
        }
    }

private:
    void spawn()
    {
        const Vector2 p(frand(140, 480), frand(560, 900));
        const Color c = hsv(frand(0, 1), 0.55f, 0.95f);
        switch (rand() % 4)
        {
        case 0: ball(p, frand(10, 20), 1.0f, c); break;
        case 1: ngon(p, 3 + rand() % 4, frand(14, 22), 1.5f, c, frand(0, 6)); break;
        case 2: capsule(p, 18, 8, 1.0f, c, frand(0, 6)); break;
        default: compound(p, {ConvexPart::box({20, 6}), ConvexPart::box({6, 18}, {0, 18})}, 1.5f, c, frand(0, 6)); break;
        }
    }

    RigidBody *portalA = nullptr, *portalB = nullptr, *pad = nullptr;
    std::map<RigidBody *, float> cooldown;
    int teleports = 0;
    bool rain = true;
    float timer = 0.0f;
};

// ================================================================================================================
// Tower smash: a castle of boxes, polygons and capsules, and a cannon
// ================================================================================================================
class TowerSmashDemo : public MoreDemo
{
public:
    const char *getName() const override { return "Tower Smash"; }

    void drawImGui() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 210), ImGuiCond_FirstUseEver);
        ImGui::Begin("Tower Smash", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::SliderFloat("Angle (deg)", &angle, 0.0f, 70.0f);
        ImGui::SliderFloat("Power", &power, 800.0f, 4000.0f);
        if (ImGui::Button("Fire (F)", ImVec2(120, 34)) || keyEdge(SDL_SCANCODE_F, fWas))
            fire();
        ImGui::SameLine();
        if (ImGui::Button("Rebuild"))
            requestReset();
        ImGui::Text("Blocks: %d", blocks);
        tip("Stable stacks of boxes, triangles and capsule columns fall asleep and wake on impact.");
        ImGui::End();
    }

    void drawWorld() override
    {
        const float a = angle * 0.0174533f;
        const Vector2 m(160, 110);
        Renderer2D::DrawLine(m, m + Vector2(std::cos(a), std::sin(a)) * 70.0f, {255, 255, 255, 220}, 4.0f);
    }

protected:
    void build() override
    {
        room();
        box({160, 80}, {60, 20}, 0.0f, rgb(80, 80, 90));
        blocks = 0;
        const Color brick = rgb(190, 110, 90);
        for (int tower = 0; tower < 2; tower++)
        {
            const float x = 1100.0f + tower * 520.0f;
            for (int i = 0; i < 9; i++)
            {
                add(box({x - 36, 82.0f + 44.0f * i}, {30, 20}, 0.8f, brick));
                add(box({x + 36, 82.0f + 44.0f * i}, {30, 20}, 0.8f, brick));
            }
            add(polygon({x, 82.0f + 44.0f * 9 + 20}, {{-80, -20}, {80, -20}, {0, 55}}, 1.2f, rgb(90, 90, 160)));
        }
        // a pyramid of crates between the towers, with a few capsule logs lying on top
        for (int r = 0; r < 5; r++)
            for (int k = 0; k < 5 - r; k++)
                add(box({1290.0f + 62.0f * k + 31.0f * r, 82.0f + 42.0f * r}, {30, 20}, 0.6f, hsv(0.12f + 0.04f * r, 0.5f, 0.95f)));
        add(capsule({1352, 82.0f + 42.0f * 5 + 12.0f}, 36, 11, 0.8f, rgb(220, 200, 120)));
    }

private:
    void add(RigidBody *b) { if (b) blocks++; }

    void fire()
    {
        const float a = angle * 0.0174533f;
        BodyDef d = def({160 + std::cos(a) * 80.0f, 110 + std::sin(a) * 80.0f}, 6.0f, rgb(40, 40, 48));
        d.restitution = 0.1f;
        RigidBody *b = track(worldRef->createCircle(20, d));
        b->velocity = Vector2(std::cos(a), std::sin(a)) * power;
    }

    float angle = 28.0f, power = 2400.0f;
    int blocks = 0;
    bool fWas = false;
};
