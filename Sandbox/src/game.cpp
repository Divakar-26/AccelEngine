#include "game.h"
#include <iostream>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>
#include <AccelEngine/ForceRegistry.h>
#include "renderer2D.h"

static float physicsTimeMs = 0.0f;
static float renderTimeMs = 0.0f;
static float physicsHistory[600] = {0};
static float renderHistory[600] = {0};
static int historyIndex = 0;

Vector2 debugMouseWorld;
bool showDebugCoords = true;

RigidBody *grabbed = nullptr;
Vector2 grabOffset;
bool mouseDown = false;

using namespace AccelEngine;

ForceRegistry registry;
Gravity *g;

SDL_Texture *boxTex;

static Color RandomColor()
{
    return {
        float(rand() % 256),
        float(rand() % 256),
        float(rand() % 256),
        255.0f};
}

void FindMinMax(const float *data, int size, float &outMin, float &outMax);

Game::Game(int W_W, int W_H)
{
    WINDOW_H = W_H;
    WINDOW_W = W_W;
}

bool Game::Init(const char *title)
{
    window = SDL_CreateWindow(title, WINDOW_W, WINDOW_H, SDL_WINDOW_RESIZABLE);
    renderer = SDL_CreateRenderer(window, NULL);

    if (!ui.init(window, renderer))
    {
        std::cout << "Something wrong with IMGUI" << std::endl;
        return false;
    }

    Renderer2D::init(renderer, WINDOW_W, WINDOW_H);

    camera.x = 0;
    camera.y = 0;
    camera.zoom = 1.0f;

    // ---------------------------------------------------------
    // GRAVITY
    // ---------------------------------------------------------
    g = new Gravity(Vector2(0, -980));

    // ---------------------------------------------------------
    // MODULAR BRIDGE SETTINGS
    // ---------------------------------------------------------
    int howMany = 2;

    int plankCount          = 5 * howMany;
    float plankW            = 100.0f / howMany;
    float plankH            = 50.0f / howMany;
    float gap               = 50.0f / howMany;
    float offset            = 10.0f / howMany;
    float springK           = 100.0f;
    float springDamping     = 500.0f;

    int pivotOffsetFromEdges = 300;

    // ---------------------------------------------------------
    // CREATE SUPPORTS
    // ---------------------------------------------------------
    RigidBody *pivot = getBody(
        pivotOffsetFromEdges,
        WINDOW_H / 4,
        500, 50,
        0.0f,
        0.0f,
        {0, 255, 0, 255}
    );

    RigidBody *pivot2 = getBody(
        WINDOW_W - pivotOffsetFromEdges,
        WINDOW_H / 4,
        500, 50,
        0.0f,
        0.0f,
        {0, 255, 0, 255}
    );

    world.addBody(pivot);
    world.addBody(pivot2);
    bodies.push_back(pivot);
    bodies.push_back(pivot2);

    // ---------------------------------------------------------
    // CREATE BRIDGE
    // ---------------------------------------------------------
    std::vector<RigidBody *> bridge;

    float halfPivotW  = pivot->getWidth()  * 0.5f;
    float halfPivot2W = pivot2->getWidth() * 0.5f;
    float halfPlankW  = plankW * 0.5f;

    float startX = pivot->position.x + halfPivotW + gap + halfPlankW;
    float step   = plankW + gap;
    float y      = pivot->position.y;

    for (int i = 0; i < plankCount; i++)
    {
        float x = startX + i * step;

        RigidBody *p = getBody(
            x,
            y,
            plankW,
            plankH,
            1.0f,
            0.0f,
            {255, 0, 0, 255}
        );

        // connect left side
        if (i == 0)
        {
            // pivot → first plank
            Spring *s = new Spring(
                Vector2(-halfPlankW + offset, 0),
                pivot,
                Vector2(halfPivotW - offset, 0),
                springK,
                gap
            );
            s->damping = springDamping;
            registry.add(p, s);
        }
        else
        {
            // plank[i-1] → plank[i]
            Spring *s = new Spring(
                Vector2(-halfPlankW + offset, 0),
                bridge[i - 1],
                Vector2(halfPlankW - offset, 0),
                springK,
                gap
            );
            s->damping = springDamping;
            registry.add(p, s);
        }

        // connect last to pivot2
        if (i == plankCount - 1)
        {
            Spring *s = new Spring(
                Vector2(halfPlankW - offset, 0),
                pivot2,
                Vector2(-halfPivot2W + offset, 0),
                springK,
                gap
            );
            s->damping = springDamping;
            registry.add(p, s);
        }

        world.addBody(p);
        bodies.push_back(p);
        bridge.push_back(p);
    }

    running = true;
    return true;
}


void Game::handleEvent()
{
    SDL_Event e;
    // handle ui

    while (SDL_PollEvent(&e))
    {
        if (ui.handleEvent(e))
        {
            continue;
        }
        if (e.type == SDL_EVENT_QUIT)
        {
            running = false;
        }
        if (e.type == SDL_EVENT_KEY_DOWN)
        {
            cameraControls(e);
        }
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            SDL_FPoint w = Renderer2D::screenToWorld(e.button.x, e.button.y);
            float mx = w.x;
            float my = w.y;
            mouseDown = true;

            if (e.button.button == SDL_BUTTON_LEFT)
            {
                gradBodies(mx, my);
            }
            else if (e.button.button == SDL_BUTTON_RIGHT)
            {
                addAABB(mx, my);
            }
            else if (e.button.button == SDL_BUTTON_MIDDLE)
            {
                addCircle(mx, my);
            }
        }

        if (e.type == SDL_EVENT_MOUSE_BUTTON_UP)
        {
            if (e.button.button == SDL_BUTTON_LEFT)
            {
                grabbed = nullptr;
                mouseDown = false;
            }
        }

        if (e.type == SDL_EVENT_MOUSE_MOTION && grabbed)
        {
            SDL_FPoint w = Renderer2D::screenToWorld(e.motion.x, e.motion.y);
            float mx = w.x;
            float my = w.y;

            Vector2 target = Vector2(mx, my) + grabOffset;

            grabbed->position = target;
            grabbed->velocity = Vector2(0, 0);
            grabbed->rotation = 0;
        }
    }
}

void Game::update(float dt)
{
    const int substeps = 20;
    const real h = dt / (real)substeps;

    Uint64 startPhysics = SDL_GetPerformanceCounter();

    if (grabbed && mouseDown)
    {
        float sx, sy;
        SDL_GetMouseState(&sx, &sy);

        SDL_FPoint w = Renderer2D::screenToWorld(sx, sy);
        Vector2 target = Vector2(w.x, w.y) + grabOffset;

        grabbed->position = target;
        grabbed->velocity = Vector2(0, 0);
        grabbed->rotation = 0;
    }

    for (int i = 0; i < substeps; ++i)
    {
        world.startFrame();
        registry.updateForces(h);
        world.step(h, 1);
    }

    // std::cout<<bodies.size()<<std::endl;

    Uint64 endPhysics = SDL_GetPerformanceCounter();
    physicsTimeMs = (float)(endPhysics - startPhysics) * 1000.0f / SDL_GetPerformanceFrequency();
}

void Game::render()
{
    Renderer2D::setCamera(camera);
    static int sampleCounter = 0;

    Uint64 startRender = SDL_GetPerformanceCounter();
    ui.DrawAddBody(world, bodies);

    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    SDL_RenderClear(renderer);

    for (auto *b : bodies)
    {
        // Vector2 screenPos = WorldToScreen(b->position, WINDOW_H);
        SDL_Color c = {b->c.r, b->c.g, b->c.b, b->c.a};

        if (b->shapeType == ShapeType::AABB)
        {
            Renderer2D::DrawRectangle(b->position.x, b->position.y, b->aabb.halfSize.x * 2, b->aabb.halfSize.y * 2, b->orientation, c);
        }
        else if (b->shapeType == ShapeType::CIRCLE)
        {
            Renderer2D::DrawCircle(b->position.x, b->position.y, b->circle.radius, b->orientation, c);
        }
    }

    // std::vector<Contact> contacts = world.getContacts();

    // for (auto it : contacts)
    // {
    //     for (auto it2 : it.contactPoints)
    //     {
    //         Renderer2D::DrawCircle(it2.x, it2.y, 3,  0.0,  SDL_Color{255, 0, 0, 255});
    //     }
    // }

    Renderer2D::drawSprings(registry);

    float minVal, maxVal;
    FindMinMax(physicsHistory, 200, minVal, maxVal);

    ImGui::Begin("Profiler");

    ImGui::Text("Physics: %.3f ms", physicsTimeMs);
    ImGui::ProgressBar(physicsTimeMs / 16.67f, ImVec2(200, 20));

    ImGui::Spacing();

    ImGui::Text("Render: %.3f ms", renderTimeMs);
    ImGui::ProgressBar(renderTimeMs / 16.67f, ImVec2(200, 20));

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Physics Time (Last 100 frames)");
    ImGui::PlotLines("##phys", physicsHistory, 100, historyIndex, nullptr, minVal, maxVal, ImVec2(600, 160));

    ImGui::Spacing();

    ImGui::Text("Render Time (Last 100 frames)");
    ImGui::PlotLines("##rend", renderHistory, 100, 0, nullptr, 0, 20, ImVec2(300, 80));

    ImGui::End();

    if (showDebugCoords)
    {
        ImGui::Begin("Mouse World", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::Text("World X: %.2f", debugMouseWorld.x);
        ImGui::Text("World Y: %.2f", debugMouseWorld.y);

        ImGui::Separator();
        if (ImGui::Button("Hide"))
            showDebugCoords = false;

        ImGui::End();
    }

    ui.DrawFrame();

    Uint64 endRender = SDL_GetPerformanceCounter();

    sampleCounter++;

    if (sampleCounter >= 300) // take a sample every 10 frames
    {
        physicsHistory[historyIndex] = physicsTimeMs;
        renderHistory[historyIndex] = renderTimeMs;

        historyIndex = (historyIndex + 1) % 600;

        sampleCounter = 0; // reset
    }

    SDL_RenderPresent(renderer);
}

void FindMinMax(const float *data, int size, float &outMin, float &outMax)
{
    outMin = data[0];
    outMax = data[0];

    for (int i = 1; i < size; i++)
    {
        if (data[i] < outMin)
            outMin = data[i];
        if (data[i] > outMax)
            outMax = data[i];
    }

    // Expand range slightly to avoid zero-line
    float range = outMax - outMin;
    if (range < 0.0001f)
        range = 0.0001f;

    outMin -= range * 0.1f;
    outMax += range * 0.1f;
}

void Game::showFPS(float realDt, float fixedDt)
{
    float fps = 1.0f / realDt;
    float frameMs = realDt * 1000.0f;
    float fixedMs = fixedDt * 1000.0f;

    char title[256];
    snprintf(title, sizeof(title),
             "AccelEngine | FPS: %.1f | Frame: %.2f ms | Step: %.2f ms | Bodies: %zu",
             fps, frameMs, fixedMs, bodies.size());

    SDL_SetWindowTitle(window, title);
}

void Game::addCircle(float x, float y)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 1.0f;
    b->restitution = 1.0f; // Add some restitution

    b->shapeType = ShapeType::CIRCLE;
    b->circle.radius = 10 + rand() % 50;

    // FIX 1: Correct circle inertia formula
    // I = (1/2) * m * r² for solid circle
    real mass = 1.0f / b->inverseMass;
    real inertia = 0.5f * mass * b->circle.radius * b->circle.radius;

    // FIX 2: Avoid division by zero
    if (inertia > 1e-6f)
    {
        b->inverseInertia = 1.0f / inertia;
    }
    else
    {
        b->inverseInertia = 0.0f;
    }

    b->velocity = {0, 0};
    b->calculateDerivativeData();
    b->c = RandomColor();

    world.addBody(b);
    bodies.push_back(b);
    registry.add(b, g);
}

void Game::addAABB(float x, float y)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 1.0f;
    b->restitution = 0.3f; // Add restitution

    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = {20 + rand() % 50, 20 + rand() % 50};

    // FIX 3: Correct AABB inertia formula with proper order
    // I = (1/12) * m * (w² + h²) for rectangle
    real mass = 1.0f / b->inverseMass;
    real width = b->aabb.halfSize.x * 2.0f;
    real height = b->aabb.halfSize.y * 2.0f;
    real inertia = (1.0f / 12.0f) * mass * (width * width + height * height);

    // FIX 4: Avoid division by zero
    if (inertia > 1e-6f)
    {
        b->inverseInertia = 1.0f / inertia;
    }
    else
    {
        b->inverseInertia = 0.0f;
    }
    b->angularDamping = 0.98;

    b->velocity = {0, 0};
    b->calculateDerivativeData();
    b->c = RandomColor();

    world.addBody(b);
    bodies.push_back(b);
    registry.add(b, g);
}

void Game::addAABB(float x, float y, float w, float h, float orientation)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 0.0f;
    b->restitution = 0.3f;

    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = {w / 2, h / 2};
    b->orientation = orientation * (3.14159265f / 180.0f);

    b->inverseInertia = 0.0f;
    b->angularDamping = 0.98;

    b->velocity = {0, 0};
    b->calculateDerivativeData();
    b->c = RandomColor();

    world.addBody(b);
    bodies.push_back(b);

    std::cout << "Hello" << std::endl;
    // registry.add(b, g);
}

RigidBody *Game::getBody(float x, float y, float w, float h, float mass1, float orientation, SDL_Color c)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = mass1;
    b->restitution = 0.3f;

    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = {w / 2, h / 2};
    b->orientation = orientation * (3.14159265f / 180.0f);

    // FIX 3: Correct AABB inertia formula with proper order
    // I = (1/12) * m * (w² + h²) for rectangle
    real mass = 1.0f / b->inverseMass;
    real width = b->aabb.halfSize.x * 2.0f;
    real height = b->aabb.halfSize.y * 2.0f;
    real inertia = (1.0f / 12.0f) * mass * (width * width + height * height);

    if (inertia > 1e-6f)
    {
        b->inverseInertia = 1.0f / inertia;
    }
    else
    {
        b->inverseInertia = 0.0f;
    }
    b->angularDamping = 0.98;

    b->velocity = {0, 0};
    b->calculateDerivativeData();
    b->c = {(float)c.r, (float)c.g, (float)c.b, (float)c.a};

    return b;
}

Vector2 Game::WorldToScreen(const Vector2 &w, float screenHeight)
{
    return Vector2(w.x, screenHeight - w.y);
}
Vector2 Game::ScreenToWorld(const Vector2 &s, float screenHeight)
{
    return Vector2(s.x, screenHeight - s.y);
}
Vector2 Game::getMouseWorld()
{
    float sx, sy;
    SDL_GetMouseState(&sx, &sy);

    SDL_FPoint w = Renderer2D::screenToWorld(sx, sy);

    return Vector2(w.x, w.y);
}

void Game::gradBodies(float mx, float my)
{
    for (auto *b : bodies)
    {
        if (b->inverseMass == 0.0f)
            continue; // static bodies not draggable

        if (b->shapeType == ShapeType::AABB)
        {
            if (mx >= b->position.x - b->aabb.halfSize.x &&
                mx <= b->position.x + b->aabb.halfSize.x &&
                my >= b->position.y - b->aabb.halfSize.y &&
                my <= b->position.y + b->aabb.halfSize.y)
            {
                grabbed = b;
                grabOffset = b->position - Vector2(mx, my);
                break;
            }
        }
        else if (b->shapeType == ShapeType::CIRCLE)
        {
            Vector2 d = b->position - Vector2(mx, my);
            if (d.magnitude() <= b->circle.radius)
            {
                grabbed = b;
                grabOffset = d;
                break;
            }
        }
    }
}

void Game::cameraControls(SDL_Event &e)
{
    switch (e.key.key)
    {
    case SDLK_LEFT:
        camera.x -= 50;
        break;
    case SDLK_RIGHT:
        camera.x += 50;
        break;
    case SDLK_UP:
        camera.y += 50;
        break;
    case SDLK_DOWN:
        camera.y -= 50;
        break;
    case SDLK_LCTRL:
    {
        debugMouseWorld = getMouseWorld();
        showDebugCoords = true;
    }
    break;
    case SDLK_Q:
    {
        float msx, msy;
        SDL_GetMouseState(&msx, &msy);

        SDL_FPoint before = Renderer2D::screenToWorld(msx, msy);

        camera.zoom *= 1.1f;
        SDL_FPoint after = Renderer2D::screenToWorld(msx, msy);

        camera.x += before.x - after.x;
        camera.y += before.y - after.y;
    }
    break;

    case SDLK_E:
    {
        float msx, msy;
        SDL_GetMouseState(&msx, &msy);

        SDL_FPoint before = Renderer2D::screenToWorld(msx, msy);

        camera.zoom /= 1.1f;

        SDL_FPoint after = Renderer2D::screenToWorld(msx, msy);

        camera.x += before.x - after.x;
        camera.y += before.y - after.y;
    }
    break;

    default:
        break;
    }
}