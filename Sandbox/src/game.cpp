#include "game.h"
#include <iostream>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>
#include "renderer2D.h"

static float physicsTimeMs = 0.0f;
static float renderTimeMs = 0.0f;
static float physicsHistory[600] = {0};
static float renderHistory[600] = {0};
static int historyIndex = 0;

using namespace AccelEngine;

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

    // init UI
    if (!ui.init(window, renderer))
    {
        std::cout << "Somethine wrong with IMGUI ----- Cannot Initialised" << std::endl;
        return false;
    }

    Renderer2D::init(renderer, WINDOW_W, WINDOW_H);

    // body
    body.position = Vector2(WINDOW_W / 2.0f, 100.0f);
    body.inverseMass = 1.0f;
    body.orientation = 0;
    body.velocity = Vector2(0, 0);
    body.angularDamping = 1.0f;
    body.rotation = 0.0f;
    body.shapeType = ShapeType::AABB;
    body.aabb.halfSize = Vector2(30.0f, 30.0f);
    body.linearDamping = 1.0f;
    body.inverseInertia = 1.0f / 30.0f;
    body.c = {200, 0, 0, 255};

    // ground
    ground.position = Vector2(927, -247);
    ground.inverseMass = 0.0f;
    ground.shapeType = ShapeType::AABB;
    ground.aabb.halfSize = Vector2(800.0f, 50.0f);
    ground.restitution = 0.8f;
    ground.c = {200, 200, 0, 255};



    addAABB(66, -247 - 100, 1600, 100, 90.0f);
    addAABB(1782, -247 - 100, 1600, 100, 90.0f);

    // adding bodies to world and bodies
    world.addBody(&ground);
    world.addBody(&body);
    bodies.push_back(&body);
    bodies.push_back(&ground);

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
            switch (e.key.key)
            {
            case SDLK_D:
                body.velocity = Vector2(100, 0);
                break;
            case SDLK_A:
                body.velocity = Vector2(-100, 0);
                break;
            case SDLK_S:
                body.velocity = Vector2(0, -100);
                break;
            case SDLK_W:
                body.velocity = Vector2(0, 100);
                break;
            case SDLK_LCTRL:
                ui.addBody(world, bodies);
                break;
            default:
                break;
            }
        }
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            float mx = e.button.x;
            float my = WINDOW_H - e.button.y;

            if (e.button.button == SDL_BUTTON_LEFT)
            {
                addCircle(mx, my);
            }
            else if (e.button.button == SDL_BUTTON_RIGHT)
            {
                addAABB(mx, my);
            }
        }
    }
}

void Game::update(float dt)
{

    world.startFrame();

    for (auto &it : bodies)
    {
        if (it->inverseMass > 0.0f)
        {
            it->addForce(Vector2(0, -980));
        }
    }

    Uint64 startPhysics = SDL_GetPerformanceCounter();
    world.step(dt, 20);
    Uint64 endPhysics = SDL_GetPerformanceCounter();

    physicsTimeMs = (float)(endPhysics - startPhysics) * 1000.0f / SDL_GetPerformanceFrequency();
}

void Game::render()
{
    static int sampleCounter = 0;

    Uint64 startRender = SDL_GetPerformanceCounter();
    ui.DrawAddBody(world, bodies);

    SDL_SetRenderDrawColor(renderer, 25, 25, 40, 255);
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
            Renderer2D::DrawCircle(b->position.x, b->position.y, b->circle.radius, c);
        }
    }

    // std::vector<Contact> contacts = world.getContacts();

    // for (auto it : contacts)
    // {
    //     for (auto it2 : it.contactPoints)
    //     {
    //         Renderer2D::DrawCircle(it2.x, it2.y, 3, SDL_Color{255, 0, 0, 255});
    //     }
    // }

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
}

void Game::addAABB(float x, float y, float w, float h, float orientation)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 0.0f;
    b->restitution = 0.3f; // Add restitution

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
    b->c = RandomColor();

    world.addBody(b);
    bodies.push_back(b);
}