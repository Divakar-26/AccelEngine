#include "game.h"
#include <iostream>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/collision_narrow.h>
#include <AccelEngine/collision_resolve.h>
#include "renderer2D.h"

using namespace AccelEngine;

SDL_Texture *boxTex;

void DrawCircle(SDL_Renderer *, float, float, int);

Game::Game(int W_W, int W_H)
{
    WINDOW_H = W_H;
    WINDOW_W = W_W;
}

bool Game::Init(const char *title)
{
    window = SDL_CreateWindow(title, WINDOW_W, WINDOW_H, SDL_WINDOW_RESIZABLE);
    renderer = SDL_CreateRenderer(window, NULL);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL3_InitForSDLRenderer(window, renderer))
    {
        SDL_Log("ImGui SDL3 init failed!");
        return false;
    }
    if (!ImGui_ImplSDLRenderer3_Init(renderer))
    {
        SDL_Log("ImGui SDL3 renderer init failed!");
        return false;
    }

    Renderer2D::init(renderer, WINDOW_W, WINDOW_H);

    // ----------------------------------------
    // FIXED: Position body in visible area (middle-top)
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

    ground.position = Vector2(927, -247); // Center at bottom
    ground.inverseMass = 0.0f;            // Use 0 for truly static objects
    ground.shapeType = ShapeType::AABB;
    ground.aabb.halfSize = Vector2(800.0f, 50.0f);
    ground.restitution = 0.8f;

    world.addBody(&ground);
    world.addBody(&body);
    bodies.push_back(&body);
    bodies.push_back(&ground);

    boxTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 60, 60);
    SDL_SetRenderTarget(renderer, boxTex);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderTarget(renderer, nullptr);

    running = true;
    return true;
}

void Game::handleEvent()
{
    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        ImGui_ImplSDL3_ProcessEvent(&e);

        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
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
                addBody();
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
    showFPS(dt);

    world.startFrame();

    for (auto &it : bodies)
    {
        if (it->inverseMass > 0.0f)
        {
            it->addForce(Vector2(0, 0));
        }
    }

    world.step(dt, 6);

        for (auto *b : bodies)
        {
            b->c = {255, 255, 255, 255};
        }
    // if (!contacts.empty())
    // {
    //     std::cout << "Actual collisions: " << contacts.size() << std::endl;

    //     for (auto *b : bodies)
    //     {
    //         b->c = {255, 255, 255, 255};
    //     }

    //     for (auto &contact : contacts)
    //     {
    //         contact.a->c = {0, 255, 0, 255};
    //         contact.b->c = {0, 255, 0, 255};

    //         CollisionResolve::Solve(contact, dt);
    //     }
    // }
    // else
    // {

    // }
}

void Game::render()
{
    imguiAddBodyMenu();

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

    // std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
    // CoarseCollision::FindPotentialPairs(&world, potentialPairs);

    // std::vector<Contact> contacts;
    // NarrowCollision::FindContacts(&world, potentialPairs, contacts);

    // for (auto it : contacts)
    // {
    //     for(auto it2 : it.contactPoints){
    //         std::cout<<it2.x<<" "<<it2.y<<std::endl;
    //         Vector2 screenPoint = WorldToScreen(it2, WINDOW_H);
    // DrawCircle(renderer, screenPoint.x, screenPoint.y, 5);
    //     }
    // }

    ImGui::Render();
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

    SDL_RenderPresent(renderer);
}

void Game::addBody()
{
    static bool seeded = false;
    if (!seeded)
    {
        std::srand((unsigned)std::time(nullptr));
        seeded = true;
    }

    RigidBody *b = new RigidBody();

    float margin = 60.0f;
    float x = margin + static_cast<float>(std::rand()) / RAND_MAX * (WINDOW_W - 2 * margin);
    float y = margin + static_cast<float>(std::rand()) / RAND_MAX * (WINDOW_H * 0.4f);

    b->position = Vector2(x, y);

    b->inverseMass = 1.0f;
    // Calculate proper inertia for a rectangle: I = (1/12) * m * (w² + h²)

    // {
    //     float width = b->aabb.halfSize.x * 2;
    //     float height = b->aabb.halfSize.y * 2;
    //     float mass = 1.0f / b->inverseMass;
    //     float inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
    //     b->inverseInertia = 1.0f / inertia;
    // }
    // else
    {
        b->shapeType = ShapeType::CIRCLE;
        b->circle.radius = 10 + rand() % 50;
    }

    b->velocity = Vector2(0, 0);
    b->angularDamping = 0.99f;
    b->rotation = 0.0f;

    b->calculateDerivativeData();

    world.addBody(b);
    bodies.push_back(b);
}

void Game::showFPS(float dt)
{
    fps = 1.0f / dt;
    frameTimeMs = dt * 1000.0f;

    char title[128];
    snprintf(title, sizeof(title),
             "AccelEngine | FPS: %.1f | Frame: %.2f ms | Bodies: %zu",
             fps, frameTimeMs, bodies.size());

    SDL_SetWindowTitle(window, title);
}

void Game::imguiAddBodyMenu()
{
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    // --- Scene Editor UI ---
    ImGui::Begin("Scene Editor");

    if (ImGui::Button("Add Body"))
        addBody();

    static int selectedIndex = -1;
    ImGui::Separator();
    ImGui::Text("Bodies:");

    for (int i = 0; i < bodies.size(); ++i)
    {
        char label[32];
        snprintf(label, sizeof(label), "Body %d", i);
        if (ImGui::Selectable(label, selectedIndex == i))
            selectedIndex = i;
    }

    if (selectedIndex >= 0 && selectedIndex < bodies.size())
    {
        RigidBody *selected = bodies[selectedIndex];
        ImGui::Separator();
        ImGui::Text("Transform");
        ImGui::DragFloat2("Position", (float *)&selected->position, 1.0f);
        ImGui::SliderAngle("Orientation", (float *)&selected->orientation, -180.0f, 180.0f);
        ImGui::DragFloat2("Velocity", (float *)&selected->velocity, 0.5f);
    }

    ImGui::End();
}

void Game::addCircle(float x, float y)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 1.0f;  
    b->restitution = 0.8f;

    b->shapeType = ShapeType::CIRCLE;
    b->circle.radius = 10 + rand() % 50;

    b->velocity = {0, 0};
    b->calculateDerivativeData();

    world.addBody(b);
    bodies.push_back(b);
}

void Game::addAABB(float x, float y)
{
    RigidBody *b = new RigidBody();
    b->position = {x, y};
    b->inverseMass = 1.0f;

    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = {20 + rand() % 50, 20 + rand() % 50}; // random sizes

    b->velocity = {0, 0};
    b->calculateDerivativeData();

    world.addBody(b);
    bodies.push_back(b);
}
