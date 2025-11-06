#include "game.h"
#include <iostream>
#include <AccelEngine/collision_coarse.h>

using namespace AccelEngine;

SDL_Texture *boxTex;

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

    // ----------------------------------------
    body.position = Vector2(300, 300);
    body.inverseMass = 1.0f;
    body.orientation = 0;
    body.velocity = Vector2(0, 0);
    body.angularDamping = 0.99f;
    // this is angular velocity -> it calms down after feew second becuase of anhgular damping
    body.rotation = 1.0f;
    world.addBody(&body);

    bodies.push_back(&body);

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
                body.velocity = Vector2(10, 0);
                break;
            case SDLK_A:
                body.velocity = Vector2(-10, 0);
                break;
            case SDLK_S:
                body.velocity = Vector2(0, 10);
                break;
            case SDLK_W:
                body.velocity = Vector2(0, -10);
                break;
            case SDLK_LCTRL:
                addBody();
                break;
            default:
                break;
            }
        }
        else if (e.type == SDL_EVENT_KEY_UP)
        {
            body.velocity = Vector2(0, 0);
        }
    }
}

void Game::update(float dt)
{

    world.startFrame();
    world.runPhysics(dt);

    // Coarse collision check
    std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
    CoarseCollision::FindPotentialPairs(&world, potentialPairs);

    if (!potentialPairs.empty())
    {
        std::cout << "Possible collisions: " << potentialPairs.size() << std::endl;
    }
}

void Game::render()
{
    // --- Start ImGui frame ---
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

    // --- Render scene ---
    SDL_SetRenderDrawColor(renderer, 25, 25, 40, 255);
    SDL_RenderClear(renderer);

    for (auto *b : bodies)
    {
        SDL_FRect dst = {b->position.x, b->position.y, 60, 60};
        SDL_FPoint center = {30, 30};

        // apply per-body tint color
        SDL_SetTextureColorMod(boxTex, b->c.r, b->c.g, b->c.b);
        SDL_SetTextureAlphaMod(boxTex, b->c.a);

        SDL_RenderTextureRotated(renderer, boxTex, nullptr, &dst,
                                 b->orientation, &center, SDL_FLIP_NONE);
    }

    // --- Draw ImGui ---
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
    float y = margin + static_cast<float>(std::rand()) / RAND_MAX * (WINDOW_H - 2 * margin);

    b->position = Vector2(x, y);

    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = Vector2(30.0f, 30.0f); // matches your 60x60 rendering

    b->inverseMass = 1.0f;
    b->velocity = Vector2(0, 0);
    b->angularDamping = 0.99f;
    b->rotation = 0.0f;

    b->calculateDerivativeData();

    world.addBody(b);
    bodies.push_back(b);
}
