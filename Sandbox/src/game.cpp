#include "game.h"
#include <iostream>
#include <AccelEngine/collision_coarse.h>
#include <AccelEngine/narrow_collision.h>
#include <AccelEngine/collision_resolver.h>

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
    // FIXED: Position body in visible area (middle-top)
    body.position = Vector2(WINDOW_W / 2.0f, 100.0f);
    body.inverseMass = 1.0f;
    body.orientation = 0;
    body.velocity = Vector2(0, 0);
    body.angularDamping = 0.99f;
    body.rotation = 0.0f;
    body.shapeType = ShapeType::AABB;
    body.aabb.halfSize = Vector2(30.0f, 30.0f);
    body.linearDamping = 0.98f;
    body.inverseInertia = 1.0f / 30.0f;

    // FIXED: Position ground at bottom of screen
    ground.position = Vector2(WINDOW_W / 2.0f, WINDOW_H - 25.0f); // Center at bottom
    ground.inverseMass = 0.0f; // Use 0 for truly static objects
    ground.shapeType = ShapeType::AABB;
    ground.aabb.halfSize = Vector2(400.0f, 50.0f);

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
    // ADD GRAVITY to all dynamic bodies
    for (auto *b : bodies)
    {
        if (b->inverseMass > 0.0001f) // Only apply gravity to dynamic bodies
        {
            b->velocity = (Vector2(0, 100.0f));
        }
    }
    
    fps = 1.0f / dt;
    frameTimeMs = dt * 1000.0f;

    char title[128];
    snprintf(title, sizeof(title),
             "AccelEngine | FPS: %.1f | Frame: %.2f ms | Bodies: %zu",
             fps, frameTimeMs, bodies.size());

    SDL_SetWindowTitle(window, title);

    world.startFrame();
    world.runPhysics(dt);

    std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
    CoarseCollision::FindPotentialPairs(&world, potentialPairs);

    std::vector<Contact> contacts;
    NarrowCollision::FindContacts(&world, potentialPairs, contacts);

    if (!contacts.empty())
    {
        ContactResolver::ResolveContacts(contacts, dt); // ENABLE THIS!
        std::cout << "Actual collisions: " << contacts.size() << std::endl;
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

    // First render all bodies
    for (auto *b : bodies)
    {
        SDL_FRect dst = {
            b->position.x - b->aabb.halfSize.x,
            b->position.y - b->aabb.halfSize.y,
            b->aabb.halfSize.x * 2,
            b->aabb.halfSize.y * 2};

        SDL_FPoint center = {30, 30};

        // apply per-body tint color
        SDL_SetTextureColorMod(boxTex, b->c.r, b->c.g, b->c.b);
        SDL_SetTextureAlphaMod(boxTex, b->c.a);

        float degrees = b->orientation * (180.0f / M_PI);
        SDL_RenderTextureRotated(renderer, boxTex, nullptr, &dst, degrees, &center, SDL_FLIP_NONE);
    }

    // --- Find and draw contact points ---
    std::vector<std::pair<RigidBody *, RigidBody *>> potentialPairs;
    CoarseCollision::FindPotentialPairs(&world, potentialPairs);

    std::vector<Contact> contacts;
    NarrowCollision::FindContacts(&world, potentialPairs, contacts);

    // Draw contact points as small green circles
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    for (const auto &contact : contacts)
    {
        for (int i = 0; i < contact.contactCount; ++i)
        {
            const Vector2 &point = contact.contactPoints[i];
            
            // Draw a small circle at the contact point
            const float radius = 4.0f;
            const int segments = 12;
            
            for (int j = 0; j < segments; ++j)
            {
                float angle1 = 2.0f * M_PI * j / segments;
                float angle2 = 2.0f * M_PI * (j + 1) / segments;
                
                SDL_FPoint p1 = {
                    point.x + radius * cosf(angle1),
                    point.y + radius * sinf(angle1)
                };
                SDL_FPoint p2 = {
                    point.x + radius * cosf(angle2),
                    point.y + radius * sinf(angle2)
                };
                
                SDL_RenderLine(renderer, p1.x, p1.y, p2.x, p2.y);
            }
            
            // Optional: Draw a small cross for better visibility
            SDL_RenderLine(renderer, point.x - 3, point.y, point.x + 3, point.y);
            SDL_RenderLine(renderer, point.x, point.y - 3, point.x, point.y + 3);
        }
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
    float y = margin + static_cast<float>(std::rand()) / RAND_MAX * (WINDOW_H * 0.4f);

    b->position = Vector2(x, y);
    b->shapeType = ShapeType::AABB;
    b->aabb.halfSize = Vector2(30.0f, 30.0f);

    b->inverseMass = 1.0f;
    // Calculate proper inertia for a rectangle: I = (1/12) * m * (w² + h²)
    float width = b->aabb.halfSize.x * 2;
    float height = b->aabb.halfSize.y * 2;
    float mass = 1.0f / b->inverseMass;
    float inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
    b->inverseInertia = 1.0f / inertia;

    b->velocity = Vector2(0, 0);
    b->angularDamping = 0.99f;
    b->rotation = 0.0f;

    b->calculateDerivativeData();

    world.addBody(b);
    bodies.push_back(b);
}