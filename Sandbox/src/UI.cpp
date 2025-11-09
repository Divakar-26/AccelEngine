#include "UI.h"

#include <SDL3/SDL.h>
#include <cstdlib> // for rand()
#include <ctime>   // for time()
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <iostream>

using namespace AccelEngine;

bool UI::init(SDL_Window *window, SDL_Renderer *renderer)
{
    this->renderer = renderer;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplSDL3_InitForSDLRenderer(window, renderer))
    {
        // SDL_Log("ImGui SDL3 init failed!");
        return false;
    }
    if (!ImGui_ImplSDLRenderer3_Init(renderer))
    {
        // SDL_Log("ImGui SDL3 renderer init failed!");
        return false;
    }

    return true;
}

bool UI::handleEvent(SDL_Event &e)
{
    ImGui_ImplSDL3_ProcessEvent(&e);

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureMouse)
    {
        return true;
    }
}

void UI::DrawFrame()
{


    ImGui::Render();
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
}

void UI::DrawAddBody(World &world, std::vector<RigidBody *> &bodies)
{
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    // --- Scene Editor UI ---
    ImGui::Begin("Scene Editor");

    if (ImGui::Button("Add Body"))
        addBody(world, bodies);

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

void UI::addBody(World &world, std::vector<RigidBody *> &bodies)
{
    static bool seeded = false;
    if (!seeded)
    {
        std::srand((unsigned)std::time(nullptr));
        seeded = true;
    }

    RigidBody *b = new RigidBody();

    float margin = 60.0f;
    float x = margin + static_cast<float>(std::rand()) / RAND_MAX * (1000 - 2 * margin);
    float y = margin + static_cast<float>(std::rand()) / RAND_MAX * (500 * 0.4f);

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