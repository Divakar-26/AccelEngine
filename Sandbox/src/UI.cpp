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
    // First pass event to ImGui's backend
    ImGui_ImplSDL3_ProcessEvent(&e);

    ImGuiIO &io = ImGui::GetIO();

    if (io.WantCaptureMouse)
        return true;

    if (io.WantCaptureKeyboard)
        return true;

    return false;
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

    ImGui::Begin("Scene Editor", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // ============================================================
    // TOP: Creation + Global Controls
    // ============================================================

    ImGui::Text("Create Body");

    static int newShape = 0;
    ImGui::RadioButton("AABB", &newShape, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Circle", &newShape, 1);

    static float spawnW = 50;
    static float spawnH = 50;
    static float spawnR = 25;

    if (newShape == 0)
    {
        ImGui::DragFloat("Width", &spawnW, 1.f, 5.f, 500.f);
        ImGui::DragFloat("Height", &spawnH, 1.f, 5.f, 500.f);
    }
    else
    {
        ImGui::DragFloat("Radius", &spawnR, 1.f, 5.f, 500.f);
    }

    static bool spawnCollisionEnabled = true;
    ImGui::Checkbox("Enable Collision for New Bodies",&spawnCollisionEnabled);

    if (ImGui::Button("Add Body"))
    {
        RigidBody* b = new RigidBody();

        b->position = {500, 500};
        b->velocity = {0, 0};

        if (newShape == 0)
        {
            b->shapeType = ShapeType::AABB;
            b->aabb.halfSize = {spawnW / 2, spawnH / 2};

            float mass = 1.f;
            b->inverseMass = 1.f / mass;

            float inertia = (1.f/12.f) * mass * (spawnW*spawnW + spawnH*spawnH);
            b->inverseInertia = 1.f / inertia;
        }
        else
        {
            b->shapeType = ShapeType::CIRCLE;
            b->circle.radius = spawnR;

            float mass = 1.f;
            b->inverseMass = 1.f / mass;

            float inertia = 0.5f * mass * spawnR * spawnR;
            b->inverseInertia = 1.f / inertia;
        }

        b->enableCollision = spawnCollisionEnabled;
        b->calculateDerivativeData();
        world.addBody(b);
        bodies.push_back(b);
    }

    ImGui::Separator();

    // ============================================================
    // SELECTED BODY INSPECTOR
    // ============================================================

    static int selectedIndex = -1;

    ImGui::Text("Inspector");
    if (selectedIndex >= 0 && selectedIndex < bodies.size())
    {
        RigidBody* b = bodies[selectedIndex];

        ImGui::BeginChild("InspectorSection", ImVec2(0, 250), true);

        ImGui::Text("Body %d", selectedIndex);

        ImGui::Checkbox("Enable Collision", &b->enableCollision);

        ImGui::DragFloat2("Position", (float*)&b->position, 0.5f);

        float degrees = b->orientation * 180.0f / 3.14159265f;
        ImGui::DragFloat("Rotation (deg)", &degrees, 0.2f, -180.f, 180.f, "%.2f");
        b->orientation = degrees * 3.14159265f / 180.f;

        ImGui::DragFloat2("Velocity", (float*)&b->velocity, 0.2f);
        ImGui::DragFloat("Angular Vel", &b->rotation, 0.1f);
        ImGui::DragFloat("Angular Damping", &b->angularDamping, 0.01f, 0.f, 1.f);

        float mass = 1.f / b->inverseMass;
        if (ImGui::DragFloat("Mass", &mass, 0.1f, 0.01f, 10000.f))
            b->inverseMass = 1.f / mass;

        float inertia = b->inverseInertia > 0 ? 1.f / b->inverseInertia : 0.f;
        if (ImGui::DragFloat("Inertia", &inertia, 0.1f, 0.01f, 100000.f))
            b->inverseInertia = 1.f / inertia;

        ImGui::DragFloat("Restitution", &b->restitution, 0.01f, 0.f, 1.f);
        ImGui::DragFloat("Orientation (raw)", &b->orientation, 0.01f, -10000.f, 10000.f, "%.4f");


        if (b->shapeType == ShapeType::AABB)
        {
            ImGui::Text("AABB");
            ImGui::DragFloat2("Half Size", (float*)&b->aabb.halfSize, 1.f);
        }
        else
        {
            ImGui::Text("Circle");
            ImGui::DragFloat("Radius", &b->circle.radius, 1.f, 1.f, 500.f);
        }

        ImGui::ColorEdit4("Color", (float*)&b->c);

        ImGui::EndChild();
    }

    ImGui::Separator();

    // ============================================================
    // BODY LIST AT BOTTOM
    // ============================================================

    ImGui::Text("Bodies");

    ImGui::BeginChild("BodiesList", ImVec2(0, 200), true);

    for (int i = 0; i < bodies.size(); i++)
    {
        char label[64];
        snprintf(label, sizeof(label), "Body %d", i);

        if (ImGui::Selectable(label, selectedIndex == i))
            selectedIndex = i;
    }

    ImGui::EndChild();

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