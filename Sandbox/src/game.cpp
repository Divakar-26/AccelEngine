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

    // init UI
    if(!ui.init(window, renderer)){
        std::cout<<"Somethine wrong with IMGUI ----- Cannot Initialised"<<std::endl;
        return false;
    }

    Renderer2D::init(renderer, WINDOW_W, WINDOW_H);

    //body
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

    // ground
    ground.position = Vector2(927, -247);
    ground.inverseMass = 0.0f;            
    ground.shapeType = ShapeType::AABB;
    ground.aabb.halfSize = Vector2(800.0f, 50.0f);
    ground.restitution = 0.8f;

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
    //handle ui

    
    while (SDL_PollEvent(&e))
    {
        if(ui.handleEvent(e)){
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
                ui.addBody(world,bodies);
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

    std::cout << dt << std::endl;

    world.step(dt, 10);

    for (auto *b : bodies)
    {
        b->c = {255, 255, 255, 255};
    }
}

void Game::render()
{
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

    std::vector<Contact> contacts = world.getContacts();

    for (auto it : contacts)
    {
        for (auto it2 : it.contactPoints)
        {
            Renderer2D::DrawCircle(it2.x, it2.y, 3, SDL_Color{255,0,0,255});
        }
    }


    ui.DrawFrame();
    SDL_RenderPresent(renderer);
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
    b->restitution = 0.0f;

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
