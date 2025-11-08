#pragma once

#include <SDL3/SDL.h>
#include <vector>
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"

#include <cstdlib> // for rand()
#include <ctime>   // for time()

using namespace AccelEngine;

class Game
{
public:
    Game(int W_W, int W_H);

    bool Init(const char *title);
    void handleEvent();
    void update(float dt);
    void render();

    SDL_Texture *getRotatedTexture(float w, float h, float rotation, SDL_Color c);

    bool isRunning()
    {
        return running;
    }

    void addBody();
    void showFPS(float dt);
    void imguiAddBodyMenu();

    inline Vector2 WorldToScreen(const Vector2 &w, float screenHeight)
    {
        return Vector2(w.x, screenHeight - w.y);
    }
    inline Vector2 ScreenToWorld(const Vector2 &s, float screenHeight)
    {
        return Vector2(s.x, screenHeight - s.y);
    }

    void addCircle(float x, float y);
    void addAABB(float x, float y);

private:
    int WINDOW_W, WINDOW_H;
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;

    bool running;

    std::vector<RigidBody *> bodies;
    World world;

    RigidBody body;
    RigidBody ground;

    float fps = 0.0f;
    float frameTimeMs = 0.0f;
};