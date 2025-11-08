#pragma once

#include <SDL3/SDL.h>
#include <vector>
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include "UI.h"



using namespace AccelEngine;

class Game
{
public:
    Game(int W_W, int W_H);

    bool Init(const char *title);
    void handleEvent();
    void update(float dt);
    void render();

    bool isRunning()
    {
        return running;
    }

    void showFPS(float realDt, float fixedDt);

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

    UI ui;

    bool running;

    std::vector<RigidBody *> bodies;
    World world;

    RigidBody body;
    RigidBody ground;

    float fps = 0.0f;
    float frameTimeMs = 0.0f;
};