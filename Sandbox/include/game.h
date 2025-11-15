#pragma once
#include <SDL3/SDL.h>
#include <vector>
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/world.h>
#include <renderer2D.h>
#include "UI.h"
#include "InputManager.h"
#include "InputActions.h"

using namespace AccelEngine;

class InputManager;
class InputActions;

class Game
{
public:
    Game(int W_W, int W_H);

    bool Init(const char *title);
    void handleEvent();
    void cameraControls(const SDL_KeyboardEvent &  e);
    void update(float dt);
    void render();

    bool isRunning()
    {
        return running;
    }
    void showFPS(float realDt, float fixedDt);
    Vector2 WorldToScreen(const Vector2 &w, float screenHeight);
    Vector2 ScreenToWorld(const Vector2 &s, float screenHeight);
    Vector2 getMouseWorld();

    void gradBodies(float mx, float my);
    void addCircle(float x, float y);
    void addAABB(float x, float y);
    void addAABB(float x, float y, float w, float h, float orientation);

    RigidBody *getBody(float x, float y, float w, float h, float mass, float orientation, SDL_Color c);

    bool spawnBoxHeld = false;
    bool spawnCircleHeld = false;
    bool mouseDown = false;

    RigidBody *grabbed = nullptr;
    Vector2 grabOffset;

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

    Camera camera;

    InputManager *inputMgr;
    InputActions *inputAct;
};