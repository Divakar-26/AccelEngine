#include <SDL3/SDL.h>
#include <iostream>
#include <AccelEngine/core.h>
#include <AccelEngine/body.h>
#include <AccelEngine/ForceGenerator.h>
#include <AccelEngine/world.h>

using namespace AccelEngine;

int main()
{
    if (SDL_Init(SDL_INIT_VIDEO) == 0)
    {
        std::cerr << "SDL failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_Window *window = SDL_CreateWindow("RigidBody World", 800, 600, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);

    // =========================
    // SETUP PHYSICS WORLD
    // =========================
    World world;

    // --- Create one rigid body ---
    RigidBody body;
    body.inverseMass = 1.0f / 5.0f;    // 5 kg
    body.inverseInertia = 1.0f / 2.0f; // rotational inertia
    body.position = Vector2(400, 100);
    body.orientation = 0;
    body.linearDamping = 0.99f;
    body.angularDamping = 0.9995f;
    body.velocity = Vector2(0, 0);

    world.addBody(&body);

    // --- Create environment forces ---
    Gravity gravity(Vector2(0, 100)); // Downward force

    // Waterline at y = 400
    Buoyancy buoy(Vector2(0, 0), 50.0f, 2.0f, 400.0f, 1000.0f);

    // Aerodynamic drag-like surface
    Matrix2 dragTensor;
    dragTensor.set(-0.5f, 0.0f, 0.0f, -0.5f);
    Aero aero(dragTensor, Vector2(0, 0));

    // Optional simple wind
    Vector2 wind(5000.0f, 0.0f);

    // =========================
    // SDL LOOP
    // =========================
    const float FIXED_DT = 1.0f / 60.0f;
    Uint64 lastTime = SDL_GetTicks();
    bool running = true;

    while (running)
    {
        // ------------------------
        // EVENT HANDLING
        // ------------------------
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_EVENT_QUIT)
                running = false;

            if (e.type == SDL_EVENT_KEY_DOWN)
            {
                if (e.key.key == SDLK_RIGHT)
                    body.addForce(Vector2(500.0f, 0));
                else if (e.key.key == SDLK_LEFT)
                    body.addForce(Vector2(-500.0f, 0));
                else if (e.key.key == SDLK_UP)
                    body.addForce(Vector2(0, -1500.0f));
                else if (e.key.key == SDLK_SPACE)
                    body.addForceAtBodyPoint(Vector2(0, -2000.0f), Vector2(25, 0));
            }
        }

        // ------------------------
        // FIXED TIMESTEP
        // ------------------------
        Uint64 now = SDL_GetTicks();
        float frameTime = (now - lastTime) / 1000.0f;
        lastTime = now;

        // ------------------------
        // PHYSICS UPDATE
        // ------------------------
        world.startFrame();

        Vector2 localPoint(-30, -15); // top-right corner of box
        body.velocity.x += 1;
        // Apply forces manually (no registry yet)
        gravity.updateForce(&body, FIXED_DT);
        buoy.updateForce(&body, FIXED_DT);
        aero.updateForce(&body, FIXED_DT);

        // Run physics on all bodies
        world.runPhysics(FIXED_DT);

        // ------------------------
        // RENDERING
        // ------------------------
        SDL_SetRenderDrawColor(renderer, 25, 25, 40, 255);
        SDL_RenderClear(renderer);

        std::cout << "y: " << body.position.y;

        SDL_Texture *boxTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 60, 60);
        SDL_SetRenderTarget(renderer, boxTex);
        SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderTarget(renderer, nullptr);

        // Draw waterline
        SDL_SetRenderDrawColor(renderer, 60, 80, 180, 180);
        SDL_RenderLine(renderer, 0, 400, 800, 400);

        Vector2 worldPoint = body.getPointInWorldSpace(Vector2(-30, -15));
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderLine(renderer, worldPoint.x, worldPoint.y,
                       worldPoint.x + 50, worldPoint.y); // short line showing force dir

        // Draw rigid body
        SDL_FRect dst = {body.position.x, body.position.y, 60, 60};
        SDL_FPoint center = {30, 30}; // rotation pivot = center of box
        SDL_RenderTextureRotated(renderer, boxTex, nullptr, &dst, body.orientation, &center, SDL_FLIP_NONE);

        SDL_RenderPresent(renderer);
        SDL_Delay(16); // ~60 FPS
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
