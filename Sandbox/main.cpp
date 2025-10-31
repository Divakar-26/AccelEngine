#include <SDL3/SDL.h>
#include <iostream>
#include <AccelEngine/core.h>
#include <AccelEngine/particle.h>
#include <AccelEngine/pfgen.h>

using namespace AccelEngine;

int main()
{
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("Physics Test", 800, 600, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);

    Particle p;
    p.inverseMass = 1.0f;
    // p.damping = 0.99f;
    p.position = Vector2(100, 100);
    p.acceleration = Vector2(0, 0);
    p.velocity = Vector2(0, 0);

    Particle p1;
    p1.inverseMass = 1.0f;
    // p.damping = 0.99f;
    p1.position = Vector2(500, 100);
    p1.acceleration = Vector2(0, 0);
    p1.velocity = Vector2(0, 0);

    // make a registry and also a force generator called gravity
    ParticleForceRegistry r;
    ParticleGravity gravity(Vector2(0, 0));

    // k = 100 and rest lenght = 300 px -> 100 = more spring like
    ParticleSpring spring1(&p1, 1.0f, 300.0f);
    ParticleSpring spring2(&p, 1.0f, 300.0f);
    r.add(&p, &spring1);
    r.add(&p1, &spring2);

    // add it in registry
    ParticleDrag drag(0.5f, 0.1f);
    r.add(&p, &drag);
    r.add(&p, &gravity);

    bool running = true;
    Uint64 lastTime = SDL_GetTicks();
    bool upHeld = false;
    while (running)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_EVENT_QUIT)
                running = false;
            if (e.type == SDL_EVENT_KEY_DOWN)
            {
                if (e.key.key == SDLK_RIGHT)
                {
                    p.position.x += 10.0f;
                }
                else if(e.key.key == SDLK_R){
                    p.position = Vector2(100,100);
                }
                
            }
        }

        Uint64 currentTime = SDL_GetTicks();
        float duration = (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        r.updateForces(duration);
        p.integrate(duration);
        p1.integrate(duration);

        SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
        SDL_FRect rect = {p.position.x, p.position.y, 100, 100};
        SDL_RenderFillRect(renderer, &rect);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_FRect rect2 = {p1.position.x, p1.position.y, 100, 100};
        SDL_RenderFillRect(renderer, &rect2);

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_RenderLine(renderer, p.position.x + 50, p.position.y + 50,
                       p1.position.x + 50, p1.position.y + 50);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
