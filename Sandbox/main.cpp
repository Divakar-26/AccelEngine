#include <SDL3/SDL.h>
#include <iostream>
#include <AccelEngine/core.h>
#include <AccelEngine/particle.h>
#include <AccelEngine/pfgen.h>
#include <AccelEngine/ParticleContact.h>
#include <AccelEngine/ParticleWorld.h>

using namespace AccelEngine;

int main()
{
    if (SDL_Init(SDL_INIT_VIDEO) == 0)
    {
        std::cerr << "SDL failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    SDL_Window *window = SDL_CreateWindow("Physics Test", 800, 600, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);

    Particle p;
    p.inverseMass = 1.0f;
    p.position = Vector2(100, 100);
    p.acceleration = Vector2(0, 0);
    p.velocity = Vector2(0, 0);

    Particle ground;
    ground.inverseMass = 0.00001f; 
    ground.position = Vector2(120, 500);
    ground.acceleration = Vector2(0, 0);
    ground.velocity = Vector2(0, 0);


    ParticleGravity gravity(Vector2(0, 980)); 
    ParticleDrag drag(0, 0);

    ParticleWorld world(100, 100);
    world.addParticle(&p);
    world.addParticle(&ground);
    world.getForceRegistry().add(&p, &gravity);
    world.getForceRegistry().add(&p, &drag);

    ParticleContact contact;

    bool running = true;


    const float FIXED_DT = 1.0f / 60.0f; 
    float accumulator = 0.0f;
    Uint64 lastTime = SDL_GetTicks();

    const float size = 100.0f;

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
                    p.velocity.x += 200.0f;
                else if (e.key.key == SDLK_LEFT)
                    p.velocity.x -= 200.0f;
                else if (e.key.key == SDLK_R)
                {
                    p.position = Vector2(100, 100);
                    p.velocity = Vector2(0, 0);
                }
            }
        }

        Uint64 currentTime = SDL_GetTicks();
        float frameTime = (currentTime - lastTime) / 1000.0f;
        if (frameTime > 0.25f) frameTime = 0.25f;
        lastTime = currentTime;
        accumulator += frameTime;


        while (accumulator >= FIXED_DT)
        {
            world.startFrame();


            if (p.position.x < ground.position.x + size &&
                p.position.x + size > ground.position.x &&
                p.position.y < ground.position.y + size &&
                p.position.y + size > ground.position.y)
            {
                contact.particle[0] = &p;
                contact.particle[1] = &ground;

                Vector2 normal = (p.position - ground.position);
                normal.normalize();
                Vector2 relativeVel = p.getVelocity() - ground.getVelocity();
                if (relativeVel * normal > 0)
                    normal.invert();

                contact.contactNormal = normal;
                contact.restitution = 0.0f;
                contact.resolve(FIXED_DT);
            }

            world.runPhysics(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
        SDL_FRect rectP = {p.position.x, p.position.y, size, size};
        SDL_RenderFillRect(renderer, &rectP);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_FRect rectG = {ground.position.x, ground.position.y, size, size};
        SDL_RenderFillRect(renderer, &rectG);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
