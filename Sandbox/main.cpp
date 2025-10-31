    #include <SDL3/SDL.h>
    #include <iostream>
    #include <AccelEngine/core.h>
    #include <AccelEngine/particle.h>

    using namespace AccelEngine;

    int main()
    {
        SDL_Init(SDL_INIT_VIDEO);
        SDL_Window *window = SDL_CreateWindow("Physics Test", 800, 600, SDL_WINDOW_RESIZABLE);
        SDL_Renderer *renderer = SDL_CreateRenderer(window, nullptr);

        Particle p;
        p.inverseMass = 1.0f;
        // p.damping = 0.99f;
        p.position = Vector3(100, 100, 0);
        p.acceleration = Vector3(0,980,0);
        p.velocity = Vector3(100, 0, 0);

        

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
                if(e.type == SDL_EVENT_KEY_DOWN){
                    upHeld = true;
                }
                else if(e.type == SDL_EVENT_KEY_UP){
                    upHeld = false;
                }
            }

            Uint64 currentTime = SDL_GetTicks();
            float duration = (currentTime - lastTime) / 1000.0f;
            lastTime = currentTime;

            p.addForce(Vector3(0, 10, 0));
            p.addForce(Vector3(0, -10, 0));

            // Integrate physics
            p.integrate(duration);

            SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
            SDL_RenderClear(renderer);

            SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
            SDL_FRect rect = {p.position.x, p.position.y, 100, 100};
            SDL_RenderFillRect(renderer, &rect);

            SDL_RenderPresent(renderer);
        }

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
