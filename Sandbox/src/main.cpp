#include "game.h"

int main()
{
    Game game(800, 600);
    game.Init("hello");

    const float FIXED_DT = 1.0f / 60.0f; // physics tick rate (60 Hz)
    Uint64 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;

    while (game.isRunning())
    {
        Uint64 now = SDL_GetTicks();
        float frameTime = (now - lastTime) / 1000.0f; 
        lastTime = now;

        if (frameTime > 0.25f)
            frameTime = 0.25f;

        accumulator += frameTime;

        game.handleEvent();

        while (accumulator >= FIXED_DT)
        {
            game.update(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        game.render();
    }

    return 0;
}
