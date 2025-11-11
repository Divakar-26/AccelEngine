#include "game.h"
#include <iostream>
int main()
{
    Game game(1920, 1000);
    game.Init("hello");

    const float FIXED_DT = 1.0f / 60.0f;

    Uint64 lastTime = SDL_GetTicks();
    float accumulator = 0.0f;

    while (game.isRunning())
    {
        Uint64 now = SDL_GetTicks();
        float realDt = (now - lastTime) / 1000.0f;
        lastTime = now;

        if (realDt > 0.25f)
            realDt = 0.25f;

        accumulator += realDt;

        game.handleEvent();

        while (accumulator >= FIXED_DT)
        {
            game.update(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        game.render();

        static float perfTimer = 0.0f;
        perfTimer += realDt;

        if (perfTimer >= 0.5f) // print every half second
        {
            std::cout << "Step time: " << realDt * 1000.0f << " ms" << std::endl;
            perfTimer = 0.0f;
        }

        static float titleTimer = 0.0f;
        titleTimer += realDt;

        if (titleTimer >= 1.0f)
        {
            game.showFPS(realDt, FIXED_DT);
            titleTimer = 0.0f;
        }
    }

    return 0;
}
