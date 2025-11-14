#pragma once
#include <SDL3/SDL.h>

class Game;

class InputActions {
public:
    void init(Game* game);

    void onKeyDown(const SDL_KeyboardEvent& e);
    void onKeyUp(const SDL_KeyboardEvent& e);
    void onMouseDown(const SDL_MouseButtonEvent& e);
    void onMouseUp(const SDL_MouseButtonEvent& e);
    void onMouseMove(const SDL_MouseMotionEvent& e);

private:
    Game* game = nullptr;
};
