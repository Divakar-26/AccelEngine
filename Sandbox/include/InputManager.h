#pragma once
#include <SDL3/SDL.h>

class InputActions;

class InputManager {
public:
    void init(InputActions* actions);
    void process(SDL_Event& e);

private:
    InputActions* actions = nullptr;
};
