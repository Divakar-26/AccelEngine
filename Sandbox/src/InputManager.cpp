#include "InputManager.h"
#include "InputActions.h"

void InputManager::init(InputActions* a) {
    actions = a;
}

void InputManager::process(SDL_Event& e) {

    switch (e.type) {

    case SDL_EVENT_KEY_DOWN:
        actions->onKeyDown(e.key);
        break;

    case SDL_EVENT_KEY_UP:
        actions->onKeyUp(e.key);
        break;

    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        actions->onMouseDown(e.button);
        break;

    case SDL_EVENT_MOUSE_BUTTON_UP:
        actions->onMouseUp(e.button);
        break;

    case SDL_EVENT_MOUSE_MOTION:
        actions->onMouseMove(e.motion);
        break;
    }
}
