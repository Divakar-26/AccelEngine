#include "InputActions.h"
#include "game.h"       // source file include is allowed, no circular include

void InputActions::init(Game* g) {
    game = g;
}

void InputActions::onKeyDown(const SDL_KeyboardEvent& e) {
    switch (e.key) {

    case SDLK_B:
        game->spawnBoxHeld = true;
        break;

    case SDLK_C:
        game->spawnCircleHeld = true;
        break;

    default:
        game->cameraControls(e);
    }
}

void InputActions::onKeyUp(const SDL_KeyboardEvent& e) {
    if (e.key == SDLK_B) game->spawnBoxHeld = false;
    if (e.key == SDLK_C) game->spawnCircleHeld = false;
}

void InputActions::onMouseDown(const SDL_MouseButtonEvent& e) {
    float mx, my;
    SDL_FPoint w = Renderer2D::screenToWorld(e.x, e.y);
    mx = w.x;
    my = w.y;

    game->mouseDown = true;

    if (e.button == SDL_BUTTON_LEFT)
        game->gradBodies(mx, my);
    else if (e.button == SDL_BUTTON_RIGHT)
        game->addAABB(mx, my);
    else if (e.button == SDL_BUTTON_MIDDLE)
        game->addCircle(mx, my);
}

void InputActions::onMouseUp(const SDL_MouseButtonEvent& e) {
    if (e.button == SDL_BUTTON_LEFT) {
        game->grabbed = nullptr;
        game->mouseDown = false;
    }
}

void InputActions::onMouseMove(const SDL_MouseMotionEvent& e) {
    if (!game->grabbed) return;

    SDL_FPoint w = Renderer2D::screenToWorld(e.x, e.y);
    Vector2 target = Vector2(w.x, w.y) + game->grabOffset;

    game->grabbed->position = target;
    game->grabbed->velocity = {0,0};
    game->grabbed->rotation = 0;
}
