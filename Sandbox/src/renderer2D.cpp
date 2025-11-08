#include "renderer2D.h"

SDL_Renderer *Renderer2D::renderer = nullptr;
SDL_Texture *Renderer2D::rectangle = nullptr;
SDL_Texture *Renderer2D::circle = nullptr;
int Renderer2D::WINDOW_W = 0;
int Renderer2D::WINDOW_H = 0;

void Renderer2D::init(SDL_Renderer *r, int w, int h)
{
    renderer = r;

    rectangle = GetTexture("../Sandbox/assets/rectangle.png");
    circle = GetTexture("../Sandbox/assets/circle.png");

    if (!rectangle || !circle)
    {
        SDL_Log("Cannot load images for rectangle or cirlce ", SDL_GetError());
    }

    WINDOW_W = w;
    WINDOW_H = h;
}

SDL_Texture *Renderer2D::GetTexture(const char *path)
{
    SDL_Surface *s = IMG_Load(path);
    SDL_Texture *t = SDL_CreateTextureFromSurface(renderer, s);

    SDL_DestroySurface(s);

    return t;
}

void Renderer2D::DrawRectangle(float x, float y, float w, float h, float orientation, SDL_Color outline)
{
    SDL_FRect dst = {
        x - w / 2,
        WINDOW_H - y - h / 2,
        w,
        h};

    SDL_FPoint center = {w / 2, h / 2};
    SDL_SetTextureColorMod(rectangle, outline.r, outline.g, outline.b);
    SDL_SetTextureAlphaMod(rectangle, outline.a);

    float degrees = -orientation * (180.0f / 3.14159);

    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &dst, degrees, &center, SDL_FLIP_NONE);
}

void Renderer2D::DrawCircle(float x, float y, float radius, SDL_Color outline)
{
    SDL_FRect dst = {
        x - radius,
        WINDOW_H - y - radius,
        radius * 2,
        radius  *2};

    SDL_FPoint center = {x, y};
    SDL_SetTextureColorMod(circle, outline.r, outline.g, outline.b);
    SDL_SetTextureAlphaMod(circle, outline.a);

    SDL_RenderTextureRotated(renderer, circle, nullptr, &dst, 0, &center, SDL_FLIP_NONE);
}