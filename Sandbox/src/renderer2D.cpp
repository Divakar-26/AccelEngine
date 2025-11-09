#include "renderer2D.h"

SDL_Renderer *Renderer2D::renderer = nullptr;
SDL_Texture *Renderer2D::rectangle = nullptr;
SDL_Texture *Renderer2D::circle = nullptr;
int Renderer2D::WINDOW_W = 0;
int Renderer2D::WINDOW_H = 0;

static const float OUTLINE_THICKNESS = 2.0f;

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

void Renderer2D::DrawRectangle(float x, float y, float w, float h, float orientation, SDL_Color fill)
{
    float degrees = -orientation * (180.0f / 3.14159f);

    float sx = x - w/2;
    float sy = WINDOW_H - y - h/2;

    float inset = OUTLINE_THICKNESS;

    // ----- Outer rect pivot -----
    SDL_FPoint outerCenter = { w/2, h/2 };

    // ----- Inner rect pivot -----
    float innerW = w - inset * 2;
    float innerH = h - inset * 2;
    SDL_FPoint innerCenter = { innerW / 2, innerH / 2 };

    // --------------------
    // 1. WHITE OUTER RECT
    // --------------------
    SDL_FRect outerRect = { sx, sy, w, h };

    SDL_SetTextureColorMod(rectangle, 255, 255, 255);
    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &outerRect,
                             degrees, &outerCenter, SDL_FLIP_NONE);

    // --------------------
    // 2. COLORED INNER RECT
    // --------------------
    SDL_FRect innerRect = { sx + inset, sy + inset, innerW, innerH };

    SDL_SetTextureColorMod(rectangle, fill.r, fill.g, fill.b);
    SDL_RenderTextureRotated(renderer, rectangle, nullptr, &innerRect,
                             degrees, &innerCenter, SDL_FLIP_NONE);
}



void Renderer2D::DrawCircle(float x, float y, float radius, SDL_Color outline)
{
    // ---- 1. White outline ----
    SDL_FRect outlineDst = {
        x - radius - OUTLINE_THICKNESS,
        WINDOW_H - y - radius - OUTLINE_THICKNESS,
        (radius * 2) + OUTLINE_THICKNESS * 2,
        (radius * 2) + OUTLINE_THICKNESS * 2};

    SDL_FPoint center = {radius, radius};

    SDL_SetTextureColorMod(circle, 255, 255, 255);
    SDL_SetTextureAlphaMod(circle, 255);
    SDL_RenderTextureRotated(renderer, circle, nullptr, &outlineDst, 0, &center, SDL_FLIP_NONE);

    // ---- 2. Draw body color ----
    SDL_FRect dst = {
        x - radius,
        WINDOW_H - y - radius,
        radius * 2,
        radius * 2};

    SDL_SetTextureColorMod(circle, outline.r, outline.g, outline.b);
    SDL_SetTextureAlphaMod(circle, outline.a);

    SDL_RenderTextureRotated(renderer, circle, nullptr, &dst, 0, &center, SDL_FLIP_NONE);
}
