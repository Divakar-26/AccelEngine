#pragma once

#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <AccelEngine/ForceRegistry.h>

struct Camera{
    float x, y;
    float zoom = 1.0f;
    // right now now rotation
};

class Renderer2D
{
    public:

        static void init(SDL_Renderer * renderer, int w, int h);
        static void DrawRectangle(float x, float y, float w, float h, float orientation , SDL_Color outline);
        static void DrawCircle(float x, float y , float radius, float orientation,  SDL_Color outline);

        static void drawSprings(AccelEngine::ForceRegistry & registry);
        static SDL_Texture * GetTexture(const char * path);
        static SDL_Texture * rectangle;
        static SDL_Texture * circle;    

        static int WINDOW_W, WINDOW_H;
        static Camera camera;
        static void setCamera(const Camera & c);
        static SDL_FPoint worldToScreen(float x, float y);
        static SDL_FPoint screenToWorld(float sx, float sy);

    private:
        static SDL_Renderer * renderer;
};