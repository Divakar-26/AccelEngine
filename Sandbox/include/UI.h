#pragma once


#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include <AccelEngine/world.h>
#include <vector>

using namespace AccelEngine;

class UI{
    public:

        bool init(SDL_Window * window, SDL_Renderer * renderer);
        void DrawFrame();
        bool handleEvent(SDL_Event & e);

        void addBody(World & world, std::vector<RigidBody*> &bodies);

        void DrawAddBody(World & world, std::vector<RigidBody*> &bodies);
        

    private:
        SDL_Renderer * renderer;

};