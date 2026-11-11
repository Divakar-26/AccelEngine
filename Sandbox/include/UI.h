#pragma once


#include <imgui.h> // context, backends and the draw pass belong to SIMPEngine's ImGuiLayer
#include <AccelEngine/world.h>
#include <vector>

using namespace AccelEngine;

class UI{
    public:


        void addBody(World & world, std::vector<RigidBody*> &bodies);

        void DrawAddBody(World & world, std::vector<RigidBody*> &bodies);
        

};