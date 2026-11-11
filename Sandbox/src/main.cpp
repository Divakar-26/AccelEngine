// AccelEngine Sandbox, running on SIMPEngine: its window, GL context, batched renderer and ImGui layer.
// The physics stays in this repository (AccelEngine); SIMPEngine only draws and delivers input.
#include "game.h"

#include <Engine/Application/Application.h>
#include <Engine/Core/Log.h>
#include <Engine/Core/VFS.h>
#include <Engine/Layer/Layer.h>

#include <SDL3/SDL.h>
#include <iostream>
#include <memory>
#include <vector>

namespace
{
    // Collects raw SDL events as they are queued; the layer replays them at the start of its update.
    // (SIMPEngine hands layers its own Event types, which carry no mouse position or modifier state.)
    std::vector<SDL_Event> g_events;

    bool collectEvent(void *, SDL_Event *e)
    {
        switch (e->type)
        {
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP:
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_MOUSE_MOTION:
        case SDL_EVENT_WINDOW_RESIZED:
            g_events.push_back(*e);
            break;
        default:
            break;
        }
        return true;
    }

    class SandboxLayer : public SIMPEngine::Layer
    {
    public:
        SandboxLayer() : Layer("Sandbox") {}

        void OnAttach() override
        {
            game = std::make_unique<Game>(1920, 1000);
            game->Init("AccelEngine Sandbox");
            SDL_AddEventWatch(collectEvent, nullptr);
        }

        void OnDetach() override
        {
            SDL_RemoveEventWatch(collectEvent, nullptr);
            game.reset();
        }

        void OnUpdate(SIMPEngine::TimeStep ts) override
        {
            std::vector<SDL_Event> events;
            events.swap(g_events);
            for (const SDL_Event &e : events)
                game->processEvent(e);

            // fixed 1/60 s steps; at most 2 per frame so heavy scenes slow down instead of collapsing
            const float FIXED_DT = 1.0f / 60.0f;
            const float realDt = std::min(ts.GetSeconds(), 0.25f);
            lastRealDt = realDt;
            accumulator += realDt;

            const int MAX_UPDATES = 2;
            int updates = 0;
            while (accumulator >= FIXED_DT && updates < MAX_UPDATES)
            {
                game->update(FIXED_DT);
                accumulator -= FIXED_DT;
                updates++;
            }
            if (updates == MAX_UPDATES)
                accumulator = 0.0f;

            titleTimer += realDt;
            if (titleTimer >= 1.0f && realDt > 0.0f)
            {
                game->showFPS(realDt, FIXED_DT);
                titleTimer = 0.0f;
            }
        }

        void OnRender() override
        {
            game->render();
        }

    private:
        std::unique_ptr<Game> game;
        float accumulator = 0.0f;
        float lastRealDt = 0.0f;
        float titleTimer = 0.0f;
    };

    class SandboxApp : public SIMPEngine::Application
    {
    public:
        static SIMPEngine::ApplicationSpecification spec()
        {
            SIMPEngine::ApplicationSpecification s;
            s.Title = "AccelEngine Sandbox";
            s.Width = 1920;
            s.Height = 1000;
            s.VSync = true;
            return s;
        }

        SandboxApp() : Application(spec())
        {
            PushLayer(std::make_unique<SandboxLayer>());
        }
    };
}

SIMPEngine::Application *SIMPEngine::CreateApplication()
{
    return new SandboxApp();
}

int main()
{
    SIMPEngine::Log::Init();
    auto *app = SIMPEngine::CreateApplication();
    app->Run();
    delete app;
    return 0;
}
