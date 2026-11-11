    #include "game.h"
    #include <iostream>
    #include <AccelEngine/collision_coarse.h>
    #include <AccelEngine/collision_narrow.h>
    #include <AccelEngine/collision_resolve.h>
    #include <AccelEngine/ForceRegistry.h>
    #include "renderer2D.h"
    #include "allDemo.h"
    #include <Engine/Application/Application.h>

    static float physicsTimeMs = 0.0f;
    static float renderTimeMs = 0.0f;
    static float physicsHistory[600] = {0};
    static float renderHistory[600] = {0};
    static int historyIndex = 0;

    bool showBVH = false;
    bool showSleeping = true; // draw sleeping bodies dimmed
bool showContacts = false;
    // Substeps per 1/60 s update (was a hard-coded 50). 6 keeps the joint demos (See-Saw) steady;
    // 4 is ~1.5x cheaper and fine for pure contact scenes, so lower it in the Physics panel for huge piles.
    int physicsSubsteps = 6;

    bool spawnBoxHeld = false;
    bool spawnCircleHeld = false;
    float spawnCooldown = 0.001f;
    float spawnTimer = 0.0f;

    Vector2 debugMouseWorld;
    bool showDebugCoords = true;

    RigidBody *grabbed = nullptr;
    Vector2 grabOffset;
    bool mouseDown = false;
    bool eraserActive = false;

    using namespace AccelEngine;

    ForceRegistry registry;
    Gravity *g;

    Demo *activeDemo = nullptr;
static int startDemoIndex = 0;
    std::vector<Demo *> demos;

    static Color RandomColor()
    {
        return {
            float(rand() % 256),
            float(rand() % 256),
            float(rand() % 256),
            255.0f};
    }

    void FindMinMax(const float *data, int size, float &outMin, float &outMax);

    Game::Game(int W_W, int W_H)
    {
        WINDOW_H = W_H;
        WINDOW_W = W_W;
    }

    bool Game::Init(const char *title)
    {
        // the window, GL context and ImGui come from SIMPEngine (SandboxApp in main.cpp)
        window = SIMPEngine::Application::Get().GetWindow().GetNativeWindow();
        SDL_SetWindowTitle(window, title);
        int ww = WINDOW_W, wh = WINDOW_H;
        SDL_GetWindowSize(window, &ww, &wh);
        WINDOW_W = ww;
        WINDOW_H = wh;

        Renderer2D::init(WINDOW_W, WINDOW_H);

        camera.x = 0;
        camera.y = 0;
        camera.zoom = 1.0f;

        inputAct = new InputActions();
        inputMgr = new InputManager();

        inputAct->init(this);
        inputMgr->init(inputAct);

        g = new Gravity(Vector2(0, -980));

        // with warm starting and 4-6 substeps, 2 velocity iterations per substep are enough
        world.settings.velocityIterations = 2;

        demos.push_back(new ShapesDemo());
        demos.push_back(new CarDemo());
        demos.push_back(new RagdollDemo());
        demos.push_back(new FilterDemo());
        demos.push_back(new PinballDemo());
        demos.push_back(new CatapultDemo());
        demos.push_back(new HillClimbDemo());
        demos.push_back(new BridgeHingeDemo());
        demos.push_back(new MarbleRunDemo());
        demos.push_back(new PortalDemo());
        demos.push_back(new TowerSmashDemo());
        demos.push_back(new LogoDemo());
        demos.push_back(new DamBreakDemo());
        demos.push_back(new GaltonDemo());
        demos.push_back(new WreckingBallDemo());
        demos.push_back(new GravityWellDemo());
        demos.push_back(new MixerDemo());
        demos.push_back(new HourglassDemo());
        demos.push_back(new ChainReactionDemo());
        demos.push_back(new SiegeDemo());
        demos.push_back(new GravityTiltDemo());
        demos.push_back(new BridgeDemo());
        demos.push_back(new SoftBodyDemo());
        demos.push_back(new SeeSawDemo());
        demos.push_back(new FrictionDemo());
        demos.push_back(new DominoDemo());
        demos.push_back(new ClothDemo());
        demos.push_back(new EngineDemo());
        demos.push_back(new RestitutionDemo());
        demos.push_back(new NewtonsCradle());
        demos.push_back(new StressDemo());
        
        // SANDBOX_DEMO=<index or name fragment> starts with that demo (handy for screenshots / automated checks)
        if (const char *want = getenv("SANDBOX_DEMO"))
        {
            for (int i = 0; i < (int)demos.size(); i++)
                if (std::to_string(i) == want || std::string(demos[i]->getName()).find(want) != std::string::npos)
                {
                    startDemoIndex = i;
                    break;
                }
        }
        activeDemo = demos[startDemoIndex];
        activeDemo->init(world, bodies, registry, g);

        running = true;
        return true;
    }

    void Game::processEvent(const SDL_Event &e)
    {
        if (e.type == SDL_EVENT_WINDOW_RESIZED)
        {
            WINDOW_W = e.window.data1;
            WINDOW_H = e.window.data2;
            Renderer2D::resize(WINDOW_W, WINDOW_H);
            return;
        }

        // keep the ImGui panels usable: while ImGui wants the mouse (or text input) the scene does not react.
        // (WantCaptureKeyboard is always true here: SIMPEngine enables keyboard navigation.)
        // (a button release always goes through so a grabbed body is never stuck to the cursor)
        ImGuiIO &io = ImGui::GetIO();
        const bool mouseEvent = e.type == SDL_EVENT_MOUSE_BUTTON_DOWN || e.type == SDL_EVENT_MOUSE_MOTION;
        const bool keyEvent = e.type == SDL_EVENT_KEY_DOWN;
        if ((mouseEvent && io.WantCaptureMouse) || (keyEvent && io.WantTextInput))
            return;

        Renderer2D::setCamera(camera);
        SDL_Event copy = e;
        inputMgr->process(copy);
        Renderer2D::setCamera(camera);
    }

    void Game::update(float dt)
    {

        aProfileData.clear();
        Renderer2D::setCamera(camera);
        spawnTimer -= dt;

        if (spawnTimer <= 0.0f)
        {
            float msx, msy;
            SDL_GetMouseState(&msx, &msy);

            SDL_FPoint w = Renderer2D::screenToWorld(msx, msy);
            float mx = w.x;
            float my = w.y;

            if (spawnBoxHeld)
            {
                addAABB(mx, my);
                spawnTimer = spawnCooldown;
            }

            if (spawnCircleHeld)
            {
                addCircle(mx, my);
                spawnTimer = spawnCooldown;
            }
        }

        const int substeps = physicsSubsteps;
        const real h = dt / (real)substeps;

        Uint64 startPhysics = SDL_GetPerformanceCounter();

        if (grabbed && mouseDown)
        {
            float sx, sy;
            SDL_GetMouseState(&sx, &sy);

            SDL_FPoint w = Renderer2D::screenToWorld(sx, sy);
            Vector2 target = Vector2(w.x, w.y) + grabOffset;

            float mx = w.x;
            float my = w.y;

            grabbed->position = target;
            grabbed->velocity = Vector2(0, 0);
            grabbed->rotation = 0;
            grabbed->wakeUp(); // otherwise a held body (zero velocity) would fall asleep
        }

        for (int i = 0; i < substeps; ++i)
        {
            world.startFrame();

            registry.updateForces(h);

            if (activeDemo)
            {
                activeDemo->update();
            }
            world.step(h, 1);
        }

        Uint64 endPhysics = SDL_GetPerformanceCounter();
        physicsTimeMs = (float)(endPhysics - startPhysics) * 1000.0f / SDL_GetPerformanceFrequency();
    }

    void Game::render()
    {
        static int sampleCounter = 0;

        Uint64 startRender = SDL_GetPerformanceCounter();

        // many thousands of bodies: drop the extras that double the primitive count
        const bool crowd = bodies.size() > 6000;
        Renderer2D::Style &st = Renderer2D::style;
        const Renderer2D::Style userStyle = st;
        if (crowd) { st.shadows = false; st.shading = false; }
        Renderer2D::beginFrame(camera);

        if (showBVH)
            world.broadPhase.forEachNode([](const Vector2 &mn, const Vector2 &mx)
                                         { Renderer2D::DrawAABBOutline(mn.x, mn.y, mx.x, mx.y, {255, 255, 0, 255}); });

        for (auto *b : bodies)
        {
            SDL_Color c = {(Uint8)b->c.r, (Uint8)b->c.g, (Uint8)b->c.b, (Uint8)b->c.a};
            if (showSleeping && !b->isAwake && !b->isStatic())
            {
                c.r = (Uint8)(c.r * 0.35f);
                c.g = (Uint8)(c.g * 0.35f);
                c.b = (Uint8)(c.b * 0.35f);
            }
            Renderer2D::DrawBody(*b, c);
        }

        Renderer2D::drawSprings(registry);
        Renderer2D::drawJoints(world.getJoints());
        if (showContacts)
            Renderer2D::DrawContacts(world.getContacts());
        if (activeDemo)
            activeDemo->drawWorld();
        Renderer2D::endFrame(); // everything above is one batched GL draw; ImGui is drawn after this layer
        st = userStyle;

        ui.DrawAddBody(world, bodies);

        if (activeDemo)
        {
            activeDemo->drawImGui();
        }

        ImGui::SetNextWindowPos(ImVec2(1560, 40), ImGuiCond_FirstUseEver);
        ImGui::Begin("Demo Selector", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        int &selectedDemoIndex = startDemoIndex; // the demo started in Init

        ImGui::Text("Select Demo");
        ImGui::Separator();

        for (int i = 0; i < demos.size(); ++i)
        {
            bool isSelected = (selectedDemoIndex == i);
            if (ImGui::Selectable(demos[i]->getName(), isSelected))
            {
                selectedDemoIndex = i;

                world.clear();
                bodies.clear();
                registry.clear();
                g->gravity = Vector2(0, -980); // some demos change the direction

                activeDemo = demos[i];
                activeDemo->init(world, bodies, registry, g);
            }

            if (isSelected)
                ImGui::SetItemDefaultFocus();
        }

        ImGui::End();

        float minVal, maxVal;
        FindMinMax(physicsHistory, 200, minVal, maxVal);


        ImGui::SetNextWindowPos(ImVec2(1210, 40), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340, 560), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Physics"))
        {
            PhysicsSettings &ps = world.settings;
            ImGui::Text("Bodies: %zu  awake: %d / %d dynamic", bodies.size(), world.getAwakeBodyCount(), world.getDynamicBodyCount());
            ImGui::Text("Physics: %.2f ms/update   Render (CPU): %.2f ms", physicsTimeMs, renderTimeMs);
            ImGui::Checkbox("Sleeping", &ps.enableSleep);
            ImGui::SameLine();
            ImGui::Checkbox("Dim sleeping bodies", &showSleeping);
            if (ImGui::CollapsingHeader("Rendering (SIMPEngine)"))
            {
                Renderer2D::Style &rs = Renderer2D::style;
                ImGui::Checkbox("Shadows", &rs.shadows);
                ImGui::SameLine();
                ImGui::Checkbox("Shading", &rs.shading);
                ImGui::Checkbox("Grid", &rs.grid);
                ImGui::SameLine();
                ImGui::Checkbox("Joint details", &rs.jointDetails);
                ImGui::Checkbox("Show contacts", &showContacts);
                ImGui::Checkbox("Show BVH", &showBVH);
                ImGui::TextDisabled("(shadows/shading switch off above 6000 bodies)");
            }
            if (ImGui::Button("Wake all"))
                world.wakeAll();
            ImGui::SliderFloat("Sleep lin. tol (px/s)", &ps.sleepLinearTolerance, 0.5f, 30.0f);
            ImGui::SliderFloat("Sleep ang. tol (rad/s)", &ps.sleepAngularTolerance, 0.005f, 0.5f);
            ImGui::SliderFloat("Time to sleep (s)", &ps.timeToSleep, 0.1f, 3.0f);
            ImGui::Separator();
            ImGui::SliderInt("Substeps per update", &physicsSubsteps, 1, 50);
            ImGui::SliderInt("Worker threads (-1 auto)", &ps.workerThreads, -1, 31);
            ImGui::SliderInt("Velocity iterations", &ps.velocityIterations, 1, 20);
            ImGui::SliderInt("Position iterations", &ps.positionIterations, 0, 10);
            ImGui::Checkbox("Warm starting", &ps.warmStarting);
            ImGui::SliderFloat("Linear slop (px)", &ps.linearSlop, 0.0f, 3.0f);
            ImGui::SliderFloat("Baumgarte", &ps.baumgarte, 0.0f, 1.0f);
            ImGui::SliderFloat("Restitution threshold (px/s)", &ps.restitutionThreshold, 0.0f, 300.0f);
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(1210, 620), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340, 300), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Profiler")){
            for(auto & [name, data] : aProfileData){
                double avg = data.totalTime / (double)data.callCount;

                ImGui::Text("%s", name.c_str());
                ImGui::SameLine(200);
                ImGui::Text("Total : %.3f ms", data.totalTime);
                ImGui::Text("Total : %.3f ms", avg);
                ImGui::Text("Calls : %d", data.callCount);

            }
        }
        ImGui::End();


        if (showDebugCoords)
        {
            ImGui::SetNextWindowPos(ImVec2(10, 40), ImGuiCond_FirstUseEver);
        ImGui::Begin("Mouse World", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

            ImGui::Text("World X: %.2f", debugMouseWorld.x);
            ImGui::Text("World Y: %.2f", debugMouseWorld.y);

            ImGui::Separator();
            if (ImGui::Button("Hide"))
                showDebugCoords = false;

            ImGui::End();
        }

        Uint64 endRender = SDL_GetPerformanceCounter();
        renderTimeMs = (float)(endRender - startRender) * 1000.0f / SDL_GetPerformanceFrequency();

        sampleCounter++;

        if (sampleCounter >= 300) // take a sample every 10 frames
        {
            physicsHistory[historyIndex] = physicsTimeMs;
            renderHistory[historyIndex] = renderTimeMs;

            historyIndex = (historyIndex + 1) % 600;

            sampleCounter = 0; // reset
        }

    }

    void FindMinMax(const float *data, int size, float &outMin, float &outMax)
    {
        outMin = data[0];
        outMax = data[0];

        for (int i = 1; i < size; i++)
        {
            if (data[i] < outMin)
                outMin = data[i];
            if (data[i] > outMax)
                outMax = data[i];
        }

        // Expand range slightly to avoid zero-line
        float range = outMax - outMin;
        if (range < 0.0001f)
            range = 0.0001f;

        outMin -= range * 0.1f;
        outMax += range * 0.1f;
    }

    void Game::showFPS(float realDt, float fixedDt)
    {
        float fps = 1.0f / realDt;
        float frameMs = realDt * 1000.0f;
        float fixedMs = fixedDt * 1000.0f;

        char title[256];
        snprintf(title, sizeof(title),
                "AccelEngine | FPS: %.1f | Frame: %.2f ms | Step: %.2f ms | Bodies: %zu",
                fps, frameMs, fixedMs, bodies.size());

        SDL_SetWindowTitle(window, title);
    }

    void Game::addCircle(float x, float y)
    {
        RigidBody *b = new RigidBody();
        b->position = {x, y};
        b->inverseMass = 1.0f;
        b->restitution = 1.0f; // Add some restitution

        b->shapeType = ShapeType::CIRCLE;
        b->circle.radius = 10 + rand() % 50;

        b->calculateInertia();
        b->velocity = {0, 0};
        b->calculateDerivativeData();
        b->c = RandomColor();

        world.addBody(b);
        bodies.push_back(b);
        registry.add(b, g);
    }

    void Game::addAABB(float x, float y)
    {
        RigidBody *b = new RigidBody();
        b->position = {x, y};
        b->inverseMass = 0.25f;
        b->restitution = 0.3f; // Add restitution

        b->shapeType = ShapeType::AABB;
        b->aabb.halfSize = {20 + rand() % 50, 20 + rand() % 50};

        b->calculateInertia();
        b->angularDamping = 0.98;

        b->velocity = {0, 0};
        b->calculateDerivativeData();
        b->c = RandomColor();

        world.addBody(b);
        bodies.push_back(b);
        registry.add(b, g);
    }

    void Game::addAABB(float x, float y, float w, float h, float orientation)
    {
        RigidBody *b = new RigidBody();
        b->position = {x, y};
        b->inverseMass = 0.0f;
        b->restitution = 0.3f;

        b->shapeType = ShapeType::AABB;
        b->aabb.halfSize = {w / 2, h / 2};
        b->orientation = orientation * (3.14159265f / 180.0f);

        b->inverseInertia = 0.0f;
        b->angularDamping = 0.98;

        b->velocity = {0, 0};
        b->calculateDerivativeData();
        b->c = RandomColor();

        world.addBody(b);
        bodies.push_back(b);
    }

    RigidBody *Game::getBody(float x, float y, float w, float h, float mass1, float orientation, SDL_Color c)
    {
        RigidBody *b = new RigidBody();
        b->position = {x, y};
        b->inverseMass = mass1;
        b->restitution = 0.3f;

        b->shapeType = ShapeType::AABB;
        b->aabb.halfSize = {w / 2, h / 2};
        b->orientation = orientation * (3.14159265f / 180.0f);

        // FIX 3: Correct AABB inertia formula with proper order
        // I = (1/12) * m * (w² + h²) for rectangle
        real mass = 1.0f / b->inverseMass;
        real width = b->aabb.halfSize.x * 2.0f;
        real height = b->aabb.halfSize.y * 2.0f;
        real inertia = (1.0f / 12.0f) * mass * (width * width + height * height);

        if (inertia > 1e-6f)
        {
            b->inverseInertia = 1.0f / inertia;
        }
        else
        {
            b->inverseInertia = 0.0f;
        }
        b->angularDamping = 0.98;

        b->velocity = {0, 0};
        b->calculateDerivativeData();
        b->c = {(float)c.r, (float)c.g, (float)c.b, (float)c.a};

        return b;
    }

    Vector2 Game::WorldToScreen(const Vector2 &w, float screenHeight)
    {
        return Vector2(w.x, screenHeight - w.y);
    }
    Vector2 Game::ScreenToWorld(const Vector2 &s, float screenHeight)
    {
        return Vector2(s.x, screenHeight - s.y);
    }
    Vector2 Game::getMouseWorld()
    {
        float sx, sy;
        SDL_GetMouseState(&sx, &sy);

        SDL_FPoint w = Renderer2D::screenToWorld(sx, sy);

        return Vector2(w.x, w.y);
    }

    void Game::gradBodies(float mx, float my)
    {
        // queryPoint handles every shape (and rotated boxes) and prefers dynamic bodies over static ones
        RigidBody *b = world.queryPoint(Vector2(mx, my));
        if (b && b->inverseMass > 0.0f) // static bodies are not draggable
        {
            grabbed = b;
            grabOffset = b->position - Vector2(mx, my);
        }
    }

    void Game::cameraControls(const SDL_KeyboardEvent &e)
    {
        switch (e.key)
        {
        case SDLK_LEFT:
            camera.x -= 50;
            break;
        case SDLK_RIGHT:
            camera.x += 50;
            break;
        case SDLK_UP:
            camera.y += 50;
            break;
        case SDLK_DOWN:
            camera.y -= 50;
            break;
        case SDLK_LCTRL:
        {
            debugMouseWorld = getMouseWorld();
            showDebugCoords = true;
        }
        break;
        case SDLK_Q:
        {
            float msx, msy;
            SDL_GetMouseState(&msx, &msy);

            Renderer2D::setCamera(camera);
            SDL_FPoint before = Renderer2D::screenToWorld(msx, msy);

            camera.zoom *= 1.1f;
            Renderer2D::setCamera(camera);
            SDL_FPoint after = Renderer2D::screenToWorld(msx, msy);

            camera.x += before.x - after.x;
            camera.y += before.y - after.y;
        }
        break;

        case SDLK_E:
        {
            float msx, msy;
            SDL_GetMouseState(&msx, &msy);

            Renderer2D::setCamera(camera);
            SDL_FPoint before = Renderer2D::screenToWorld(msx, msy);

            camera.zoom /= 1.1f;
            Renderer2D::setCamera(camera);

            SDL_FPoint after = Renderer2D::screenToWorld(msx, msy);

            camera.x += before.x - after.x;
            camera.y += before.y - after.y;
        }
        break;
        case SDLK_A:
        {
            eraserActive = !eraserActive;
            std::cout << "Eraser: " << (eraserActive ? "ON" : "OFF") << std::endl;
        }
        break;
        case SDLK_G:
        {
            showBVH != showBVH;
            std::cout << "Showing BVH" << std::endl;
        }
        break;
        default:
            break;
        }
    }
