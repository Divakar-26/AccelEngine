# Sandbox (demo app)

Front end for exercising the engine, running on **SIMPEngine** (`~/Desktop/SDLGameEngine`, installed system-wide): its window, GL context, batched renderer and ImGui layer. Nothing here ships with the library.

## How it is wired

- `main.cpp`: `SandboxApp : SIMPEngine::Application` pushes one `SandboxLayer`. The layer runs the fixed 1/60 s loop (max 2 updates/frame) in `OnUpdate`, calls `Game::render()` in `OnRender`, and replays SDL events it collected with `SDL_AddEventWatch` into `Game::processEvent` (SIMPEngine's own events carry no mouse position). SIMPEngine owns `ImGui::NewFrame/Render`: demos and panels just call `ImGui::` inside `render()`.
- `Renderer2D` is a thin layer over `SIMPEngine::Renderer`. `beginFrame(camera)` clears and sets an y-up ortho projection (camera.x/y = world point at the window's bottom-left, zoom = px per unit); everything drawn after goes into one batch (sorted by z) and `endFrame()` flushes it. ImGui draws afterwards, over the scene. `worldToScreen/screenToWorld` keep their old meaning, and `ImGui::GetBackgroundDrawList()` overlays (Galton curve) still work.
- `Renderer2D::DrawBody` draws every shape type: AA circles/boxes, polygons as filled fans with a lighter centre, capsules, compounds (outlines first, then fills, so seams vanish), sensors (translucent), one-way platforms (bright rim + chevrons). Toggle shadows/shading/grid/joint details in the Physics window ("Rendering"); above 6000 bodies shadows and shading switch off by themselves. Joints: rods, hinge pins (limit wedge, green when motorised), springs/suspension as zig-zags, weld crosses. `Demo::drawWorld()` is the hook for world-space labels (`Renderer2D::DrawText`).
- Keyboard: SIMPEngine enables ImGui keyboard navigation, so `WantCaptureKeyboard` is always true. Gate on `WantTextInput`.
- The vendored `include/imgui` is no longer built or included; ImGui comes from the installed SIMPEngine.
- The post-build step copies SIMPEngine's fonts to `<build>/Sandbox/engine/fonts`, where it looks for them. `imgui.ini` lives in the working directory (run from `build/`).

### Feature demos (`featureDemos.h`)
Shapes, Car, Hinges (ragdolls + motor) and Filters (layers, sensors, one-way) derive from `FeatureDemo`. Unlike the older demos they create bodies with `World::createX` (so the World frees them: `worldRef->clear()` is the whole reset) and register them in `bodies` only for drawing/picking via `track()`. Helpers: `box/ball/capsule/polygon/ngon/compound`, `room()`, `requestReset()`. `moreDemos.h` adds Pinball (Z/M flippers), Catapult (F or FIRE), Hill Climb (L/J/K), Hinged Bridge, Marble Run, Portals and Tower Smash (F fires) on a `MoreDemo` base (`segment`, `key`, `keyEdge`). Start any demo directly with `SANDBOX_DEMO=<index or name fragment> ./Sandbox/Sandbox`. Car: L/J drive, K brake (polled with `SDL_GetKeyboardState`, only while no text field is active).

## Adding a demo

1. Create `include/demo/myDemo.h` with a class deriving from `Demo` (`include/demo/demo.h`). Implement:
   - `getName()`
   - `init(World&, std::vector<RigidBody*>& bodies, ForceRegistry&, ForceGenerator* gravity)`
   - optionally `update()` (called once per substep) and `drawImGui()` (called once per frame)
2. Build bodies with the protected helpers `makeAABB(...)` / `makeCircle(...)`. They set the shape, inertia and derived data, add the body to both `world` and `bodies`, and register gravity when `invMass > 0`. Pass `invMass = 0` for static ground and walls.
3. At the start of `init`, call `world.joints.clear()`, because `World::clear()` doesn't clear joints.
4. Add the header to `include/demo/allDemo.h` and add `demos.push_back(new MyDemo());` in `Game::Init` (`src/game.cpp`).

Use `dominoDemo.h` as a minimal template (`stressDemo.h` shows an ImGui panel that spawns bodies). `bridgeDemo.h`, `clothDemo.h` and `softBodyDemo.h` show joints and springs.

### Showcase demos (`showcase.h`)
Dam Break, Galton Board, Wrecking Ball, Gravity Well, Color Mixer, Hourglass, Chain Reaction, Siege and Gravity Tilt derive from `ShowcaseDemo` instead of `Demo`. Its rules:
- **Hooks:** implement `build()` (create the scene) and optionally `step()` (runs once per substep: motors, emitters).
- **Reset:** `requestReset()` rebuilds on the next substep and frees the old bodies and joints.
- **Helpers:** `box`, `ball`, `segment` (a static plank between two points), `link` (a `DistanceJoint` the demo owns), `hsv`/`rgb` colours, `mouseWorld()` and `tip()`.
- **Tuned values:** they were checked headless, so re-check them if you change them.
  - Galton: bead air drag 0.005, triangular lattice with slanted side walls. Without these the bins come out flat or two-humped instead of a bell.
  - Mixer: both paddles turn the same way. Turning in opposite directions doesn't mix.
  - Dam Break: house storeys must not overlap at build time.

## Notes

- `makeAABB`'s `size` argument is a **half-size**.
- The `color` argument is currently overwritten by `randomColor()`.
- Scenes are laid out in y-up pixel space. The default window is 1920×1000, and the ground usually sits around y = 40.
- `Game::update` runs `physicsSubsteps` (default 6) substeps per 1/60 s tick, adjustable in the Physics window. Joint demos need ≥ 6.
- Drawing is batched by SIMPEngine and ordered by z (`kZBackground` ... `kZText` in renderer2D.cpp), not by call order. Give new draw code an explicit z.
- The Stress Demo's "Stress Test" window spawns N bodies in a grid at once (up to 50k), for large-scale tests.
- Body rendering and the BVH overlay live in `Game::render` and `Renderer2D`.
- `.cpp` files are listed explicitly in `Sandbox/CMakeLists.txt`, so add new ones there. Header-only demos need no CMake change.
- `Game::gradBodies` picks with `World::queryPoint` (works for every shape and rotated boxes).
- Zoom (Q/E) is now centred on the cursor; E used to fall through into the eraser toggle (fixed).
