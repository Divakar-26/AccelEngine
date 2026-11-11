# AccelEngine

A small 2D rigid-body physics engine (C++20, static lib) plus a demo app (`Sandbox`) that renders with the author's own SIMPEngine (OpenGL batch renderer, window, ImGui layer; see `~/Desktop/SDLGameEngine`).
Shapes: circles, oriented boxes, convex polygons, capsules and compound bodies. Joints: distance, gear, revolute (hinge/pin with limits and motor), wheel (suspension + motor), weld. Filtering: categories/masks/groups, sensors, one-way platforms.
There is no test suite in the repo yet; the Sandbox demos and ad-hoc headless programs are how changes get verified.

## Build & run

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DACCELENGINE_BUILD_SANDBOX=ON   # first time only
cmake --build build -j8
cd build && ./Sandbox/Sandbox      # run from INSIDE build/: textures load from ../Sandbox/assets
```

- The `build/` directory is already configured (Release, Sandbox ON). Usually you only need `cmake --build build -j8`.
- The Sandbox needs **SIMPEngine installed system-wide** (`sudo cmake --install build/release --prefix /usr/local` in SDLGameEngine; `find_package(SIMPEngine)`). It links SIMPEngine's libs but NOT its bundled `libAccelEngine.a`: that copy is older and would clash with this repo's AccelEngine (`Sandbox/CMakeLists.txt` builds the link list by hand and adds SIMPEngine's include dir as SYSTEM so this repo's `<AccelEngine/...>` headers win). If SIMPEngine's `PhysicsSystem` ever needs a symbol that changed here, the link fails: keep the legacy two-argument `NarrowCollision::FindContacts` overload.
- The engine itself (AccelEngine/) needs nothing but a C++20 compiler.
- The engine is built with `-O3 -march=native` (AccelEngine/CMakeLists.txt), whatever the build type. Add `-g` locally when debugging.
- `World::step` is multithreaded (internal `ThreadPool`, `PhysicsSettings::workerThreads`, default = hardware threads − 1). Set `workerThreads = 0` to debug single-threaded.
- Verify a change by building, then running the relevant demo (pick it in the "Select Demo" ImGui window).
- The engine library builds and links on its own (no Sandbox include path needed). Useful checks: compile `AccelEngine/src/*.cpp` plus a test `main` with `-fsanitize=address,undefined` and with `-fsanitize=thread`; both were clean after the stability work.

## Layout

```
AccelEngine/                   the engine (library)
  include/AccelEngine/
    precision.h                `real` = float
    core.h                     Vector2, Matrix2, Vector3 (header-only math)
    shape.h                    ConvexPart (rounded convex polygon: circle/capsule/polygon piece of a body) + WorldPoly
    body.h                     RigidBody: state, integrate(), sleep state, filtering/sensor/one-way fields, AABB/inertia helpers (header-only)
    world.h                    World::step() pipeline (threaded), sleeping islands, CollisionEvent (header-only)
    thread_pool.h              internal fork-join pool used by World (not public API)
    BVH.h                      BVHTree broad phase: contiguous preorder nodes, refit between rebuilds, `margin` = speculative distance
    collision_narrow.h         Contact struct (manifold with per-point depths) + SAT/clipping narrow phase; Collide() = every manifold of a pair
    collision_resolve.h        PhysicsSettings + ContactSolver (the live solver). `CollisionResolve` statics are legacy/unused
    joint.h                    Joint base, DistanceJoint (soft via compliance), GearJoint, RevoluteJoint, WeldJoint, WheelJoint
    ForceGenerator.h           Gravity, Spring, AnchoredSpring, Aero*, Buoyancy (header-only)
    ForceRegistry.h            body -> ForceGenerator registrations
    profiler.h                 PROFILE_SCOPE macro -> global aProfileData map
    collision_coarse.h         O(n^2) AABB pair finder: legacy, World uses BVH instead
    particle.h, pfgen.h, ParticleContact.h, ParticleWorld.h
                               a separate legacy particle/mass-aggregate system, not used by World or Sandbox
  src/                         .cpp for BVH, collision_*, particle*
Sandbox/                       demo app (SIMPEngine window + GL renderer + ImGui layer)
  src/main.cpp                 SandboxApp (SIMPEngine::Application) + SandboxLayer: fixed-step loop, replays SDL events to Game
  src/game.cpp                 owns World, ForceRegistry, Gravity, demo list, per-frame render
  src/renderer2D.cpp           thin layer over SIMPEngine::Renderer: camera, bodies of every shape, joints, springs, overlays
  src/UI.cpp, Input*.cpp       ImGui panels, input to actions
  include/demo/*.h             one header-only Demo subclass per scene (see Sandbox/CLAUDE.md)
  include/imgui/               VENDORED Dear ImGui. Do not read or edit (see below)
TODOs.txt, profiling.txt, skipped.txt   the author's notes: roadmap, perf baselines, deferred items
```

## How a frame runs

`main.cpp` runs a fixed 1/60 s accumulator loop, capped at 2 updates per frame (the leftover time is dropped so heavy scenes slow down instead of collapsing). `Game::update(dt)` splits each tick into `physicsSubsteps` substeps (default 6; was a hard-coded 50), with `world.settings.velocityIterations = 2`. Each substep runs:

1. `world.startFrame()`: clears force/torque accumulators and recomputes derived data.
2. `registry.updateForces(h)`: gravity, springs and so on. The **ForceRegistry lives in Sandbox, not World** (TODO #3).
3. `activeDemo->update()`
4. `world.step(h, 1)`, which runs the numbered `PROFILE_SCOPE` phases shown in the Sandbox "Profiler" window:
   1. **Integrate** (parallel): assign `worldIndex`, `integrate` every awake body (semi-implicit Euler)
   2. **Broad phase**: `BVHTree::build(bodies, pool)` (full rebuild every 8 builds or when the body set changes, otherwise a refit; leaves refreshed in parallel) and `findPairs(pairs, pool)`. Only subtrees containing an awake body are traversed, so every pair has at least one awake, dynamic body.
   3. **Narrow phase** (parallel, per-chunk outputs concatenated in order): per pair `shouldCollide` (layers/groups, joints with `collideConnected = false`), then `NarrowCollision::Collide` (SAT + clipping; circles and boxes use the tuned fast paths, everything else the generic rounded-convex path; a compound pair can yield several contacts). Sensor pairs only produce an event, one-way platforms drop contacts coming from behind. Also collects collision events and sleeping bodies to wake.
   4. **Events + wake**: sleeping bodies touched by awake ones (or jointed to them) wake, keeping their sleep timer
   5. joint `preSolve`, then **Solve prepare**: per-body solver arrays (parallel), greedy graph colouring (serial), constraint setup + warm-start lookup (parallel)
   6. **Solve velocities**: `warmStart` + `solveVelocities` × `velocityIterations`, colour by colour, each colour in parallel
   7. **Solve finish**: velocities written back, warm-start cache rebuilt (lock-free per-owner lists)
   8. **Joints**: `solve` × `jointIterations` (100), serial
   9. **Positions**: `solvePositions(positionIterations)` (coloured, parallel; moves bodies, never adds velocity) + derived data
   10. **Sleep**: per-body timers (parallel); every 4th substep, if some body could sleep, union-find islands and put fully-still islands to sleep

All tunables live in `World::settings` (`PhysicsSettings`, collision_resolve.h) and are exposed in the Sandbox "Physics" ImGui window.

### Contact solver (collision_resolve.cpp)
- Sequential impulses with accumulated, clamped impulses: normal ≥ 0, and friction within ±μ·normal.
- 2-point manifolds use Box2D's 2×2 block solver. Solving point 0 then point 1 left a torque bias that made stacks lean and fall, so keep the block solver.
- Warm starting: impulses are cached per body pair on the body with the lower `worldIndex`, and matched by contact position in that body's local frame. Without warm starting, tall stacks keep residual velocity and never sleep.
- Graph colouring: constraints sharing a moving body never get the same colour (32 colours, plus a serial overflow bucket), so a colour can be solved in parallel. Bodies the solver can't move are read but never written (`movA`/`movB`), which avoids data races on the ground.
- Determinism: every parallel phase writes per-chunk buffers concatenated in chunk order, and colouring is serial, so results don't depend on thread timing.
- Restitution applies only when the approach speed exceeds `restitutionThreshold`, so resting contacts don't micro-bounce.
- Speculative contacts: pairs within `linearSlop` px count as touching (separation > 0 lets bodies approach until the gap closes). This keeps contacts and warm starting alive between substeps.

## World API (all additive: the original `addBody/addJoint/step(dt, substeps)` calls still work)

```cpp
World world;                                  // settings, gravity and forces are members
BodyDef d; d.position = {600, 300}; d.mass = 2; d.restitution = 0.2f;
RigidBody *ball = world.createCircle(20, d);  // owned by the world; also createBox(halfSize, d); mass <= 0 = static
world.createDistanceJoint(a, b, localA, localB, restLength);
world.createPolygon(points, d); world.createRegularPolygon(5, 30, d); world.createCapsule(halfLength, radius, d);
world.createCompound({ConvexPart::box({40, 10}), ConvexPart::circle({30, 20}, 12)}, d);   // parts are in the body frame
world.createRevoluteJoint(a, b, worldAnchor)->setLimits(-0.5f, 0.5f);                      // or ->setMotor(speed, maxTorque)
world.createWheelJoint(chassis, wheel, wheel->position, {0, 1})->setSpring(4.0f, 0.7f);    // + setMotor, setLimits
world.createWeldJoint(a, b);
world.step(1.0f / 60.0f);                     // settings.substeps (6) substeps; applies world.gravity and world.forces
world.removeBody(ball);                       // also drops its joints, forces, contacts, events; wakes what rested on it
world.queryPoint(p); world.queryAABB(min, max, out); world.raycast(origin, dir, maxDist, hit);   // linear scans
ball->applyImpulse(j); ball->setPose(pos, angle); ball->setVelocity(v); ball->setMass(m);        // all wake the body
```
- `step(dt)` applies `World::gravity` and `World::forces` itself; `step(dt, substeps)` applies **neither** (caller does `startFrame()` + its own registry per substep, as the Sandbox does). Don't use both styles on one world or gravity is applied twice.
- Safety net (`PhysicsSettings::maxLinearSpeed`/`maxAngularSpeed`): runaway speeds are clamped, and a body whose state becomes NaN/inf is reset, switched off (`enableCollision = false`, asleep) and counted in `getDisabledBodyCount()`. Invalid `dt` (<= 0, NaN) or `substeps < 1` make `step` do nothing.
- `World::clear()` now also drops joints, forces, contacts and the broad-phase tree.

## Shapes, filtering and the new joints

- **Parts model:** `POLYGON`, `CAPSULE` and `COMPOUND` bodies keep their geometry in `RigidBody::parts` (`ConvexPart`, local frame, body origin = centre of mass). A part is a rounded polygon: 1 vertex = circle, 2 = capsule, 3..8 = convex polygon, `radius` rounds it. `ConvexPart::polygon` takes any point cloud and builds the (CCW, <= 8 vertex) hull. `CIRCLE` and `AABB` bodies keep the original fields and fast paths.
- **Origin placement:** `createPolygon/createCapsule/createCompound` put the *frame origin you built the shape in* at `BodyDef::position`; the body's centre of mass (and `body->position`) is wherever the area-weighted centroid is. Parts are re-centred on it.
- **Generic collider** (`NarrowCollision::CollideConvex`): SAT over edge normals, reference/incident clipping, closest-point handling for rounded vertex-vertex contacts, circle-vs-polygon by closest feature. Normal always points A -> B. Parts with only a bounding-circle overlap are skipped.
- **Several contacts per pair** are normal now (compounds). The warm-start cache matches by position, so it copes; do not assume one `Contact` per body pair.
- **Filtering:** `categoryBits/maskBits/groupIndex` (Box2D rules: shared positive group always collides, shared negative never). `Joint::collideConnected` (false for the new joints, true for the old ones). Checked in the BVH pair test and again in the narrow phase.
- **Sensors** (`isSensor`): no contact forces, never wake sleepers, report `CollisionEvent{.., sensor = true}` (`World::getSensorEvents()`). Only pairs with at least one awake dynamic body are tested, so a static sensor sees awake bodies only. In the Sandbox (`step(h, 1)` per substep) events are those of the last substep, which is exactly "overlapping now".
- **One-way platforms** (`oneWay`, `oneWayNormal` in the platform's local frame): a contact is kept only if its normal faces the front (dot > 0.5) and the other body is not moving away faster than 1 px/s. Thin platforms only: a body fully inside the platform may be pushed out the wrong side.
- **Joints** (new ones): velocity-level like `DistanceJoint` (`preSolve` once per substep, `solve` x `jointIterations`, Baumgarte bias in `detail::kJointBeta`), raw `inverseMass`. Both bodies must be non-null: pin to the world with a static body. Revolute angle limits must lie within (-pi, pi) and are measured from the creation pose. Motor torque limits are impulses per substep (`dt * maxTorque`).
- **Tunings that were checked headless** (Sandbox car): wheel motor torque 60000 with a 7 kg chassis and two 1.2 kg wheels climbs a 16 degree ramp; suspension 4 Hz / 0.7.

## Conventions

- **Units and axes:** world units are pixels, and **y points up** (gravity is `(0, -980)`). `Renderer2D::worldToScreen` flips y. Rotation is in radians, counter-clockwise.
- **Contact normal** points from `contact.a` to `contact.b`. The resolver subtracts the impulse from A and adds it to B.
- **Static or kinematic bodies:** `inverseMass = 0` (and `inverseInertia = 0`). `lockPosition` / `lockRotation` / `ignoreGravity` flags give kinematic-like behaviour. `isStatic()` means infinite mass or both locks set. The contact solver uses `getEffectiveInverseMass/Inertia()`, so locked axes act as infinite mass. Joints still use the raw `inverseMass`.
- **Sleeping:** sleeping bodies are not integrated and not solved. Code that moves a body by hand (sets `position` or `velocity`, drags it, edits it in the UI) must call `wakeUp()`. Gravity and `addForce` do **not** wake a body. A `Spring` wakes a sleeping end if the other end is awake. Set `canSleep = false` for bodies driven by demo code, like the engine wheel. Static bodies never connect islands.
- "AABB" in `ShapeType::AABB` actually means an **oriented box** with `aabb.halfSize`. `worldAABBMin/Max` is the real axis-aligned bound.
- After setting `position`, `orientation` or the shape by hand, call `calculateDerivativeData()`. After setting mass or shape, call `calculateInertia()`.
- Use `real` for engine math and `Vector2::cross` for the 2D scalar cross product. Everything is in `namespace AccelEngine`.
- Style: 4-space indent, Allman braces, `#pragma once`, include headers as `<AccelEngine/xxx.h>`. Some names are misspelled (`SolveVelocityWithRoatation...`, `getHeigt`, `restLenght`). **Keep the existing spellings** unless asked to rename, because callers depend on them.

## Known issues (read before changing related code)

- **Dense random piles of plain boxes never fall asleep** (maxw ~0.1-0.3 rad/s after 60 s with 120 boxes in a 1000 px container, 2 velocity iterations). Pre-existing, unchanged by the shape work; small stacks and single bodies sleep fine. Concave compounds (L shapes) pile up livelier still.
- `Color` in body.h is a rendering concern living in the engine (the engine itself no longer includes or links anything from the Sandbox; `BVHTree::draw()` was replaced by `forEachNode`).
- **Ownership:** only bodies/joints made with `World::createCircle/createBox/createDistanceJoint` are freed by the World. Bodies added with `addBody()` and joints with `addJoint()` stay yours (`clear()`/`removeBody()` just drop the pointers). The Sandbox demos still use the old `new` + `addBody` style and leak on demo switch (the showcase demos free their own scene).
- `RigidBody()` now defaults `restitution` to 0 and `shapeType` to CIRCLE with radius 0. Still set both explicitly.
- `GetCollisionEvents()` now covers the whole `step()` call (deduplicated when substeps > 1). The Sandbox calls `step(h, 1)` per substep, so there it still only holds the last substep.
- The `CollisionResolve::*` static functions are the old solver, now unused. Change `ContactSolver` instead.
- Overlapping static bodies used to produce NaN (0/0 in the old solver). The BVH's active-subtree pruning and `ContactSolver::prepare` skip such pairs. Don't remove those guards.
- **Joints need ≥ 6 substeps:** at 4 substeps the See-Saw keeps jittering (~4 px/s) and never sleeps. Newton's Cradle loses energy faster at low substep counts (peak speed ~75 px/s vs ~140 at 50 substeps).
- `ThreadPool::parallelFor` must not be nested, and must only be called from one thread at a time.
- **Benchmarking:** this machine has 4 P-cores (CPUs 0–7 with HT) and 4 E-cores (8–11), and other software often keeps it loaded. Compare `perf stat -e cycles:u` on a pinned single thread (`taskset -c 2`, `workerThreads = 0`); wall-clock multithreaded numbers are noisy. Make sure test scenes have no overlapping static bodies when comparing against the original engine, because its NaN bug makes it look artificially fast.
- `Sandbox/include/imgui` is an embedded git repo with no `.gitmodules` entry, so `git clone --recursive` will not fetch it. **The Sandbox no longer uses it** (ImGui comes from the installed SIMPEngine, 1.92.2 with docking); it can be removed.
- SIMPEngine's `ImGuiLayer` enables keyboard navigation, so `io.WantCaptureKeyboard` is always true there. Gate keyboard input on `io.WantTextInput` (as `Game::processEvent` and `CarDemo` do).

## Perf baselines

`profiling.txt` holds the author's older numbers. The numbered `PROFILE_SCOPE` phases in `World::step` are shown per update in the Sandbox "Profiler" window, and the Physics window shows physics vs render ms.

Headless benchmark: an N-body grid (half boxes, half circles, size 5) dropped into a container, 4 substeps, 2 s simulated, everything awake. CPU cycles were measured on one pinned P-core.

| bodies | original engine, 1 core | current, 1 core | current, all threads (machine under load) |
|---|---|---|---|
| 10k | 31.9 G cycles | 16.5 G cycles | ≈ 13 ms/frame |
| 20k | 69.2 G cycles | 33.0 G cycles | ≈ 20–30 ms/frame |
| 50k | – | – | ≈ 115 ms/frame (heavily loaded machine) |

The current engine does 2 velocity iterations per substep (the original did 1) and stays stable; the original collapses stacks. Resting piles fall asleep and then cost almost nothing. Don't regress these numbers without saying so.

## Don't read / don't edit

- `Sandbox/include/imgui/**` is third-party, about 115 MB including its `.git`. If you need the ImGui API, `imgui.h` is the only file worth opening. Never modify anything there.
- `build/**` holds generated CMake output. Use the build commands; don't browse it.
- `Sandbox/assets/*` are binary images.
- `.vscode/` holds editor settings and isn't relevant.
