![Logo](./images/logo.png)

# AccelEngine
---
AccelEngine is a small and lightweight 2D physics engine, made to be used in 2D games and simulations.
Right now it supports Circle and AABB bodies.

---

## Features

- #### Physics
    - Rigid Bodies(Circle , Boxes)
    - SAT Narrow-phase collision detection
    - Broad-phase collision using AABB and BVH
    - Collision resolution with friction and restitution
    - Springs, distance joints and constraints

- #### Rendering (if using Sandbox to test)
    - SDL-based 2D renderer
    - Transform system

- #### Core Engine
    - Modular structure (World, RigidBodies, ForceRegistry, ParticleWorld)
    - Easy embedding into games or editors
---
## Build Instructions

### Build the core engine

1. Clone the Repo

```sh
    git clone https://github.com/Divakar-26/AccelEngine
    cd AccelEngine
```

2. Build with CMake

```sh
    mkdir build
    cd build
    cmake ..
    make
```

### Build With the Sandbox (demo)

1. Clone the Repo
```sh
    git clone --recursive https://github.com/Divakar-26/AccelEngine
    cd AccelEngine
```

2. Build with CMake and SDL (SDL3 Should be installed System-wide)

```sh
    mkdir build
    cd build
    cmake .. -DACCELENGINE_BUILD_SANDBOX=ON
    make
    
```

3. Run the Demo

```sh
    ./Sandbox/Sandbox
```

---

## Using AccelEngine in Another Project

To use AccelEngine as Library:
```
add_subdirectory(AccelEngine)
target_link_libraries(YourProject PRIVATE AccelEngine)
```

Then include it:
```cpp
#include <AccelEngine/core.h>
#include <AccelEngine/world.h>
```

## License

MIT License