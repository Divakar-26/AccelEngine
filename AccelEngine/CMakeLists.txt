add_library(AccelEngine STATIC 
    src/particle.cpp
    src/pfgen.cpp
    src/ParticleContact.cpp
    src/ParticleWorld.cpp
    src/collision_coarse.cpp
    src/collision_narrow.cpp
    src/collision_resolve.cpp
    src/BVH.cpp
)

target_compile_options(AccelEngine PRIVATE -O3 -march=native)

target_include_directories(AccelEngine PUBLIC include)
