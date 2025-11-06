add_library(AccelEngine STATIC 
    src/particle.cpp
    src/pfgen.cpp
    src/ParticleContact.cpp
    src/ParticleWorld.cpp
    src/collision_coarse.cpp
    src/narrow_collision.cpp
    src/collision_resolver.cpp
)

target_include_directories(AccelEngine PUBLIC include)
