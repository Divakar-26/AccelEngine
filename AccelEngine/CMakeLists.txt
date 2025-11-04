add_library(AccelEngine STATIC 
    src/particle.cpp
    src/pfgen.cpp
    src/ParticleContact.cpp
    src/ParticleWorld.cpp
)

target_include_directories(AccelEngine PUBLIC include)
