add_library(AccelEngine STATIC 
    src/particle.cpp
    src/pfgen.cpp
    src/ParticleContact.cpp
)

target_include_directories(AccelEngine PUBLIC include)
