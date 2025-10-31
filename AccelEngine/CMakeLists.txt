add_library(AccelEngine STATIC 
    src/particle.cpp
    src/pfgen.cpp
)

target_include_directories(AccelEngine PUBLIC include)
