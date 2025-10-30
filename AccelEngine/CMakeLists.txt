add_library(AccelEngine STATIC 
    src/main.cpp
    src/particle.cpp
)

target_include_directories(AccelEngine PUBLIC include)
