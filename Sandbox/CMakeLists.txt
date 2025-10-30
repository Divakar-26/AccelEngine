add_executable(Sandbox main.cpp)

# Find SDL3 package
find_package(SDL3 REQUIRED CONFIG)

# Link with AccelEngine and SDL3
target_link_libraries(Sandbox
    PRIVATE
        AccelEngine
        SDL3::SDL3
)

# Include engine headers
target_include_directories(Sandbox PRIVATE
    ${CMAKE_SOURCE_DIR}/AccelEngine/include
)
