add_executable(Sandbox 
    src/main.cpp
    src/game.cpp

    # ImGui core
    include/imgui/imgui.cpp
    include/imgui/imgui_draw.cpp
    include/imgui/imgui_tables.cpp
    include/imgui/imgui_widgets.cpp

    # Optional: SDL + OpenGL backends
    include/imgui/backends/imgui_impl_sdl3.cpp
    include/imgui/backends/imgui_impl_sdlrenderer3.cpp
)


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
    ${CMAKE_SOURCE_DIR}/Sandbox/include
    ${CMAKE_SOURCE_DIR}/Sandbox/include/imgui
)
