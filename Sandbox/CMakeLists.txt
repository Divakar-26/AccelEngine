# The Sandbox renders with SIMPEngine (GL renderer, window, ImGui layer), installed system-wide
# (see SDLGameEngine/docs/USING_THE_LIBRARY.md). Physics comes from THIS repository's AccelEngine.
find_package(SIMPEngine REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

add_executable(Sandbox
    src/main.cpp
    src/game.cpp
    src/renderer2D.cpp
    src/UI.cpp
    src/InputManager.cpp
    src/InputActions.cpp
)

# SIMPEngine's own link list minus its bundled copy of AccelEngine: that one is older and would clash with
# the AccelEngine target of this repository.
set(_simp_libs "")
foreach(_lib libEngine.a libimgui.a libglad.a liblua.a libyaml-cpp.so libspdlog.so libSDL3_image.so libSDL3.so)
    list(APPEND _simp_libs "${SIMPEngine_LIBRARY_DIR}/${_lib}")
endforeach()

target_link_libraries(Sandbox PRIVATE AccelEngine ${_simp_libs} OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS})
target_link_options(Sandbox PRIVATE "LINKER:-rpath,${SIMPEngine_LIBRARY_DIR}")
target_compile_definitions(Sandbox PRIVATE
    ENGINE_PROFILER_ENABLE_IMGUI SIMP_VERSION_STRING="1.0.0"
    SPDLOG_SHARED_LIB FMT_SHARED SPDLOG_COMPILED_LIB)

# This repository's headers first; SIMPEngine's as SYSTEM includes so its (older) <AccelEngine/...> copy never wins.
target_include_directories(Sandbox PRIVATE
    ${CMAKE_SOURCE_DIR}/AccelEngine/include
    ${CMAKE_SOURCE_DIR}/Sandbox/include
    ${CMAKE_SOURCE_DIR}/Sandbox/include/demo)
target_include_directories(Sandbox SYSTEM PRIVATE ${SIMPEngine_INCLUDE_DIR} ${SIMPEngine_INCLUDE_DIR}/Engine)

# SIMPEngine looks for its fonts in <exe>/engine/fonts
add_custom_command(TARGET Sandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SIMPEngine_ASSETS_DIR}/fonts" "$<TARGET_FILE_DIR:Sandbox>/engine/fonts")
