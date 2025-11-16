option(ACCELENGINE_BUILD_SANDBOX "Build the Sandbox demo" OFF)

cmake_minimum_required(VERSION 3.20)
project(AccelEngine)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add subprojects
add_subdirectory(AccelEngine)
if(ACCELENGINE_BUILD_SANDBOX)
    add_subdirectory(Sandbox)
endif()

