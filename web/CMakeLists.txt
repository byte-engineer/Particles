cmake_minimum_required(VERSION 3.30)

# Specify the compiler
set(CMAKE_C_COMPILER emcc)
set(CMAKE_CXX_COMPILER emcc)
set(CMAKE_BUILD_TYPE Release)

project(Main LANGUAGES C CXX)
include_directories(
    "${PROJECT_SOURCE_DIR}/../3rd/include"
    "${PROJECT_SOURCE_DIR}/src/"
    )

link_directories("${PROJECT_SOURCE_DIR}/../3rd/lib")


# add_executable(Main src/tests/interactive.cpp)
add_executable(Main src/tests/comparison.cpp)
target_link_libraries(Main PRIVATE raylib winmm)
