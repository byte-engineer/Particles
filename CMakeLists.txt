# cmake_minimum_required(VERSION 3.30)

# # Specify the compiler
# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang++)
# set(CMAKE_BUILD_TYPE Release)

# project(Main LANGUAGES C CXX)
# include_directories(
#     "${PROJECT_SOURCE_DIR}/3rd/include"
#     "${PROJECT_SOURCE_DIR}/src/"
#     )

# link_directories("${PROJECT_SOURCE_DIR}/3rd/lib")

# # add_executable(Main src/tests/interactive.cpp)
# add_executable(Main src/tests/comparison.cpp)
# target_link_libraries(Main PRIVATE raylib winmm)



###################################################
cmake_minimum_required(VERSION 3.10)  # CMake 3.30 is too new, 3.10+ is fine

# Set the project
project(Main LANGUAGES C CXX)

set(CMAKE_C_COMPILER emcc)
set(CMAKE_CXX_COMPILER em++)

set(CMAKE_BUILD_TYPE Release)

include_directories(
    "${PROJECT_SOURCE_DIR}/../3rd/include"
    "${PROJECT_SOURCE_DIR}/src/"
)

link_directories("${PROJECT_SOURCE_DIR}/../3rd/lib")

set(CMAKE_EXECUTABLE_SUFFIX ".html")  # Generates .html file for browser execution

add_executable(Main src/tests/comparison.cpp)

target_compile_options(Main PRIVATE 
    -s USE_WEBGL2=1         # Enable WebGL2 (for rendering if needed)
    -s ALLOW_MEMORY_GROWTH=1  # Allow dynamic memory growth
    -s WASM=1              # Ensure WebAssembly output
)

target_link_libraries(Main PRIVATE 
    "-s USE_SDL=2"  # Example: Using SDL2 instead of WinMM (Windows-only)
)
