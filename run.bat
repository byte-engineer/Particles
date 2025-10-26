@echo off


cmake -G "Ninja" -S . -B build
if errorlevel 1 (
    echo CMake configuration failed.
    exit /b 1
)

cmake --build build
if errorlevel 1 (
    echo Build failed.
    exit /b 1
)

.\build\Main.exe