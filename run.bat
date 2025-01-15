@echo off

cls
if exist build (
    rmdir /s /q build
)

mkdir build
cd build

cmake -G "Ninja" ../
if errorlevel 1 (
    echo CMake configuration failed.
    exit /b 1
)

ninja
if errorlevel 1 (
    echo Build failed.
    exit /b 1
)
cls
Main.exe