#pragma once
#include <raylib.h>
#include <vec.hpp>

namespace Draw
{
    void CircleWithStrok(vec2 position, int rad, int strok, Color color, Color strok_Color)
    {
        // DrawCircle(position.x, position.y, rad, strok_Color);
        DrawCircle(position.x, position.y, rad - strok, color);
    }
}