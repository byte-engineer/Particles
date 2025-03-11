#include <raylib.h>

#include <vector>
#include <string>


#include <vec.hpp>
#include <draw.hpp>
#include <utils.hpp>
#include <object.hpp>



int main()
{

    vec2 win = vec2 {700, 700}; 


    // particales vector
    std::vector<particle::Circle> circles;


    // Object Creation
    for (int i = 0; i < 100; ++i)
    {
        particle::Circle newCircle(randomPos(win), 7, 1);
        newCircle.setRestitution(0.95f);
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
    }



    // Window initilization.
    InitWindow(win.x , win.y, "Simulation");
    SetTargetFPS(120);


    while (!WindowShouldClose())
    {
        float ts = GetFrameTime();
        onUpdate(circles, win, ts);
    }

    CloseWindow();
}