#include <raylib.h>

#include <vector>
#include <string>


#include <vec.hpp>
#include <draw.hpp>
#include <utils.hpp>
#include <object.hpp>



int main()
{
    particle::Circle circle( vec2{100, 100}, 10);
    circle.setColor(GREEN);
    circle.setAcceleration(vec2{98, 0});



    InitWindow(400, 400, "Test");
    SetTargetFPS(60);
    float ts = 1/60.; 

    while (!WindowShouldClose())
    {
        BeginDrawing();

        {
            circle.clearForces();
            circle.addForce(vec2 { 0, 981.f } * circle.getMass() );        // f = ma
            // circle.addForce(-10 * (circle.getVelocity()*circle.getVelocity()));
        }

        ClearBackground(Color { 227, 192, 118, 255 });
        circle.sideCollisions({400, 400}, true);
        circle.updateVelocity(ts);
        circle.updatePosition(ts);
        circle.setRestitution(0.1);
        circle.draw();
        EndDrawing();
    }
    CloseWindow();
}