#include <raylib.h>

#include <vector>
#include <string>
#include <execution>
#include <algorithm>

#include <vec.hpp>
#include <draw.hpp>
#include <utils.hpp>
#include <object.hpp>



void onUpdate(std::vector<particle::Circle>& circles, vec2 winSize, float ts)
{
    BeginDrawing();
    ClearBackground(Color { 220, 220, 220, 255 });


    
    std::for_each(std::execution::par, circles.begin(), circles.end(), [&](particle::Circle& circle) {
                // Updates
                circle.updateVelocity(ts);
                circle.sideCollisions(winSize);
                circle.updatePosition(ts);
        
        
                if (circle.getVelocity().len() < 0.01)
                {
                    circle.setVelocity(vec2{0.0, 0.0});
                }
        
                for (particle::Circle& other : circles)
                {
                    if (circle.isCollide(other))
                    {
                        // circle.correctOverlap(other);
                        circle.updateCollisionVelocity(other);
                        // circle.correctOverlap(other);
                    }
                }
        
        
                // Drawing
                circle.draw();
        
    });

    


    // for (particle::Circle& circle : circles)
    // {
    //     // Updates
    //     circle.updateVelocity(ts);
    //     circle.sideCollisions(winSize);
    //     circle.updatePosition(ts);


    //     if (circle.getVelocity().len() < 0.01)
    //     {
    //         circle.setVelocity(vec2{0.0, 0.0});
    //     }

    //     for (particle::Circle& other : circles)
    //     {
    //         if (circle.isCollide(other))
    //         {
    //             // circle.correctOverlap(other);
    //             circle.updateCollisionVelocity(other);
    //             // circle.correctOverlap(other);
    //         }
    //     }


    //     // Drawing
    //     circle.draw();
    // }

    DrawFPS(4, 4);
    DrawText(std::to_string(circles.size()).c_str(), 4, 20, 20, RED);

    EndDrawing();
}



int main()
{
    vec2 win = vec2 {600, 700}; 

    // particales vector
    std::vector<particle::Circle> circles;

    int N = 2000;

    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(vec2{(float)(i*10.0), (float)(i*10.0)}, utils::random(5, 10) );
        newCircle.setVelocity(vec2{utils::random(-400, 400), utils::random(-400, 400)});
        newCircle.setRestitution(1.0f);
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