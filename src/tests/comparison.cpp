#include <raylib.h>

#include <vector>
#include <string>
#include <execution>

#include <mutex>

#include <vec.hpp>
#include <draw.hpp>
#include <utils.hpp>
#include <object.hpp>

std::mutex mtx;

void onUpdate(std::vector<particle::Circle>& circles, vec2 winSize, float ts)
{
    BeginDrawing();
    ClearBackground(Color { 220, 220, 220, 255 });


    std::mutex collisionMutex; // Mutex for collision handling
    
    std::for_each(std::execution::par, circles.begin(), circles.end(), 
        [&circles, &ts, &winSize, &collisionMutex](particle::Circle& circle)
        {
            // Updates
            circle.updateVelocity(ts);
            circle.sideCollisions(winSize);
            circle.updatePosition(ts);
    
            // Collision handling
            std::for_each(std::execution::par, circles.begin(), circles.end(), 
                [&circle, &collisionMutex](particle::Circle& other)
                {
                    if (&circle != &other && circle.isCollide(other)) // Avoid self-collision
                    {
                        std::lock_guard<std::mutex> lock(collisionMutex);
                        circle.updateCollisionVelocity(other);
                    }
                });
    
            // Drawing (No need for a mutex here since rendering is usually thread-safe per object)
            circle.draw();
        });
    
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