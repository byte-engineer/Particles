#include "raylib.h"

#include <vector>
#include <string>

#include "vec.hpp"
#include "draw.hpp"
#include "utils.hpp"
#include "object.hpp"


vec2 randomPos(vec2 win)
{
    return (vec2::random().normalized()*75)+(win/2);
}

float random(float min, float max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand(min, max);
    return rand(gen);
}

int main()
{
    utils::Timer timer; // Calculate the time.

    // ++++++++++++++++++++++++++++++++++++++++

    vec2 win = vec2{700, 700};
    float Restitution = 1.0f;
    bool roof = false;

    // Large balles:
    int lObj_num = 0;
    vec2 Lvelocity = vec2::random(00.0f, 00.0f);
    float Lradius = utils::random(100, 100.0f);

    // Balls
    // rad range
    int obj_num = 0;
    float rMin = 10; 
    float rMax = 6; 
    int sColor = 128;

    // ++++++++++++++++++++++++++++++++++++++++

    // Circles Vector.
    std::vector<particle::Circle> circles;

    for (int i=0; i<lObj_num; i++)
    {
        particle::Circle circle(vec2::random(win.x/2, win.y/2), Lradius);
        circle.setVelocity(Lvelocity);  
        circle.setColor(utils::randColor());
        circle.addForce(vec2 { 0, 98 } * circle.getMass());
        circles.push_back(circle);
    }

    // Object creation.
    for (int i = 0; i < obj_num; i++) {
        float radius = random(rMin, rMax);
        particle::Circle circle(vec2::random(0 + radius, win.y - radius), radius);
        circle.setColor(utils::randColor(sColor));
        circle.setVelocity(vec2 {utils::random(-1, 1), 0});
        circle.addForce(vec2 { 0, 98.1 } * circle.getMass());
        circles.push_back(circle);
    }




    // Initializations
    InitWindow(win.x, win.y, "Simulation");
    SetTargetFPS(60);
    int count = 0;

        
    while (!WindowShouldClose())
    {
        BeginDrawing();
        // Updates
        count++;
        float ts = GetFrameTime();
        DrawFPS(4, 4);
        DrawText(std::to_string((int)circles.size()).c_str(), 4, 30, 20, RED);
        ClearBackground(Color { 227, 192, 118, 255 });

        // key handelling
        if (IsKeyDown(KEY_S)) 
        {
            for (int i = 0; i < 5; ++i)
            {    
                float radius = random(rMin, rMax);
                particle::Circle circle(vec2{GetMousePosition().x, GetMousePosition().y}, radius);
                circle.setColor(utils::randColor(sColor));
                // circle.setVelocity(vec2 {utils::random(-1, 1), 0});
                circle.addForce(vec2 { 0, 98.1 } * circle.getMass());
                circles.push_back(circle);
            }
        }

        if (IsKeyDown(KEY_D))
        {
            for (int i = 0; i < 5; ++i)
                if (!circles.empty())
                {
                    circles.pop_back();
                }
        }


        // Handle collisions
        for (size_t i = 0; i < circles.size(); ++i)
        {
            for (size_t j = i+1; j < circles.size(); ++j)  // i + 1
            {
                if (circles[i].isCollide(circles[j]))
                {
                    circles[i].updateCollisionVelocity(circles[j]);
                    circles[i].correctOverlap(circles[j]);
                }
            }
        }

        // Update each circle
        for (particle::Circle& circle : circles)
        {
            circle.updateVelocity(ts);
            circle.sideCollisions(win, roof);
            circle.updatePosition(ts);

            // circle.clearForces();


            // Drawing
            circle.draw();
        }

        EndDrawing();

        // // Checks
        // if (count % 60 == 0)
        // {
        //     float KEs = 0;
        //     float Ms = 0;
        //     for (particle::Circle& cir : circles) {
        //         KEs += cir.getKE();
        //         Ms += cir.getMomentum();
        // }
        //     std::cout << "Kinetic Energy: " << KEs << std::endl;
        //     std::cout << "Momentum: " << Ms << std::endl;
        //     std::cout << "==================" << std::endl;
        // }

    }
    CloseWindow();
    return 0;
}

