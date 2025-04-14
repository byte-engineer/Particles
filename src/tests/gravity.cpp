#include "raylib.h"

#include <vector>
#include <string>
#include <thread>
#include <execution>
#include <mutex>

#include "vec.hpp"
#include "draw.hpp"
#include "utils.hpp"
#include "object.hpp"



void keyEvent(std::vector<particle::Circle>& circles)
{
    vec2 mousePos = vec2 {GetMousePosition().x, GetMousePosition().y};
    vec2 mouseDlta= vec2 {GetMouseDelta().x, GetMouseDelta().y};

    if (IsMouseButtonPressed(1))
    {
        particle::Circle cir({GetMousePosition().x, GetMousePosition().y}, 10, 100);
        cir.setVelocity({0, 0});
        cir.setColor(BLACK);
        circles.push_back(cir);
    }    

    if (IsMouseButtonDown(0))
    {
        if (!circles.empty())
        {
            for (size_t i = 0; i < circles.size(); ++i)
            {
                float distance = (mousePos - circles[i].getPosition()).len();
                if (distance < 70 && distance > 1e-3f)
                {
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDlta * 50 / (distance)));
                }
            }
        }
    }



    if (IsKeyDown(KEY_S) && (circles.size() < 1500))
    {
        for (int i = 0; i < 1; ++i)
        {
            particle::Circle newCircle(mousePos, 7, 1);
            newCircle.setRestitution(0.95f);
            newCircle.setColor(utils::randColor(128));
            circles.push_back(newCircle);
        }
    }

    if (IsKeyDown(KEY_C))
    {
        circles.clear();
    }

}


void onUpdate(std::vector<particle::Circle>& circles, vec2 winSize, float ts)
{
    BeginDrawing();
    ClearBackground(Color { 227, 192, 118, 255 });
    keyEvent(circles);
    DrawFPS(4, 4);
    DrawText(std::to_string(circles.size()).c_str(), 4, 20, 20, RED);

    for (particle::Circle& circle : circles)
    {
        // Settings
        {
            circle.clearForces();
            // circle.addForce(vec2 { 0, 98.1f } * circle.getMass() );        // f = ma
            circle.addForce(circle.getRaduis() * circle.getVelocity() * -1 * 5);

            // path
            circle.path.push_back(circle.getPosition());

            if (circle.path.size() > 20)
            {
                circle.path.erase(circle.path.begin());
            }


            for (particle::Circle& other : circles)
            {
                if (&circle != &other)
                {
                    vec2 pos = other.getPosition() - circle.getPosition();
                    float distanceSquared = pos.lengthSequared();

                    if (distanceSquared > 0.001f)
                    {
                        float distance = sqrt(distanceSquared);
                        vec2 direction = pos / distance;

                        // Compute gravitational force: F = G * (m1 * m2) / r^2
                        float G = 100.0f; // Gravitational constant :|
                        float forceMagnitude = G * (other.getMass() * circle.getMass()) / distanceSquared;

                        vec2 gravity = direction * forceMagnitude;

                        circle.addForce(gravity);
                    }
                }
            }
        }

        // Updates
        circle.updateVelocity(ts);
        circle.sideCollisions(winSize, true);
        circle.updatePosition(ts);

        for (particle::Circle& other : circles)
        {
            if (circle.isCollide(other))
            {
                // circle.correctOverlap(other);
                circle.updateCollisionVelocity(other);
                circle.correctOverlap(other);
            }
        }

        // std::vector<vec2> masses;
        // std::vector<vec2> positions;
        // for (particle::Circle circle : circles)
        // {
            
        // }


        // Drawing
        for (size_t i = 1; i < circle.path.size(); i++)
        {
            DrawLineV({circle.path[i - 1].x, circle.path[i - 1].y}, {circle.path[i].x, circle.path[i].y}, RED);
        }
        circle.draw();
    }

    EndDrawing();
}



int main()
{

    vec2 win = vec2 {1000, 700}; 


    // particales vector
    std::vector<particle::Circle> circles;


    // Object Creation
    for (int i = 0; i < 700; ++i)
    {
        float radius = utils::random(5, 10);
        vec2 pos = {utils::random(0 + radius, win.x - radius), utils::random(0 + radius, win.y - radius)};
        vec2 vel =  {utils::random(-1000 , 1000), utils::random(-100 , 100)};
        particle::Circle newCircle(pos, radius, 1);
        newCircle.setRestitution(0.95f);
        newCircle.setVelocity(vel);
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
    }

    // Window initilization.
    InitWindow(win.x , win.y, "Simulation");
    SetTargetFPS(60);


    while (!WindowShouldClose())
    {
        float ts = GetFrameTime();
        onUpdate(circles, win, ts);
    }

    CloseWindow();
}
