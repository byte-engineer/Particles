#include <raylib.h>

#include <vector>
#include <string>
#include <thread>
#include <execution>
#include <mutex>

#include <vec.hpp>
#include <draw.hpp>
#include <utils.hpp>
#include <object.hpp>



void keyEvent(std::vector<particle::Circle>& circles)
{
    vec2 mousePos = vec2 {GetMousePosition().x, GetMousePosition().y};\
    vec2 mouseDlta= vec2 {GetMouseDelta().x, GetMouseDelta().y};

    if (IsKeyDown(KEY_S) && (circles.size() < 1500))
    {
        for (int i = 0; i < 3; ++i)
        {
            particle::Circle newCircle(mousePos, 7, 1);
            newCircle.setRestitution(0.95f);
            newCircle.setColor(utils::randColor(128));
            circles.push_back(newCircle);
        }
    }


    if (IsKeyDown(KEY_D))
    {
        for (int i = 0; i < 3 && !circles.empty(); ++i)
        {
        if (!circles.empty())
            {
                circles.pop_back();
            }
        }
    }

    if (IsKeyPressed(KEY_C))
    {
        circles.clear();
    }

    size_t index = 0;
    if (IsMouseButtonDown(1))
    {
        if (!circles.empty())
        {
            float closest = 1000.0f;
            for (size_t i = 0; i < circles.size(); ++i)
            {
                float distance = (mousePos - circles[i].getPosition()).len();
                if (distance < closest)
                {
                    closest = distance;
                    index = i;
                }
            }
            if (closest < 1000.0f)
            {
                circles[index].moveTo(mousePos);
                circles[index].setVelocity(mouseDlta * 43);
            }
        }
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
            circle.addForce(vec2 { 0, 98.1f } * circle.getMass() );        // f = ma
        }

        // Updates
        circle.updateVelocity(ts);
        circle.sideCollisions(winSize, false);
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


        // Drawing
        circle.draw();
    }

    EndDrawing();
}



int main()
{

    vec2 win = vec2 {700, 700}; 


    // particales vector
    std::vector<particle::Circle> circles;


    // Object Creation
    for (int i = 0; i < 100; ++i)
    {
        particle::Circle circle(win/2, 10);
        circle.setColor(RED);
        circles.push_back(circle);
    }



    // Window initilization.
    InitWindow(win.x , win.y, "Simulation");
    // SetTargetFPS(60);


    while (!WindowShouldClose())
    {
        float ts = GetFrameTime();
        onUpdate(circles, win, ts);
    }

    CloseWindow();
}
