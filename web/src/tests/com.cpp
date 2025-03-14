#include "raylib.h"

#include <vector>
#include <string>

#include "vec.hpp"
#include "draw.hpp"
#include "utils.hpp"
#include "object.hpp"



vec2 randomPos(vec2 win)
{
    return (vec2::random().normalized()*100)+(win/2);
}


void keyEvent(std::vector<particle::Circle>& circles)
{
    vec2 mousePos = vec2 {GetMousePosition().x, GetMousePosition().y};\
    vec2 mouseDlta= vec2 {GetMouseDelta().x, GetMouseDelta().y};

    if (IsKeyPressed(KEY_S) && (circles.size() < 1500))
    {
        for (int i = 0; i < 1; ++i)
        {
            particle::Circle newCircle(mousePos, 7, 1);
            newCircle.setVelocity(vec2::zeros());
            newCircle.setRestitution(0.95f);
            newCircle.setColor(utils::randColor(128));
            circles.push_back(newCircle);
        }
    }


    if (IsKeyPressed(KEY_D))
    {
        for (int i = 0; i < 1 && !circles.empty(); ++i)
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
                circles[index].setVelocity(mouseDlta * 20);
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
                if (distance < 7000 && distance > 1e-3f)
                {
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDlta * 40 / (distance)));
                }
            }
        }
    }
}


// OnUpdate
void onUpdate(std::vector<particle::Circle>& circles, vec2 winSize, float ts)
{
    BeginDrawing();
    ClearBackground(Color { 227, 192, 118, 255 });

    keyEvent(circles);
    DrawFPS(4, 4);
    DrawText(std::to_string(circles.size()).c_str(), 4, 20, 20, RED);

    std::vector<vec2> positions; 

    for (particle::Circle& circle : circles)
    {
        // Settings
        {
            circle.clearForces();
            // circle.addForce(vec2 { 0, 98.1f } * circle.getMass() );        // f = ma


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
                        float G = 2000.0f; // Gravitational constant.
                        float forceMagnitude = G * (other.getMass() * circle.getMass()) / distanceSquared;

                        vec2 gravity = direction * forceMagnitude;

                        circle.addForce(gravity);
                    }
                }
            }

        }


        for (particle::Circle& other : circles)
        {
            if (circle.isCollide(other))
            {
                circle.correctOverlap(other);
                circle.updateCollisionVelocity(other);
                circle.correctOverlap(other);
            }
        }
        // Updates
        circle.updateVelocity(ts);
        // circle.sideCollisions(winSize, true);
        circle.updatePosition(ts);




        positions.push_back(circle.getPosition());

        // Drawing
        circle.draw();
    }

    if (!positions.empty())
    {

        vec2 positionsSum = vec2::zeros();
        for (vec2 pos : positions)
            positionsSum += pos;

        vec2 averagePosition = positionsSum / positions.size();
        DrawCircle(averagePosition.x, averagePosition.y, 4, RED);
    }

    EndDrawing();
}



int main()
{

    vec2 win = vec2 {700, 700}; 


    // particales vector
    std::vector<particle::Circle> circles;


    // Object Creation
    for (int i = 0; i < 1; ++i)
    {
        particle::Circle newCircle(randomPos(win), 7, 1);
        newCircle.setVelocity(vec2::zeros());
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
