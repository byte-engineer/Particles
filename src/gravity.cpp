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

    if (IsMouseButtonPressed(1))
    {
        particle::Circle cir({GetMousePosition().x, GetMousePosition().y}, 10, 200);
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
            circle.addForce(circle.getRaduis() * circle.getVelocity() * -1 * 0.8);

            for (particle::Circle& other : circles)
            {
                if (&circle != &other)
                {
                    vec2 pos = other.getPosition() - circle.getPosition();
                    float distanceSquared = pos.lengthSequared();

                    if (distanceSquared > 0.01f)
                    {
                        float distance = sqrt(distanceSquared);
                        vec2 direction = pos / distance;

                        // Compute gravitational force: F = G * (m1 * m2) / r^2
                        float G = 60.6f; // Gravitational constant.
                        float forceMagnitude = G * (other.getMass() * circle.getMass()) / distanceSquared;

                        vec2 gravity = direction * forceMagnitude;

                        circle.addForce(gravity);
                    }
                }
            }

        }

        // Updates
        circle.updateVelocity(ts);
        // circle.sideCollisions(winSize, true);
        circle.updatePosition(ts);

        for (particle::Circle& other : circles)
        {
            if (circle.isCollide(other))
            {
                circle.correctOverlap(other);
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

    vec2 win = vec2 {1000, 700}; 


    // particales vector
    std::vector<particle::Circle> circles;


    // Object Creation
    for (int i = 0; i < 800; ++i)
    {
        float radius = utils::random(3, 10);
        vec2 pos = {utils::random(0 + radius, win.x - radius), utils::random(0 + radius, win.y - radius)};
        vec2 vel =  {utils::random(-100 , 100), utils::random(-100 , 100)};
        particle::Circle newCircle(pos, radius, 1);
        newCircle.setRestitution(0.95f);
        newCircle.setVelocity({0, 0});
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
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
