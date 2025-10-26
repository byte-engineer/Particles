#include <raylib.h>

#include <string>
#include <vector>

#include "draw.hpp"
#include "object.hpp"
#include "utils.hpp"
#include "vec.hpp"

vec2 randomPos(vec2 win)
{
    return (vec2::random().normalized() * 75) + (win / 2);
}

void keyEvent(std::vector<particle::Circle> &circles)
{
    vec2 mousePos = vec2{GetMousePosition().x, GetMousePosition().y};
    vec2 mouseDlta = vec2{GetMouseDelta().x, GetMouseDelta().y};

    if (IsKeyDown(KEY_S) && (circles.size() < 20000))
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

void onUpdate(std::vector<particle::Circle> &circles, vec2 winSize, float ts)
{
    BeginDrawing();
    ClearBackground(Color{227, 192, 118, 255});

    keyEvent(circles);
    DrawFPS(4, 4);
    DrawText(std::to_string(circles.size()).c_str(), 4, 20, 20, RED);

    for (particle::Circle &circle : circles)
    {
        // Settings
        {
            circle.clearForces();
            circle.addForce(vec2{0, 98.1f} * circle.getMass()); // f = ma
        }

        // Updates
        circle.updateVelocity(ts);
        circle.sideCollisions(winSize, false);
        circle.updatePosition(ts);

        if (circle.getVelocity().len() < 0.01)
        {
            circle.setVelocity(vec2{0.0, 0.0});
        }

        for (particle::Circle &other : circles)
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
    SetTraceLogLevel(LOG_NONE);

    vec2 win = vec2{700, 700};

    // particales vector
    std::vector<particle::Circle> circles;

    int N = 800; // Number of circles
    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(randomPos(win), ((i / (float)N) * 4) + 2, 1); // 100 * ((float)N / i)
        newCircle.setRestitution(0.9f);
        newCircle.setColor(Color{(unsigned char)((i / (float)N) * 255), 128, 128, 255}); // utils::randColor(128)
        circles.push_back(newCircle);
    }

    // one spicial ball
    particle::Circle spicialCircle(win / 2, 10, 1000);
    spicialCircle.setColor(RED);
    circles.push_back(spicialCircle);

    // Window initilization.
    InitWindow(win.x, win.y, "Simulation");
    SetTargetFPS(120);

    while (!WindowShouldClose())
    {
        float ts = GetFrameTime();
        onUpdate(circles, win, ts);
    }

    CloseWindow();
}