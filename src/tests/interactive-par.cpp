#include <raylib.h>

#include <algorithm>
#include <execution>
#include <string>
#include <vector>

// Custom headers
#include "draw.hpp"
#include "object-runge-kutta.hpp"
#include "utils.hpp"
#include "vec.hpp"

// --- CONSTANTS AND GLOBAL VARIABLES ---
const vec2 GRAVITY = vec2{0, 9.81f * 10};
// Reduced iterations for a speed boost. Can be tuned back up if stability suffers.
const int POSITION_CORRECTION_ITERATIONS = 4;
// Define a fixed, stable time step for physics updates (120 times per second)
const float PHYSICS_DT = 1.0f / 120.0f;
// Accumulator to track time for sub-stepping
float accumulator = 0.0f;

vec2 randomPos(vec2 win)
{
    // A slight modification to ensure particles start within bounds
    return (vec2::random().normalized() * 75) + (win / 2);
}

void keyEvent(std::vector<particle::Circle> &circles)
{
    vec2 mousePos = vec2{GetMousePosition().x, GetMousePosition().y};
    vec2 mouseDlta = vec2{GetMouseDelta().x, GetMouseDelta().y};

    // Add circles
    if (IsKeyDown(KEY_S) && (circles.size() < 20000))
    {
        for (int i = 0; i < 3; ++i)
        {
            particle::Circle newCircle(mousePos, 20.f);
            newCircle.setRestitution(0.95f);
            newCircle.setColor(utils::randColor(128));
            circles.push_back(newCircle);
        }
    }

    // Delete circles
    if (IsKeyDown(KEY_D))
    {
        for (int i = 0; i < 3 && !circles.empty(); ++i)
        {
            circles.pop_back();
        }
    }

    // Clear all
    if (IsKeyPressed(KEY_C))
    {
        circles.clear();
    }

    size_t index = 0;

    // Select and drag closest particle (Right Click)
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

    // Apply force to nearby particles (Left Click)
    if (IsMouseButtonDown(0))
    {
        if (!circles.empty())
        {
            for (size_t i = 0; i < circles.size(); ++i)
            {
                float distance = (mousePos - circles[i].getPosition()).len();
                if (distance < 70 && distance > 1e-3f)
                {
                    // Apply instantaneous velocity change (impulse)
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDlta * 50 / (distance)));
                }
            }
        }
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Handles all the physics updates with a fixed time step (dt).
 * This function is called multiple times per frame if needed.
 */
void physicsStep(std::vector<particle::Circle> &circles, vec2 winSize, float dt)
{
    // --- PHASE 1: O(N) Independent Updates (PARALLELIZED) ---
    // All updates that don't depend on other particles.
    std::for_each(std::execution::par, circles.begin(), circles.end(), [winSize, dt](particle::Circle &circle) {
        // 1. Force Application
        circle.clearForces();
        circle.addForce(GRAVITY * circle.getMass());

        // 2. Integration: RK4 updates position and velocity
        circle.updatePhysics(dt);

        // 3. Boundary Collisions (Wall/Floor): Apply after integration
        circle.sideCollisions(winSize);

        // Damping/Resting Check
        if (circle.getVelocity().len() < 0.01)
        {
            circle.setVelocity(vec2{0.0, 0.0});
        }
    });

    // --- PHASE 2: O(N^2) Collision Resolution (SEQUENTIAL & ITERATIVE) ---
    // This is the bottleneck, but the iterations were reduced from 8 to 4.
    for (int k = 0; k < POSITION_CORRECTION_ITERATIONS; ++k)
    {
        for (size_t i = 0; i < circles.size(); ++i)
        {
            for (size_t j = i + 1; j < circles.size(); ++j)
            {
                if (circles[i].isCollide(circles[j]))
                {
                    // 1. Resolve Overlap (Position Correction)
                    circles[i].correctOverlap(circles[j]);

                    // 2. Resolve Velocity (Impulse): ONLY run once per frame
                    if (k == 0)
                    {
                        circles[i].updateCollisionVelocity(circles[j]);
                    }
                }
            }
        }
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Handles all the user input and rendering.
 * This function is called exactly once per rendering frame.
 */
void renderStep(std::vector<particle::Circle> &circles, vec2 winSize)
{
    BeginDrawing();
    ClearBackground(Color{227, 192, 118, 255});

    // Input events (which should be outside the physics loop)
    keyEvent(circles);

    // Draw UI
    DrawFPS(4, 4);
    DrawText(std::to_string(circles.size()).c_str(), 4, 20, 20, RED);

    // --- PHASE 3: O(N) Drawing (SEQUENTIAL) ---
    for (particle::Circle &circle : circles)
    {
        circle.draw();
    }

    EndDrawing();
}

// --------------------------------------------------------------------------------

int main()
{
    // Initialize random seed
    srand(time(NULL));

    SetTraceLogLevel(LOG_NONE);

    vec2 win = vec2{600 * 2, 700};

    std::vector<particle::Circle> circles;

    int N = 1000; // Number of circles
    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(randomPos(win), ((i / (float)N) * 15) + 1.f, 1);
        newCircle.setRestitution(0.9f);
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
    }

    // one spicial ball (high mass)
    particle::Circle spicialCircle(win / 2, 10, 1000);
    spicialCircle.setColor(RED);
    circles.push_back(spicialCircle);

    // Window initilization.
    InitWindow(win.x, win.y, "Parallel C++ Simulation (Optimized Fixed Time Step)");
    SetTargetFPS(120);

    while (!WindowShouldClose())
    {
        // Get the actual time since the last render frame
        float frameTime = GetFrameTime();
        accumulator += frameTime;

        // --- Physics Sub-stepping Loop (Fixed Time Step) ---
        // Run physics logic using the fixed time step until the accumulator is empty
        while (accumulator >= PHYSICS_DT)
        {
            physicsStep(circles, win, PHYSICS_DT);
            accumulator -= PHYSICS_DT;
        }

        // --- Rendering Step ---
        renderStep(circles, win);
    }

    CloseWindow();
}