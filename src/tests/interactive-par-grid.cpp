// interactive.cpp

#include <raylib.h>

#include <algorithm>
#include <cmath> // For std::floor
#include <execution>
#include <string>
#include <vector>

// Custom headers
#include "draw.hpp"
#include "object-runge-kutta-grid.hpp"
#include "utils.hpp"
#include "vec.hpp"

// --- CONSTANTS AND GLOBAL VARIABLES ---

// --- GRID CONSTANTS ---
constexpr int GRID_WIDTH = 10;
constexpr int GRID_HEIGHT = 10;
float CELL_SIZE_X = 0.0f; // Will be calculated in main()
float CELL_SIZE_Y = 0.0f; // Will be calculated in main()

struct GridCell
{
    // Store pointers for fast insertion and no copies
    std::vector<particle::Circle *> particles;
};

// Global grid, initialized in main
std::vector<GridCell> collisionGrid;
// ----------------------

constexpr vec2 GRAVITY = vec2{0, 9.81f * 10};
// Increased iterations for stability, now that the collision check is O(N)
constexpr int POSITION_CORRECTION_ITERATIONS = 8;
constexpr float PHYSICS_DT = 1.0f / 120.0f;
float accumulator = 0.0f;

vec2 randomPos(const vec2 win)
{
    // A slight modification to ensure particles start within bounds
    return (vec2::random().normalized() * 75) + (win / 2);
}

void keyEvent(std::vector<particle::Circle> &circles)
{
    const vec2 mousePos = vec2{GetMousePosition().x, GetMousePosition().y};
    const vec2 mouseDelta = vec2{GetMouseDelta().x, GetMouseDelta().y};

    // Add circles
    if (IsKeyDown(KEY_S) && (circles.size() < 20000))
    {
        for (int i = 0; i < 3; ++i)
        {
            particle::Circle newCircle(mousePos, 5.f);
            newCircle.setRestitution(0.999f);
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
                circles[index].setVelocity(mouseDelta * 43);
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
                if (float distance = (mousePos - circles[i].getPosition()).len(); distance < 70 && distance > 1e-2f)
                {
                    // Apply instantaneous velocity change (impulse)
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDelta * 100 / (distance + 1e-2f)));
                }
            }
        }
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Maps each particle to its cell in the spatial grid. O(N) operation.
 */
void populateGrid(std::vector<particle::Circle> &circles)
{
    // Clear the grid for the new frame
    std::for_each(std::execution::par, collisionGrid.begin(), collisionGrid.end(),
                  [](GridCell &cell) { cell.particles.clear(); });

    // Populate the grid
    for (particle::Circle &circle : circles)
    {
        // Calculate the cell coordinates (i, j)
        int i = static_cast<int>(std::floor(circle.getPosition().x / CELL_SIZE_X));
        int j = static_cast<int>(std::floor(circle.getPosition().y / CELL_SIZE_Y));

        // Clamp to grid boundaries
        i = std::max(0, std::min(i, GRID_WIDTH - 1));
        j = std::max(0, std::min(j, GRID_HEIGHT - 1));

        // Calculate the 1D index
        int index = i + j * GRID_WIDTH;

        // Check for validity before pushing (should be redundant with clamping)
        if (index >= 0 && index < collisionGrid.size())
        {
            collisionGrid[index].particles.push_back(&circle);
        }
    }
}

/**
 * @brief Resolves collisions by checking particles against their neighbors in the grid.
 * @param isInitialPass True for the first iteration (k=0), allowing velocity resolution.
 */
void resolveGridCollisions(bool isInitialPass)
{
    // Create a vector of all grid cell indices to iterate over in parallel
    std::vector<int> cell_indices(collisionGrid.size());
    std::iota(cell_indices.begin(), cell_indices.end(), 0);

    // Parallel execution over all grid cells
    std::for_each(std::execution::par, cell_indices.begin(), cell_indices.end(), [isInitialPass](int index) {
        int current_i = index % GRID_WIDTH;
        int current_j = index / GRID_WIDTH;

        // Iterate over the 3x3 neighborhood (own cell + 8 neighbors)
        for (int dj = -1; dj <= 1; ++dj)
        {
            for (int di = -1; di <= 1; ++di)
            {
                int neighbor_i = current_i + di;
                int neighbor_j = current_j + dj;

                // Check boundary conditions for the neighbor cell
                if (neighbor_i >= 0 && neighbor_i < GRID_WIDTH && neighbor_j >= 0 && neighbor_j < GRID_HEIGHT)
                {
                    int neighbor_index = neighbor_i + neighbor_j * GRID_WIDTH;

                    // Only check unique pairs. The simplest way for a grid is:
                    // 1. Check current cell against itself (only i < j pairs)
                    // 2. Check current cell against neighbors that have not been fully processed yet
                    //    (e.g., those below or to the right, depending on iteration order)

                    // Optimization: We check all pairs between the current cell and its neighbor,
                    // but we must avoid checking the same pair twice (i against j, and j against i)
                    // and avoid redundant checks with previously processed cells.

                    // Simple heuristic to ensure pairs are unique:
                    // 1. Always check current cell against itself (di=0, dj=0). The inner loop handles uniqueness.
                    // 2. Only check against neighbors that haven't processed THIS cell yet.
                    // The following check ensures each pair (cell, neighbor) is processed only once:
                    if (neighbor_index < index)
                    {
                        // Skip if this neighbor has already processed the current cell
                        // This prevents processing the (A, B) pair when processing cell A
                        // and then processing the (B, A) pair when processing cell B.
                        continue;
                    }

                    std::vector<particle::Circle *> &list1 = collisionGrid[index].particles;
                    std::vector<particle::Circle *> &list2 = collisionGrid[neighbor_index].particles;

                    for (particle::Circle *circle1 : list1)
                    {
                        for (particle::Circle *circle2 : list2)
                        {
                            // Skip self-comparison if list1 and list2 are the same cell
                            if (circle1 == circle2)
                            {
                                continue;
                            }

                            // Narrow-phase check
                            if (circle1->isCollide(*circle2))
                            {
                                // 1. Resolve Overlap (Position Correction)
                                circle1->correctOverlap(*circle2);

                                // 2. Resolve Velocity (Impulse): ONLY run once per frame
                                if (isInitialPass)
                                {
                                    circle1->updateCollisionVelocity(*circle2);
                                }
                            }
                        }
                    }
                }
            }
        }
    });
}

/**
 * @brief Handles all the physics updates with a fixed time step (dt).
 * This function is called multiple times per frame if needed.
 */
void physicsStep(std::vector<particle::Circle> &circles, vec2 winSize, float dt)
{
    // --- PHASE 1: O(N) Independent Updates (PARALLELIZED) ---
    std::for_each(std::execution::par, circles.begin(), circles.end(), [winSize, dt](particle::Circle &circle) {
        // 1. Force Application
        circle.clearForces();
        circle.addForce(GRAVITY * circle.getMass());

        // 2. Integration: RK4 updates position and velocity
        circle.updatePhysics(dt);

        // 3. Boundary Collisions (Wall/Floor): Apply after integration
        circle.sideCollisions(winSize, true);

        // (TODO) remove this
        circle.setMass(circle.getVelocity().len());

        // Damping/Resting Check
        if (circle.getVelocity().len() < 0.01)
        {
            circle.setVelocity(vec2{0.0, 0.0});
        }
    });

    // --- PHASE 2: O(N) Collision Resolution (GRID-BASED & PARALLELIZED) ---
    // The previous O(N^2) brute-force check is replaced with these two steps.

    // 1. Broad-Phase: Populate the grid (O(N))
    populateGrid(circles);

    // 2. Narrow-Phase + Position Correction: Iterate and resolve collisions (O(N) expected in practice)
    for (int k = 0; k < POSITION_CORRECTION_ITERATIONS; ++k)
    {
        resolveGridCollisions(k == 0);
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
    srand(time(nullptr));

    SetTraceLogLevel(LOG_NONE);

    const vec2 win = vec2{600 * 2, 700};

    // Calculate cell sizes based on window dimensions
    CELL_SIZE_X = win.x / GRID_WIDTH;
    CELL_SIZE_Y = win.y / GRID_HEIGHT;

    // Initialize the grid with the correct size
    collisionGrid.resize(GRID_WIDTH * GRID_HEIGHT);

    std::vector<particle::Circle> circles;

    int N = 2000; // Number of circles
    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(randomPos(win), 8, 1); // ((i / (float)N) * 8)
        newCircle.setRestitution(.9);
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
    }

    // one spicial ball (high mass)
    particle::Circle spicialCircle(win / 2, 10, 1);
    spicialCircle.setColor(RED);
    // circles.push_back(spicialCircle);

    // Window initilization.
    InitWindow(win.x, win.y, "Parallel C++ Simulation (Grid Optimized)");
    SetTargetFPS(60);

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