// gravity-optim.cpp

#include <raylib.h>

#include <algorithm>
#include <cmath> // For std::floor, std::sqrt, std::pow
#include <execution>
#include <numeric> // For std::iota
#include <string>
#include <vector>

// Custom headers
#include "draw.hpp"
#include "object-runge-kutta-grid.hpp"
#include "utils.hpp"
#include "vec.hpp"

// --- CONSTANTS AND GLOBAL VARIABLES ---

// --- GRID CONSTANTS ---
const int GRID_WIDTH = 100;
const int GRID_HEIGHT = 100;
float CELL_SIZE_X = 0.0f; // Will be calculated in main
float CELL_SIZE_Y = 0.0f; // Will be calculated in main

struct GridCell
{
    // Store pointers for fast insertion and no copies
    std::vector<particle::Circle *> particles;
};

// Global grid, initialized in main
std::vector<GridCell> collisionGrid;
// ----------------------

// New Constant for N-Body Force
// Set to 0.0 to disable. Positive for attraction (like gravity), negative for repulsion.
const float ATTRACTION_CONSTANT_G = 100000.0f;
// The default gravity has been reduced or removed to highlight the N-body force
const vec2 GRAVITY = vec2{0, 0.0f}; // Set to 0.0f to let the N-body force dominate
// Increased iterations for stability, now that the collision check is O(N)
const int POSITION_CORRECTION_ITERATIONS = 8;
const float PHYSICS_DT = 1.0f / 120.0f;
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
            particle::Circle newCircle(mousePos, 5.f);
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
                if (distance < 70 && distance > 1e-2f)
                {
                    // Apply instantaneous velocity change (impulse)
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDlta * 100 / (distance + 1e-2f)));
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

// --------------------------------------------------------------------------------

/**
 * @brief Applies attraction/repulsion forces using the spatial grid for broad-phase optimization.
 * This is effective for forces that rapidly decrease with distance (e.g., inverse-square)
 * and when the interaction radius is roughly one cell size.
 */
void applyGridForces()
{
    if (std::abs(ATTRACTION_CONSTANT_G) < 1e-3f)
        return; // Skip if force is negligible

    // Create a vector of all grid cell indices to iterate over in parallel
    std::vector<int> cell_indices(collisionGrid.size());
    std::iota(cell_indices.begin(), cell_indices.end(), 0);

    // Parallel execution over all grid cells
    std::for_each(std::execution::par, cell_indices.begin(), cell_indices.end(), [](int index) {
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

                    // Only check unique pairs. Using neighbor_index <= index ensures (A, B) is
                    // processed once when A is the 'smaller' index (or when A == B).
                    if (neighbor_index > index)
                    {
                        // Skip if this neighbor has a larger index.
                        // We will process the pair later when we are at the neighbor's index.
                        continue;
                    }

                    std::vector<particle::Circle *> &list1 = collisionGrid[index].particles;
                    std::vector<particle::Circle *> &list2 = collisionGrid[neighbor_index].particles;

                    for (particle::Circle *circle1 : list1)
                    {
                        for (particle::Circle *circle2 : list2)
                        {
                            // Skip self-comparison (only needed if index == neighbor_index)
                            if (circle1 == circle2)
                            {
                                continue;
                            }

                            vec2 pos_diff = circle2->getPosition() - circle1->getPosition();
                            float distanceSquared = pos_diff.lengthSequared();

                            // Use a small epsilon to avoid division by zero for near-zero distances
                            if (distanceSquared > 1e-3f)
                            {
                                float distance = std::sqrt(distanceSquared);
                                // The direction from circle1 to circle2
                                vec2 direction = pos_diff / distance;

                                // Compute gravitational force magnitude: F = G * (m1 * m2) / r^2
                                // The 'G' constant controls attraction/repulsion
                                float forceMagnitude =
                                    ATTRACTION_CONSTANT_G * (circle2->getMass() * circle1->getMass()) / distanceSquared;

                                vec2 force = direction * forceMagnitude;

                                // Force on circle1 is 'force'. Force on circle2 is '-force' (Newton's 3rd Law).
                                // Since forces are being applied in parallel, we must apply the full force and
                                // rely on the accumulator to be consistent for each particle's total force.
                                // NOTE: This structure requires the particle's addForce to be thread-safe
                                // OR (the simpler and safer route here) to only modify 'circle1'.
                                // For an N-body simulation, we should only apply to ONE particle in the pair
                                // and process ALL pairs.

                                // Since we iterate over all unique (circle1, circle2) pairs (where index <=
                                // neighbor_index): We ONLY apply the force to circle1, which means all other forces on
                                // circle1 from its neighbors are also being added in this loop. We also need to add the
                                // reaction force to circle2.
                                circle1->addForce(force);
                                circle2->addForce(force * -1.0f); // Newton's 3rd law
                            }
                        }
                    }
                }
            }
        }
    });
}

// --------------------------------------------------------------------------------

/**
 * @brief Resolves collisions by checking particles against their neighbors in the grid.
 * @param isInitialPass True for the first iteration (k=0), allowing velocity resolution.
 */
void resolveGridCollisions(bool isInitialPass)
{
    // ... (This function is unchanged from the original code) ...
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

                    // Simple heuristic to ensure pairs are unique:
                    // Only check against neighbors that haven't processed THIS cell yet.
                    if (neighbor_index < index)
                    {
                        // Skip if this neighbor has already processed the current cell
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
    // 1. Broad-Phase: Populate the grid (O(N))
    populateGrid(circles);

    // --- PHASE 1: O(N) Independent Updates + N-Body Force (PARALLELIZED) ---
    std::for_each(std::execution::par, circles.begin(), circles.end(), [winSize, dt](particle::Circle &circle) {
        // 1. Force Application (Reset)
        circle.clearForces();
        circle.addForce(GRAVITY * circle.getMass());

        // **NOTE:** The N-Body force (Attraction/Repulsion) is calculated globally in `applyGridForces`
        // and *added* to the particles' force accumulators, which is why it's called
        // *after* this initial loop's force clearing.
    });

    // 2. N-Body Force Calculation (Grid Optimized)
    // This function populates the force accumulators of the circles based on their neighbors.
    applyGridForces(); // New N-body force step

    // 3. Integration & Boundary Collisions (Re-parallelized for clarity)
    std::for_each(std::execution::par, circles.begin(), circles.end(), [winSize, dt](particle::Circle &circle) {
        // 3a. Integration: RK4 updates position and velocity based on total force
        circle.updatePhysics(dt);

        // 3b. Boundary Collisions (Wall/Floor): Apply after integration
        circle.sideCollisions(winSize, true);

        // (TODO) remove this (It's a debugging line from your original code)
        circle.setMass(circle.getVelocity().len());

        // 3c. Damping/Resting Check
        if (circle.getVelocity().len() < 0.01)
        {
            circle.setVelocity(vec2{0.0, 0.0});
        }
    });

    // --- PHASE 2: O(N) Collision Resolution (GRID-BASED & PARALLELIZED) ---
    // The grid was populated at the start, so we can now use it.

    // 4. Narrow-Phase + Position Correction: Iterate and resolve collisions (O(N) expected in practice)
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
    DrawText(("G: " + std::to_string(ATTRACTION_CONSTANT_G)).c_str(), 4, 40, 20, BLACK);

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

    // Calculate cell sizes based on window dimensions
    CELL_SIZE_X = win.x / GRID_WIDTH;
    CELL_SIZE_Y = win.y / GRID_HEIGHT;

    // Initialize the grid with the correct size
    collisionGrid.resize(GRID_WIDTH * GRID_HEIGHT);

    std::vector<particle::Circle> circles;

    int N = 100; // Number of circles
    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(randomPos(win), 4, 1); // ((i / (float)N) * 8)
        newCircle.setVelocity({0.f, 0.f});
        newCircle.setRestitution(.5);
        newCircle.setColor(utils::randColor(128));
        circles.push_back(newCircle);
    }

    // one spicial ball (high mass)
    // particle::Circle spicialCircle(win / 2, 10, 100);
    // spicialCircle.setColor(RED);
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