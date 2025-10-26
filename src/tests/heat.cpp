// interactive.cpp

#include <raylib.h>

#include <algorithm>
#include <cmath> 
#include <execution> // For std::execution::par
#include <string>
#include <vector>
#include <numeric> // For std::iota
#include <iostream>

// Custom headers (Assumed to be available)
#include "draw.hpp" 
#include "heatObject.cpp" // Contains particle::Circle (Assumed to have position, velocity, mass, etc.)
#include "utils.hpp"
#include "vec.hpp"

// --- SPH-LIKE CONSTANTS AND GLOBAL VARIABLES ---

// --- GRID CONSTANTS ---
constexpr int GRID_WIDTH = 50; // Increased resolution for better SPH/fluid results
constexpr int GRID_HEIGHT = 25;
float CELL_SIZE_X = 0.0f; // Will be calculated in main()
float CELL_SIZE_Y = 0.0f; // Will be calculated in main()

// The characteristic radius (h) for SPH-like interactions. Should be roughly 2-3 times the particle radius.
// Since particles have a radius of ~3, h=10 is a good starting point.
constexpr float KERNEL_RADIUS_H = 15.0f; 
constexpr float KERNEL_RADIUS_H_SQ = KERNEL_RADIUS_H * KERNEL_RADIUS_H;

// --- SPH Fluid Constants ---
constexpr float RHO_0 = 1.0f; // Reference Density (Atmospheric/Base Density)
constexpr float STIFFNESS_K = 0.1f; // Controls the strength of the pressure/compressibility
constexpr float VISCOSITY_NU = 15.0f; // SPH-like Viscosity Coefficient (Higher for more "smoky"/dampened flow)
constexpr float GRAVITY_MAGNITUDE = 9.81f * 100;

struct GridCell
{
    std::vector<particle::Circle *> particles;
};

// Global grid, initialized in main
std::vector<GridCell> collisionGrid;
// ----------------------

// Time step remains small for stability
constexpr float PHYSICS_DT = 1.0f / 120.0f; 
float accumulator = 0.0f;

// --- FIRE SIMULATOR CONSTANTS ---
constexpr float TEMP_MIN = 0.0f;
constexpr float TEMP_MAX = 1.0f;
// The rate at which the bottom heater adds temperature
constexpr float HEAT_RATE = 1.0f; // Increased heat rate
// The rate at which particles cool (dissipate heat)
constexpr float COOLING_RATE = 0.2f;
// Maximum upward force applied due to heat (buoyancy)
constexpr float MAX_BUOYANCY_ACCEL = 5.0f; // Scaled to acceleration
// The y-coordinate fraction that defines the "hot zone" (e.g., bottom 35%)
constexpr float HOT_ZONE_HEIGHT_FRAC = 0.15f;
// Rate of temperature exchange upon collision (Still necessary for heat diffusion)
constexpr float HEAT_TRANSFER_RATE = 0.5f;

// --- TURBULENCE CONSTANTS (FOR FLICKER EFFECT) ---
// Strength of the oscillating horizontal force (Increased for better flicker)
constexpr float TURBULENCE_STRENGTH = 200.0f; 
float turbulenceTimer = 0.0f; // Global timer for the sine wave

// **EXTERNAL Temperature Data Structure**
std::vector<float> temperatures; 
// **EXTERNAL Density & Pressure Data Structures**
std::vector<float> densities; 
std::vector<float> pressures; 
// ----------------------------

/**
 * @brief Helper function to get the index of a particle from its pointer.
 * NOTE: Safe only if 'circles' vector is not resized.
 */
size_t getCircleIndex(particle::Circle *p, const std::vector<particle::Circle> &circles)
{
    if (p < circles.data() || p >= circles.data() + circles.size()) {
        std::cerr << "Error: Pointer out of range in getCircleIndex!" << std::endl;
        return 0; 
    }
    return std::distance(circles.data(), static_cast<const particle::Circle*>(p));
}

// SPH-like Kernel Function (Poly6 is common, but a simpler linear or quadratic decay works)
// Using a simplified quadratic decay (similar to the Spiky Kernel's form without normalization)
float kernel(float r_sq) {
    if (r_sq >= KERNEL_RADIUS_H_SQ) return 0.0f;
    float r = std::sqrt(r_sq);
    float h = KERNEL_RADIUS_H;
    // Simple quadratic falloff: (h-r)^2
    return (h - r) * (h - r);
}

// Gradient of the kernel (used for Pressure Force)
vec2 kernelGradient(const vec2& relativePos, float r_sq) {
    if (r_sq >= KERNEL_RADIUS_H_SQ || r_sq < 1e-6f) return vec2{0, 0};
    float r = std::sqrt(r_sq);
    float h = KERNEL_RADIUS_H;
    // Gradient of (h-r)^2 is -2*(h-r) * relativePos.normalized()
    float magnitude = -2.0f * (h - r);
    return relativePos.normalized() * magnitude;
}

// Laplacian of the kernel (used for Viscosity)
float kernelLaplacian(float r_sq) {
    if (r_sq >= KERNEL_RADIUS_H_SQ || r_sq < 1e-6f) return 0.0f;
    float r = std::sqrt(r_sq);
    float h = KERNEL_RADIUS_H;
    // Laplacian of (h-r)^2 is 2
    return 2.0f; // Simplified constant Laplacian for the quadratic kernel
}

// --- INPUT & RENDER FUNCTIONS (Kept largely the same) ---

vec2 randomPos(const vec2 win)
{
    return {
        (float)GetRandomValue(50, (int)win.x - 50), 
        (float)GetRandomValue((int)win.y - 200, (int)win.y - 50)
    };
}

void keyEvent(std::vector<particle::Circle> &circles)
{
    const vec2 mousePos = vec2{GetMousePosition().x, GetMousePosition().y};
    const vec2 mouseDelta = vec2{GetMouseDelta().x, GetMouseDelta().y};
    const float initial_radius = 3.0f; // Smaller initial radius

    // Add circles
    if (IsKeyDown(KEY_S) && (circles.size() < 20000))
    {
        for (int i = 0; i < 3; ++i)
        {
            vec2 spawnPos = {mousePos.x + GetRandomValue(-initial_radius, initial_radius), 
                             mousePos.y + GetRandomValue(-initial_radius, initial_radius)};
            // Small radius for smoke/fluid
            particle::Circle newCircle(spawnPos, initial_radius, 1.0f); 
            newCircle.setRestitution(0.0f); // Set low/zero restitution for fluid
            circles.push_back(newCircle);
            temperatures.push_back(0.8f); 
            densities.push_back(RHO_0); // Initial density
            pressures.push_back(0.0f); // Initial pressure
        }
    }

    // Delete circles (Cleanup all auxiliary vectors)
    if (IsKeyDown(KEY_D))
    {
        for (int i = 0; i < 3 && !circles.empty(); ++i)
        {
            circles.pop_back();
            temperatures.pop_back();
            densities.pop_back();
            pressures.pop_back();
        }
    }

    // Clear all
    if (IsKeyPressed(KEY_C))
    {
        circles.clear();
        temperatures.clear();
        densities.clear();
        pressures.clear();
    }
    
    // ... (Mouse drag/force logic remains, as it's useful for interaction)
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
                    circles[i].setVelocity(circles[i].getVelocity() + (mouseDelta * 100 / (distance + 1e-2f)));
                }
            }
        }
    }
}

// ... (populateGrid remains the same, but the grid size has changed) ...
void populateGrid(std::vector<particle::Circle> &circles)
{
    std::for_each(std::execution::par, collisionGrid.begin(), collisionGrid.end(),
                  [](GridCell &cell) { cell.particles.clear(); });

    for (particle::Circle &circle : circles)
    {
        int i = static_cast<int>(std::floor(circle.getPosition().x / CELL_SIZE_X));
        int j = static_cast<int>(std::floor(circle.getPosition().y / CELL_SIZE_Y));

        i = std::max(0, std::min(i, GRID_WIDTH - 1));
        j = std::max(0, std::min(j, GRID_HEIGHT - 1));

        int index = i + j * GRID_WIDTH;

        if (index >= 0 && index < collisionGrid.size())
        {
            collisionGrid[index].particles.push_back(&circle);
        }
    }
}

/**
 * @brief Calculates density and pressure for each particle using SPH kernel.
 */
void calculateDensityAndPressure(std::vector<particle::Circle> &circles)
{
    std::vector<int> cell_indices(collisionGrid.size());
    std::iota(cell_indices.begin(), cell_indices.end(), 0);

    // Phase 1: Calculate Density
    std::for_each(std::execution::par, cell_indices.begin(), cell_indices.end(), 
        [&circles](int index) { 
        
        int current_i = index % GRID_WIDTH;
        int current_j = index / GRID_WIDTH;

        // Iterate over the 3x3 neighborhood 
        for (int dj = -1; dj <= 1; ++dj)
        {
            for (int di = -1; di <= 1; ++di)
            {
                int neighbor_i = current_i + di;
                int neighbor_j = current_j + dj;

                if (neighbor_i >= 0 && neighbor_i < GRID_WIDTH && neighbor_j >= 0 && neighbor_j < GRID_HEIGHT)
                {
                    int neighbor_index = neighbor_i + neighbor_j * GRID_WIDTH;

                    // Ensure we only process a cell-pair once
                    if (neighbor_index < index) continue; 

                    std::vector<particle::Circle *> &list1 = collisionGrid[index].particles;
                    std::vector<particle::Circle *> &list2 = collisionGrid[neighbor_index].particles;

                    for (particle::Circle *circle1 : list1)
                    {
                        size_t index1 = getCircleIndex(circle1, circles);
                        float mass1 = circle1->getMass();

                        for (particle::Circle *circle2 : list2)
                        {
                            size_t index2 = getCircleIndex(circle2, circles);
                            float mass2 = circle2->getMass();

                            if (circle1 == circle2) {
                                // Contribution of itself to its own density
                                densities[index1] += mass1 * kernel(0.0f);
                                continue;
                            }
                            
                            // Prevent double counting within the same cell
                            if (index == neighbor_index && circle1 > circle2) continue;

                            vec2 relativePos = circle1->getPosition() - circle2->getPosition();
                            float distance_sq = relativePos.x * relativePos.x + relativePos.y * relativePos.y;
                            
                            if (distance_sq < KERNEL_RADIUS_H_SQ) 
                            {
                                float w = kernel(distance_sq);
                                
                                // Density is a sum of (mass * kernel)
                                densities[index1] += mass2 * w;
                                // Symmetrical update for the neighbor
                                if (index1 != index2) {
                                    densities[index2] += mass1 * w;
                                }
                            }
                        }
                    }
                }
            }
        }
    });

    // Phase 2: Calculate Pressure from Density
    // The pressure is based on the ideal gas/Tait equation: P = k * (rho - rho_0)
    for (size_t i = 0; i < circles.size(); ++i) {
        pressures[i] = STIFFNESS_K * (densities[i] - RHO_0);
        // Pressure must be positive (repulsive)
        if (pressures[i] < 0.0f) pressures[i] = 0.0f;
    }
}

/**
 * @brief Applies SPH Pressure and Viscosity forces.
 */
void applySPHForces(std::vector<particle::Circle> &circles)
{
    // Clear densities for recalculation
    std::fill(densities.begin(), densities.end(), 0.0f);

    // 1. Calculate new Density and Pressure
    calculateDensityAndPressure(circles);

    std::vector<int> cell_indices(collisionGrid.size());
    std::iota(cell_indices.begin(), cell_indices.end(), 0);

    // 2. Calculate and apply SPH forces (Pressure and Viscosity)
    std::for_each(std::execution::par, cell_indices.begin(), cell_indices.end(), 
        [&circles](int index) { 
        
        int current_i = index % GRID_WIDTH;
        int current_j = index / GRID_WIDTH;

        for (int dj = -1; dj <= 1; ++dj)
        {
            for (int di = -1; di <= 1; ++di)
            {
                int neighbor_i = current_i + di;
                int neighbor_j = current_j + dj;

                if (neighbor_i >= 0 && neighbor_i < GRID_WIDTH && neighbor_j >= 0 && neighbor_j < GRID_HEIGHT)
                {
                    int neighbor_index = neighbor_i + neighbor_j * GRID_WIDTH;

                    // Only need to iterate over (index, neighbor_index) where neighbor_index >= index
                    if (neighbor_index < index) continue; 

                    std::vector<particle::Circle *> &list1 = collisionGrid[index].particles;
                    std::vector<particle::Circle *> &list2 = collisionGrid[neighbor_index].particles;

                    for (particle::Circle *circle1 : list1)
                    {
                        size_t index1 = getCircleIndex(circle1, circles);
                        float mass1 = circle1->getMass();
                        float density1 = densities[index1];
                        float pressure1 = pressures[index1];

                        for (particle::Circle *circle2 : list2)
                        {
                            if (circle1 == circle2) continue;
                            if (index == neighbor_index && circle1 > circle2) continue;

                            size_t index2 = getCircleIndex(circle2, circles);
                            float mass2 = circle2->getMass();
                            float density2 = densities[index2];
                            float pressure2 = pressures[index2];

                            vec2 relativePos = circle1->getPosition() - circle2->getPosition();
                            float distance_sq = relativePos.x * relativePos.x + relativePos.y * relativePos.y;
                            
                            if (distance_sq < KERNEL_RADIUS_H_SQ) 
                            {
                                // --- Pressure Force (Based on Pressure Gradient) ---
                                // The formula: F_p = -m_i * sum_j [ (p_i/rho_i^2 + p_j/rho_j^2) * m_j * grad(W) ]
                                float avg_pressure_term = (pressure1 / (density1 * density1) + pressure2 / (density2 * density2)) * 0.5f;
                                vec2 grad_w = kernelGradient(relativePos, distance_sq);
                                vec2 pressure_force = grad_w * mass1 * mass2 * avg_pressure_term * -1.0f; // * -1.0f for repulsion
                                
                                // Add force symmetrically
                                circle1->addForce(pressure_force);
                                circle2->addForce(pressure_force * -1.0f);

                                // --- Viscosity Force (Based on Velocity Laplacian) ---
                                // The formula: F_v = m_i * sum_j [ mu * (v_j - v_i) / rho_j * laplacian(W) ]
                                vec2 relativeVel = circle2->getVelocity() - circle1->getVelocity();
                                float lap_w = kernelLaplacian(distance_sq);

                                // Simplified Viscosity term (assuming mass1 = mass2 = 1.0)
                                vec2 viscosity_force = relativeVel * VISCOSITY_NU * lap_w / density2;
                                
                                // Apply force (viscosity is always attractive/damping)
                                circle1->addForce(viscosity_force); 
                                circle2->addForce(viscosity_force * -1.0f);
                            }
                        }
                    }
                }
            }
        }
    });
}


/**
 * @brief Resolves collisions and heat transfer. (Only Heat Transfer and Position Correction remain)
 */
void resolveGridCollisions(std::vector<particle::Circle> &circles, bool isInitialPass) 
{
    std::vector<int> cell_indices(collisionGrid.size());
    std::iota(cell_indices.begin(), cell_indices.end(), 0);

    // Parallel execution over all grid cells
    std::for_each(std::execution::par, cell_indices.begin(), cell_indices.end(), 
        [isInitialPass, &circles](int index) { 
        int current_i = index % GRID_WIDTH;
        int current_j = index / GRID_WIDTH;

        for (int dj = -1; dj <= 1; ++dj)
        {
            for (int di = -1; di <= 1; ++di)
            {
                int neighbor_i = current_i + di;
                int neighbor_j = current_j + dj;

                if (neighbor_i >= 0 && neighbor_i < GRID_WIDTH && neighbor_j >= 0 && neighbor_j < GRID_HEIGHT)
                {
                    int neighbor_index = neighbor_i + neighbor_j * GRID_WIDTH;

                    if (neighbor_index < index) continue; 
                    
                    std::vector<particle::Circle *> &list1 = collisionGrid[index].particles;
                    std::vector<particle::Circle *> &list2 = collisionGrid[neighbor_index].particles;

                    for (particle::Circle *circle1 : list1)
                    {
                        for (particle::Circle *circle2 : list2)
                        {
                            if (circle1 == circle2) continue;
                            if (index == neighbor_index && circle1 > circle2) continue;

                            // Narrow-phase check is now just a check for overlap (position correction)
                            if (circle1->isCollide(*circle2)) // circle1->isCollide() checks if distance < r1+r2
                            {
                                // 1. Resolve Overlap (Position Correction) - Essential for non-SPH overlap
                                // KEEPING position correction for stability and simplicity over pure SPH PBD.
                                circle1->correctOverlap(*circle2);

                                if (isInitialPass)
                                {
                                    // *** Removed circle1->updateCollisionVelocity(*circle2); ***
                                    
                                    // 2. Heat Transfer
                                    size_t index1 = getCircleIndex(circle1, circles);
                                    size_t index2 = getCircleIndex(circle2, circles);

                                    float temp1 = temperatures[index1];
                                    float temp2 = temperatures[index2];

                                    // Move each temperature towards the average
                                    float delta_temp = (temp2 - temp1) * HEAT_TRANSFER_RATE;

                                    // Apply the transfer (ensuring both are updated symmetrically)
                                    temperatures[index1] += delta_temp;
                                    temperatures[index2] -= delta_temp;
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
 * @brief Maps temperature (0.0 to 1.0) to a fire color gradient. (Reworked for more translucent smoke)
 */
Color getFireColor(float t)
{
    t = std::min(1.0f, std::max(0.0f, t)); // Clamp t

    // Define Key Colors:
    // SMOKE: Darker, more translucent
    const Color SMOKE_COLOR = {50, 50, 60, 50}; 
    // Ignition/Base Fire
    const Color RED_COLOR = {255, 60, 0, 180}; 
    const Color ORANGE_COLOR = {255, 165, 0, 200}; 
    // Hottest part is yellow/white and highly opaque
    const Color YELLOW_COLOR = {255, 255, 150, 255}; 

    if (t < 0.2f) { // 0.0 to 0.2 (Smoke)
        float local_t = t / 0.2f; 
        // Interpolate from SMOKE_COLOR (cold) to RED_COLOR (ignition temp)
        return Color{
            (unsigned char)(SMOKE_COLOR.r + (RED_COLOR.r - SMOKE_COLOR.r) * local_t),
            (unsigned char)(SMOKE_COLOR.g + (RED_COLOR.g - SMOKE_COLOR.g) * local_t),
            (unsigned char)(SMOKE_COLOR.b + (RED_COLOR.b - SMOKE_COLOR.b) * local_t),
            (unsigned char)(SMOKE_COLOR.a + (RED_COLOR.a - SMOKE_COLOR.a) * local_t) // Alpha ramps up
        };
    } else if (t < 0.5f) { // 0.2 to 0.5 (Red to Orange)
        float local_t = (t - 0.2f) / 0.3f; 
        // Interpolate from RED_COLOR to ORANGE_COLOR
        return Color{
            (unsigned char)(RED_COLOR.r + (ORANGE_COLOR.r - RED_COLOR.r) * local_t),
            (unsigned char)(RED_COLOR.g + (ORANGE_COLOR.g - RED_COLOR.g) * local_t),
            (unsigned char)(RED_COLOR.b + (ORANGE_COLOR.b - RED_COLOR.b) * local_t),
            (unsigned char)(RED_COLOR.a + (ORANGE_COLOR.a - RED_COLOR.a) * local_t)
        };
    } else { // 0.5 to 1.0 (Orange to Yellow/White)
        float local_t = (t - 0.5f) / 0.5f; 
        // Interpolate from ORANGE_COLOR to YELLOW_COLOR
        return Color{
            (unsigned char)(ORANGE_COLOR.r + (YELLOW_COLOR.r - ORANGE_COLOR.r) * local_t),
            (unsigned char)(ORANGE_COLOR.g + (YELLOW_COLOR.g - ORANGE_COLOR.g) * local_t),
            (unsigned char)(ORANGE_COLOR.b + (YELLOW_COLOR.b - ORANGE_COLOR.b) * local_t),
            (unsigned char)(ORANGE_COLOR.a + (YELLOW_COLOR.a - ORANGE_COLOR.a) * local_t) // Alpha to max (255)
        };
    }
}


/**
 * @brief Handles all the physics updates with a fixed time step (dt).
 */
void physicsStep(std::vector<particle::Circle> &circles, vec2 winSize, float dt)
{
    // Update turbulence timer for chaotic motion
    turbulenceTimer += dt;
    float lateralStrength = std::sin(turbulenceTimer * 5.0f); 
    const vec2 turbulenceForce = {TURBULENCE_STRENGTH * lateralStrength, 0};

    const float HOT_ZONE_HEIGHT = winSize.y * HOT_ZONE_HEIGHT_FRAC;

    std::vector<size_t> indices(circles.size());
    std::iota(indices.begin(), indices.end(), 0);

    // --- PHASE 0: Populate Grid for Inter-Particle Forces and Collisions ---
    populateGrid(circles);

    // --- PHASE 1: O(N) Independent Forces & Temp Update (PARALLELIZED) ---
    std::for_each(std::execution::par, indices.begin(), indices.end(), [winSize, dt, HOT_ZONE_HEIGHT, turbulenceForce, &circles](size_t i) {
        particle::Circle &circle = circles[i];
        float &currentTemp = temperatures[i]; 
        float mass = circle.getMass();

        // 1. Temperature Update (Heat/Cooling)
        float newTemp = currentTemp;

        if (circle.getPosition().y > winSize.y - HOT_ZONE_HEIGHT)
        {
            float heatFactor = (circle.getPosition().y - (winSize.y - HOT_ZONE_HEIGHT)) / HOT_ZONE_HEIGHT;
            newTemp += HEAT_RATE * heatFactor * dt;
        }
        else
        {
            newTemp -= COOLING_RATE * dt;
        }

        newTemp = std::min(TEMP_MAX, std::max(TEMP_MIN, newTemp));
        currentTemp = newTemp;

        // 2. Force Application: Clear forces and apply independent forces
        circle.clearForces();

        // **Gravity is NOT applied, it's implicitly part of Buoyancy in the fluid model.**
        
        // **Buoyancy (Convection Force)**
        // Buoyancy is inversely related to density (or directly related to temperature).
        // The colder the particle (closer to TEMP_MIN), the less the buoyant acceleration.
        // It drives the main upward motion of smoke/fire.
        float buoyancyScale = (currentTemp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN); // 0.0 to 1.0
        vec2 buoyancy = vec2{0, -MAX_BUOYANCY_ACCEL * buoyancyScale * mass}; // Upward (negative y) force
        circle.addForce(buoyancy);

        // **Flicker/Turbulence Force**
        circle.addForce(turbulenceForce * mass * buoyancyScale); // Turbulence scales with heat

    });
    
    // --- PHASE 2: SPH Forces (Density, Pressure, Viscosity) ---
    applySPHForces(circles);

    // --- PHASE 3: O(N) Integration and Boundary (PARALLELIZED) ---
    std::for_each(std::execution::par, indices.begin(), indices.end(), [winSize, dt, &circles](size_t i) {
        particle::Circle &circle = circles[i];

        // 1. Integration: Update position and velocity (RK4 is generally better than Verlet for force-based)
        circle.updatePhysics(dt);

        // 2. Boundary Collisions (Wall/Floor): Now with near-zero restitution
        circle.sideCollisions(winSize, true);
    });

    // --- PHASE 4: O(N) Position Correction and Heat Transfer (GRID-BASED & PARALLELIZED) ---
    for (int k = 0; k < 1; ++k) // Reduced iterations since SPH handles packing
    {
        resolveGridCollisions(circles, k == 0); 
    }
}

// ... (renderStep remains the same) ...

void renderStep(std::vector<particle::Circle> &circles, vec2 winSize)
{
    BeginDrawing();
    ClearBackground(Color{10, 10, 30, 255}); 

    keyEvent(circles);

    DrawFPS(4, 4);
    DrawText(("Particles: " + std::to_string(circles.size())).c_str(), 4, 20, 20, WHITE);

    // --- PHASE 5: O(N) Drawing (SEQUENTIAL) ---
    for (size_t i = 0; i < circles.size(); ++i)
    {
        particle::Circle &circle = circles[i];
        float currentTemp = temperatures[i]; 

        Color tempColor = getFireColor(currentTemp);
        
        // Dynamic radius for visual puffiness, but smaller base radius
        float baseRadius = 3.0f;
        float radiusScale = 0.8f + currentTemp * 0.4f; 
        circle.setRadius(baseRadius * radiusScale); 
        circle.setColor(tempColor);
        circle.draw();
    }

    // Draw the "Heater" zone indicator
    DrawRectangle(0, winSize.y - (winSize.y * HOT_ZONE_HEIGHT_FRAC), winSize.x, winSize.y * HOT_ZONE_HEIGHT_FRAC, ColorAlpha(RED, 0.15f));
    DrawText("HEAT SOURCE (S to spawn particles near mouse)", 10, winSize.y - 20, 16, WHITE);


    EndDrawing();
}

// --------------------------------------------------------------------------------

int main()
{
    srand(time(nullptr));

    SetTraceLogLevel(LOG_NONE);

    // Increased window size for more dramatic fluid simulation
    const vec2 win = vec2{1400, 700}; 

    // Calculate cell sizes based on window dimensions
    CELL_SIZE_X = win.x / GRID_WIDTH;
    CELL_SIZE_Y = win.y / GRID_HEIGHT;

    // The grid cell size should be at least KERNEL_RADIUS_H to guarantee 3x3 neighbor search finds all.
    if (CELL_SIZE_X < KERNEL_RADIUS_H || CELL_SIZE_Y < KERNEL_RADIUS_H) {
         // This check is crucial for correctness in a SPH grid.
         std::cerr << "WARNING: Grid cell size is smaller than KERNEL_RADIUS_H. Neighbor search may fail." << std::endl;
    }


    collisionGrid.resize(GRID_WIDTH * GRID_HEIGHT);

    std::vector<particle::Circle> circles;

    // Start with a denser initial particle count
    int N = 5000; 
    
    // Object Creation
    for (int i = 0; i < N; ++i)
    {
        particle::Circle newCircle(randomPos(win), 3.0f, 1.0f); // Smaller radius
        newCircle.setRestitution(0.0f); // Low restitution for fluid
        circles.push_back(newCircle);
        temperatures.push_back(TEMP_MIN); 
        // Initialize auxiliary SPH vectors
        densities.push_back(RHO_0); 
        pressures.push_back(0.0f); 
    }

    InitWindow(win.x, win.y, "Optimized SPH-like Fire and Smoke Convection Simulation");
    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        float frameTime = GetFrameTime();
        accumulator += frameTime;

        // --- Physics Sub-stepping Loop (Fixed Time Step) ---
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