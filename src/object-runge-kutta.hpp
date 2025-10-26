#pragma once
#include <draw.hpp>
#include <iostream>
#include <vec.hpp>
#include <vector>

// Assuming 'vec2' is a class with the following operators/methods:
// vec2 operator+(const vec2& other)
// vec2 operator-(const vec2& other)
// vec2 operator*(float scalar)
// vec2 operator/(float scalar)
// vec2::zeros()
// vec2::random(float min, float max)
// len()
// dot(const vec2& other)
// distance(const vec2& other)

// Assuming 'PI' is defined
// Assuming 'Draw::CircleWithStrok' and 'Color' (like BLACK) are defined

namespace particle
{
// --- RK4 Helper Structures ---

/**
 * @brief Represents the state of the particle at a given time.
 * The state vector for this 2D system is [pos.x, pos.y, vel.x, vel.y]
 */
struct State
{
    vec2 pos;
    vec2 velocity;
};

/**
 * @brief Represents the derivative of the state vector (slope).
 * The derivative vector is [d_pos/dt, d_velocity/dt] = [velocity, acceleration]
 */
struct Derivative
{
    vec2 d_pos;      // Change in position (i.e., velocity)
    vec2 d_velocity; // Change in velocity (i.e., acceleration)

    // Overload operators for vector math required by RK4
    Derivative operator*(float scalar) const
    {
        return {d_pos * scalar, d_velocity * scalar};
    }
    Derivative operator+(const Derivative &other) const
    {
        return {d_pos + other.d_pos, d_velocity + other.d_velocity};
    }
};

// ----------------------------

class Object
{
  public:
    Object(vec2 pos, float area, float density) : m_pos(pos), m_area(area), m_density(density)
    {
        s_count++;
        m_acceleratation = vec2::zeros();
        m_forces.reserve(5);
        m_isStatic = false;

        // Note: The loop seems counter-intuitive,
        // but the subsequent line overwrites it anyway.
        do
        {
            m_velocity = vec2::random(-1.f, 1.f);
        } while (m_velocity.len() <= 1);

        m_velocity = vec2::random(-200.f, 200.f);
    }
    static int s_count;

    void addForce(vec2 force)
    {
        m_forces.push_back(force);
    }
    float getMass()
    {
        return m_area * m_density;
    }
    float getMomentum()
    {
        return getMass() * m_velocity.len();
    }
    vec2 getPosition()
    {
        return m_pos;
    }

    float getKE()
    {
        float mag = m_velocity.len();
        return 0.5f * getMass() * (mag * mag);
    }

    vec2 getAllForces()
    {
        vec2 allForces = vec2::zeros();
        for (vec2 force : m_forces)
        {
            allForces += force;
        }
        return allForces;
    }

    void clearForces()
    {
        m_forces.clear();
    }

    void setVelocity(vec2 velosity)
    {
        m_velocity = velosity;
    }

    vec2 getVelocity()
    {
        return m_velocity;
    }

    // OLD: void updatePosition(float ts) -> REMOVED. Use updatePhysics.
    void updatePosition(float ts)
    {
        updatePhysics(ts);
    }

    void setRestitution(float restitution)
    {
        m_restitution = restitution;
    }

    float getRestitution()
    {
        return m_restitution;
    }

    // --- RK4 Implementation Functions ---

    /**
     * @brief Returns the current state (position and velocity) of the object.
     */
    State getCurrentState() const
    {
        return {m_pos, m_velocity};
    }

    /**
     * @brief Calculates the derivatives (velocity and acceleration) for a given state.
     * @param state The state at which to calculate the derivatives.
     */
    Derivative getDerivative(const State &state)
    {
        // Calculate Acceleration (F/m) based on the forces currently applied to the object
        // Assumes forces (m_forces) are set before calling the main RK4 step.
        vec2 totalForce = getAllForces();
        float mass = getMass();
        vec2 acceleration = totalForce / mass;

        Derivative deriv;
        deriv.d_pos = state.velocity;    // dx/dt = velocity
        deriv.d_velocity = acceleration; // dv/dt = acceleration
        return deriv;
    }

    /**
     * @brief Integrates the physics over a time step 'ts' using Runge-Kutta 4.
     */
    void updatePhysics(float ts)
    {
        if (m_isStatic)
            return;

        // Step 1: Get k1 (Derivative at t)
        State initial_state = getCurrentState();
        Derivative k1 = getDerivative(initial_state);

        // Step 2: Get k2 (Derivative at t + 0.5*ts, using initial state + 0.5*ts*k1)
        State state_k2 = {initial_state.pos + k1.d_pos * (0.5f * ts),
                          initial_state.velocity + k1.d_velocity * (0.5f * ts)};
        Derivative k2 = getDerivative(state_k2);

        // Step 3: Get k3 (Derivative at t + 0.5*ts, using initial state + 0.5*ts*k2)
        State state_k3 = {initial_state.pos + k2.d_pos * (0.5f * ts),
                          initial_state.velocity + k2.d_velocity * (0.5f * ts)};
        Derivative k3 = getDerivative(state_k3);

        // Step 4: Get k4 (Derivative at t + ts, using initial state + ts*k3)
        State state_k4 = {initial_state.pos + k3.d_pos * ts, initial_state.velocity + k3.d_velocity * ts};
        Derivative k4 = getDerivative(state_k4);

        // Weighted sum of derivatives: (k1 + 2*k2 + 2*k3 + k4) / 6
        Derivative final_deriv = k1 + (k2 * 2.0f) + (k3 * 2.0f) + k4;
        final_deriv = final_deriv * (1.0f / 6.0f);

        // Update the state (position and velocity) using the final average derivative
        m_pos += final_deriv.d_pos * ts;
        m_velocity += final_deriv.d_velocity * ts;

        // Note: m_acceleratation is now only for display/debugging purposes.
        // Its actual use for integration is inside getDerivative.
        m_acceleratation = final_deriv.d_velocity;
    }

    // ------------------------------------

    virtual void moveTo(vec2 location) = 0;
    virtual void move(vec2 dPos) = 0;

    void dispState()
    {
        std::cout << "Position           :" << m_pos.x << ", " << m_pos.y << std::endl;
        std::cout << "Velosity           :" << m_velocity.x << " ," << m_velocity.y << std::endl;
        std::cout << "Acceleration       :" << m_acceleratation.x << ", " << m_acceleratation.x << std::endl;
        std::cout << "Mass               :" << getMass() << std::endl;
        std::cout << "Kinatic Energy     :" << getKE() << std::endl;
        std::cout << "Momentum           :" << getMomentum() << std::endl;
    }

    ~Object()
    {
        s_count--;
    }

  protected:
    vec2 m_pos;
    vec2 m_velocity;
    vec2 m_acceleratation; // Used for display, but calculated inside getDerivative for RK4
    float m_area;          // deal with this as a volume.
    float m_density;
    float m_restitution;
    bool m_isStatic;
    std::vector<vec2> m_forces;
}; // object

int Object::s_count = 0;

// ----------------------------------------------------------------------

class Circle : public Object
{
  public:
    Circle(vec2 pos, float raduis, float density = 1.0f) : Object(pos, PI * raduis * raduis, density), m_radius(raduis)
    {
        // ...
    }

    void draw()
    {
        Draw::CircleWithStrok(m_pos, m_radius, 1, m_color, BLACK);
    }

    float getRaduis()
    {
        return m_radius;
    }

    void moveTo(vec2 location) override
    {
        m_pos = location;
    }

    void move(vec2 dPos) override
    {
        m_pos += dPos;
    }

    // OLD: void updatePosition(float ts) -> REMOVED. Use Object::updatePhysics.
    // OLD: void updateVelocity(float ts) -> REMOVED. Use Object::updatePhysics.

    void updatePosition(float ts)
    {
        updatePhysics(ts);
    }

    void updateVelocity(float ts)
    {
        updatePhysics(ts);
    }

    void sideCollisions(vec2 winSize, bool roof = true)
    {
        // Clamp position to boundary
        if (m_pos.x - m_radius <= 0)
            m_pos.x = m_radius;
        else if (m_pos.x + m_radius >= winSize.x)
            m_pos.x = winSize.x - m_radius;

        if (m_pos.y - m_radius <= 0 && roof)
            m_pos.y = m_radius;
        else if (m_pos.y + m_radius >= winSize.y)
            m_pos.y = winSize.y - m_radius;

        // Handle collisions with left or right walls
        if (m_pos.x - m_radius <= 0 || m_pos.x + m_radius >= winSize.x)
        {
            // Reflect velocity and apply restitution
            m_velocity.x *= -m_restitution;
        }

        // Handle collisions with top or bottom walls
        if (((m_pos.y - m_radius <= 0) && roof) || m_pos.y + m_radius >= winSize.y)
        {
            // Reflect velocity and apply restitution
            m_velocity.y *= -m_restitution;
        }
    }

    bool isCollide(particle::Circle &other)
    {
        if (!(other.getPosition() == this->getPosition()))
        {
            float d = getPosition().distance(other.getPosition());
            if (d < (getRaduis() + other.getRaduis()))
                return true;
        }
        return false;
    }

    void updateCollisionVelocity(particle::Circle &other)
    {
        float m1 = this->getMass();
        float m2 = other.getMass();
        float e = m_restitution; // Assuming both objects share the same restitution.

        vec2 pos1 = this->getPosition();
        vec2 pos2 = other.getPosition();

        vec2 v1 = this->getVelocity();
        vec2 v2 = other.getVelocity();

        vec2 relativePos = pos2 - pos1;
        vec2 relativeVelocity = v2 - v1;

        float distance = relativePos.len();
        if (distance < 1e-6f)
            distance = 1e-6f; // Avoid division by zero

        vec2 collisionAxis = relativePos / distance;

        float velocityAlongAxis = relativeVelocity.dot(collisionAxis);

        if (velocityAlongAxis > 0)
            return; // Already separating

        // Impulse scalar (J)
        float impulse = (-(1.0f + e) * velocityAlongAxis) / (1.0f / m1 + 1.0f / m2);

        vec2 impulseVector = impulse * collisionAxis;
        this->setVelocity(v1 - impulseVector / m1);
        other.setVelocity(v2 + impulseVector / m2);
    }

    void correctOverlap(particle::Circle &other)
    {
        vec2 relativePos = other.getPosition() - this->getPosition();
        float distance = relativePos.len();

        if (distance < 1e-6f)
            return; // Prevent division by zero if positions are identical

        vec2 collisionAxis = relativePos / distance;

        float overlap = this->getRaduis() + other.getRaduis() - distance;
        if (overlap > 0)
        {
            // Simple position correction: move each by half the overlap
            vec2 correction = collisionAxis * (overlap / 2.0f);
            this->moveTo(this->getPosition() - correction);
            other.moveTo(other.getPosition() + correction);
        }
    }

    void setColor(Color col)
    {
        m_color = col;
    }

    Color getColor()
    {
        return m_color;
    }

    vec2 getAcceleration()
    {
        return m_acceleratation;
    }

    void setAcceleration(vec2 acceleration)
    {
        m_acceleratation = acceleration;
    }

    // OLD: void updateVelocity(float ts) -> REMOVED. Use Object::updatePhysics.

    std::vector<vec2> path;

  private:
    float m_radius;
    Color m_color;

}; // Circle

} // namespace particle