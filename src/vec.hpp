#pragma once
#include <cmath>
#include <random>
#include <limits> // For std::numeric_limits

// Define this in your build system (e.g., -DPARTICLE_DEBUG=1) to enable printing
// #define PARTICLE_DEBUG 1
#if defined(PARTICLE_DEBUG) && PARTICLE_DEBUG == 1
#include <iostream>
#include <ostream>
#endif

#undef PI

// Modern C++ constants are type-safe and namespace-scoped
static constexpr double PI = 3.14159265358979323846;
static constexpr float TORAD = static_cast<float>(PI / 180.0);
static constexpr float TODEG = static_cast<float>(180.0 / PI);

struct vec2
{
    float x;
    float y;

    /**
     * @brief Checks if two floats are within an epsilon (tolerance) of each other.
     */
    static bool isNear(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::abs(a - b) <= epsilon;
    }

    static constexpr vec2 zeros() noexcept { return vec2{ 0.0f, 0.0f }; }
    static constexpr vec2 ones() noexcept { return vec2{ 1.0f, 1.0f }; }

    float dot(const vec2& other) const noexcept { return x * other.x + y * other.y; }

    float len() const noexcept {
        return std::sqrt(x*x + y*y);
    }

    /**
     * @brief Returns the squared length (magnitude) of the vector.
     * Faster than len() as it avoids sqrt.
     */
    float lenSq() const noexcept {
        return x*x + y*y;
    }

    float distance(const vec2& other) const noexcept {
        const float dx = other.x - x;
        const float dy = other.y - y;
        return std::sqrt(dx*dx + dy*dy);
    }

    /**
     * @brief Returns the squared distance to another vector.
     * Faster than distance() as it avoids sqrt. Ideal for comparisons.
     */
    float distanceSq(const vec2& other) const noexcept {
        const float dx = other.x - x;
        const float dy = other.y - y;
        return (dx*dx + dy*dy);
    }

    static vec2 random(float min = -1.0f, float max = 1.0f) {
        // This static generator is not thread-safe, but common for simple projects.
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(min, max);
        return vec2{ dist(gen), dist(gen) };
    }

    vec2& normalize() noexcept {
        float length = len();
        // Use an epsilon to prevent division by near-zero
        if (length > 1e-6f) {
            x /= length;
            y /= length;
        } else {
            x = 0.0f;
            y = 0.0f;
        }
        return *this;
    }

    vec2 normalized() const noexcept {
        vec2 result = *this;
        return result.normalize();
    }

    vec2& setMag(float magnitude) noexcept {
        normalize();
        x *= magnitude;
        y *= magnitude;
        return *this;
    }

    float angle() const noexcept {
        return std::atan2(y, x) * TODEG;
    }

    float fullAngle() const noexcept {
        if (isNear(x, 0.0f) && isNear(y, 0.0f)) return 0.0f;
        
        float ang = angle(); // angle() uses atan2, which handles quadrants
        if (ang < 0.0f) ang += 360.0f;
        return ang;
    }

    static vec2 from_angle(float deg) noexcept {
        float rad = deg * TORAD;
        return vec2{ std::cos(rad), std::sin(rad) };
    }
};

// --- Arithmetic operators (scalar) ---
inline vec2 operator*(float scalar, const vec2& v) noexcept {
    return vec2{scalar * v.x, scalar * v.y};
}
inline vec2 operator*(const vec2& v, float scalar) noexcept {
    return vec2{v.x * scalar, v.y * scalar};
}
inline vec2 operator/(const vec2& v, float scalar) noexcept {
    return vec2{v.x / scalar, v.y / scalar};
}
inline vec2 operator+(const vec2& v, float scalar) noexcept {
    return vec2{v.x + scalar, v.y + scalar};
}
inline vec2 operator-(const vec2& v, float scalar) noexcept {
    return vec2{v.x - scalar, v.y - scalar};
}

// --- Arithmetic operators (vec2) ---
inline vec2 operator+(const vec2& a, const vec2& b) noexcept {
    return vec2{a.x + b.x, a.y + b.y};
}
inline vec2 operator-(const vec2& a, const vec2& b) noexcept {
    return vec2{a.x - b.x, a.y - b.y};
}
inline vec2 operator*(const vec2& a, const vec2& b) noexcept {
    return vec2{a.x * b.x, a.y * b.y};
}
inline vec2 operator/(const vec2& a, const vec2& b) noexcept {
    return vec2{a.x / b.x, a.y / b.y};
}

// --- Comparison (Epsilon-based) ---
inline bool operator==(const vec2& a, const vec2& b) noexcept {
    return vec2::isNear(a.x, b.x) && vec2::isNear(a.y, b.y);
}
inline bool operator!=(const vec2& a, const vec2& b) noexcept {
    return !(a == b);
}

// --- Assignment operators (vec2) ---
inline vec2& operator+=(vec2& a, const vec2& b) noexcept {
    a.x += b.x; a.y += b.y; return a;
}
inline vec2& operator-=(vec2& a, const vec2& b) noexcept {
    a.x -= b.x; a.y -= b.y; return a;
}
inline vec2& operator*=(vec2& a, const vec2& b) noexcept {
    a.x *= b.x; a.y *= b.y; return a;
}
inline vec2& operator/=(vec2& a, const vec2& b) noexcept {
    a.x /= b.x; a.y /= b.y; return a;
}

// --- Assignment operators (scalar) ---
inline vec2& operator+=(vec2& v, float s) noexcept {
    v.x += s; v.y += s; return v;
}
inline vec2& operator-=(vec2& v, float s) noexcept {
    v.x -= s; v.y -= s; return v;
}
inline vec2& operator*=(vec2& v, float s) noexcept {
    v.x *= s; v.y *= s; return v;
}
inline vec2& operator/=(vec2& v, float s) noexcept {
    v.x /= s; v.y /= s; return v;
}

// --- Debug Printing (Conditional) ---
#if defined(PARTICLE_DEBUG) && PARTICLE_DEBUG == 1
inline std::ostream& operator<<(std::ostream& os, const vec2& v) {
    os << "{ x: " << v.x << ", y: " << v.y << " }";
    return os;
}
#endif // PARTICLE_DEBUG