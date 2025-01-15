#pragma once
#include <cmath>
#include <iostream>
#include <random>

// #define PI 3.14159265358979323846
#define TORAD (PI/180)
#define TODEG (180/PI)

struct vec2
{
    float x;
    float y;

    static vec2 zeros() { return vec2{ 0, 0 }; }
    static vec2 ones() { return vec2{ 1, 1 }; }

    float dot(const vec2& other) const { return x * other.x + y * other.y; }

    float len() const {
        return std::sqrt(x*x + y*y);
    }

    float distance(const vec2& other) const {
        return std::sqrt( (other.x - x)*(other.x - x) + (other.y - y)*(other.y - y) );
    }

    static vec2 random(float min=-1, float max=1) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(min, max);
        return vec2{ dist(gen), dist(gen) };
    }

    // Returns reference to allow chaining
    vec2& normalize() {
        float length = len();
        if (length != 0) {
            x /= length;
            y /= length;
        } else {
            x = 0.0f;
            y = 0.0f;
        }
        return *this;
    }

    // Returns new normalized vector
    vec2 normalized() const {
        vec2 result = *this;
        return result.normalize();
    }

    vec2& setMag(float magnitude) {
        normalize();
        x *= magnitude;
        y *= magnitude;
        return *this;
    }

    float angle() const {
        return std::atan2(y, x) * TODEG;
    }

    float fullAngle() const {
        if (x == 0 && y == 0) return 0.0f;
        if (x == 0) return (y > 0) ? 90.0f : 270.0f;
        
        float ang = angle();
        if (ang < 0) ang += 360.0f;
        return ang;
    }

    static vec2 from_angle(float deg) {
        float rad = deg * TORAD;
        return vec2{ std::cos(rad), std::sin(rad) };
    }

    void display() const {
        std::cout << "--------------------------------------\n"
                  << "Components: { "<< x << ", " << y << " }\n"
                  << "Length    : "<< len() << "\n"
                  << "Angle     : "<< fullAngle() << " deg\n"
                  << "--------------------------------------\n";
    }
};

// Operator declarations would follow...


// Arithmetic operations: vec2 with scalar
vec2 operator*(float scalar, const vec2& other) {
    return vec2{scalar * other.x, scalar * other.y};
}

vec2 operator/(float scalar, const vec2& other) {
    return vec2{scalar / other.x, scalar / other.y};
}

vec2 operator+(float scalar, const vec2& other) {
    return vec2{scalar + other.x, scalar + other.y};
}

vec2 operator-(float scalar, const vec2& other) {
    return vec2{scalar - other.x, scalar - other.y};
}

// Arithmetic operations: vec2 with scalar
vec2 operator*(const vec2& first, float scalar) {
    return vec2{first.x * scalar, first.y * scalar};
}

vec2 operator/(const vec2& first, float scalar) {
    return vec2{first.x / scalar, first.y / scalar};
}

vec2 operator+(const vec2& first, float scalar) {
    return vec2{first.x + scalar, first.y + scalar};
}

vec2 operator-(const vec2& first, float scalar) {
    return vec2{first.x - scalar, first.y - scalar};
}

// Arithmetic operations: vec2 with vec2
vec2 operator+(const vec2& first, const vec2& other) {
    return vec2{first.x + other.x, first.y + other.y};
}

vec2 operator-(const vec2& first, const vec2& other) {
    return vec2{first.x - other.x, first.y - other.y};
}

vec2 operator*(const vec2& first, const vec2& other) {
    return vec2{first.x * other.x, first.y * other.y};
}

vec2 operator/(const vec2& first, const vec2& other) {
    return vec2{first.x / other.x, first.y / other.y};
}

// Comparison operators
bool operator==(const vec2& first, const vec2& other) {
    return (first.x == other.x) && (first.y == other.y);
}

bool operator!=(const vec2& first, const vec2& other) {
    return !(first == other);
}

bool operator<(const vec2& first, const vec2& other) {
    return (first.x < other.x) && (first.y < other.y);
}

bool operator<=(const vec2& first, const vec2& other) {
    return (first.x <= other.x) && (first.y <= other.y);
}

bool operator>(const vec2& first, const vec2& other) {
    return (first.x > other.x) && (first.y > other.y);
}

bool operator>=(const vec2& first, const vec2& other) {
    return (first.x >= other.x) && (first.y >= other.y);
}

// Assignment operators
vec2& operator+=(vec2& lhs, const vec2& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    return lhs;
}

vec2& operator-=(vec2& lhs, const vec2& rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    return lhs;
}

vec2& operator*=(vec2& lhs, const vec2& rhs) {
    lhs.x *= rhs.x;
    lhs.y *= rhs.y;
    return lhs;
}

vec2& operator/=(vec2& lhs, const vec2& rhs) {
    lhs.x /= rhs.x;
    lhs.y /= rhs.y;
    return lhs;
}

// Assignment operators with scalar
vec2& operator+=(vec2& lhs, float scalar) {
    lhs.x += scalar;
    lhs.y += scalar;
    return lhs;
}

vec2& operator-=(vec2& lhs, float scalar) {
    lhs.x -= scalar;
    lhs.y -= scalar;
    return lhs;
}

vec2& operator*=(vec2& lhs, float scalar) {
    lhs.x *= scalar;
    lhs.y *= scalar;
    return lhs;
}

vec2& operator/=(vec2& lhs, float scalar) {
    lhs.x /= scalar;
    lhs.y /= scalar;
    return lhs;
}