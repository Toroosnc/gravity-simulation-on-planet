#pragma once

#include <string>

struct Vec2 {
    double x{};
    double y{};

    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
};

struct Body {
    std::string name;
    Vec2 position;
    Vec2 velocity;
    double mass;
    int radius;
    const char* color;
    double orbit_radius{};
};
