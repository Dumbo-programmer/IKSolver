#pragma once
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <algorithm>

namespace ik {

struct Vector2 {
    double x, y;

    /**
     * @brief Default constructor (zero vector)
     */
    Vector2() : x(0), y(0) {}
    /**
     * @brief Construct with x and y values
     */
    Vector2(double x_, double y_) : x(x_), y(y_) {}

    // Arithmetic operators
    Vector2 operator+(const Vector2& other) const { return {x + other.x, y + other.y}; }
    Vector2 operator-(const Vector2& other) const { return {x - other.x, y - other.y}; }
    Vector2 operator*(double scalar) const { return {x * scalar, y * scalar}; }
    Vector2 operator/(double scalar) const { return {x / scalar, y / scalar}; }

    Vector2& operator+=(const Vector2& other) { x += other.x; y += other.y; return *this; }
    Vector2& operator-=(const Vector2& other) { x -= other.x; y -= other.y; return *this; }
    Vector2& operator*=(double scalar) { x *= scalar; y *= scalar; return *this; }
    Vector2& operator/=(double scalar) { x /= scalar; y /= scalar; return *this; }

    // Norms
    double length() const { return std::sqrt(x * x + y * y); }
    double lengthSquared() const { return x * x + y * y; }
    Vector2 normalized() const {
        double len = length();
        return (len > EPSILON) ? Vector2(x / len, y / len) : Vector2(0, 0);
    }

    // Dot & cross
    double dot(const Vector2& other) const { return x * other.x + y * other.y; }
    double cross(const Vector2& other) const { return x * other.y - y * other.x; } // scalar in 2D

    // Rotation (2D only)
    Vector2 rotated(double radians) const {
        double c = std::cos(radians);
        double s = std::sin(radians);
        return {x * c - y * s, x * s + y * c};
    }

    // Angle operations
    double angle() const { return std::atan2(y, x); }
    double angleTo(const Vector2& other) const {
        return std::atan2(cross(other), dot(other));
    }
    
    // Perpendicular vector (90-degree rotation)
    Vector2 perpendicular() const { return {-y, x}; }

    // Reflection
    Vector2 reflect(const Vector2& normal) const {
        return *this - normal * (2.0 * this->dot(normal));
    }

    // Linear interpolation
    Vector2 lerp(const Vector2& other, double t) const {
        return *this + (other - *this) * t;
    }

    // Distance to another vector
    double distanceTo(const Vector2& other) const {
        return (*this - other).length();
    }
    
    // Vectors are approximately equal
    bool isApproximatelyEqual(const Vector2& other, double epsilon = EPSILON) const {
        return isApproximatelyEqual(x, other.x, epsilon) && isApproximatelyEqual(y, other.y, epsilon);
    }
    
    // Clamp components
    Vector2 clamp(double minVal, double maxVal) const {
        return {clamp(x, minVal, maxVal), clamp(y, minVal, maxVal)};
    }

    // debug
    friend std::ostream& operator<<(std::ostream& os, const Vector2& v) {
        os << "(" << v.x << ", " << v.y << ")";
        return os;
    }
};

} 
