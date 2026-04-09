
#pragma once
#include <cmath>
#include <iostream>
#include <algorithm>
#include "iksolver/MathUtils.h"

namespace ik {

struct Vector3 {
    double x, y, z;

    /**
     * @brief Default constructor (zero vector)
     */
    Vector3() : x(0), y(0), z(0) {}
    /**
     * @brief Construct with x, y, z values
     */
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Arithmetic operators
    Vector3 operator+(const Vector3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vector3 operator-(const Vector3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vector3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vector3 operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }

    Vector3& operator+=(const Vector3& other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vector3& operator-=(const Vector3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
    Vector3& operator*=(double scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vector3& operator/=(double scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    // Norms
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    double lengthSquared() const { return x * x + y * y + z * z; }
    Vector3 normalized() const {
        double len = length();
        return (len > ik::EPSILON) ? Vector3(x / len, y / len, z / len) : Vector3(0, 0, 0);
    }

    // Dot & cross
    double dot(const Vector3& other) const { return x * other.x + y * other.y + z * other.z; }
    Vector3 cross(const Vector3& other) const {
        return {y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
    }

    // Angle operations
    double angleTo(const Vector3& other) const {
        const double denom = this->length() * other.length();
        if (denom <= ik::EPSILON) {
            return 0.0;
        }
        double cosTheta = this->dot(other) / denom;
        return std::acos(ik::clamp<double>(cosTheta, -1.0, 1.0));
    }
    
    // Reflection
    Vector3 reflect(const Vector3& normal) const {
        return *this - normal * (2.0 * this->dot(normal));
    }

    // Linear interpolation
    Vector3 lerp(const Vector3& other, double t) const {
        return *this + (other - *this) * t;
    }

    // Distance to another vector
    double distanceTo(const Vector3& other) const {
        return (*this - other).length();
    }
    
    // Vectors are approximately equal
    bool isApproximatelyEqual(const Vector3& other, double epsilon = EPSILON) const {
         return ik::isApproximatelyEqual<double>(x, other.x, epsilon) &&
             ik::isApproximatelyEqual<double>(y, other.y, epsilon) &&
             ik::isApproximatelyEqual<double>(z, other.z, epsilon);
    }

    // Clamp components
    Vector3 clamp(double minVal, double maxVal) const {
        return {ik::clamp<double>(x, minVal, maxVal),
            ik::clamp<double>(y, minVal, maxVal),
            ik::clamp<double>(z, minVal, maxVal)};
    }

    // project onto another vector
    Vector3 projectOnto(const Vector3& other) const {
        const double lenSq = other.lengthSquared();
        if (lenSq <= ik::EPSILON) {
            return Vector3(0.0, 0.0, 0.0);
        }
        return other * (this->dot(other) / lenSq);
    }

    // debug
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

} 
