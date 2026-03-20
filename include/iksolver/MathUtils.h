#pragma once
#include <cmath>
#include <algorithm>
#include <limits>

namespace ik {

// mathematical constants
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double HALF_PI = PI * 0.5;
constexpr double EPSILON = 1e-9;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// utility functions
template<typename T>
constexpr T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
}

template<typename T>
constexpr T lerp(T a, T b, T t) {
    return a + t * (b - a);
}

template<typename T>
constexpr T square(T x) {
    return x * x;
}

// angle utilities
inline double normalizeAngle(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle <= -PI) angle += TWO_PI;
    return angle;
}

inline double angleDifference(double a, double b) {
    return normalizeAngle(a - b);
}

inline double shortestAngularDistance(double from, double to) {
    return normalizeAngle(to - from);
}

// floating point comparison
template<typename T>
bool isApproximatelyEqual(T a, T b, T epsilon = static_cast<T>(EPSILON)) {
    return std::abs(a - b) < epsilon;
}

template<typename T>
bool isApproximatelyZero(T value, T epsilon = static_cast<T>(EPSILON)) {
    return std::abs(value) < epsilon;
}

// safe mathematical operations
inline double safeSqrt(double x) {
    return std::sqrt(std::max(0.0, x));
}

inline double safeAcos(double x) {
    return std::acos(clamp(x, -1.0, 1.0));
}

inline double safeAsin(double x) {
    return std::asin(clamp(x, -1.0, 1.0));
}

// random number utilities
inline double randomInRange(double min, double max) {
    return min + (max - min) * (static_cast<double>(rand()) / RAND_MAX);
}

} 