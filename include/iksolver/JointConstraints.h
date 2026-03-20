#pragma once
#include "./Vector2.h"
#include "./Vector3.h"
#include <limits>

namespace ik {

// constraint 
struct JointConstraint2D {
    double minAngle = -std::numeric_limits<double>::infinity();
    double maxAngle = std::numeric_limits<double>::infinity();
    bool enabled = false;
    
    JointConstraint2D() = default;
    JointConstraint2D(double min, double max) : minAngle(min), maxAngle(max), enabled(true) {}
    
    // apply constraint to an angle (in rads)
    double applyConstraint(double angle) const {
        if (!enabled) return angle;
        return std::clamp(angle, minAngle, maxAngle);
    }
    
    // angle is within constraints
    bool isWithinConstraints(double angle) const {
        if (!enabled) return true;
        return angle >= minAngle && angle <= maxAngle;
    }
};

// more complex 3D joint constraints (spherical joints, etc.)
struct JointConstraint3D {
    Vector3 axis = {0, 0, 1}; // rotation axis
    double minAngle = -std::numeric_limits<double>::infinity();
    double maxAngle = std::numeric_limits<double>::infinity();
    bool enabled = false;
    
    JointConstraint3D() = default;
    JointConstraint3D(const Vector3& rotationAxis, double min, double max) 
        : axis(rotationAxis.normalized()), minAngle(min), maxAngle(max), enabled(true) {}
    
    // apply constraint (simplified for now)
    double applyConstraint(double angle) const {
        if (!enabled) return angle;
        return std::clamp(angle, minAngle, maxAngle);
    }
    
    bool isWithinConstraints(double angle) const {
        if (!enabled) return true;
        return angle >= minAngle && angle <= maxAngle;
    }
};

} 