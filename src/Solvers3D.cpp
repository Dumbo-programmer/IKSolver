#include "../include/iksolver/Solvers3D.h"
#include "../include/iksolver/MathUtils.h"
#include <cmath>

namespace ik {

// CCD 3D implementation
bool CCDSolver3D::Solve(IKChain3D& chain, const Vector3& target, const SolverSettings& settings) {
    if (chain.size() < 2) return false;
    
    for (int iteration = 0; iteration < settings.maxIterations; ++iteration) {
        double error = performCCDStep(chain, target, settings);
        
        if (error < settings.tolerance) {
            return true;
        }
    }
    
    return false;
}

double CCDSolver3D::performCCDStep(IKChain3D& chain, const Vector3& target, const SolverSettings& settings) {
    Vector3 endEffector = chain.getEndEffector();
    
    // work backwards from the end effector
    for (int i = static_cast<int>(chain.size()) - 2; i >= 0; --i) {
        Vector3 jointPos = chain.getJointPosition(i);
        Vector3 toEnd = (endEffector - jointPos).normalized();
        Vector3 toTarget = (target - jointPos).normalized();
        
        // compute rotation axis and angle
        Vector3 rotationAxis = computeRotationAxis(toEnd, toTarget);
        double rotationAngle = computeRotationAngle(toEnd, toTarget);
        
        // Limit the rotation angle
        if (std::abs(rotationAngle) > settings.maxAngleChange) {
            rotationAngle = (rotationAngle > 0) ? settings.maxAngleChange : -settings.maxAngleChange;
        }
        
        // apply constraints if enabled
        if (settings.respectConstraints) {
            auto& joint = chain.getJoints()[i];
            if (joint.constraint.enabled) {
                double currentAngle = chain.getJointAngle(i);
                double newAngle = currentAngle + rotationAngle;
                newAngle = joint.constraint.applyConstraint(newAngle);
                rotationAngle = newAngle - currentAngle;
            }
        }
        
        // apply rotation to all joints after this one
        if (std::abs(rotationAngle) > EPSILON) {
            chain.setJointAngle(i, chain.getJointAngle(i) + rotationAngle);
            chain.updateForwardKinematics();
            endEffector = chain.getEndEffector();
        }
    }
    
    return (endEffector - target).length();
}

Vector3 CCDSolver3D::computeRotationAxis(const Vector3& from, const Vector3& to) {
    Vector3 axis = from.cross(to);
    if (axis.lengthSquared() < EPSILON) {
        // Vectors are parallel, return any perpendicular vector
        if (std::abs(from.x) < 0.9) {
            return Vector3(1, 0, 0).cross(from).normalized();
        } else {
            return Vector3(0, 1, 0).cross(from).normalized();
        }
    }
    return axis.normalized();
}

double CCDSolver3D::computeRotationAngle(const Vector3& from, const Vector3& to) {
    double cosAngle = clamp(from.dot(to), -1.0, 1.0);
    return std::acos(cosAngle);
}

// FABRIK 3D implementation
bool FABRIKSolver3D::Solve(IKChain3D& chain, const Vector3& target, const SolverSettings& settings) {
    if (chain.size() < 2) return false;
    
    Vector3 basePosition = chain.getJointPosition(0);
    
    // target is reachable
    if (!chain.isTargetReachable(target)) {
        // stretch towards target
        Vector3 direction = (target - basePosition).normalized();
        Vector3 currentPos = basePosition;
        
        for (size_t i = 0; i < chain.size() - 1; ++i) {
            auto& joints = chain.getJoints();
            currentPos += direction * joints[i].length;
            chain.setJointPosition(static_cast<int>(i + 1), currentPos);
        }
        return false;
    }
    
    for (int iteration = 0; iteration < settings.maxIterations; ++iteration) {
        // backward reaching
        backwardReach(chain, target);
        
        // forward reaching
        forwardReach(chain, basePosition);
        
        // check convergence
        Vector3 endEffector = chain.getEndEffector();
        if ((endEffector - target).length() < settings.tolerance) {
            return true;
        }
    }
    
    return false;
}

void FABRIKSolver3D::backwardReach(IKChain3D& chain, const Vector3& target) {
    auto& joints = chain.getJoints();
    
    // set end effector to target
    chain.setJointPosition(static_cast<int>(joints.size() - 1), target);
    
    // work backwards
    for (int i = static_cast<int>(joints.size()) - 2; i >= 0; --i) {
        Vector3 currentPos = chain.getJointPosition(i);
        Vector3 nextPos = chain.getJointPosition(i + 1);
        
        Vector3 direction = (currentPos - nextPos).normalized();
        Vector3 newPos = nextPos + direction * joints[i].length;
        
        chain.setJointPosition(i, newPos);
    }
}

void FABRIKSolver3D::forwardReach(IKChain3D& chain, const Vector3& basePosition) {
    auto& joints = chain.getJoints();
    
    // set base to original position
    chain.setJointPosition(0, basePosition);
    
    // work forwards
    for (size_t i = 0; i < joints.size() - 1; ++i) {
        Vector3 currentPos = chain.getJointPosition(static_cast<int>(i));
        Vector3 nextPos = chain.getJointPosition(static_cast<int>(i + 1));
        
        Vector3 direction = (nextPos - currentPos).normalized();
        Vector3 newPos = currentPos + direction * joints[i].length;
        
        chain.setJointPosition(static_cast<int>(i + 1), newPos);
    }
}

}
