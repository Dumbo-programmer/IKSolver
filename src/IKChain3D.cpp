#include "../include/iksolver/IKChain3D.h"
#include "../include/iksolver/MathUtils.h"

namespace ik {

IKChain3D::IKChain3D(const Vector3& root, int numJoints, double segmentLength) 
    : basePosition(root) {
    joints.reserve(numJoints);
    Vector3 pos = root;
    for (int i = 0; i < numJoints; i++) {
        joints.emplace_back(pos, segmentLength);
        pos.z += segmentLength; // default orientation along +Z
    }
}

IKChain3D::IKChain3D(const std::vector<Vector3>& jointPositions) {
    if (jointPositions.size() < 2) return;
    
    basePosition = jointPositions[0];
    joints.reserve(jointPositions.size());
    
    for (size_t i = 0; i < jointPositions.size(); ++i) {
        double length = 0.0;
        if (i < jointPositions.size() - 1) {
            length = (jointPositions[i + 1] - jointPositions[i]).length();
        }
        joints.emplace_back(jointPositions[i], length);
    }
}

void IKChain3D::setRoot(const Vector3& root) {
    Vector3 offset = root - basePosition;
    basePosition = root;
    
    for (auto& joint : joints) {
        joint.position += offset;
    }
}

void IKChain3D::setTarget(const Vector3& t) {
    target = t;
}

Vector3 IKChain3D::getEndEffector() const {
    if (!joints.empty())
        return joints.back().position;
    return Vector3();
}

Vector3 IKChain3D::getJointPosition(int index) const {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return Vector3();
    return joints[index].position;
}

void IKChain3D::setJointPosition(int index, const Vector3& pos) {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return;
    joints[index].position = pos;
    
    // Update length to next joint
    if (index < static_cast<int>(joints.size()) - 1) {
        joints[index].length = (joints[index + 1].position - pos).length();
    }
}

void IKChain3D::setJointAngle(int index, double angle) {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return;
        
    // Apply constraints if enabled
    if (joints[index].constraint.enabled) {
        angle = joints[index].constraint.applyConstraint(angle);
    }
    
    joints[index].angle = normalizeAngle(angle);
}

double IKChain3D::getJointAngle(int index) const {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return 0.0;
    return joints[index].angle;
}

void IKChain3D::setJointRotationAxis(int index, const Vector3& axis) {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return;
    joints[index].rotationAxis = axis.normalized();
}

Vector3 IKChain3D::getJointRotationAxis(int index) const {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return Vector3(0, 0, 1);
    return joints[index].rotationAxis;
}

void IKChain3D::setJointConstraint(int index, const JointConstraint3D& constraint) {
    if (index < 0 || index >= static_cast<int>(joints.size()))
        return;
    joints[index].constraint = constraint;
}

void IKChain3D::updateForwardKinematics() {
    if (joints.empty()) return;
    
    joints[0].position = basePosition;
    
    for (size_t i = 0; i < joints.size() - 1; ++i) {
        // Simplified FK - just move along current direction
        Vector3 direction = joints[i].rotationAxis;
        joints[i + 1].position = joints[i].position + direction * joints[i].length;
    }
}

double IKChain3D::getTotalLength() const {
    double total = 0.0;
    for (const auto& joint : joints) {
        total += joint.length;
    }
    return total;
}

bool IKChain3D::isTargetReachable(const Vector3& target) const {
    double distanceToTarget = basePosition.distanceTo(target);
    double totalLength = getTotalLength();
    return distanceToTarget <= totalLength + EPSILON;
}

bool IKChain3D::isValid() const {
    if (joints.empty()) return false;
    
    for (const auto& joint : joints) {
        if (joint.length < 0.0) return false;
        if (!std::isfinite(joint.position.x) || !std::isfinite(joint.position.y) || !std::isfinite(joint.position.z)) return false;
        if (!std::isfinite(joint.angle)) return false;
    }
    
    return true;
}

void IKChain3D::computeLengthsFromPositions() {
    for (size_t i = 0; i < joints.size() - 1; ++i) {
        joints[i].length = (joints[i + 1].position - joints[i].position).length();
    }
    if (!joints.empty()) {
        joints.back().length = 0.0; // end effector
    }
}

}
