#include "../include/iksolver/IKChain2D.h"
#include "../include/iksolver/MathUtils.h"

namespace ik {

// chain and compute bone lengths automatically
IKChain2D::IKChain2D(const std::vector<Vector2>& jointPositions) {
    if (jointPositions.size() < 2)
        return; //  least two joints

    basePosition = jointPositions[0];
    
    for (size_t i = 0; i < jointPositions.size(); ++i) {
        Bone2D bone;
        bone.position = jointPositions[i];
        if (i < jointPositions.size() - 1) {
            bone.length = (jointPositions[i + 1] - jointPositions[i]).length();
            bone.angle = (jointPositions[i + 1] - jointPositions[i]).angle();
        } else {
            bone.length = 0.0; // end effector has no outgoing length
            bone.angle = 0.0;
        }
        bones.push_back(bone);
    }
}

//chain with specific lengths and base position
IKChain2D::IKChain2D(const Vector2& base, const std::vector<double>& lengths) 
    : basePosition(base) {
    if (lengths.empty()) return;
    
    Vector2 currentPos = base;
    bones.reserve(lengths.size() + 1);
    
    // Add base joint
    bones.emplace_back(currentPos, lengths[0], 0.0);
    
    // Add subsequent joints
    for (size_t i = 1; i < lengths.size(); ++i) {
        currentPos.x += lengths[i-1]; // Default to horizontal chain
        bones.emplace_back(currentPos, lengths[i], 0.0);
    }
    
    // Add end effector
    currentPos.x += lengths.back();
    bones.emplace_back(currentPos, 0.0, 0.0);
}

// Return position of the last joint
Vector2 IKChain2D::getEndEffector() const {
    if (bones.empty())
        return Vector2(0,0);
    return bones.back().position;
}

// Get position of any joint
Vector2 IKChain2D::getJointPosition(int index) const {
    if (index < 0 || index >= static_cast<int>(bones.size()))
        return Vector2(0, 0);
    return bones[index].position;
}

// Move a specific joint
void IKChain2D::setJointPosition(int index, const Vector2& pos) {
    if (index < 0 || index >= static_cast<int>(bones.size()))
        return;
    bones[index].position = pos;

    // Update the length and angle to the next bone
    if (index < static_cast<int>(bones.size()) - 1) {
        Vector2 toNext = bones[index + 1].position - pos;
        bones[index].length = toNext.length();
        bones[index].angle = toNext.angle();
    }
}

// Set joint angle (absolute)
void IKChain2D::setJointAngle(int index, double angle) {
    if (index < 0 || index >= static_cast<int>(bones.size()) - 1)
        return;
        
    // Apply constraints if enabled
    if (bones[index].constraint.enabled) {
        angle = bones[index].constraint.applyConstraint(angle);
    }
    
    bones[index].angle = normalizeAngle(angle);
    updateForwardKinematics();
}

// get joint angle
double IKChain2D::getJointAngle(int index) const {
    if (index < 0 || index >= static_cast<int>(bones.size()) - 1)
        return 0.0;
    return bones[index].angle;
}

// base position and update entire chain
void IKChain2D::setBasePosition(const Vector2& base) {
    Vector2 offset = base - basePosition;
    basePosition = base;
    
    for (auto& bone : bones) {
        bone.position += offset;
    }
}

// add constraint to a joint
void IKChain2D::setJointConstraint(int index, const JointConstraint2D& constraint) {
    if (index < 0 || index >= static_cast<int>(bones.size()))
        return;
    bones[index].constraint = constraint;
}

// forward kinematics: update all joint positions based on angles
void IKChain2D::updateForwardKinematics() {
    if (bones.empty()) return;
    
    bones[0].position = basePosition;
    
    for (size_t i = 0; i < bones.size() - 1; ++i) {
        Vector2 direction(std::cos(bones[i].angle), std::sin(bones[i].angle));
        bones[i + 1].position = bones[i].position + direction * bones[i].length;
    }
}

// get total length of the chain
double IKChain2D::getTotalLength() const {
    double total = 0.0;
    for (const auto& bone : bones) {
        total += bone.length;
    }
    return total;
}

// target is reachable
bool IKChain2D::isTargetReachable(const Vector2& target) const {
    double distanceToTarget = basePosition.distanceTo(target);
    double totalLength = getTotalLength();
    return distanceToTarget <= totalLength + EPSILON;
}

// chain integrity
bool IKChain2D::isValid() const {
    if (bones.empty()) return false;
    
    for (const auto& bone : bones) {
        if (bone.length < 0.0) return false;
        if (!std::isfinite(bone.position.x) || !std::isfinite(bone.position.y)) return false;
        if (!std::isfinite(bone.angle)) return false;
    }
    
    return true;
}

} 
