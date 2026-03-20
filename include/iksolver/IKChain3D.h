#pragma once
#include <vector>
#include "Vector3.h"
#include "JointConstraints.h"

namespace ik {

struct Joint3D {
    Vector3 position;
    double length; // distance to next joint
    Vector3 rotationAxis; // local rotation axis
    double angle; // rotation angle around axis
    JointConstraint3D constraint; // joint constraints

    Joint3D(const Vector3& pos, double len = 0.0, const Vector3& axis = {0, 0, 1}) 
        : position(pos), length(len), rotationAxis(axis.normalized()), angle(0) {}
};

class IKChain3D {
public:
    IKChain3D() = default;
    IKChain3D(const Vector3& root, int numJoints, double segmentLength);
    IKChain3D(const std::vector<Vector3>& jointPositions);

    // chain management
    void setRoot(const Vector3& root);
    void setTarget(const Vector3& target);
    Vector3 getEndEffector() const;
    Vector3 getJointPosition(int index) const;
    void setJointPosition(int index, const Vector3& pos);
    
    // angle management
    void setJointAngle(int index, double angle);
    double getJointAngle(int index) const;
    void setJointRotationAxis(int index, const Vector3& axis);
    Vector3 getJointRotationAxis(int index) const;
    
    // constraints
    void setJointConstraint(int index, const JointConstraint3D& constraint);
    
    // forward kinematics
    void updateForwardKinematics();
    
    // utility functions
    double getTotalLength() const;
    bool isTargetReachable(const Vector3& target) const;
    bool isValid() const;
    size_t size() const { return joints.size(); }
    size_t getDegreesOfFreedom() const { return joints.empty() ? 0 : joints.size() - 1; }
    
    const std::vector<Joint3D>& getJoints() const { return joints; }
    std::vector<Joint3D>& getJoints() { return joints; }

private:
    std::vector<Joint3D> joints;
    Vector3 target;
    Vector3 basePosition;
    
    void computeLengthsFromPositions();
};

}