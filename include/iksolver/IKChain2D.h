#pragma once
#include <vector>
#include <cmath>
#include "./Vector2.h"
#include "./JointConstraints.h"

namespace ik {

struct Bone2D {
    Vector2 position; // joint position
    double length;    // length to next joint (or end effector)
    double angle;     // absolute angle in world space (radians)
    JointConstraint2D constraint; // joint constraints
    
    Bone2D() : position(0, 0), length(0), angle(0) {}
    Bone2D(const Vector2& pos, double len, double ang = 0) 
        : position(pos), length(len), angle(ang) {}
};

class IKChain2D {
public:
    std::vector<Bone2D> bones;
    Vector2 basePosition; // root position of the chain

    // construct chain from vector of joint positions
    IKChain2D(const std::vector<Vector2>& jointPositions);
    
    // construct chain with specific lengths and base position
    IKChain2D(const Vector2& base, const std::vector<double>& lengths);

    // get the position of the end effector (last joint)
    Vector2 getEndEffector() const;
    
    // get position of any joint
    Vector2 getJointPosition(int index) const;

    // mov a specific joint
    void setJointPosition(int index, const Vector2& pos);
    
    // set joint angle (absolute)
    void setJointAngle(int index, double angle);
    
    // get joint angle
    double getJointAngle(int index) const;
    
    // set base position and update entire chain
    void setBasePosition(const Vector2& base);
    
    // add constraint
    void setJointConstraint(int index, const JointConstraint2D& constraint);
    
    // forward kinematics
    void updateForwardKinematics();
    
    // get total length of the chain
    double getTotalLength() const;

    // check if target is reachable
    bool isTargetReachable(const Vector2& target) const;
    
    // validate chain integrity
    bool isValid() const;

    // total number of joints
    size_t size() const { return bones.size(); }
    
    // get number of degrees of freedom
    size_t getDegreesOfFreedom() const { return bones.empty() ? 0 : bones.size() - 1; }
};

} 
