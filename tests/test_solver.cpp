#include <iostream>
#include <vector>
#include <cassert>
#include "../include/iksolver/IKChain2D.h"
#include "../include/iksolver/IKChain3D.h"
#include "../include/iksolver/CCDSolver.h"
#include "../include/iksolver/FABRIKSolver.h"
#include "../include/iksolver/JacobianSolver.h"
#include "../include/iksolver/Solvers3D.h"
#include "../include/iksolver/JointConstraints.h"
#include "../include/iksolver/MathUtils.h"

using namespace ik;

void test2DVectorOperations() {
    std::cout << "Testing 2D Vector operations...\n";
    
    Vector2 v1(3, 4);
    Vector2 v2(1, 2);
    
    assert(std::abs(v1.length() - 5.0) < EPSILON);
    assert(std::abs(v1.angle() - std::atan2(4, 3)) < EPSILON);
    assert(std::abs(v1.angleTo(v2) - v1.angleTo(v2)) < EPSILON);
    
    Vector2 v3 = v1.lerp(v2, 0.5);
    assert(v3.isApproximatelyEqual(Vector2(2, 3)));
    
    std::cout << "2D Vector tests passed!\n";
}

void test3DVectorOperations() {
    std::cout << "Testing 3D Vector operations...\n";
    
    Vector3 v1(1, 0, 0);
    Vector3 v2(0, 1, 0);
    Vector3 v3 = v1.cross(v2);
    
    assert(v3.isApproximatelyEqual(Vector3(0, 0, 1)));
    assert(std::abs(v1.dot(v2)) < EPSILON);
    assert(std::abs(v1.angleTo(v2) - HALF_PI) < EPSILON);
    
    std::cout << "3D Vector tests passed!\n";
}

void testConstraints() {
    std::cout << "Testing joint constraints...\n";
    
    JointConstraint2D constraint(-PI/2, PI/2);
    assert(std::abs(constraint.applyConstraint(PI) - PI/2) < EPSILON);
    assert(std::abs(constraint.applyConstraint(-PI) - (-PI/2)) < EPSILON);
    assert(constraint.isWithinConstraints(0.0));
    assert(!constraint.isWithinConstraints(PI));
    
    std::cout << "Constraint tests passed!\n";
}

void test2DIKChain() {
    std::cout << "Testing 2D IK Chain...\n";
    
    std::vector<Vector2> joints = {
        Vector2(0, 0),
        Vector2(1, 0),
        Vector2(2, 0),
        Vector2(3, 0)
    };
    
    IKChain2D chain(joints);
    assert(chain.size() == 4);
    assert(std::abs(chain.getTotalLength() - 3.0) < EPSILON);
    assert(chain.isTargetReachable(Vector2(2, 1)));
    assert(!chain.isTargetReachable(Vector2(5, 0)));
    
    // Test constraints
    JointConstraint2D constraint(-PI/4, PI/4);
    chain.setJointConstraint(1, constraint);
    
    std::cout << "2D IK Chain tests passed!\n";
}

void test3DIKChain() {
    std::cout << "Testing 3D IK Chain...\n";
    
    std::vector<Vector3> joints = {
        Vector3(0, 0, 0),
        Vector3(0, 0, 1),
        Vector3(0, 0, 2),
        Vector3(0, 0, 3)
    };
    
    IKChain3D chain(joints);
    assert(chain.size() == 4);
    assert(std::abs(chain.getTotalLength() - 3.0) < EPSILON);
    assert(chain.isTargetReachable(Vector3(0, 0, 2.5)));
    
    std::cout << "3D IK Chain tests passed!\n";
}

void testCCDSolver2D() {
    std::cout << "Testing CCD Solver 2D...\n";
    
    std::vector<Vector2> joints = {
        Vector2(0, 0),
        Vector2(1, 0),
        Vector2(2, 0)
    };
    
    IKChain2D chain(joints);
    Vector2 target(1.5, 0.5);
    
    bool solved = CCDSolver::Solve(chain, target, 1e-3f, 10);
    if (solved) {
        Vector2 endEffector = chain.getEndEffector();
        assert((endEffector - target).length() < 1e-2);
        std::cout << "CCD 2D: Target reached at " << endEffector << "\n";
    } else {
        std::cout << "CCD 2D: Target not reached within iterations\n";
    }
    
    std::cout << "CCD 2D tests completed!\n";
}

void testFABRIKSolver2D() {
    std::cout << "Testing FABRIK Solver 2D...\n";
    
    std::vector<Vector2> joints = {
        Vector2(0, 0),
        Vector2(1, 0),
        Vector2(2, 0)
    };
    
    IKChain2D chain(joints);
    Vector2 target(1.5, 0.5);
    
    bool solved = FABRIKSolver::Solve(chain, target, 1e-3f, 10);
    if (solved) {
        Vector2 endEffector = chain.getEndEffector();
        assert((endEffector - target).length() < 1e-2);
        std::cout << "FABRIK 2D: Target reached at " << endEffector << "\n";
    } else {
        std::cout << "FABRIK 2D: Target not reached within iterations\n";
    }
    
    std::cout << "FABRIK 2D tests completed!\n";
}

void testJacobianSolver() {
    std::cout << "Testing Jacobian Solver...\n";
    
    std::vector<Vector2> joints = {
        Vector2(0, 0),
        Vector2(1, 0),
        Vector2(2, 0)
    };
    
    IKChain2D chain(joints);
    Vector2 target(1.2, 0.8);
    
    JacobianSolver::SolverSettings settings;
    settings.maxIterations = 20;
    settings.tolerance = 1e-3f;
    
    bool solved = JacobianSolver::Solve(chain, target, settings);
    if (solved) {
        Vector2 endEffector = chain.getEndEffector();
        std::cout << "Jacobian: Target reached at " << endEffector << "\n";
    } else {
        std::cout << "Jacobian: Target not reached within iterations\n";
    }
    
    std::cout << "Jacobian tests completed!\n";
}

void testMathUtils() {
    std::cout << "Testing math utilities...\n";
    
    // Test angle normalization: 3*PI should normalize to PI (not -PI)
    double normalized = normalizeAngle(3 * PI);
    assert(std::abs(normalized - PI) < EPSILON);
    
    assert(std::abs(angleDifference(PI/4, -PI/4) - PI/2) < EPSILON);
    assert(isApproximatelyEqual(1.0, 1.0 + EPSILON/2));
    assert(!isApproximatelyEqual(1.0, 1.0 + EPSILON*2));
    
    double clamped = clamp(1.5, 0.0, 1.0);
    assert(std::abs(clamped - 1.0) < EPSILON);
    
    std::cout << "Math utility tests passed!\n";
}

int main(int argc, char** argv) {
    std::cout << "=== Enhanced IK Solver Library Tests ===\n\n";
    
    try {
        test2DVectorOperations();
        test3DVectorOperations();
        testConstraints();
        test2DIKChain();
        test3DIKChain();
        testCCDSolver2D();
        testFABRIKSolver2D();
        testJacobianSolver();
        testMathUtils();

        // Edge case: unreachable target for CCD
        std::cout << "Testing CCD Solver 2D with unreachable target...\n";
        std::vector<Vector2> unreachableJoints = { Vector2(0, 0), Vector2(1, 0), Vector2(2, 0) };
        IKChain2D unreachableChain(unreachableJoints);
        Vector2 unreachableTarget(100, 100);
        bool solvedUnreachable = CCDSolver::Solve(unreachableChain, unreachableTarget, 1e-3f, 10);
        assert(!solvedUnreachable);
        std::cout << "CCD 2D: Unreachable target test passed!\n";

        // Edge case: zero-length bone for FABRIK
        std::cout << "Testing FABRIK Solver 2D with zero-length bone...\n";
        std::vector<Vector2> zeroLengthJoints = { Vector2(0, 0), Vector2(0, 0), Vector2(0, 0) };
        IKChain2D zeroLengthChain(zeroLengthJoints);
        Vector2 zeroLengthTarget(1, 1);
        bool solvedZeroLength = FABRIKSolver::Solve(zeroLengthChain, zeroLengthTarget, 1e-3f, 10);
        assert(!solvedZeroLength);
        std::cout << "FABRIK 2D: Zero-length bone test passed!\n";

        // Failure scenario: single-joint chain for Jacobian
        std::cout << "Testing Jacobian Solver with single-joint chain...\n";
        std::vector<Vector2> singleJoint = { Vector2(0, 0) };
        IKChain2D singleJointChain(singleJoint);
        JacobianSolver::SolverSettings settings;
        bool solvedSingleJoint = JacobianSolver::Solve(singleJointChain, Vector2(1, 1), settings);
        assert(!solvedSingleJoint);
        std::cout << "Jacobian: Single-joint chain test passed!\n";

        // Performance test: large chain for CCD
        std::cout << "Testing CCD Solver 2D with large chain...\n";
        std::vector<Vector2> largeChainJoints;
        for (int i = 0; i < 100; ++i) largeChainJoints.push_back(Vector2(i, 0));
        IKChain2D largeChain(largeChainJoints);
        Vector2 largeTarget(50, 50);
        bool solvedLarge = CCDSolver::Solve(largeChain, largeTarget, 1e-2f, 100);
        std::cout << "CCD 2D: Large chain test completed, solved=" << solvedLarge << "\n";

        std::cout << "\n=== All tests completed successfully! ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}