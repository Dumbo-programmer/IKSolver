#pragma once
#include "./IKChain3D.h"
#include "./Vector3.h"

namespace ik {

/**
 * @class CCDSolver3D
 * @brief Cyclic Coordinate Descent (CCD) solver for 3D inverse kinematics.
 *
 * Provides static methods to solve a 3D kinematic chain using CCD algorithm.
 */
class CCDSolver3D {
public:
    /**
     * @struct SolverSettings
     * @brief Settings for CCDSolver3D.
     */
    struct SolverSettings {
        float tolerance = 1e-3f; ///< Acceptable distance to target
        int maxIterations = 50; ///< Maximum number of iterations
        float maxAngleChange = 0.26f; ///< Max angle change per iteration
        bool respectConstraints = true; ///< Enforce joint constraints
        SolverSettings() = default;
    };

    /**
     * @brief Solve a 3D IK chain using CCD.
     * @param chain The IKChain3D to solve.
     * @param target The target position for the end effector.
     * @param settings Solver settings.
     * @return True if target is reached within tolerance, false otherwise.
     */
    static bool Solve(IKChain3D& chain, const Vector3& target, const SolverSettings& settings);

private:
    /**
     * @brief Perform a single CCD iteration step.
     */
    static double performCCDStep(IKChain3D& chain, const Vector3& target, const SolverSettings& settings);

    /**
     * @brief Compute rotation axis needed to align two vectors.
     */
    static Vector3 computeRotationAxis(const Vector3& from, const Vector3& to);
    /**
     * @brief Compute rotation angle needed to align two vectors.
     */
    static double computeRotationAngle(const Vector3& from, const Vector3& to);
};

/**
 * @class FABRIKSolver3D
 * @brief FABRIK solver for 3D inverse kinematics.
 *
 * Provides static methods to solve a 3D kinematic chain using FABRIK algorithm.
 */
class FABRIKSolver3D {
public:
    /**
     * @struct SolverSettings
     * @brief Settings for FABRIKSolver3D.
     */
    struct SolverSettings {
        float tolerance = 1e-3f; ///< Acceptable distance to target
        int maxIterations = 50; ///< Maximum number of iterations
        bool respectConstraints = true; ///< Enforce joint constraints
        SolverSettings() = default;
    };

    /**
     * @brief Solve a 3D IK chain using FABRIK.
     * @param chain The IKChain3D to solve.
     * @param target The target position for the end effector.
     * @param settings Solver settings.
     * @return True if target is reached within tolerance, false otherwise.
     */
    static bool Solve(IKChain3D& chain, const Vector3& target, const SolverSettings& settings);

private:
    /**
     * @brief Backward reaching phase for FABRIK.
     */
    static void backwardReach(IKChain3D& chain, const Vector3& target);
    /**
     * @brief Forward reaching phase for FABRIK.
     */
    static void forwardReach(IKChain3D& chain, const Vector3& basePosition);
};

} 
