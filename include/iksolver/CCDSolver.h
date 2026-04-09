#pragma once
#include "./IKChain2D.h"
#include "./Vector2.h"

namespace ik {

/**
 * @enum CCDResult
 * @brief Error codes for CCDSolver. Because sometimes things go sideways.
 */
enum CCDResult {
    CCD_SUCCESS = 0,
    CCD_INVALID_INPUT = 1,
    CCD_UNREACHABLE_TARGET = 2,
    CCD_MAX_ITERATIONS_EXCEEDED = 3
};

/**
 * @class CCDSolver
 * @brief Cyclic Coordinate Descent (CCD) solver for 2D inverse kinematics.
 *
 * Provides a static method to solve a 2D kinematic chain using CCD algorithm.
 */
class CCDSolver {
public:
    /**
     * @brief Solve a 2D IK chain using CCD.
     * @param chain The IKChain2D to solve.
     * @param target The target position for the end effector.
     * @param tolerance Acceptable distance to target.
     * @param maxIterations Maximum number of iterations.
     * @return CCDResult error code (see enum above).
     * @note Returns CCD_INVALID_INPUT for invalid chain or solver settings.
     * @note Returns CCD_UNREACHABLE_TARGET when the target is beyond the chain reach.
     */
    static CCDResult Solve(IKChain2D& chain, const Vector2& target, float tolerance = 1e-2f, int maxIterations = 50);
};

}
