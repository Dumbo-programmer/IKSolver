#pragma once
#include "./IKChain2D.h"
#include "./Vector2.h"

namespace ik {

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
     * @return CCDResult error code (see enum below).
     * @throws std::invalid_argument if input is invalid. (Don't feed the solver junk!)
     * @note If you get CCD_UNREACHABLE_TARGET, try moving the target closer. Or buy longer bones.
     */
    static bool Solve(IKChain2D& chain, const Vector2& target, float tolerance = 1e-2f, int maxIterations = 50);

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
};

}
