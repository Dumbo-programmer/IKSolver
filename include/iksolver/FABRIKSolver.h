#pragma once
#include "./IKChain2D.h"

namespace ik {

/**
 * @enum FABRIKResult
 * @brief Error codes for FABRIKSolver. Because sometimes things go sideways.
 */
enum FABRIKResult {
    FABRIK_SUCCESS = 0,
    FABRIK_INVALID_INPUT = 1,
    FABRIK_UNREACHABLE_TARGET = 2,
    FABRIK_MAX_ITERATIONS_EXCEEDED = 3
};

/**
 * @class FABRIKSolver
 * @brief Forward And Backward Reaching Inverse Kinematics (FABRIK) solver for 2D chains.
 *
 * Provides a static method to solve a 2D kinematic chain using the FABRIK algorithm.
 */
class FABRIKSolver {
public:
    /**
     * @brief Solve a 2D IK chain using FABRIK.
     * @param chain The IKChain2D to solve.
     * @param target The target position for the end effector.
     * @param tolerance Acceptable distance to target.
     * @param maxIterations Maximum number of iterations.
     * @return FABRIKResult error code (see enum above).
     * @throws std::invalid_argument if input is invalid. (Don't feed the solver junk!)
     * @note If you get FABRIK_UNREACHABLE_TARGET, try moving the target closer. Or buy longer bones.
     */
    static FABRIKResult Solve(IKChain2D& chain, const Vector2& target, float tolerance = 1e-2f, int maxIterations = 50);
};

}
