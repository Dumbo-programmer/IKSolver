#pragma once
#include "./IKChain2D.h"
#include "./Vector2.h"
#include <vector>

namespace ik {

/**
 * @class JacobianSolver
 * @brief Jacobian-based solver for 2D inverse kinematics.
 *
 * Provides static methods to solve a 2D kinematic chain using Jacobian inverse kinematics.
 */
class JacobianSolver {
public:
    /**
     * @struct SolverSettings
     * @brief Settings for Jacobian solver.
     */
    struct SolverSettings {
        float tolerance = 1e-3f; ///< Acceptable distance to target
        int maxIterations = 100; ///< Maximum number of iterations
        float dampingFactor = 0.1f; ///< Damping factor for least squares
        float stepSize = 0.1f; ///< Step size for angle updates
        bool useDampedLeastSquares = true; ///< Use damped least squares
        SolverSettings() = default;
    };

    /**
     * @brief Solve a 2D IK chain using Jacobian inverse kinematics.
     * @param chain The IKChain2D to solve.
     * @param target The target position for the end effector.
     * @param settings Solver settings.
     * @return JacobianResult error code (see enum below).
     * @throws std::invalid_argument if input is invalid. (Don't feed the solver junk!)
     * @note If you get JACOBIAN_UNREACHABLE_TARGET, try moving the target closer. Or buy longer bones.
     */
    static bool Solve(IKChain2D& chain, const Vector2& target, const SolverSettings& settings);

    /**
     * @enum JacobianResult
     * @brief Error codes for JacobianSolver. Because sometimes things go sideways.
     */
    enum JacobianResult {
        JACOBIAN_SUCCESS = 0,
        JACOBIAN_INVALID_INPUT = 1,
        JACOBIAN_UNREACHABLE_TARGET = 2,
        JACOBIAN_MAX_ITERATIONS_EXCEEDED = 3
    };

private:
    /**
     * @brief Compute the Jacobian matrix for the chain.
     */
    static std::vector<std::vector<double>> computeJacobian(const IKChain2D& chain);

    /**
     * @brief Compute pseudo-inverse using damped least squares.
     */
    static std::vector<std::vector<double>> dampedPseudoInverse(
        const std::vector<std::vector<double>>& jacobian, 
        double damping);

    /**
     * @brief Matrix-vector multiplication utility.
     */
    static std::vector<double> matrixVectorMultiply(
        const std::vector<std::vector<double>>& matrix,
        const std::vector<double>& vector);
};

}