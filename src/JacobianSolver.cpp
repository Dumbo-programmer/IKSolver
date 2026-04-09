#include "../include/iksolver/JacobianSolver.h"
#include "../include/iksolver/MathUtils.h"
#include <cmath>
#include <iostream>

namespace ik {

JacobianResult JacobianSolver::Solve(IKChain2D& chain, const Vector2& target, const JacobianSolver::SolverSettings& settings) {
    if (chain.size() < 2 || settings.maxIterations <= 0 || settings.tolerance <= 0.0f) {
        return JACOBIAN_INVALID_INPUT;
    }

    double totalLength = 0.0;
    for (size_t i = 0; i < chain.size() - 1; ++i) {
        totalLength += chain.bones[i].length;
    }

    if ((target - chain.bones[0].position).length() > totalLength + EPSILON) {
        return JACOBIAN_UNREACHABLE_TARGET;
    }

    for (int iteration = 0; iteration < settings.maxIterations; ++iteration) {
        Vector2 endEffector = chain.getEndEffector();
        Vector2 error = target - endEffector;

        if (error.length() < settings.tolerance) {
            return JACOBIAN_SUCCESS;
        }

        auto jacobian = computeJacobian(chain);
        if (jacobian.empty()) {
            return JACOBIAN_MAX_ITERATIONS_EXCEEDED;
        }

        std::vector<double> errorVec = {error.x, error.y};

        std::vector<std::vector<double>> pseudoInv;
        if (settings.useDampedLeastSquares) {
            pseudoInv = dampedPseudoInverse(jacobian, settings.dampingFactor);
        } else {
            pseudoInv = dampedPseudoInverse(jacobian, 0.0);
        }

        std::vector<double> deltaAngles = matrixVectorMultiply(pseudoInv, errorVec);
        if (deltaAngles.empty()) {
            return JACOBIAN_MAX_ITERATIONS_EXCEEDED;
        }

        for (size_t i = 0; i < deltaAngles.size() && i < chain.getDegreesOfFreedom(); ++i) {
            double currentAngle = chain.getJointAngle(static_cast<int>(i));
            double newAngle = currentAngle + deltaAngles[i] * settings.stepSize;
            chain.setJointAngle(static_cast<int>(i), newAngle);
        }
    }

    Vector2 finalError = target - chain.getEndEffector();
    if (finalError.length() < settings.tolerance) {
        return JACOBIAN_SUCCESS;
    }

    return JACOBIAN_MAX_ITERATIONS_EXCEEDED;
}

std::vector<std::vector<double>> JacobianSolver::computeJacobian(const IKChain2D& chain) {
    size_t dof = chain.getDegreesOfFreedom();
    if (dof == 0) return {};
    
    std::vector<std::vector<double>> jacobian(2, std::vector<double>(dof, 0.0));
    
    Vector2 endEffector = chain.getEndEffector();
    
    for (size_t i = 0; i < dof; ++i) {
        Vector2 jointPos = chain.getJointPosition(static_cast<int>(i));
        Vector2 toEnd = endEffector - jointPos;
        
        // for 2D rotational joints, the jacobian column is the cross product
        // of the joint axis (0,0,1) with the vector from joint to end effector
        jacobian[0][i] = -toEnd.y; // -sin component
        jacobian[1][i] = toEnd.x;  // cos component
    }
    
    return jacobian;
}

std::vector<std::vector<double>> JacobianSolver::dampedPseudoInverse(
    const std::vector<std::vector<double>>& jacobian, 
    double damping) {
    size_t rows = jacobian.size();
    size_t cols = jacobian[0].size();

    std::vector<std::vector<double>> result(cols, std::vector<double>(rows, 0.0));

    // J^T * (J * J^T + λ²I)^(-1) for a 2xN Jacobian.
    if (rows != 2 || cols == 0) {
        return result;
    }

    double j00 = 0.0;
    double j01 = 0.0;
    double j11 = 0.0;

    for (size_t k = 0; k < cols; ++k) {
        j00 += jacobian[0][k] * jacobian[0][k];
        j01 += jacobian[0][k] * jacobian[1][k];
        j11 += jacobian[1][k] * jacobian[1][k];
    }

    const double lambda2 = damping * damping;
    const double a = j00 + lambda2;
    const double b = j01;
    const double c = j01;
    const double d = j11 + lambda2;

    const double det = a * d - b * c;
    if (std::abs(det) <= EPSILON) {
        return result;
    }

    const double inv00 = d / det;
    const double inv01 = -b / det;
    const double inv10 = -c / det;
    const double inv11 = a / det;

    for (size_t i = 0; i < cols; ++i) {
        const double jt0 = jacobian[0][i];
        const double jt1 = jacobian[1][i];
        result[i][0] = jt0 * inv00 + jt1 * inv10;
        result[i][1] = jt0 * inv01 + jt1 * inv11;
    }

    return result;
}

std::vector<double> JacobianSolver::matrixVectorMultiply(
    const std::vector<std::vector<double>>& matrix,
    const std::vector<double>& vector) {
    if (matrix.empty() || matrix[0].size() != vector.size()) {
        return {};
    }
    std::vector<double> result(matrix.size(), 0.0);
    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = 0; j < vector.size(); ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
    return result;
}

} // end namespace ik
