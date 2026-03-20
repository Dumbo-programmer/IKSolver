#include "../include/iksolver/JacobianSolver.h"
#include "../include/iksolver/MathUtils.h"
#include <cmath>

namespace ik {

enum JacobianResult {
    JACOBIAN_SUCCESS = 0,
    JACOBIAN_INVALID_INPUT = 1,
    JACOBIAN_UNREACHABLE_TARGET = 2,
    JACOBIAN_MAX_ITERATIONS_EXCEEDED = 3
};

bool JacobianSolver::Solve(IKChain2D& chain, const Vector2& target, const SolverSettings& settings) {
    // Input validation: because even Jacobian needs boundaries.
    if (chain.size() < 2) {
        throw std::invalid_argument("JacobianSolver: Chain must have at least 2 joints. Otherwise, it's just a lonely bone.");
    }
    if (settings.maxIterations <= 0) {
        throw std::invalid_argument("JacobianSolver: maxIterations must be positive. Even lazy algorithms iterate at least once.");
    }
    if (settings.tolerance <= 0) {
        throw std::invalid_argument("JacobianSolver: tolerance must be positive. Perfectionism is overrated.");
    }

    // Check if target is reachable
    double totalLength = 0.0;
    for (size_t i = 0; i < chain.size() - 1; ++i) {
        totalLength += chain.bones[i].length;
    }
    if ((target - chain.bones[0].position).length() > totalLength) {
        std::cerr << "JacobianSolver: Target unreachable. Returning JACOBIAN_UNREACHABLE_TARGET.\n";
        return JACOBIAN_UNREACHABLE_TARGET;
    }

    for (int iteration = 0; iteration < settings.maxIterations; ++iteration) {
        Vector2 endEffector = chain.getEndEffector();
        Vector2 error = target - endEffector;

        //if we've reached the target
        if (error.length() < settings.tolerance) {
            // Success! Like finding a bug and squashing it.
            return JACOBIAN_SUCCESS;
        }

        // compute Jacobian matrix
        auto jacobian = computeJacobian(chain);
        if (jacobian.empty()) continue;

        // error vector (2D: x, y)
        std::vector<double> errorVec = {error.x, error.y};

        // compute pseudo-inverse and solve for joint angle changes
        std::vector<std::vector<double>> pseudoInv;
        if (settings.useDampedLeastSquares) {
            pseudoInv = dampedPseudoInverse(jacobian, settings.dampingFactor);
        } else {
            // pseudo-inverse for this example
            pseudoInv = dampedPseudoInverse(jacobian, 0.01);
        }

        // compute joint angle changes
        std::vector<double> deltaAngles = matrixVectorMultiply(pseudoInv, errorVec);

        // apply changes with step size
        for (size_t i = 0; i < deltaAngles.size() && i < chain.getDegreesOfFreedom(); ++i) {
            double currentAngle = chain.getJointAngle(static_cast<int>(i));
            double newAngle = currentAngle + deltaAngles[i] * settings.stepSize;
            chain.setJointAngle(static_cast<int>(i), newAngle);
        }
    }

    // check final error
    Vector2 finalError = target - chain.getEndEffector();
    if (finalError.length() < settings.tolerance) {
        return JACOBIAN_SUCCESS;
    }
    std::cerr << "JacobianSolver: Max iterations exceeded. Returning JACOBIAN_MAX_ITERATIONS_EXCEEDED.\n";
    return JACOBIAN_MAX_ITERATIONS_EXCEEDED;
}
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
    
    // simplified damped pseudo-inverse computation
    // for a more robust implementation, use proper matrix libraries
    std::vector<std::vector<double>> result(cols, std::vector<double>(rows, 0.0));
    
    // J^T * (J * J^T + λ²I)^(-1)
    // simplified implementation for 2xN Jacobian
    
    if (rows != 2) return result; // only handle 2D case for now
    
    for (size_t i = 0; i < cols; ++i) {
        for (size_t j = 0; j < rows; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < cols; ++k) {
                sum += jacobian[j][k] * jacobian[j][k];
            }
            
            if (sum + damping * damping > EPSILON) {
                result[i][j] = jacobian[j][i] / (sum + damping * damping);
            }
        }
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

} 
