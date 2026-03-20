#include "../include/iksolver/FABRIKSolver.h"

#include <cmath>
#include <algorithm>

namespace ik {


enum FABRIKResult {
    FABRIK_SUCCESS = 0,
    FABRIK_INVALID_INPUT = 1,
    FABRIK_UNREACHABLE_TARGET = 2,
    FABRIK_MAX_ITERATIONS_EXCEEDED = 3
};

bool FABRIKSolver::Solve(IKChain2D& chain, const Vector2& target, float tolerance, int maxIterations) {
    // Input validation: because even FABRIK needs boundaries.
    size_t n = chain.size();
    if (n < 2) {
        throw std::invalid_argument("FABRIKSolver: Chain must have at least 2 joints. Otherwise, it's just a lonely bone.");
    }
    if (maxIterations <= 0) {
        throw std::invalid_argument("FABRIKSolver: maxIterations must be positive. Even lazy algorithms iterate at least once.");
    }
    if (tolerance <= 0) {
        throw std::invalid_argument("FABRIKSolver: tolerance must be positive. Perfectionism is overrated.");
    }

    Vector2 basePos = chain.bones[0].position;

    // Total length of the chain
    double totalLength = 0.0;
    for (size_t i = 0; i < n - 1; ++i) totalLength += chain.bones[i].length;

    // Target is reachable?
    if ((target - basePos).length() > totalLength) {
        // Stretch chain towards target
        for (size_t i = 0; i < n - 1; ++i) {
            Vector2 dir = (target - chain.bones[i].position).normalized();
            chain.bones[i + 1].position = chain.bones[i].position + dir * chain.bones[i].length;
        }
        std::cerr << "FABRIKSolver: Target unreachable. Returning FABRIK_UNREACHABLE_TARGET.\n";
        return FABRIK_UNREACHABLE_TARGET;
    }

    Vector2 endEffector = chain.getEndEffector();
    int iter = 0;

    while ((endEffector - target).length() > tolerance && iter < maxIterations) {
        // --- backward reaching ---
        chain.bones.back().position = target;
        for (int i = (int)n - 2; i >= 0; --i) {
            Vector2 dir = (chain.bones[i].position - chain.bones[i + 1].position).normalized();
            chain.bones[i].position = chain.bones[i + 1].position + dir * chain.bones[i].length;
        }

        // --- forward reaching ---
        chain.bones[0].position = basePos;
        for (size_t i = 0; i < n - 1; ++i) {
            Vector2 dir = (chain.bones[i + 1].position - chain.bones[i].position).normalized();
            chain.bones[i + 1].position = chain.bones[i].position + dir * chain.bones[i].length;
        }

        endEffector = chain.getEndEffector();
        ++iter;
    }

    if ((endEffector - target).length() <= tolerance) {
        // Success! Like finding a parking spot right in front of the store.
        return FABRIK_SUCCESS;
    }
    // Max iterations exceeded. Maybe the chain needs more coffee.
    std::cerr << "FABRIKSolver: Max iterations exceeded. Returning FABRIK_MAX_ITERATIONS_EXCEEDED.\n";
    return FABRIK_MAX_ITERATIONS_EXCEEDED;
}

} 
