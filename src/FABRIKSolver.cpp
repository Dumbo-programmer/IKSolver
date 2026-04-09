#include "../include/iksolver/FABRIKSolver.h"

#include <algorithm>
#include <cmath>

namespace ik {


FABRIKResult FABRIKSolver::Solve(IKChain2D& chain, const Vector2& target, float tolerance, int maxIterations) {
    size_t n = chain.size();
    if (n < 2 || maxIterations <= 0 || tolerance <= 0.0f) {
        return FABRIK_INVALID_INPUT;
    }

    Vector2 basePos = chain.bones[0].position;

    double totalLength = 0.0;
    for (size_t i = 0; i < n - 1; ++i) {
        totalLength += chain.bones[i].length;
    }

    if ((target - basePos).length() > totalLength) {
        // Stretch chain toward target while preserving segment lengths.
        for (size_t i = 0; i < n - 1; ++i) {
            Vector2 dir = (target - chain.bones[i].position).normalized();
            chain.bones[i + 1].position = chain.bones[i].position + dir * chain.bones[i].length;
        }
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
        return FABRIK_SUCCESS;
    }

    return FABRIK_MAX_ITERATIONS_EXCEEDED;
}

} 
