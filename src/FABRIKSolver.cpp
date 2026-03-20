#include "../include/iksolver/FABRIKSolver.h"

#include <cmath>
#include <algorithm>

namespace ik {


FABRIKResult FABRIKSolver::Solve(IKChain2D& chain, const Vector2& target, float tolerance, int maxIterations) {
    // Input validation: because even FABRIK needs boundaries. And a sense of humor.
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
    // The base position: where all good chains begin their journey.

    // Total length of the chain
    double totalLength = 0.0;
    for (size_t i = 0; i < n - 1; ++i) totalLength += chain.bones[i].length;
    // If your chain is too short, consider stretching. Or just move the target closer.

    // Target is reachable? If not, it's like trying to reach the cookie jar on the top shelf.
    if ((target - basePos).length() > totalLength) {
        // Stretch chain towards target
        for (size_t i = 0; i < n - 1; ++i) {
            Vector2 dir = (target - chain.bones[i].position).normalized();
            chain.bones[i + 1].position = chain.bones[i].position + dir * chain.bones[i].length;
            // If you can't reach the target, just stretch. It's not cheating, it's FABRIK.
        }
        std::cerr << "FABRIKSolver: Target unreachable. Returning FABRIK_UNREACHABLE_TARGET.\n";
        return FABRIK_UNREACHABLE_TARGET;
    }

    Vector2 endEffector = chain.getEndEffector();
    int iter = 0;

    // The main loop: where the magic happens. Or the bugs. Who knows?
    while ((endEffector - target).length() > tolerance && iter < maxIterations) {
        // --- backward reaching ---
        chain.bones.back().position = target;
        for (int i = (int)n - 2; i >= 0; --i) {
            Vector2 dir = (chain.bones[i].position - chain.bones[i + 1].position).normalized();
            chain.bones[i].position = chain.bones[i + 1].position + dir * chain.bones[i].length;
            // Backward phase: like retracing your steps after losing your keys.
        }

        // --- forward reaching ---
        chain.bones[0].position = basePos;
        for (size_t i = 0; i < n - 1; ++i) {
            Vector2 dir = (chain.bones[i + 1].position - chain.bones[i].position).normalized();
            chain.bones[i + 1].position = chain.bones[i].position + dir * chain.bones[i].length;
            // Forward phase: like heading out for snacks after finding your keys.
        }

        endEffector = chain.getEndEffector();
        ++iter;
        // If you're still not at the target, keep iterating. Or just give up and order pizza.
    }

    if ((endEffector - target).length() <= tolerance) {
        // Success! Like finding a parking spot right in front of the store.
        // If you reached the target, celebrate! Or just move on to the next bug.
        return FABRIK_SUCCESS;
    }
    // Max iterations exceeded. Maybe the chain needs more coffee.
    std::cerr << "FABRIKSolver: Max iterations exceeded. Returning FABRIK_MAX_ITERATIONS_EXCEEDED.\n";
    // If you get here, the algorithm is tired. Give it a break, or more iterations.
    return FABRIK_MAX_ITERATIONS_EXCEEDED;
}

} 
