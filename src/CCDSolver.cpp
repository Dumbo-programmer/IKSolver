#include "../include/iksolver/CCDSolver.h"
#include "../include/iksolver/IKChain2D.h"
#include "../include/iksolver/Vector2.h"
#include <cmath>
#include <stdio.h>
#include <algorithm>
namespace ik {

CCDResult CCDSolver::Solve(ik::IKChain2D& chain, const ik::Vector2& target, float tolerance, int maxIterations) {
    // Let's check for invalid input, because bugs are like ninjas: silent but deadly.
    if (chain.bones.empty()) {
        throw std::invalid_argument("CCDSolver: Chain is empty. Did you forget to add bones? Bones are important!");
    }
    if (maxIterations <= 0) {
        throw std::invalid_argument("CCDSolver: maxIterations must be positive. Even procrastinators iterate at least once.");
    }
    if (tolerance <= 0) {
        throw std::invalid_argument("CCDSolver: tolerance must be positive. We all need boundaries, even algorithms.");
    }

    // Check if target is reachable
    double totalLength = 0.0;
    for (size_t i = 0; i < chain.bones.size() - 1; ++i) {
        totalLength += chain.bones[i].length;
    }
    if ((target - chain.bones[0].position).length() > totalLength) {
        // Sorry, target is out of reach. Like my dreams of becoming a professional pizza taster.
        std::cerr << "CCDSolver: Target unreachable. Returning CCD_UNREACHABLE_TARGET.\n";
        return CCD_UNREACHABLE_TARGET;
    }

    // Performance tip: If you want to go fast, remove all the jokes. But where's the fun in that?
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Looping backwards through the chain, because forward is so last season.
        for (int i = (int)chain.bones.size() - 2; i >= 0; --i) {
            Vector2 endEffector = chain.getEndEffector();
            Vector2 joint = chain.bones[i].position;
            // If you ever get lost, just follow the end effector. It's always chasing the target.

            Vector2 toEnd = (endEffector - joint).normalized();
            Vector2 toTarget = (target - joint).normalized();
            // Normalizing vectors: because we all need to be well-adjusted.

            float cosTheta = toEnd.dot(toTarget);
            cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
            float angle = std::acos(cosTheta);
            // If you don't understand the math, just pretend it's magic. That's what I do.

            // Rotate each following bone
            float cross = toEnd.cross(toTarget);
            if (cross < 0) angle = -angle;
            // Cross product: the only time crossing paths is a good thing.

            for (size_t j = i + 1; j < chain.bones.size(); ++j) {
                Vector2 rel = chain.bones[j].position - joint;
                rel = rel.rotated(angle);
                chain.bones[j].position = joint + rel;
                // If your bones start spinning, don't worry. It's just the algorithm doing its thing.
            }
        }

        if ((chain.getEndEffector() - target).length() < tolerance) {
            // Success! Like finding fries at the bottom of the bag.
            // If you reached the target, pat yourself on the back. Or let the robot do it.
            return CCD_SUCCESS;
        }
    }
    // Max iterations exceeded. Maybe try more iterations, or bribe the algorithm with coffee.
    std::cerr << "CCDSolver: Max iterations exceeded. Returning CCD_MAX_ITERATIONS_EXCEEDED.\n";
    // If you get here, the algorithm is tired. Give it a break, or more iterations.
    return CCD_MAX_ITERATIONS_EXCEEDED;
// Removed extra closing brace
}
// Add missing namespace closing brace
}
