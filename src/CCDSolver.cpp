#include "../include/iksolver/CCDSolver.h"
#include "../include/iksolver/IKChain2D.h"
#include "../include/iksolver/Vector2.h"
#include <cmath>
#include <stdio.h>
#include <algorithm>
namespace ik {

enum CCDResult {
    CCD_SUCCESS = 0,
    CCD_INVALID_INPUT = 1,
    CCD_UNREACHABLE_TARGET = 2,
    CCD_MAX_ITERATIONS_EXCEEDED = 3
};

bool CCDSolver::Solve(ik::IKChain2D& chain, const ik::Vector2& target, float tolerance, int maxIterations) {
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

    for (int iter = 0; iter < maxIterations; ++iter) {
        for (int i = (int)chain.bones.size() - 2; i >= 0; --i) {
            Vector2 endEffector = chain.getEndEffector();
            Vector2 joint = chain.bones[i].position;

            Vector2 toEnd = (endEffector - joint).normalized();
            Vector2 toTarget = (target - joint).normalized();

            float cosTheta = toEnd.dot(toTarget);
            cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
            float angle = std::acos(cosTheta);

            // Rotate each following bone
            float cross = toEnd.cross(toTarget);
            if (cross < 0) angle = -angle;

            for (size_t j = i + 1; j < chain.bones.size(); ++j) {
                Vector2 rel = chain.bones[j].position - joint;
                rel = rel.rotated(angle);
                chain.bones[j].position = joint + rel;
            }
        }

        if ((chain.getEndEffector() - target).length() < tolerance) {
            // Success! Like finding fries at the bottom of the bag.
            return CCD_SUCCESS;
        }
    }
    // Max iterations exceeded. Maybe try more iterations, or bribe the algorithm with coffee.
    std::cerr << "CCDSolver: Max iterations exceeded. Returning CCD_MAX_ITERATIONS_EXCEEDED.\n";
    return CCD_MAX_ITERATIONS_EXCEEDED;
}
}

}
