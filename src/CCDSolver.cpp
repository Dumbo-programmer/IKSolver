#include "../include/iksolver/CCDSolver.h"
#include "../include/iksolver/IKChain2D.h"
#include "../include/iksolver/Vector2.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ik {

CCDResult CCDSolver::Solve(ik::IKChain2D& chain, const ik::Vector2& target, float tolerance, int maxIterations) {
    if (chain.size() < 2 || maxIterations <= 0 || tolerance <= 0.0f) {
        return CCD_INVALID_INPUT;
    }

    if ((target - chain.bones[0].position).length() > chain.getTotalLength() + EPSILON) {
        // Keep chain fully extended toward the target direction for deterministic behavior.
        Vector2 direction = (target - chain.bones[0].position).normalized();
        for (size_t i = 0; i + 1 < chain.bones.size(); ++i) {
            chain.bones[i + 1].position = chain.bones[i].position + direction * chain.bones[i].length;
            chain.bones[i].angle = direction.angle();
        }
        return CCD_UNREACHABLE_TARGET;
    }

    for (int iter = 0; iter < maxIterations; ++iter) {
        for (int i = (int)chain.bones.size() - 2; i >= 0; --i) {
            Vector2 endEffector = chain.getEndEffector();
            Vector2 joint = chain.bones[i].position;

            if ((endEffector - joint).lengthSquared() <= EPSILON || (target - joint).lengthSquared() <= EPSILON) {
                continue;
            }

            Vector2 toEnd = (endEffector - joint).normalized();
            Vector2 toTarget = (target - joint).normalized();

            double cosTheta = std::clamp(toEnd.dot(toTarget), -1.0, 1.0);
            double angle = std::acos(cosTheta);
            if (std::abs(angle) <= EPSILON) {
                continue;
            }

            double cross = toEnd.cross(toTarget);
            if (cross < 0) angle = -angle;

            for (size_t j = i + 1; j < chain.bones.size(); ++j) {
                Vector2 rel = chain.bones[j].position - joint;
                rel = rel.rotated(angle);
                chain.bones[j].position = joint + rel;
            }

            if (i < static_cast<int>(chain.bones.size()) - 1) {
                Vector2 nextDir = (chain.bones[i + 1].position - chain.bones[i].position).normalized();
                if (nextDir.lengthSquared() > EPSILON) {
                    chain.bones[i].angle = nextDir.angle();
                }
            }
        }

        if ((chain.getEndEffector() - target).length() <= tolerance) {
            return CCD_SUCCESS;
        }
    }

    return CCD_MAX_ITERATIONS_EXCEEDED;
}

}
