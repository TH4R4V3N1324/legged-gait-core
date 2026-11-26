#ifndef GAIT_CORE__LEG_DEFINITION_HPP
#define GAIT_CORE__LEG_DEFINITION_HPP

#include <array>
#include <cmath>

#include "gait_core/types.hpp"
#include "gait_core/robot/joint_definition.hpp"

namespace gait_core {

/**
 * @brief Where a leg's coxa joint sits on the body
*/
struct LegConfig {
    Scalar yaw;                // rad, rotation of the leg frame about the body z axis
    Vec3 translation_offset;   // m, position of the coxa joint in the body frame
};

enum class LegSolver : uint8_t {
    Chain,          // a generic chain of revolute joints (ChainKinematics): works for any leg
    YawPitchPitch,  // the analytic solver (YawPitchPitchLeg) for a yaw joint then two pitch joints, e.g. coxa/femur/tibia
};

struct LegDefinition {
    LegConfig mount;                                        // as today: yaw + coxa position
    int numJoints = 0;
    std::array<JointDefinition, kMaxJointsPerLeg> joints;   // body to foot
    Vec3 foot;                                              // m, last joint to foot
    LegSolver solver = LegSolver::Chain;                    // which kinematics to use for this leg
    // m, leg frame: where this leg's foot stands relative to the robot's shared stance
    // (standReach, 0, standHeight). Kept through height and reach changes, sitting included.
    // Zero for legs that all stand alike, e.g. a radially symmetric hexapod.
    Vec3 stanceOffset = Vec3::Zero();
    // m, the sole: a rectangle on the ground around the foot point, its opposite corners in the
    // foot frame (x forward along the leg frame's x at zero joint angles, y to its left), which
    // turns with the joints about the vertical (for a leg with a hip yaw joint, with that joint).
    // Equal (the default, zero size) means a point foot.
    Vec2 soleMin = Vec2::Zero();
    Vec2 soleMax = Vec2::Zero();

    bool hasSole() const { return soleMin != soleMax; }

    /**
     * @brief Which way the sole points, seen from above
     * @param[in] q The joint angles, body to foot (numJoints of them)
     * @return rad, the foot frame's x axis about the leg frame's z axis (add mount.yaw for the body
     *         frame); 0 if q is the wrong size or the foot's x axis points straight up or down
    */
    Scalar soleYaw(const JointVec& q) const {
        if (q.size() != numJoints) {
            return 0;
        }
        Mat3 rotation = Mat3::Identity();
        for (int j = 0; j < numJoints; ++j) {
            rotation = rotation * AngleAxis(q(j), joints[j].axis).toRotationMatrix();
        }
        const Vec3 forward = rotation.col(0);
        return forward.head<2>().squaredNorm() > Scalar(1e-12) ? std::atan2(forward.y(), forward.x()) : Scalar(0);
    }
};

} // namespace gait_core

#endif  // GAIT_CORE__LEG_DEFINITION_HPP