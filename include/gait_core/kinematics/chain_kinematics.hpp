#ifndef GAIT_CORE__CHAIN_KINEMATICS_HPP
#define GAIT_CORE__CHAIN_KINEMATICS_HPP

#include "gait_core/robot/leg_definition.hpp"
#include "gait_core/kinematics/leg_kinematics.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Kinematics for any chain of revolute joints, from its LegDefinition
 * @note FK and the Jacobian are exact. IK is numerical (damped least squares from the seed),
 *       so it finds the solution nearest the seed and takes a bounded number of iterations.
 *       Use an analytic LegKinematics where there is one (YawPitchPitchLeg for explorer).
*/
class ChainKinematics : public LegKinematics {
public:
    /**
     * @brief Take a copy of the leg's chain
     * @param[in] leg The leg; its axes are normalised here, and numJoints kept to 0..kMaxJointsPerLeg
    */
    explicit ChainKinematics(const LegDefinition& leg);

    /**
     * @brief No joints and the foot at the leg frame origin, so it can sit in an array until set
    */
    ChainKinematics();

    int numJoints() const override { return leg_.numJoints; }
    Vec3 fk(const JointVec& q) const override;
    bool ik(const Vec3& target, const JointVec& seed, JointVec& q) const override;
    LegJacobian jacobian(const JointVec& q) const override;
    /**
     * @brief How the foot moves and turns with each joint
     * @return 6 x numJoints: rows 0-2 the foot point's velocity (as jacobian()), rows 3-5 the foot's
     *         angular velocity, for joint j turning at 1 rad/s (leg frame). tau = -J^T [f; m] holds
     *         the ground pushing on the foot with force f at the foot point and moment m about it.
    */
    LegJacobian6 fullJacobian(const JointVec& q) const;
    /**
     * @brief How the foot is turned: the last joint's frame (leg frame axes as columns), the frame
     *        LegDefinition::foot is given in
    */
    Mat3 footRotation(const JointVec& q) const;

    // m, how close ik must put the foot. Near a straight leg the foot barely moves with the knee
    // (~6 mm/rad on explorer), so this is ~2e-4 rad there; float resolves the foot to ~3e-8 m.
    static constexpr Scalar IK_TOLERANCE = Scalar(1e-6);

private:
    LegDefinition leg_;   // a copy, so this never depends on the RobotDefinition outliving it

    Scalar descend(const Vec3& target, JointVec& q) const;
    void settle(const Vec3& target, JointVec& q) const;
};

}  // namespace gait_core

#endif  // GAIT_CORE__CHAIN_KINEMATICS_HPP
