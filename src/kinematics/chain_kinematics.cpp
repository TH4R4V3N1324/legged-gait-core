#include "gait_core/kinematics/chain_kinematics.hpp"

#include <algorithm>
#include <array>

namespace gait_core {

ChainKinematics::ChainKinematics() {
    leg_.mount = {Scalar(0), Vec3::Zero()};
    leg_.numJoints = 0;
    leg_.foot = Vec3::Zero();
}

ChainKinematics::ChainKinematics(const LegDefinition& leg) : leg_(leg) {
    leg_.numJoints = std::clamp(leg_.numJoints, 0, kMaxJointsPerLeg);
    for (int j = 0; j < leg_.numJoints; ++j) {
        leg_.joints[j].axis.normalize();
    }
}

/**
 * @brief Foot position from the joint angles
 * @param[in] q One angle per joint, body to foot (rad)
 * @return The foot position in the leg frame (m)
 * @note Walks the chain: step to each joint's origin and turn about its axis, both given at zero
 *       angles, so each is first turned by every joint before it. Then step out to the foot.
*/
Vec3 ChainKinematics::fk(const JointVec& q) const {
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    for (int j = 0; j < leg_.numJoints; ++j) {
        const JointDefinition& joint = leg_.joints[j];
        position += rotation * joint.origin;
        rotation = rotation * AngleAxis(q(j), joint.axis).toRotationMatrix();
    }
    return position + rotation * leg_.foot;
}

/**
 * @brief Joint angles that put the foot at target, by damped least squares from the seed
 * @param[in] target The foot position in the leg frame (m)
 * @param[in] seed Where to start, normally the current angles; zeros if it's the wrong size
 * @param[out] q The angles, always within the joint limits
 * @return true if the foot is within IK_TOLERANCE of target. false if not: out of reach, in which
 *         case q is the closest pose found, never further from target than the seed.
 * @note Solves from the seed, so normally finds the solution nearest it. If that doesn't converge
 *       it solves again from the middle of every joint's range and keeps the better: a seed on a
 *       singularity, such as a straight leg at power up, can't move the foot along the leg at all.
*/
bool ChainKinematics::ik(const Vec3& target, const JointVec& seed, JointVec& q) const {
    if (seed.size() == numJoints()) {
        q = seed;
    } else {
        q = JointVec::Zero(numJoints());
    }
    Scalar error = descend(target, q);
    if (error < IK_TOLERANCE) {
        if (numJoints() > 3) {
            settle(target, q);
        }
        return true;
    }

    JointVec middle(numJoints());
    for (int j = 0; j < leg_.numJoints; ++j) {
        middle(j) = (leg_.joints[j].lower + leg_.joints[j].upper) / 2;
    }
    const Scalar middleError = descend(target, middle);
    if (middleError < error) {
        q = middle;
        error = middleError;
    }
    return error < IK_TOLERANCE;
}

/**
 * @brief Ease the spare joints of a leg with more than three towards the middle of their ranges,
 *        without moving the foot
 * @param[in] target The foot position q reaches (leg frame, m)
 * @param[in,out] q A solution for target; moved a little along the poses that keep the foot there
 * @note With more joints than the foot's three directions, every foot position has a family of
 *       poses, and solving from the last one keeps whichever the leg's path happened to leave it in:
 *       after a walk the same stance came back with joints up to 0.19 rad apart, and they could drift
 *       towards their limits. Each solve moves POSTURE_STEP of the way towards mid-range, projected
 *       so the foot (to first order) doesn't move, then corrects the foot again. Kept only if the
 *       foot is still within IK_TOLERANCE.
*/
void ChainKinematics::settle(const Vec3& target, JointVec& q) const {
    constexpr Scalar POSTURE_STEP = Scalar(0.02);   // share of the way to mid-range per solve
    const LegJacobian J = jacobian(q);
    const Mat3 JJt = J * J.transpose();
    if (std::abs(JJt.determinant()) < Scalar(1e-12)) {
        return;   // near a singularity the foot-preserving directions aren't well defined
    }
    JointVec pull(numJoints());
    for (int j = 0; j < leg_.numJoints; ++j) {
        pull(j) = POSTURE_STEP * ((leg_.joints[j].lower + leg_.joints[j].upper) / 2 - q(j));
    }
    // Remove the part of the pull that would move the foot: (I - J^T (J J^T)^-1 J) pull
    JointVec next = q + pull - J.transpose() * JJt.ldlt().solve(J * pull);
    for (int j = 0; j < leg_.numJoints; ++j) {
        next(j) = std::clamp(next(j), leg_.joints[j].lower, leg_.joints[j].upper);
    }
    if (descend(target, next) < IK_TOLERANCE) {
        q = next;
    }
}

/**
 * @brief Damped least squares from q towards target, for at most MAX_ITERATIONS steps
 * @param[in] target The foot position in the leg frame (m)
 * @param[in,out] q Where to start; on return the closest pose found, within the joint limits
 * @return How far that pose's foot is from target (m)
 * @note Each step solves J dq = error as (J^T (J J^T + damping^2 I)^-1) error, which stays well
 *       behaved near a straight leg where J loses rank. J J^T is 3x3 for any number of joints.
 *       The damping adapts (Levenberg-Marquardt): smaller after a step that helped, so a nearly
 *       straight leg still converges in a few steps, and larger after one that didn't, which is
 *       undone. Each step also aims at most MAX_STEP closer: a target far out of reach would
 *       otherwise ask for joint moves of several radians, overshoot, and end up anywhere.
*/
Scalar ChainKinematics::descend(const Vec3& target, JointVec& q) const {
    constexpr int MAX_ITERATIONS = 20;          // bounded, so the time per tick is too
    constexpr Scalar MIN_DAMPING = Scalar(1e-6);
    constexpr Scalar MAX_DAMPING = Scalar(1.0);
    constexpr Scalar MAX_STEP = Scalar(0.05);   // m, how far one step aims to move the foot

    Scalar damping = Scalar(0.01);
    Scalar error = (target - fk(q)).norm();
    for (int i = 0; i < MAX_ITERATIONS && error >= IK_TOLERANCE; ++i) {
        Vec3 step = target - fk(q);
        const Scalar distance = step.norm();
        if (distance > MAX_STEP) {
            step *= MAX_STEP / distance;
        }
        const LegJacobian J = jacobian(q);
        const Mat3 A = J * J.transpose() + damping * damping * Mat3::Identity();
        JointVec next = q + J.transpose() * A.ldlt().solve(step);
        for (int j = 0; j < leg_.numJoints; ++j) {
            next(j) = std::clamp(next(j), leg_.joints[j].lower, leg_.joints[j].upper);
        }
        const Scalar nextError = (target - fk(next)).norm();
        if (nextError < error) {
            q = next;   // keep it, and be bolder
            error = nextError;
            damping = std::max(MIN_DAMPING, damping / 10);
        } else {
            damping = std::min(MAX_DAMPING, damping * 10);   // undo it, and be more careful
        }
    }
    return error;
}

/**
 * @brief How the foot moves with each joint
 * @param[in] q One angle per joint, body to foot (rad)
 * @return 3 x numJoints: column j is the foot's velocity (m/s) for joint j turning at 1 rad/s,
 *         which for a revolute joint is its axis crossed with the line from the joint to the foot
*/
LegJacobian ChainKinematics::jacobian(const JointVec& q) const {
    return fullJacobian(q).topRows<3>();
}

Mat3 ChainKinematics::footRotation(const JointVec& q) const {
    Mat3 rotation = Mat3::Identity();
    for (int j = 0; j < leg_.numJoints; ++j) {
        rotation = rotation * AngleAxis(q(j), leg_.joints[j].axis).toRotationMatrix();
    }
    return rotation;
}

LegJacobian6 ChainKinematics::fullJacobian(const JointVec& q) const {
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    std::array<Vec3, kMaxJointsPerLeg> jointPos;
    std::array<Vec3, kMaxJointsPerLeg> jointAxis;
    for (int j = 0; j < leg_.numJoints; ++j) {
        const JointDefinition& joint = leg_.joints[j];
        position += rotation * joint.origin;
        jointPos[j] = position;                 // where joint j is
        jointAxis[j] = rotation * joint.axis;   // which way it points (turning about it doesn't move it)
        rotation = rotation * AngleAxis(q(j), joint.axis).toRotationMatrix();
    }
    const Vec3 foot = position + rotation * leg_.foot;

    LegJacobian6 J(6, leg_.numJoints);
    for (int j = 0; j < leg_.numJoints; ++j) {
        J.col(j).head<3>() = jointAxis[j].cross(foot - jointPos[j]);
        J.col(j).tail<3>() = jointAxis[j];   // a revolute joint turns everything after it about its axis
    }
    return J;
}

}  // namespace gait_core
