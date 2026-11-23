#ifndef GAIT_CORE__LEG_KINEMATICS_HPP
#define GAIT_CORE__LEG_KINEMATICS_HPP

#include "gait_core/types.hpp"
#include <cmath>

namespace gait_core {

/**
 * @brief Abstract interface for leg kinematics
 * @note This is a pure virtual class; concrete implementations must provide the methods below
*/
class LegKinematics {
public:
    virtual ~LegKinematics() = default;

    /**
     * @brief Get the number of joints in this leg
     * @return The number of joints
    */
    virtual int numJoints() const = 0;

    /**
     * @brief Forward kinematics: compute the foot position from the joint angles
     * @param[in] jointAngles The angles of each joint, in order from the body to the foot
     * @return The position of the foot in the leg's frame
    */
    virtual Vec3 fk(const JointVec& jointAngles) const = 0;

    /**
     * @brief Inverse kinematics: compute the joint angles from the foot position
     * @param[in] target The desired position of the foot in the leg's frame
     * @param[in] seed A guess for the joint angles, used to pick a solution if there are multiple
     * @param[out] jointAngles The computed joint angles, in order from the body to the foot
     * @return True if a solution was found, false if the target is unreachable
    */
    virtual bool ik(const Vec3& target, const JointVec& seed, JointVec& jointAngles) const = 0;

    /**
     * @brief Compute the Jacobian matrix for the leg at a given joint configuration
     * @param[in] jointAngles The angles of each joint, in order from the body to the foot
     * @return The Jacobian matrix, which maps joint velocities to foot velocities
    */
    virtual LegJacobian jacobian(const JointVec& jointAngles) const = 0;

    /**
     * @brief Force the ground pushes on the foot with, from the joint torques holding the leg still
     * @param[in] jointAngles The angles of each joint, in order from the body to the foot
     * @param[in] torques The torque each joint's servo applies, in the same order (N m)
     * @param[out] force The force on the foot (leg frame, N); +z means the ground is pushing up
     * @return false if the leg is too close to singular for the force to be found
     * @note In balance torques = -J^T force; solved by least squares, which is exact for 3 joints
    */
    bool groundForce(const JointVec& jointAngles, const JointVec& torques, Vec3& force) const {
        const LegJacobian J = jacobian(jointAngles);
        if (torques.size() != J.cols()) {
            return false;
        }
        // Fewer than three joints can't resolve a 3D force. J J^T is then singular, but rounding can
        // leave its determinant above the threshold below in float, and a force from that is nonsense
        if (J.cols() < 3) {
            return false;
        }
        const Mat3 JJt = J * J.transpose();
        if (std::abs(JJt.determinant()) < Scalar(1e-12)) {
            return false;
        }
        force = -JJt.inverse() * (J * torques);
        return true;
    }
};
}

#endif  // GAIT_CORE__LEG_KINEMATICS_HPP