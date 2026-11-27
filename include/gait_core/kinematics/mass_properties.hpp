#ifndef GAIT_CORE__MASS_PROPERTIES_HPP
#define GAIT_CORE__MASS_PROPERTIES_HPP

#include "gait_core/robot/appendage_definition.hpp"
#include "gait_core/robot/leg_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Some masses as their total and first moment, which add up across bodies
*/
struct MassMoment {
    Scalar mass = 0;                  // kg
    Vec3 moment = Vec3::Zero();       // kg m, the sum of each mass times its position

    void add(Scalar m, const Vec3& position) {
        mass += m;
        moment += m * position;
    }
    MassMoment& operator+=(const MassMoment& other) {
        mass += other.mass;
        moment += other.moment;
        return *this;
    }
    /**
     * @brief Their centre of mass, in the frame the positions were given in
     * @param[in] fallback What to return with no mass
    */
    Vec3 centre(const Vec3& fallback = Vec3::Zero()) const {
        return mass > 0 ? Vec3(moment / mass) : fallback;
    }
};

/**
 * @brief A leg's links' masses where its joints put them
 * @param[in] leg The leg: each joint's mass and centreOfMass
 * @param[in] q One angle per joint, body to foot (rad); a leg of another size gives nothing
 * @return Their mass and moment, in the leg frame
 * @note Walks the chain as ChainKinematics::fk does: each link's centre of mass is in its joint's
 *       frame, which sits at the joint and turns with it.
*/
MassMoment legMass(const LegDefinition& leg, const JointVec& q);

/**
 * @brief An appendage's links' masses where its joints put them, and a load at its tip
 * @param[in] appendage The appendage: each joint's mass and centreOfMass, and its tip
 * @param[in] q One angle per joint, body to tip (rad); an appendage of another size gives nothing
 * @param[in] payload kg held at the tip
 * @return Their mass and moment, in the appendage's frame (as a leg frame: see AppendageDefinition)
*/
MassMoment appendageMass(const AppendageDefinition& appendage, const JointVec& q, Scalar payload = 0);

/**
 * @brief The joint torques that hold an appendage still against gravity
 * @param[in] appendage The appendage
 * @param[in] q One angle per joint, body to tip (rad)
 * @param[in] gravity m/s^2, gravity's acceleration in the appendage's frame (straight down is
 *            (0, 0, -9.81) with the body level)
 * @param[in] payload kg held at the tip
 * @return N m each joint applies, the same sign as JointFeedback::appendageEffort (tau = -J^T f for
 *         each link's weight f); empty if q is the wrong size
*/
JointVec appendageHoldTorques(const AppendageDefinition& appendage, const JointVec& q, const Vec3& gravity,
                              Scalar payload = 0);

/**
 * @brief How each joint's holding torque grows with a load at the tip: appendageHoldTorques per kg of payload
 * @return N m per kg, one per joint; empty if q is the wrong size
*/
JointVec appendagePayloadTorques(const AppendageDefinition& appendage, const JointVec& q, const Vec3& gravity);

}  // namespace gait_core

#endif  // GAIT_CORE__MASS_PROPERTIES_HPP
