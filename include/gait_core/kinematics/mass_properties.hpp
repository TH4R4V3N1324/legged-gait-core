#ifndef GAIT_CORE__MASS_PROPERTIES_HPP
#define GAIT_CORE__MASS_PROPERTIES_HPP

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

}  // namespace gait_core

#endif  // GAIT_CORE__MASS_PROPERTIES_HPP
