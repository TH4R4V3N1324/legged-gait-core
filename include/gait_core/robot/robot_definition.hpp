#ifndef GAIT_CORE__ROBOT_DEFINITION_HPP
#define GAIT_CORE__ROBOT_DEFINITION_HPP

#include <array>
#include <cstdint>

#include "gait_core/robot/appendage_definition.hpp"
#include "gait_core/robot/gait_pattern.hpp"
#include "gait_core/robot/leg_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Everything about the robot that the gait needs: its legs, their links and limits, and stance
 * @note SI units, REP 103 body frame (+x forward, +y left, +z up). legs[0..numLegs - 1] are used;
 *       e.g. clockwise from the front right seen from above, as explorer (gait_core::robots) does.
*/
struct RobotDefinition {
    int numLegs = 0;                         // 1..kMaxLegs
    std::array<LegDefinition, kMaxLegs> legs;    // indexed by leg number

    // Neutral stance, in each leg's frame (m)
    Scalar standReach;    // how far out from the coxa joint the foot stands
    Scalar standHeight;   // foot height relative to the hips (negative = below)

    Scalar maxStride;     // m, furthest a foot moves from its neutral position

    // m, body frame: where the robot's weight acts. The engine keeps it over the planted feet
    // (see GaitParams::min_stability_margin). The body origin is a fair guess for a symmetric robot.
    // With masses given (bodyMass > 0), this is the body's own, without its legs: the engine adds
    // each leg's links (JointDefinition::mass) where the joints put them, every tick.
    // Likewise without its appendages: the engine adds those where their joints put them.
    Vec3 centreOfMass = Vec3::Zero();
    Scalar bodyMass = 0;   // kg, the body without its legs; 0: no masses, centreOfMass is the whole robot's and fixed

    // Heads, arms and the like on the body (see AppendageDefinition): their weight, and what they
    // hold, counts where their joints put it. Only with masses (bodyMass > 0).
    int numAppendages = 0;                                   // 0..kMaxAppendages
    std::array<AppendageDefinition, kMaxAppendages> appendages;   // indexed by appendage number

    // The gaits it can walk in, selected by index (the protocol's and ROS's gait id). Each must
    // time exactly numLegs legs; the engine won't move a robot with a gait it can't run.
    int numGaits = 0;                          // 1..kMaxGaits
    std::array<GaitPattern, kMaxGaits> gaits;
};

}  // namespace gait_core

#endif  // GAIT_CORE__ROBOT_DEFINITION_HPP
