#ifndef GAIT_CORE__APPENDAGE_DEFINITION_HPP
#define GAIT_CORE__APPENDAGE_DEFINITION_HPP

#include <array>

#include "gait_core/types.hpp"
#include "gait_core/robot/joint_definition.hpp"
#include "gait_core/robot/leg_definition.hpp"

namespace gait_core {

/**
 * @brief Something on the body that isn't a leg but moves the robot's weight about: a head on a
 *        neck, an arm, a camera mast
 * @note The engine doesn't move it (the robot's own code does) but it follows it: given its joint
 *       angles each tick (JointFeedback::appendageAngles), its links' masses count towards the
 *       robot's centre of mass and weight wherever they are, and so does a load it holds at its tip:
 *       set with GaitEngine::setAppendagePayload, or worked out from the joints' efforts up to
 *       maxPayload. With torque output, GaitOutput::appendageTorques holds it all up.
 *
 *       Its frame works as a leg's does: mount.yaw turns it about the body's z axis and
 *       mount.translation_offset places its first joint on the body; each joint's origin and axis
 *       are given at zero angles, and each link's centre of mass is in its joint's frame.
*/
struct AppendageDefinition {
    LegConfig mount;                                        // its first joint's place on the body
    int numJoints = 0;                                      // 0..kMaxJointsPerLeg
    std::array<JointDefinition, kMaxJointsPerLeg> joints;   // body to tip, with their links' masses
    Vec3 tip = Vec3::Zero();                                // m, last joint to where a load is held
    // kg, the heaviest load it's taken to hold at its tip, from its joints' efforts beyond what its
    // own links need. 0 (a head, say): never estimated, so effort noise can't make up a load; a
    // payload can still be set with GaitEngine::setAppendagePayload.
    Scalar maxPayload = 0;
};

}  // namespace gait_core

#endif  // GAIT_CORE__APPENDAGE_DEFINITION_HPP
