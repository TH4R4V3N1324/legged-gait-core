#include "gait_core/kinematics/mass_properties.hpp"

namespace gait_core {

MassMoment legMass(const LegDefinition& leg, const JointVec& q) {
    MassMoment result;
    if (q.size() != leg.numJoints) {
        return result;
    }
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    for (int j = 0; j < leg.numJoints; ++j) {
        const JointDefinition& joint = leg.joints[j];
        position += rotation * joint.origin;
        rotation = rotation * AngleAxis(q(j), joint.axis.normalized()).toRotationMatrix();
        if (joint.mass > 0) {
            result.add(joint.mass, position + rotation * joint.centreOfMass);
        }
    }
    return result;
}

}  // namespace gait_core
