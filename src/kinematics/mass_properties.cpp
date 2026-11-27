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

namespace {

// Walks an appendage's chain: each joint's origin and axis, and its link's centre of mass, in the
// appendage's frame; and the tip. n is the joint count, already checked against q.
struct ChainPose {
    std::array<Vec3, kMaxJointsPerLeg> origin, axis, linkCentre;
    Vec3 tip = Vec3::Zero();
};

ChainPose walkChain(const AppendageDefinition& appendage, const JointVec& q) {
    ChainPose pose;
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    for (int j = 0; j < appendage.numJoints; ++j) {
        const JointDefinition& joint = appendage.joints[j];
        position += rotation * joint.origin;
        const Vec3 axis = joint.axis.normalized();
        pose.origin[j] = position;
        pose.axis[j] = rotation * axis;
        rotation = rotation * AngleAxis(q(j), axis).toRotationMatrix();
        pose.linkCentre[j] = position + rotation * joint.centreOfMass;
    }
    pose.tip = position + rotation * appendage.tip;
    return pose;
}

}  // namespace

MassMoment appendageMass(const AppendageDefinition& appendage, const JointVec& q, Scalar payload) {
    MassMoment result;
    if (q.size() != appendage.numJoints) {
        return result;
    }
    const ChainPose pose = walkChain(appendage, q);
    for (int j = 0; j < appendage.numJoints; ++j) {
        if (appendage.joints[j].mass > 0) {
            result.add(appendage.joints[j].mass, pose.linkCentre[j]);
        }
    }
    if (payload > 0) {
        result.add(payload, pose.tip);
    }
    return result;
}

JointVec appendageHoldTorques(const AppendageDefinition& appendage, const JointVec& q, const Vec3& gravity,
                              Scalar payload) {
    if (q.size() != appendage.numJoints) {
        return JointVec();
    }
    const ChainPose pose = walkChain(appendage, q);
    JointVec torques = JointVec::Zero(appendage.numJoints);
    // Joint j holds up every link beyond it (and the load): tau_j = -axis_j . sum (r x m g)
    for (int j = 0; j < appendage.numJoints; ++j) {
        Vec3 moment = Vec3::Zero();
        for (int k = j; k < appendage.numJoints; ++k) {
            const Scalar mass = appendage.joints[k].mass;
            if (mass > 0) {
                moment += (pose.linkCentre[k] - pose.origin[j]).cross(mass * gravity);
            }
        }
        if (payload > 0) {
            moment += (pose.tip - pose.origin[j]).cross(payload * gravity);
        }
        torques(j) = -pose.axis[j].dot(moment);
    }
    return torques;
}

JointVec appendagePayloadTorques(const AppendageDefinition& appendage, const JointVec& q, const Vec3& gravity) {
    if (q.size() != appendage.numJoints) {
        return JointVec();
    }
    const ChainPose pose = walkChain(appendage, q);
    JointVec perKg = JointVec::Zero(appendage.numJoints);
    for (int j = 0; j < appendage.numJoints; ++j) {
        perKg(j) = -pose.axis[j].dot((pose.tip - pose.origin[j]).cross(gravity));
    }
    return perKg;
}

}  // namespace gait_core
