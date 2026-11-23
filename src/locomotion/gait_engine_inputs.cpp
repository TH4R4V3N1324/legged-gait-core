// GaitEngine: sensor inputs, foot forces and leg odometry. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

/**
 * @brief Store the body's orientation, angular velocity and linear acceleration from the IMU
 * @param[in] sample The IMU reading, and when it was received
 * @note Yaw is only used by tilt mode to hold the heading; odometry provides it otherwise.
*/
void GaitEngine::setImu(const ImuSample& sample) {
    body_roll = sample.roll;
    body_pitch = sample.pitch;
    body_yaw = sample.yaw;
    body_angular_velocity = sample.angularVelocity;
    body_linear_acceleration = sample.linearAcceleration;
    last_imu_time = sample.stamp;
    has_imu = true;
}

void GaitEngine::setControllerImu(const ImuSample& sample, bool active) {
    if (!active) {
        if (controller_tilt_active) {
            setBodyPose(0, 0, 0, Vec3::Zero());
        }
        controller_tilt_active = false;
        return;
    }

    if (!controller_tilt_active) {
        controller_reference_roll = sample.roll;
        controller_reference_pitch = sample.pitch;
        controller_reference_yaw = sample.yaw;
        controller_tilt_active = true;
    }

    setBodyPose(
        std::remainder(sample.roll - controller_reference_roll, 2 * kPi),
        std::remainder(sample.pitch - controller_reference_pitch, 2 * kPi),
        std::remainder(sample.yaw - controller_reference_yaw, 2 * kPi),
        Vec3::Zero());
    last_controller_imu_time = sample.stamp;
}

bool GaitEngine::imuFresh() const {
    return has_imu && toSeconds(now_ - last_imu_time) < 0.2;
}

bool GaitEngine::controllerImuFresh() const {
    return controller_tilt_active && toSeconds(now_ - last_controller_imu_time) < 0.2;
}

/**
 * @brief Take a joint state: each foot's measured position, the foot forces and the leg odometry
 * @param[in] feedback The joint state
 * @return true if the leg odometry was updated
 * @note has_joint_states is only set once every joint has been received. A leg whose angles
 *       aren't sized to its kinematics' numJoints() counts as not received.
*/
bool GaitEngine::onJointStates(const JointFeedback& feedback) {
    bool all_found = true;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const LegKinematics& leg = kinematics(legNum);
        if (!feedback.present[legNum] || feedback.angles[legNum].size() != leg.numJoints()) {
            all_found = false;
            continue;
        }
        measured_leg_positions[legNum] = leg.fk(feedback.angles[legNum]);
        measured_angles[legNum] = feedback.angles[legNum];
    }
    has_joint_states = all_found;
    full_state_since_update = full_state_since_update || all_found;
    if (all_found && feedback.hasEffort) {
        updateFootForces(feedback);
    }

    if (has_joint_states && positions_initialized) {
        return updateLegOdometry(feedback.stamp);
    }
    return false;
}

/**
 * @brief Estimate the force on each foot from its joint efforts and decide which feet are in contact
 * @param[in] feedback The joint state, with efforts
 * @note Forces are in the body frame and low-pass filtered (foot_force_filter). Contact switches
 *       on above contact_force_on of upward force and off below contact_force_off.
*/
void GaitEngine::updateFootForces(const JointFeedback& feedback) {
    Scalar alpha = 1;
    if (has_foot_forces) {
        const Scalar dt = static_cast<Scalar>(std::max(0.0, toSeconds(feedback.stamp - last_foot_force_stamp)));
        alpha = p.foot_force_filter > 0 ? std::min(S(1.0), dt / p.foot_force_filter) : S(1.0);
    }
    last_foot_force_stamp = feedback.stamp;

    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        foot_force_updated[legNum] = false;
        Vec3 legForce;
        if (!kinematics(legNum).groundForce(feedback.angles[legNum], feedback.effort[legNum], legForce)) {
            continue;  // leg nearly straight, or efforts the wrong size: keep the last estimate
        }
        // Rotate into the body frame (a direction, so without the coxa offset)
        Vec3 bodyForce = trajectoryGen.legToBody(legNum, legForce) - trajectoryGen.legToBody(legNum, Vec3::Zero());
        // Mid-swing the foot is certainly in the air, so whatever it reads is the leg's own weight
        if (leg_swinging[legNum] && phase_progress > S(0.3) && phase_progress < S(0.7)) {
            foot_force_offset[legNum] += S(0.02) * (-bodyForce.z() - foot_force_offset[legNum]);
        }
        bodyForce.z() += foot_force_offset[legNum];
        foot_forces[legNum] = has_foot_forces ? Vec3(foot_forces[legNum] + alpha * (bodyForce - foot_forces[legNum])) : bodyForce;

        if (foot_forces[legNum].z() > p.contact_force_on) {
            foot_contact[legNum] = true;
        } else if (foot_forces[legNum].z() < p.contact_force_off) {
            foot_contact[legNum] = false;
        }
        foot_force_updated[legNum] = true;
    }
    has_foot_forces = true;

    // Blocked: the ground's net horizontal push on the feet in contact is a large share of the
    // weight, so something else is pushing back (walking freely it's ~0.07, against a wall ~0.5).
    // Filtered over ~0.3 s, with hysteresis.
    Vec2 horizontal = Vec2::Zero();
    Scalar vertical = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (foot_contact[legNum]) {
            horizontal += foot_forces[legNum].head<2>();
            vertical += foot_forces[legNum].z();
        }
    }
    const Scalar blockedAlpha = std::min(S(1.0), alpha * p.foot_force_filter / S(0.3));
    push_ratio += blockedAlpha * ((vertical > 3 * p.contact_force_on ? horizontal.norm() / vertical : S(0.0)) - push_ratio);
    const bool wasBlocked = is_blocked;
    if (push_ratio > p.blocked_force_ratio) {
        is_blocked = true;
    } else if (push_ratio < S(0.67) * p.blocked_force_ratio) {
        is_blocked = false;
    }
    if (is_blocked != wasBlocked) {
        emit(LogLevel::Info, is_blocked ? "Pushing against something: odometry ignores the legs' movement"
                                        : "No longer pushing against anything");
    }
}

/**
 * @brief Estimate the body's motion from the planted feet and add it to the leg odometry
 * @param[in] stamp The joint state timestamp
 * @return false if the timestamp was no newer than the last one, so nothing changed
 * @note Feet that are not swinging and are bearing weight are treated as planted. Also tracks the
 *       body's height from how far the planted feet move up or down, for floorRise.
*/
bool GaitEngine::updateLegOdometry(Timestamp stamp) {
    // In simulation the joint states come faster than the clock ticks, so ~30% share a timestamp
    // with the previous one. Skip those entirely, keeping the last feet, so their motion is
    // counted with the next new timestamp; replacing the feet threw it away (odometry read 20% short).
    if (has_previous_feet && stamp <= previous_feet_stamp) {
        return false;
    }
    std::array<Vec2, kMaxLegs> feet;
    feet.fill(Vec2::Zero());
    std::array<Scalar, kMaxLegs> feetZ{};
    const Eigen::Quaternion<Scalar> tiltNow = imuTilt();
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const Vec3 bodyFoot = trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]);
        // In the level frame, so tilting the body to level it doesn't count as movement
        feet[legNum] = trajectoryGen.unposedBodyPoint(bodyFoot, walk_level).head<2>();
        // Heights levelled by the body's real tilt rather than the commanded leveling: rocking
        // while walking moves the feet up and down relative to the body without it climbing
        feetZ[legNum] = (tiltNow * bodyFoot).z();
    }

    if (has_previous_feet) {
        // Only feet bearing weight, when the joint efforts say so, since a foot only stays put once
        // it's loaded. Without effort data (mock hardware) every non-swinging foot counts.
        Scalar totalLoad = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            totalLoad += std::max(S(0.0), foot_forces[legNum].z());
        }
        const bool useContact = has_foot_forces && totalLoad > 3 * p.contact_force_on;
        const Scalar dt = static_cast<Scalar>(toSeconds(stamp - previous_feet_stamp));
        std::array<Vec2, kMaxLegs> before, after;
        std::array<Scalar, kMaxLegs> beforeZ, afterZ;
        size_t plantedCount = 0, settledCount = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            const bool planted = !isSwinging(legNum) && (!useContact || (foot_contact[legNum] && previous_contact[legNum]));
            if (planted) {
                before[plantedCount] = previous_feet[legNum];
                after[plantedCount] = feet[legNum];
                ++plantedCount;
            }
            // The servos lag at touchdown, so a foot the gait calls planted is still settling onto
            // the ground for a moment, and counting that drop as the body rising made the height
            // creep up a few mm a step. Only feet the forces say are loaded: counting every foot
            // without them took the spawn drop's bounce in simulation as the body sinking 4-5 cm.
            planted_time[legNum] = planted && useContact ? planted_time[legNum] + dt : S(0.0);
            if (planted_time[legNum] > CLIMB_SETTLE_TIME) {
                beforeZ[settledCount] = previous_feet_z[legNum];
                afterZ[settledCount] = feetZ[legNum];
                ++settledCount;
            }
        }
        BodyMotion motion;
        if (LegOdometry::estimateMotion(before.data(), after.data(), plantedCount, motion)) {
            // Pushing against something, the feet scrub along the ground, so they say the body is
            // moving when it isn't: keep only the turn
            legOdometry.integrate(is_blocked ? BodyMotion{0, 0, motion.dyaw} : motion);
        }
        if (climb_anchored) {
            Scalar climb;
            if (LegOdometry::estimateClimb(beforeZ.data(), afterZ.data(), settledCount, climb)) {
                legOdometry.integrateClimb(climb);
            }
        } else {
            // The floor the robot settles on is z = 0, with the body's height above it measured from
            // the feet that have been carrying weight for a while (at least a tripod)
            Scalar sum = 0;
            int settled = 0;
            for (int legNum = 0; legNum < numLegs(); ++legNum) {
                if (planted_time[legNum] > CLIMB_ANCHOR_TIME) {
                    sum += feetZ[legNum];
                    ++settled;
                }
            }
            if (settled >= 3) {
                odom_start_height = -sum / settled - legOdometry.z();
                climb_anchored = true;
                emit(LogLevel::Info, "Leg odometry height started: body %.1f mm above the floor, from %d feet",
                    static_cast<double>(-sum / settled * 1000), settled);
            }
        }
    }
    previous_feet = feet;
    previous_feet_z = feetZ;
    previous_contact = foot_contact;
    previous_feet_stamp = stamp;
    has_previous_feet = true;
    return true;
}

/**
 * @brief How far the body is above the floor, as the footprint transform has it
 * @return The commanded body height (m) plus any lift from tilt mode; it doesn't know about
 *         climbing (floorRise does)
*/
Scalar GaitEngine::footprintHeight() const {
    return -config().currentHeight + tilt_applied.translation.z();
}

/**
 * @brief The body's roll and pitch from the IMU, as a rotation from the body to the level frame
*/
Eigen::Quaternion<Scalar> GaitEngine::imuTilt() const {
    return Eigen::Quaternion<Scalar>(AngleAxis(body_pitch, Vec3::UnitY()) * AngleAxis(body_roll, Vec3::UnitX()));
}

/**
 * @brief How far the floor under the body has risen since the leg odometry started
 * @return The leg odometry's body height less the commanded body height (m); 0 until at least
 *         three feet have carried weight long enough to measure the body's height
*/
Scalar GaitEngine::floorRise() const {
    if (!climb_anchored) {
        return 0;
    }
    return odom_start_height + legOdometry.z() - footprintHeight();
}

}  // namespace gait_core
