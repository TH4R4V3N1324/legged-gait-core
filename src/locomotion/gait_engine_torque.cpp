// GaitEngine: joint torques for robots that can command them (tau = -J^T f). One class over several
// files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

#include "gait_core/kinematics/mass_properties.hpp"

namespace gait_core {

using namespace engine_internal;

/**
 * @brief The robot's whole mass (kg): body and every link, or 0 if its definition gives none
*/
Scalar GaitEngine::totalMass() const {
    if (!(robot.bodyMass > 0)) {
        return 0;
    }
    Scalar mass = robot.bodyMass;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        for (int j = 0; j < robot.legs[legNum].numJoints; ++j) {
            mass += robot.legs[legNum].joints[j].mass;
        }
    }
    for (int i = 0; i < numAppendages(); ++i) {
        for (int j = 0; j < robot.appendages[i].numJoints; ++j) {
            mass += robot.appendages[i].joints[j].mass;
        }
        mass += appendage_payload[i];
    }
    return mass;
}

/**
 * @brief Each joint's torque for the feet to carry the robot and push it where the gait plans
 * @note The ground has to push on the feet with the robot's weight (tilted into the body frame by the
 *       IMU when it's fresh), through its centre of mass; with preview control, plus what accelerates
 *       the body along the planned path, through the planned ZMP. That's shared between the feet down
 *       so the centre of pressure is there (moved inside them if it isn't), with the least force on any
 *       one foot and none pulling: each foot's
 *       share w solves sum w = 1, sum w p = ZMP with sum w^2 least. Each planted leg's joints then
 *       hold their foot's share, tau = -J^T f (the opposite of LegKinematics::groundForce). Swinging
 *       legs get nothing: they're on position control. Each foot's share is ramped in over 50 ms after
 *       it's put down and out over 50 ms before it lifts. Needs the robot's masses; without them, or
 *       with no foot down, the torques are zero and GaitOutput::hasTorques is false.
*/
void GaitEngine::computeTorques() {
    output.hasTorques = false;
    // The joint state the engine last saw answers the command before this one
    const std::array<JointVec, kMaxLegs> lastCommand = posture_command;
    posture_command = output.joints;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        output.torques[legNum] = JointVec::Zero(output.joints[legNum].size());
        output.stiffness[legNum] = JointVec::Ones(output.joints[legNum].size());
    }
    for (int i = 0; i < numAppendages(); ++i) {
        output.appendageTorques[i] = JointVec::Zero(robot.appendages[i].numJoints);
    }
    const Scalar mass = totalMass();
    if (!p.torque_output || !output.commanded || !(mass > 0)) {
        return;
    }
    // Each appendage held up where it is, with its load (tilted by the IMU while it's fresh)
    const Vec3 gravity = gravityInBody();
    for (int i = 0; i < numAppendages(); ++i) {
        const AppendageDefinition& appendage = robot.appendages[i];
        const JointVec hold = appendageHoldTorques(appendage, appendage_angles[i],
            AngleAxis(-appendage.mount.yaw, Vec3::UnitZ()) * gravity, appendage_payload[i]);
        if (hold.size() == appendage.numJoints) {
            output.appendageTorques[i] = hold;
        }
    }

    // The feet down, where the joints sent put them (body frame), and how much of its share each
    // takes: a foot is loaded up over TORQUE_RAMP after the gait puts it down and unloaded over
    // TORQUE_RAMP before it lifts, so a foot that lands a little late (or a leg about to lift) isn't
    // asked for its whole share at once; the others make up the difference
    constexpr Scalar TORQUE_RAMP = S(0.05);   // s
    std::array<int, kMaxLegs> legs{};
    std::array<Vec3, kMaxLegs> feet;
    std::array<Vec2, kMaxLegs> ground;
    ContactPoints support;   // the polygon the centre of pressure must stay in: ground, with any soles
    int supportCount = 0;
    std::array<Scalar, kMaxLegs> ramp{};
    std::array<JointVec, kMaxLegs> angles;
    int count = 0;
    const bool walking = sit_state == SitState::STANDING && !tilt_active && config().currentMode != MODE_TILT;
    const GaitSchedule& schedule = trajectoryGen.gaitState.config;
    // With contact sensors, a foot only counts once it's really down (unless none is: then the plan)
    int touching = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        touching += foot_contact[legNum] ? 1 : 0;
    }
    const bool sensed = contact_sensed && touching > 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const bool up = (walking && leg_swinging[legNum] && phase_progress > 0) || (sensed && !foot_contact[legNum]);
        torque_down_time[legNum] = up ? S(0.0) : torque_down_time[legNum] + torque_dt;
        if (up || output.joints[legNum].size() != kinematics(legNum).numJoints()) {
            continue;
        }
        // The leg as it is (its last joint state) if known, since that's what the torque has to hold
        // up; else as it was sent
        const bool measured = has_joint_states && measured_angles[legNum].size() == output.joints[legNum].size();
        const JointVec& q = measured ? measured_angles[legNum] : output.joints[legNum];
        Scalar share = std::min(S(1.0), torque_down_time[legNum] / TORQUE_RAMP);
        if (walking && walk_advancing && !schedule.empty() && schedule.swingsIn(legNum, (phase + 1) % schedule.size())) {
            // The next phase's legs lift when this one ends (only while the gait is moving on: standing
            // or held, nothing is about to lift)
            share = std::min(share, (1 - phase_progress) * current_phase_duration / TORQUE_RAMP);
        }
        ramp[count] = std::clamp(share, S(0.0), S(1.0));
        legs[count] = legNum;
        angles[count] = q;
        feet[count] = trajectoryGen.legToBody(legNum, kinematics(legNum).fk(q));
        ground[count] = feet[count].head<2>();
        supportCount = addContactPoints(legNum, ground[count], q, support, supportCount);
        ++count;
    }
    if (count == 0) {
        return;
    }

    // The total force from the ground, and where it should act: under the centre of mass, holding its
    // weight. With preview control planning the body's path, where that plan puts the ZMP and what it
    // accelerates by. (Not the acceleration worked out from the commanded motion: on a dog's trot the
    // legs swinging move the centre of mass about enough to make that jitter by a lot.)
    const bool planned = walking && preview_started && previewPlans(config().currentGait);
    Vec2 accel = planned ? preview_controller.acceleration() : Vec2::Zero();
    const Vec2 plannedCop = planned ? preview_controller.zmp() + correction : Vec2(com.head<2>() + sway + correction);
    // Capture point feedback (capture_torque_gain): push from past where it has gone, which brings it
    // back. Measured only while balance feedback is on and walking (its error is zero otherwise).
    const Scalar omega = balance_controller.state().omega;
    const bool captureFeedback = walking && p.balance_feedback && p.capture_torque_gain > 0 && omega > 0;
    Vec2 cop = plannedCop;
    if (captureFeedback) {
        cop += p.capture_torque_gain * balance_controller.state().captureError;
    }
    Vec2 inside;
    if (stabilityMargin(support.data(), supportCount, cop) < 0 &&
        nearestStablePoint(support.data(), supportCount, cop, 0, inside)) {
        cop = inside;
    }
    // A centre of pressure moved by dp pushes the centre of mass the other way: c'' = omega^2 (c - p)
    if (captureFeedback) {
        accel -= omega * omega * (cop - plannedCop);
    }
    const Vec3 up = imuFresh() ? Vec3(imuTilt().inverse() * Vec3::UnitZ()) : Vec3::UnitZ();
    Vec3 total = mass * (BalanceController::GRAVITY * up + Vec3(accel.x(), accel.y(), 0));

    // Posture (stance_stiffness below 1): what the position control no longer holds, the torques do.
    // Each planted leg's chain says how far the body has sunk and tilted over that foot (its measured
    // joints against those commanded), averaged by load; a spring and damper on that adds lift, and
    // a moment, made by moving the centre of pressure (tau / F_z), which leaves the push over the
    // ground (the horizontal force) as it was
    const Scalar give = 1 - std::clamp(p.stance_stiffness, S(0.0), S(1.0));
    if (give > 0 && p.stance_posture_frequency > 0 && has_joint_states) {
        Scalar sag = 0, weight = 0;
        Vec3 tilt = Vec3::Zero();   // rad, body frame: the body turned from its command, over the feet
        for (int i = 0; i < count; ++i) {
            const int legNum = legs[i];
            const ChainKinematics& chain = chains[legNum];
            const JointVec& commanded = lastCommand[legNum];   // what the measured joints answer
            if (chain.numJoints() != angles[i].size() || commanded.size() != angles[i].size()) {
                continue;
            }
            const Scalar w = std::max(ramp[i], S(1e-3));
            // A foot higher in the body than commanded means the body has sunk towards it
            sag += w * (chain.fk(angles[i]).z() - chain.fk(commanded).z());
            // The foot turned by R in the leg frame means the body turned the other way over it
            const AngleAxis turned(chain.footRotation(angles[i]) * chain.footRotation(commanded).transpose());
            const Vec3 legTilt = -turned.angle() * turned.axis();
            tilt += w * Vec3(trajectoryGen.legToBody(legNum, legTilt) - trajectoryGen.legToBody(legNum, Vec3::Zero()));
            weight += w;
        }
        if (weight > 0) {
            sag /= weight;
            tilt /= weight;
            if (has_posture_sag && torque_dt > 0) {
                const Scalar raw = (sag - posture_sag) / torque_dt;
                constexpr Scalar RATE_FILTER = S(0.01);   // s
                posture_sag_rate += (torque_dt / (RATE_FILTER + torque_dt)) * (raw - posture_sag_rate);
            }
            posture_sag = sag;
            has_posture_sag = true;
            const Scalar w0 = 2 * kPi * p.stance_posture_frequency;
            const Scalar zeta = p.stance_posture_damping;
            const Scalar height = std::max(com.z() - feet[0].z(), BalanceController::MIN_HEIGHT);
            const Scalar inertia = mass * height * height;   // the body tipping about its feet
            total.z() += give * mass * (w0 * w0 * sag + 2 * zeta * w0 * posture_sag_rate);
            const Vec3 rate = imuFresh() ? body_angular_velocity : Vec3::Zero();
            const Vec2 moment = -give * inertia * (w0 * w0 * tilt.head<2>() + 2 * zeta * w0 * rate.head<2>());
            if (total.z() > 0) {
                // A vertical force F_z at p turns the body about its centre of mass by (p - c) x F_z
                cop += Vec2(-moment.y(), moment.x()) / total.z();
                if (stabilityMargin(support.data(), supportCount, cop) < 0 &&
                    nearestStablePoint(support.data(), supportCount, cop, 0, inside)) {
                    cop = inside;
                }
            }
        }
    } else {
        has_posture_sag = false;
        posture_sag_rate = 0;
    }

    // Shares, weighted by how loaded each foot may be (r, its ramp): minimise sum w_i^2 / r_i with
    // sum w = 1 and sum w p = cop, which is w_i = r_i (1/R + (p_i - mean)' S^+ (cop - mean)), the mean
    // and spread S weighted by r too. A foot ramping in or out takes less, and the centre of pressure
    // is still exactly where it should be. The pseudo-inverse covers feet in a line (or one foot).
    // Barely any weight anywhere (every foot landing or lifting at once): weigh them alike.
    const std::array<Scalar, kMaxLegs> load = ramp;   // how far each foot is loaded in, for its stiffness
    Scalar weightSum = 0;
    for (int i = 0; i < count; ++i) {
        weightSum += ramp[i];
    }
    if (weightSum < S(1e-3)) {
        for (int i = 0; i < count; ++i) {
            ramp[i] = 1;
        }
        weightSum = static_cast<Scalar>(count);
    }
    Vec2 mean = Vec2::Zero();
    for (int i = 0; i < count; ++i) {
        mean += ramp[i] * ground[i];
    }
    mean /= weightSum;
    Eigen::Matrix<Scalar, 2, 2> spread = Eigen::Matrix<Scalar, 2, 2>::Zero();
    for (int i = 0; i < count; ++i) {
        const Vec2 d = ground[i] - mean;
        spread += ramp[i] * d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 2, 2>> eigen(spread);
    Eigen::Matrix<Scalar, 2, 2> pinv = Eigen::Matrix<Scalar, 2, 2>::Zero();
    for (int k = 0; k < 2; ++k) {
        const Scalar value = eigen.eigenvalues()(k);
        if (value > S(1e-8)) {
            const Vec2 v = eigen.eigenvectors().col(k);
            pinv += (v * v.transpose()) / value;
        }
    }
    // None may pull: a negative share (the centre of pressure outside the feet it can use) is
    // dropped, and the rest scaled back up to the whole
    std::array<Scalar, kMaxLegs> share{};
    Scalar sum = 0;
    for (int i = 0; i < count; ++i) {
        share[i] = std::max(S(0.0), ramp[i] * (1 / weightSum + (ground[i] - mean).dot(pinv * (cop - mean))));
        sum += share[i];
    }
    if (!(sum > 0)) {
        return;
    }

    // Led by the torques (stance_stiffness or capture_torque_gain), a foot with a sole also pushes from
    // wherever in it the centre of pressure needs: the shares can only put it on the line between
    // feet (on one foot, at that foot), so each sole makes up the rest as far as it reaches, with a
    // moment about its foot through the leg's whole chain (its ankle)
    const bool torqueLed = p.stance_stiffness < 1 || p.capture_torque_gain > 0;
    Vec2 reached = Vec2::Zero();
    for (int i = 0; i < count; ++i) {
        reached += (share[i] / sum) * ground[i];
    }
    const Vec2 rest = cop - reached;
    // Body frame directions into the leg frame (no coxa offset)
    auto toLeg = [this](int legNum, const Vec3& v) {
        return Vec3(trajectoryGen.bodyToLeg(legNum, v) - trajectoryGen.bodyToLeg(legNum, Vec3::Zero()));
    };

    for (int i = 0; i < count; ++i) {
        const int legNum = legs[i];
        const Vec3 force = total * (share[i] / sum);   // the ground on this foot, body frame
        const Vec3 legForce = toLeg(legNum, force);
        const ChainKinematics& chain = chains[legNum];
        const bool canTurnSole = torqueLed && robot.legs[legNum].hasSole() && chain.numJoints() == angles[i].size();
        Vec2 offset = Vec2::Zero();   // where in its sole this foot pushes from, from the foot point
        if (canTurnSole && rest.squaredNorm() > 0) {
            ContactPoints sole;
            const int corners = addContactPoints(legNum, ground[i], angles[i], sole, 0);
            Vec2 target = ground[i] + rest;
            Vec2 inSole;
            if (stabilityMargin(sole.data(), corners, target) < 0 &&
                nearestStablePoint(sole.data(), corners, target, 0, inSole)) {
                target = inSole;
            }
            offset = target - ground[i];
        }
        if (offset.squaredNorm() > 0) {
            Eigen::Matrix<Scalar, 6, 1> wrench;
            wrench.head<3>() = legForce;
            wrench.tail<3>() = toLeg(legNum, Vec3(offset.x(), offset.y(), 0).cross(force));
            output.torques[legNum] = -chain.fullJacobian(angles[i]).transpose() * wrench;
        } else {
            const LegJacobian J = kinematics(legNum).jacobian(angles[i]);
            output.torques[legNum] = -J.transpose() * legForce;
        }
        // Planted, the torques carry it: the position control eases off as the foot loads up
        if (p.stance_stiffness < 1) {
            const Scalar stiffness = 1 - load[i] * (1 - std::clamp(p.stance_stiffness, S(0.0), S(1.0)));
            output.stiffness[legNum].setConstant(stiffness);
        }
    }
    output.hasTorques = true;
}

}  // namespace gait_core
