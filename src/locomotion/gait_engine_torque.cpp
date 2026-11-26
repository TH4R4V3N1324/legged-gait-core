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
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        output.torques[legNum] = JointVec::Zero(output.joints[legNum].size());
    }
    const Scalar mass = totalMass();
    if (!p.torque_output || !output.commanded || !(mass > 0)) {
        return;
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
    const Vec2 accel = planned ? preview_controller.acceleration() : Vec2::Zero();
    const Vec3 up = imuFresh() ? Vec3(imuTilt().inverse() * Vec3::UnitZ()) : Vec3::UnitZ();
    const Vec3 total = mass * (BalanceController::GRAVITY * up + Vec3(accel.x(), accel.y(), 0));
    Vec2 cop = planned ? preview_controller.zmp() + correction : Vec2(com.head<2>() + sway + correction);
    Vec2 inside;
    if (stabilityMargin(support.data(), supportCount, cop) < 0 &&
        nearestStablePoint(support.data(), supportCount, cop, 0, inside)) {
        cop = inside;
    }

    // Shares, weighted by how loaded each foot may be (r, its ramp): minimise sum w_i^2 / r_i with
    // sum w = 1 and sum w p = cop, which is w_i = r_i (1/R + (p_i - mean)' S^+ (cop - mean)), the mean
    // and spread S weighted by r too. A foot ramping in or out takes less, and the centre of pressure
    // is still exactly where it should be. The pseudo-inverse covers feet in a line (or one foot).
    // Barely any weight anywhere (every foot landing or lifting at once): weigh them alike.
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

    for (int i = 0; i < count; ++i) {
        const int legNum = legs[i];
        const Vec3 force = total * (share[i] / sum);   // the ground on this foot, body frame
        // Into the leg frame (a direction: no coxa offset)
        const Vec3 legForce = trajectoryGen.bodyToLeg(legNum, force) - trajectoryGen.bodyToLeg(legNum, Vec3::Zero());
        const LegJacobian J = kinematics(legNum).jacobian(angles[i]);
        output.torques[legNum] = -J.transpose() * legForce;
    }
    output.hasTorques = true;
}

}  // namespace gait_core
