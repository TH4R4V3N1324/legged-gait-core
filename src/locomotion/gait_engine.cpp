#include "gait_engine_internal.hpp"

#include "gait_core/kinematics/mass_properties.hpp"

// GaitEngine is one class split by area: this file builds it and holds the shared pieces
// (per-leg kinematics, logging, reach checks). The rest:
//   gait_engine_inputs.cpp    IMU, joint states, foot forces, leg odometry
//   gait_engine_commands.cpp  gait, mode, height, sit, power, body pose
//   gait_engine_tick.cpp      update(), startup, sending foot targets, sitting
//   gait_engine_walk.cpp      stepping, walking, returning to stance
//   gait_engine_terrain.cpp   terrain adaptation from foot forces
//   gait_engine_balance.cpp   tilt mode, load balancing, leveling while walking

// Moved from hexapod-ros2's gait_controller.cpp. The logic is unchanged, so the node's golden trace
// test still matches; only the ROS calls were swapped for the engine's inputs, outputs and hooks.

namespace gait_core {

using namespace engine_internal;

namespace {

/**
 * @brief The robot with numLegs kept within the per-leg arrays, since every leg loop trusts it
 * @note Only guards memory: a leg count the gaits can't schedule is a separate check.
*/
RobotDefinition withinCapacity(RobotDefinition robot) {
    robot.numLegs = std::clamp(robot.numLegs, 0, kMaxLegs);
    return robot;
}

/**
 * @brief Whether YawPitchPitchLeg (built from this leg) gives exactly this leg's kinematics
 * @param[in] leg The leg
 * @return true for a yaw joint then two pitch joints in a line, whatever the link lengths: a coxa about +z at the leg origin,
 *         then femur and tibia joints about +y spaced out along +x, and the foot out along +x
*/
bool fitsYawPitchPitch(const LegDefinition& leg) {
    if (leg.numJoints != 3) {
        return false;
    }
    auto along = [](const Vec3& v, const Vec3& direction) {
        return (v.normalized() - direction).norm() < S(1e-6);
    };
    auto outward = [](const Vec3& offset) {
        // Offsets read from a model file carry rounding noise (1e-17, say) in the zero components
        return std::abs(offset.y()) < S(1e-6) && std::abs(offset.z()) < S(1e-6) && offset.x() > S(1e-6);
    };
    return along(leg.joints[0].axis, Vec3::UnitZ()) && (leg.joints[0].origin.array().abs() < S(1e-6)).all() &&
        along(leg.joints[1].axis, Vec3::UnitY()) && outward(leg.joints[1].origin) &&
        along(leg.joints[2].axis, Vec3::UnitY()) && outward(leg.joints[2].origin) &&
        outward(leg.foot);
}

}  // namespace

GaitEngine::GaitEngine(const GaitParams& params, const RobotDefinition& geometry)
    : p(params), robot(withinCapacity(geometry)), trajectoryGen(robot),
      servo_power(params.power_on_startup) {
    if (p.initial_mode >= NUM_MODES) {
        p.initial_mode = MODE_NORMAL;
    }
    config().setMode(static_cast<Mode>(p.initial_mode));
    trajectoryGen.swingLift = p.step_height;
    trajectoryGen.swingEasing = p.swing_easing;
    target_height = config().currentHeight;
    stand_reach = config().startPos.x();
    stand_height = target_height;

    current_leg_positions.fill(Vec3::Zero());
    measured_leg_positions.fill(Vec3::Zero());
    startup_start_positions.fill(Vec3::Zero());
    swing_start.fill(Vec3::Zero());
    swing_target.fill(Vec3::Zero());
    tilt_ground_feet.fill(Vec3::Zero());
    tilt_saved_feet.fill(Vec3::Zero());
    previous_feet.fill(Vec2::Zero());
    foot_forces.fill(Vec3::Zero());
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        // Each leg's own solver: the analytic one only where it really is this leg's kinematics
        const LegDefinition& leg = robot.legs[legNum];
        chains[legNum] = ChainKinematics(leg);
        if (leg.solver == LegSolver::YawPitchPitch && fitsYawPitchPitch(leg)) {
            analytic[legNum].emplace(leg);
            leg_solver[legNum] = LegSolver::YawPitchPitch;
        } else {
            leg_solver[legNum] = LegSolver::Chain;
        }
        // A default JointVec is empty, not zero: size every leg's command to its joint count
        output.joints[legNum] = JointVec::Zero(kinematics(legNum).numJoints());
    }
    // The gaits' margins are judged with the legs in their standing stance
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        JointVec stance;
        kinematics(legNum).ik(config().startPosition[legNum], output.joints[legNum], stance);
        output.joints[legNum] = stance;
    }
    com = liveCentreOfMass();
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        output.joints[legNum] = JointVec::Zero(kinematics(legNum).numJoints());
    }
    for (int gait = 0; gait < config().numGaits(); ++gait) {
        // Whether the feet left down in each of its phases make a polygon, at the neutral stance
        const GaitSchedule schedule = config().getGaitConfig(static_cast<uint8_t>(gait));
        gait_area[gait] = !schedule.empty();
        for (size_t phase = 0; phase < schedule.size(); ++phase) {
            std::array<Vec2, kMaxLegs> down;
            int count = 0;
            for (int legNum = 0; legNum < numLegs(); ++legNum) {
                if (!schedule.swingsIn(legNum, phase)) {
                    down[count++] = trajectoryGen.legToBody(legNum, config().startPosition[legNum]).head<2>();
                }
            }
            gait_area[gait] = gait_area[gait] && BalanceController::hasArea(down.data(), count);
        }
        // Sway only a gait that needs it and that it rescues; the rest keep the body where walking puts it
        gait_margin[gait] = computeGaitMargin(static_cast<uint8_t>(gait), false);
        if (p.body_sway && gait_margin[gait] < p.min_stability_margin) {
            const Scalar swayed = computeGaitMargin(static_cast<uint8_t>(gait), true);
            if (swayed >= p.min_stability_margin) {
                gait_margin[gait] = swayed;
                gait_sways[gait] = true;
            }
        }
    }
}

/**
 * @brief Use other kinematics for one leg, e.g. an analytic solver for a different leg design
 * @param[in] legNum The leg, 0..numLegs() - 1; others are ignored
 * @param[in] kinematics Not owned, and must outlive the engine; null goes back to the built in
 *            YawPitchPitchLeg. Ignored, with a warning, if its numJoints() is not 1..kMaxJointsPerLeg.
 * @note Resets that leg's joint command to zeros of the new size, so set it before standing up.
 *       Kept when servo power is cut.
*/
void GaitEngine::setLegKinematics(int legNum, const LegKinematics* kinematics) {
    if (legNum < 0 || legNum >= numLegs()) {
        return;
    }
    if (kinematics != nullptr &&
        (kinematics->numJoints() < 1 || kinematics->numJoints() > kMaxJointsPerLeg)) {
        emit(LogLevel::Warn, "Leg %d kinematics ignored: %d joints, at most %d supported",
             legNum, kinematics->numJoints(), kMaxJointsPerLeg);
        return;
    }
    legKinematics[legNum] = kinematics;
    output.joints[legNum] = JointVec::Zero(this->kinematics(legNum).numJoints());
}

// ---------------------------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------------------------

void GaitEngine::emit(LogLevel level, const char* format, ...) {
    if (!log) {
        return;
    }
    char message[160];
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    log(level, message);
}

/**
 * @brief Whether a throttled message is due, marking it sent if so
*/
bool GaitEngine::throttleDue(int key, Scalar period) {
    if (throttled_log_sent[key] && toSeconds(now_ - last_throttled_log[key]) < static_cast<double>(period)) {
        return false;
    }
    throttled_log_sent[key] = true;
    last_throttled_log[key] = now_;
    return true;
}

void GaitEngine::emitThrottled(int key, Scalar period, LogLevel level, const char* format, ...) {
    if (!log || !throttleDue(key, period)) {
        return;
    }
    char message[160];
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    log(level, message);
}

/**
 * @brief Whether a leg can put its foot at a target
 * @param[in] legNum The leg
 * @param[in] target The foot position (leg frame)
 * @return true if IK lands the foot within 1 mm of the target with every joint inside its limits
 * @note The limits are each joint's lower/upper in the leg's definition, so any number of joints
 *       works. A leg given kinematics (setLegKinematics) with a different joint count than its
 *       definition is never reachable, as there are no limits to check it against.
*/
bool GaitEngine::reachable(int legNum, const Vec3& target) const {
    constexpr Scalar REACH_TOLERANCE = S(1e-3);  // m
    const LegKinematics& leg = kinematics(legNum);              // the solver
    const LegDefinition& definition = robot.legs[legNum];  // its joints and their limits
    if (leg.numJoints() != definition.numJoints) {
        return false;  // an override with a different chain: no limits to judge it by
    }
    JointVec angles;
    leg.ik(target, output.joints[legNum], angles);  // the result is judged below, not by ik's flag
    for (int j = 0; j < leg.numJoints(); ++j) {
        if (angles(j) < definition.joints[j].lower || angles(j) > definition.joints[j].upper) {
            return false;
        }
    }
    return (leg.fk(angles) - target).norm() <= REACH_TOLERANCE;
}

/**
 * @brief Whether every leg can put its foot at a shared pose, e.g. seated or standing
 * @param[in] target The foot position (leg frame), before each leg's stanceOffset is added
*/
bool GaitEngine::everyLegReaches(const Vec3& target) const {
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!reachable(legNum, target + config().stanceOffset[legNum])) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------
// Static stability
// ---------------------------------------------------------------------------------------------

/**
 * @brief How far the centre of mass is inside some feet, all in the body frame and seen from above
 * @param[in] feet Each leg's foot (leg frame, before the body's sway); legs whose foot is NaN are
 *            left out, so callers mark the ones in the air that way
 * @param[in] bodySway The body's sway over those feet, which carries the centre of mass with it
*/
Scalar GaitEngine::plantedMargin(const std::array<Vec3, kMaxLegs>& feet, const Vec2& bodySway) const {
    std::array<Vec2, kMaxLegs> planted;
    int count = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (feet[legNum].allFinite()) {
            planted[count++] = trajectoryGen.legToBody(legNum, feet[legNum]).head<2>();
        }
    }
    return stabilityMargin(planted.data(), count, com.head<2>() + bodySway);
}

/**
 * @brief The whole robot's centre of mass, with each leg where its joints were last commanded
 * @return Body frame (m). RobotDefinition::centreOfMass if the robot gives no masses (bodyMass 0).
 * @note Uses each leg's definition with output.joints, so a leg given other kinematics with a
 *       different joint count adds nothing.
*/
Vec3 GaitEngine::liveCentreOfMass() const {
    if (!(robot.bodyMass > 0)) {
        return robot.centreOfMass;
    }
    MassMoment total;
    total.add(robot.bodyMass, robot.centreOfMass);
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const MassMoment leg = legMass(robot.legs[legNum], output.joints[legNum]);
        if (leg.mass > 0) {
            total.add(leg.mass, trajectoryGen.legToBody(legNum, leg.centre()));
        }
    }
    return total.centre(robot.centreOfMass);
}

/**
 * @brief gaitStabilityMargin, worked out from the gait's schedule and the robot's neutral stance
 * @param[in] gait The gait
 * @param[in] sway false: with the centre of mass where it is. true: with the body swayed to the
 *            most stable point over each phase's feet.
 * @note While walking every planted foot sweeps up to maxStride from neutral, so taking maxStride
 *       off the neutral margin covers the worst a stride can do (feet translating together towards
 *       the nearest edge).
*/
Scalar GaitEngine::computeGaitMargin(uint8_t gait, bool sway) const {
    const GaitSchedule schedule = config().getGaitConfig(gait);
    if (schedule.empty()) {
        return -std::numeric_limits<Scalar>::infinity();
    }
    Scalar worst = std::numeric_limits<Scalar>::infinity();
    for (size_t phase = 0; phase < schedule.size(); ++phase) {
        std::array<Vec3, kMaxLegs> feet;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            feet[legNum] = schedule.swingsIn(legNum, phase)
                ? Vec3::Constant(std::numeric_limits<Scalar>::quiet_NaN())
                : config().startPosition[legNum];
        }
        if (sway) {
            std::array<Vec2, kMaxLegs> planted;
            int count = 0;
            for (int legNum = 0; legNum < numLegs(); ++legNum) {
                if (feet[legNum].allFinite()) {
                    planted[count++] = trajectoryGen.legToBody(legNum, feet[legNum]).head<2>();
                }
            }
            Vec2 deepest;
            worst = std::min(worst, deepestSupportPoint(planted.data(), count, deepest));
        } else {
            worst = std::min(worst, plantedMargin(feet, Vec2::Zero()));
        }
    }
    return worst - config().maxStride;
}

/**
 * @brief The pose the feet are commanded through while walking: leveling, the body's sway, and
 *        balance feedback's correction
*/
BodyPose GaitEngine::walkPose() const {
    BodyPose pose = walk_level;
    pose.translation.x() += sway.x() + correction.x();
    pose.translation.y() += sway.y() + correction.y();
    return pose;
}

/**
 * @brief Move the body's sway towards a goal, at up to sway_speed
 * @param[in] goal The sway to move towards (m, body frame)
 * @param[in] dt The time since the last tick (s)
*/
void GaitEngine::easeSway(const Vec2& goal, Scalar dt) {
    const Vec2 step = goal - sway;
    const Scalar maxStep = p.sway_speed * dt;
    sway += step.norm() > maxStep ? Vec2(step * (maxStep / step.norm())) : step;
}

/**
 * @brief Move the body over the feet planted in the current phase, if the gait sways
 * @param[in] dt The time since the last tick (s)
 * @param[in] idle No velocity command: with every foot down at a phase's start, the sway eases
 *            back to zero
 * @return Whether the phase should wait: at its start, until the body has swayed far enough over
 *         the feet that stay down (min_stability_margin) for the next leg to lift. Gives up after
 *         a phase's length, with a warning.
 * @note The body sways at up to sway_speed towards the nearest point sway_margin inside the feet
 *       that are down this phase, recomputed every tick since they slide back as the body walks.
 *       A gait that doesn't sway eases off any sway left behind (by gathering the feet at startup,
 *       or by a swaying gait before it) at the same speed.
*/
bool GaitEngine::updateSway(Scalar dt, bool idle) {
    sway_holding = false;
    if (trajectoryGen.gaitState.config.empty()) {
        easeSway(Vec2::Zero(), dt);
        return false;
    }
    std::array<Vec2, kMaxLegs> feet;
    int count = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!trajectoryGen.gaitState.config.swingsIn(legNum, phase)) {
            feet[count++] = trajectoryGen.legToBody(legNum, current_leg_positions[legNum]).head<2>();
        }
    }
    if (!gaitSways(config().currentGait)) {
        easeSway(Vec2::Zero(), dt);
        return false;
    }
    if (p.preview_control) {
        // Planned ahead; the hold stays as a safety net
        const Vec2 goal = previewSway(dt, idle, feet, count);
        return holdForSway(dt, idle, feet, count, goal);
    }
    const Vec2 centre = com.head<2>();   // body frame; over the ground it's centre + sway
    Vec2 goal = Vec2::Zero();
    if (!(idle && phase_progress == 0)) {
        Vec2 target;
        nearestStablePoint(feet.data(), count, centre + sway,
                           std::max(p.sway_margin, p.min_stability_margin), target);
        goal = target - centre;
    }
    easeSway(goal, dt);
    return holdForSway(dt, idle, feet, count, goal);
}

/**
 * @brief The sway hold: at a phase's start, wait until the body is over the feet that stay down
 * @param[in] goal Where the sway is heading, to tell a sway still getting there from one that can't
 * @return Whether the phase should wait (see updateSway)
*/
bool GaitEngine::holdForSway(Scalar dt, bool idle, const std::array<Vec2, kMaxLegs>& feet, int count, const Vec2& goal) {
    const Vec2 centre = com.head<2>();

    if (idle || phase_progress > 0) {
        sway_hold_time = 0;
        return false;
    }
    if (stabilityMargin(feet.data(), count, centre + sway) >= p.min_stability_margin) {
        sway_hold_time = 0;
        return false;
    }
    // Keep waiting while the sway is getting there, however far it has to go; give up once it has
    // arrived and still isn't stable (the goal can't be reached), or after a long while regardless
    constexpr Scalar MAX_HOLD = S(5.0);   // s
    sway_hold_time += dt;
    const Scalar remaining = (goal - sway).norm();
    if ((remaining < S(1e-4) && sway_hold_time > p.phase_duration) || sway_hold_time > MAX_HOLD) {
        emitThrottled(LOG_SWAY_TIMEOUT, 5, LogLevel::Warn,
            "Body couldn't sway over the planted feet in %.1f s (%.0f mm short): lifting the next leg anyway",
            static_cast<double>(sway_hold_time), static_cast<double>(remaining * 1000));
        return false;
    }
    sway_holding = true;
    return true;
}

Scalar GaitEngine::gaitStabilityMargin(uint8_t gait) const {
    return gait < config().numGaits() ? gait_margin[gait] : -std::numeric_limits<Scalar>::infinity();
}

int GaitEngine::unstableGait() const {
    if (p.allow_unstable_gaits) {
        return -1;
    }
    for (int gait = 0; gait < config().numGaits(); ++gait) {
        if (gait_margin[gait] < p.min_stability_margin) {
            return gait;
        }
    }
    return -1;
}

}  // namespace gait_core
