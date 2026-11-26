// GaitEngine: the tick, startup and sitting. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

GaitOutput GaitEngine::update(Scalar dt, Timestamp now) {
    walk_advancing = false;   // walk() says otherwise if it moves the gait on
    advance(dt, now);
    torque_dt = dt;
    computeTorques();
    return output;
}

/**
 * @brief update() but for the torques, which are worked out from whatever it commanded
*/
GaitOutput GaitEngine::advance(Scalar dt, Timestamp now) {
    now_ = now;
    // A leg count the gait schedules can't handle, or a gait that would tip it over: never command,
    // and keep the servos off
    if (!robotSupported()) {
        const int gait = config().unrunnableGait();
        if (numLegs() == 0 || config().numGaits() == 0) {
            emitThrottled(LOG_UNSUPPORTED_ROBOT, 5, LogLevel::Error, "Robot has no legs or no gaits: it won't move");
        } else if (gait < 0) {
            const int unstable = unstableGait();
            emitThrottled(LOG_UNSUPPORTED_ROBOT, 5, LogLevel::Error,
                "Gait %d (%s) can leave the centre of mass %.0f mm from the edge of its feet, under "
                "min_stability_margin (%.0f mm): it won't move. Set allow_unstable_gaits to run it anyway",
                unstable, config().gaitName(static_cast<uint8_t>(unstable)),
                static_cast<double>(gaitStabilityMargin(static_cast<uint8_t>(unstable)) * 1000),
                static_cast<double>(p.min_stability_margin * 1000));
        } else {
            emitThrottled(LOG_UNSUPPORTED_ROBOT, 5, LogLevel::Error,
                "Gait %d (%s) can't run on this robot's %d legs: it won't move", gait, config().gaitName(static_cast<uint8_t>(gait)), numLegs());
        }
        output.commanded = false;
        output.servoPower = false;
        return output;
    }
    if (controller_tilt_active && !controllerImuFresh()) {
        setControllerImu(ImuSample{}, false);
        emit(LogLevel::Warn, "Controller orientation timed out: tilt pose recentred");
    }
    if (full_state_since_update) {
        last_full_state = now;
        full_state_since_update = false;
    }

    // Servos that stopped answering lost their power (e-stop): don't move when it comes back
    if (servo_power && positions_initialized && p.servo_loss_timeout > 0 &&
        toSeconds(now - last_full_state) > static_cast<double>(p.servo_loss_timeout)) {
        emit(LogLevel::Warn, "No joint states for %.1f s: servo power lost (e-stop?). "
            "Send a stand command to power up and stand again", toSeconds(now - last_full_state));
        cutServoPower();
    }
    // Powering down: off once seated
    if (power_down_pending && positions_initialized && sit_state == SitState::SEATED) {
        emit(LogLevel::Info, "Seated: servos off");
        cutServoPower();
    }

    output.commanded = false;
    output.servoPower = servo_power;
    trajectoryGen.swingLift = p.step_height;
    trajectoryGen.swingEasing = p.swing_easing;

    // Run the startup sequence until the legs reach their start positions
    if (!positions_initialized) {
        positions_initialized = startup(dt);
        return output;
    }

    updateLoadBalance(dt);

    // Glide towards the height setHeight asked for, moving every foot by the same amount. Planted
    // feet follow the start positions; swings, the last commanded positions (which a swing restarts
    // from) and tilt mode's ground points carry their own heights, so they're shifted too.
    if (config().currentHeight != target_height) {
        const Scalar step = p.height_rate * dt;
        const Scalar delta = std::clamp(target_height - config().currentHeight, -step, step);
        config().setHeight(config().currentHeight + delta);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            current_leg_positions[legNum].z() += delta;
            swing_start[legNum].z() += delta;
            swing_target[legNum].z() += delta;
            tilt_ground_feet[legNum].z() += delta;
        }
    }

    // The centre of mass with the legs where they were last sent
    com = liveCentreOfMass();

    // Only walking moves the body over the ground: tilt mode and sitting start from rest
    if (config().currentMode == MODE_TILT || tilt_active || sit_state != SitState::STANDING) {
        resetBalance();
    }

    // Tilt mode, or easing the body back to neutral after leaving it
    if (config().currentMode == MODE_TILT || tilt_active) {
        tilt(dt, config().currentMode != MODE_TILT);
        return output;
    }

    if (sit_state != SitState::STANDING) {
        sitTick(dt);
        return output;
    }

    switch (config().currentMode) {
        case MODE_STRAFE:
            current_motion_intent.forward = velocity.vx;
            current_motion_intent.left = velocity.vy;
            current_motion_intent.yaw = velocity.wz;
            break;
        case MODE_NORMAL:
            // No strafing in this mode. Turning comes from wz (as Nav2 sends it) plus vy, scaled so
            // the gamepad's full linear range maps to the full yaw range
            current_motion_intent.forward = velocity.vx;
            current_motion_intent.left = 0;
            current_motion_intent.yaw = velocity.wz + velocity.vy / trajectoryGen.maxBodyRadius();
            break;
        case MODE_TILT:  // handled by tilt() above
        case NUM_MODES:
        default:
            emitThrottled(LOG_INVALID_MODE, 1, LogLevel::Warn, "Invalid or unimplemented mode");
            break;
    }
    limitAcceleration(dt);
    updateWalkLeveling(dt);
    walk(dt);
    return output;
}

/**
 * @brief Solve IK for a leg target and record it as that leg's joint command
 * @param[in] target The foot target position (leg frame)
 * @param[in] legNum The leg
 * @param[in] dt The time since the last tick (s)
 * @note The body is leveled while walking and each foot raised or lowered by load balancing.
 *       No joint goes outside the range in the leg's definition, whatever the kinematics answer
 *       (kinematics given with setLegKinematics needn't keep to it), and none moves faster than its speed limit (see jointSpeedScale): the gait slows to stay
 *       inside them, and this catches whatever it couldn't, the foot then lagging its target.
 *       Records where the foot was sent as the leg's current position.
*/
void GaitEngine::commandFoot(const Vec3& target, int legNum, Scalar dt) {
    // ik doesn't promise to read the seed before writing the result, so they can't be one vector.
    // Out of reach gives the closest pose; ik's flag is unused, as where the foot really goes is
    // checked below instead (some solvers hold the joints in range and say so, some don't).
    const LegKinematics& leg = kinematics(legNum);
    const BodyPose pose = walkPose();
    const Vec3 lift(0, 0, foot_height_offset[legNum]);
    const JointVec seed = output.joints[legNum];
    JointVec& q = output.joints[legNum];
    const Vec3 posed = trajectoryGen.posedFoot(legNum, target, pose) + lift;
    leg.ik(posed, seed, q);

    bool limited = false;        // held at a joint limit
    bool speedLimited = false;   // held to a joint speed limit
    const LegDefinition& def = robot.legs[legNum];
    if (q.size() == def.numJoints) {
        bool outOfRange = false;
        for (int j = 0; j < def.numJoints; ++j) {
            const Scalar inRange = std::clamp(q(j), def.joints[j].lower, def.joints[j].upper);
            outOfRange = outOfRange || inRange != q(j);
            q(j) = inRange;
        }
        if (outOfRange) {
            limited = true;
            emitThrottled(LOG_JOINT_RANGE_LIMITED, 5, LogLevel::Warn,
                "Leg %d asked to go past a joint limit: held at the limit, the foot misses its target", legNum + 1);
        }
    }
    if (dt > 0 && seed.size() == q.size()) {
        const int joints = std::min<int>(static_cast<int>(q.size()), def.numJoints);
        for (int j = 0; j < joints; ++j) {
            const Scalar maxStep = def.joints[j].velocityLimit(p.max_joint_speed) * dt;
            const Scalar step = q(j) - seed(j);
            if (std::abs(step) > maxStep) {
                q(j) = seed(j) + std::copysign(maxStep, step);
                speedLimited = true;
            }
        }
    }
    if (speedLimited) {
        emitThrottled(LOG_JOINT_SPEED_LIMITED, 5, LogLevel::Debug,
            "Leg %d held to its joint speed limits: the foot lags its target", legNum + 1);
    }
    // Held back, or out of reach (a solver holding its joints in range misses the target without
    // being clamped here): the foot is where the joints put it, not at its target, and the gait
    // carries on from there
    constexpr Scalar REACH_TOLERANCE = S(1e-3);   // m
    const Vec3 reached = leg.fk(q);
    if (limited || speedLimited || (reached - posed).norm() > REACH_TOLERANCE) {
        // Back in the unposed frame the targets are planned in
        current_leg_positions[legNum] = trajectoryGen.bodyToLeg(legNum,
            trajectoryGen.unposedBodyPoint(trajectoryGen.legToBody(legNum, reached - lift), pose));
    } else {
        current_leg_positions[legNum] = target;
    }
}

/**
 * @brief Command every foot towards its target, limited to max_foot_speed
 * @param[in] targets The foot target of each leg (leg frame)
 * @param[in] dt The time since the last tick (s)
*/
void GaitEngine::sendFootTargets(const std::array<Vec3, kMaxLegs>& targets, Scalar dt) {
    const Scalar maxStep = p.max_foot_speed * dt;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        Vec3 targetPos = targets[legNum];
        const Vec3 delta = targetPos - current_leg_positions[legNum];
        if (delta.norm() > maxStep) {
            targetPos = current_leg_positions[legNum] + delta.normalized() * maxStep;
        }
        commandFoot(targetPos, legNum, dt);
    }
    output.commanded = true;

    // Only reported: the gaits' margins were checked up front and tilt mode keeps its own, so this
    // means feet planted somewhere unexpected, or a gait allowed to be unstable
    std::array<Vec3, kMaxLegs> planted = targets;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (isSwinging(legNum) && phase_progress > 0) {   // at a phase's start it's still down
            planted[legNum] = Vec3::Constant(std::numeric_limits<Scalar>::quiet_NaN());
        }
    }
    // A gait only preview control plans (preview_dynamic_gaits) never has its centre of mass over its
    // feet, just the ZMP it planned this tick: judge that instead
    const bool zmpPlanned = preview_started && preview_last_tick == now_ &&
        previewPlans(config().currentGait) && !gaitSways(config().currentGait);
    const Vec2 over = zmpPlanned ? Vec2(preview_controller.zmp() - com.head<2>()) : sway;
    const Scalar margin = plantedMargin(planted, over);
    if (margin < 0 && !sway_holding) {   // holding, the next leg is still down
        emitThrottled(LOG_UNSTABLE_STANCE, 5, LogLevel::Warn, "%s %.0f mm outside the planted feet: the robot may tip",
            zmpPlanned ? "Planned ZMP" : "Centre of mass", static_cast<double>(-margin * 1000));
    }
}

/**
 * @brief Startup sequence to move legs from where they are to the seated pose
 * @param[in] dt The time since the last tick (s)
 * @return true once the sequence is complete, false while it is still running
 * @note Waits for joint states first, so the move starts from the measured leg positions instead
 *       of assuming a pose. The move takes startup_duration, or longer if a joint would otherwise
 *       exceed its speed limit. The robot is then seated, and stands up if stand_on_startup is set
 *       (or a stand was asked for meanwhile); otherwise it waits for one.
*/
bool GaitEngine::startup(Scalar dt) {
    if (!servo_power) {
        emitThrottled(LOG_SERVOS_OFF, 30, LogLevel::Info, "Servos off: send a stand command to power them up");
        return false;
    }
    if (!startup_planned) {
        if (!has_joint_states) {
            emitThrottled(LOG_WAITING_FOR_JOINTS, 5, LogLevel::Info, "Waiting for joint states from every joint");
            return false;
        }
        // Into the seated pose, whatever pose the legs start in: the stand up from there is the same
        // as after a sit request
        if (everyLegReaches(Vec3(p.sit_reach, 0, -p.sit_height)) && everyLegReaches(Vec3(p.sit_reach, 0, stand_height))) {
            config().setReach(p.sit_reach);
            config().setHeight(-p.sit_height);
            target_height = -p.sit_height;
            emit(LogLevel::Info, "Moving legs from their measured positions to the seated pose");
        } else {
            sit_state = SitState::STANDING;
            emit(LogLevel::Warn, "sit_reach and sit_height put a foot out of reach: "
                "moving legs from their measured positions straight to the standing pose");
        }
        current_leg_positions = measured_leg_positions;
        startup_start_positions = measured_leg_positions;
        // The joints start where they are: the speed limit in commandFoot counts from here
        for (int leg = 0; leg < numLegs(); ++leg) {
            output.joints[leg] = measured_angles[leg];
        }
        startup_progress = 0;
        startup_planned = true;
    }

    phase = 0;
    config().setHeight(config().currentHeight);

    // Straight line from the measured position to the start position
    auto targetsAt = [this](Scalar progress) {
        std::array<Vec3, kMaxLegs> targets;
        for (int leg = 0; leg < numLegs(); ++leg) {
            const Vec3& start = startup_start_positions[leg];
            targets[leg] = start + (config().startPosition.at(leg) - start) * progress;
        }
        return targets;
    };

    const Scalar ds = dt / p.startup_duration;
    const Scalar scale = jointSpeedScale(targetsAt(std::min(S(1.0), startup_progress + ds)), dt);
    startup_progress = std::min(S(1.0), startup_progress + ds * scale);

    const auto targets = targetsAt(startup_progress);
    for (int leg = 0; leg < numLegs(); ++leg) {
        commandFoot(targets[leg], leg, dt);
    }
    output.commanded = true;
    // Where the feet were sent: standing up from the seated pose holds them here
    current_leg_positions = targets;

    if (startup_progress >= 1) {
        emit(LogLevel::Info, "Startup sequence complete");
        // Stand straight away if asked to, or if a stand request came in during the move
        if (sit_state != SitState::STANDING && (p.stand_on_startup || sit_state == SitState::RAISING)) {
            target_height = stand_height;
            sit_state = SitState::RAISING;
            emit(LogLevel::Info, "Standing up");
        } else if (sit_state == SitState::SEATED) {
            emit(LogLevel::Info, "Seated: call sit with false to stand up");
        }
        if (seatedChanged) {
            seatedChanged(seated());
        }
        return true;
    }
    return false;
}

/**
 * @brief Evaluate every leg's trajectory target after advancing it by ds, without changing the splines
 * @param[in] ds How far to advance each spline (fraction of a phase)
 * @return The target foot position of each leg (leg frame)
 * @note Legs with no active swing or stance spline hold their commanded position
*/
std::array<Vec3, kMaxLegs> GaitEngine::legTargets(Scalar ds) {
    std::array<Vec3, kMaxLegs> targets;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        const LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];

        if (spline.active) {
            const Scalar s = std::min(S(1.0), spline.s + ds);
            targets[legNum] = (s >= 1) ? spline.P4
                : limitSwingLift(legNum, trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s),
                                 spline.P0.z() + (spline.P4.z() - spline.P0.z()) * s);
        } else if (stanceSpline.active) {
            const Scalar s = std::min(S(1.0), stanceSpline.s + ds);
            targets[legNum] = stanceSpline.P0 + (stanceSpline.P1 - stanceSpline.P0) * s;
        } else {
            targets[legNum] = current_leg_positions[legNum];
        }
    }
    return targets;
}

/**
 * @brief Work out how much to slow this tick so no joint moves faster than its speed limit
 * @note Each joint's limit is its JointDefinition::max_velocity, or max_joint_speed if it sets none
 * @param[in] targets The foot target of each leg for this tick (leg frame)
 * @param[in] dt The time since the last tick (s)
 * @return A factor from 0 to 1 to scale this tick's progress by (1 = full speed)
*/
Scalar GaitEngine::jointSpeedScale(const std::array<Vec3, kMaxLegs>& targets, Scalar dt) {
    Scalar scale = S(1.0);
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const LegKinematics& leg = kinematics(legNum);
        JointVec from;
        JointVec to;
        leg.ik(current_leg_positions[legNum], output.joints[legNum], from);
        leg.ik(targets[legNum], from, to);
        const LegDefinition& def = robot.legs[legNum];
        const int joints = std::min<int>(static_cast<int>(to.size()), def.numJoints);
        for (int j = 0; j < joints; ++j) {
            const Scalar delta = std::abs(to(j) - from(j));
            const Scalar allowed = def.joints[j].velocityLimit(p.max_joint_speed) * dt;
            if (delta > allowed) {
                scale = std::min(scale, allowed / delta);
            }
        }
    }
    return scale;
}

/**
 * @brief Sitting down or standing up, one tick at a time
 * @param[in] dt The time since the last tick (s)
 * @note Sitting steps the feet out to sit_reach a gait group at a time, then lowers the body onto
 *       the ground. Standing up is the reverse. The velocity command is ignored throughout, load
 *       balancing and walk leveling unwind, and the height glide does the lowering and raising.
*/
void GaitEngine::sitTick(Scalar dt) {
    current_motion_intent = MotionIntent{};
    updateWalkLeveling(dt);

    switch (sit_state) {
        case SitState::SPREADING:
        case SitState::GATHERING:
            if (!idleReturning) {
                if (!feetAtStart()) {
                    idleReturning = true;
                    phase_progress = 0;
                    leg_swinging.fill(false);
                } else if (sit_state == SitState::SPREADING) {
                    target_height = -p.sit_height;
                    sit_state = SitState::LOWERING;
                } else {
                    sit_state = SitState::STANDING;
                    if (seatedChanged) {
                        seatedChanged(seated());
                    }
                    emit(LogLevel::Info, "Standing");
                }
            }
            if (idleReturning) {
                returnToStart(dt);
                return;
            }
            break;
        case SitState::LOWERING:
            if (config().currentHeight == target_height) {
                sit_state = SitState::SEATED;
                emit(LogLevel::Info, "Seated");
            }
            break;
        case SitState::RAISING:
            if (config().currentHeight == target_height) {
                config().setReach(stand_reach);
                sit_state = SitState::GATHERING;
            }
            break;
        case SitState::SEATED:
        case SitState::STANDING:
            break;
    }

    // Every foot is down here, so any body sway eases back to centre before the body lowers or rises
    easeSway(Vec2::Zero(), dt);

    // Hold the feet where they are; the height glide has already moved them this tick
    sendFootTargets(current_leg_positions, dt);
}

}  // namespace gait_core
