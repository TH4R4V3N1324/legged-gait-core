// GaitEngine: tilt mode, load balancing and leveling. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

/**
 * @brief Foot positions for a body pose in tilt mode, and whether the legs can take it up safely
 * @param[in] pose The body pose relative to the planted feet
 * @param[out] targets The foot position of each leg (leg frame)
 * @return true if every foot stays within maxStride of neutral and every joint is within reach
*/
bool GaitEngine::tiltFeasible(const BodyPose& pose, std::array<Vec3, kMaxLegs>& targets) {
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        targets[legNum] = trajectoryGen.posedFoot(legNum, tilt_ground_feet[legNum], pose);
        const Vec3 offset = targets[legNum] - config().startPosition.at(legNum);
        if (std::hypot(offset.x(), offset.y()) > config().maxStride) {
            return false;
        }
        if (!reachable(legNum, targets[legNum])) {
            return false;
        }
    }
    // The centre of mass stays over the feet, judged in the frame the body stood in when tilt mode
    // began (the feet don't move; the body is posed above them)
    const Vec3 centre = trajectoryGen.unposedBodyPoint(com, pose);
    std::array<Vec2, kMaxLegs> feet;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        feet[legNum] = trajectoryGen.legToBody(legNum, tilt_ground_feet[legNum]).head<2>();
    }
    return stabilityMargin(feet.data(), numLegs(), centre.head<2>()) >= p.min_stability_margin;
}

/**
 * @brief Spread the robot's weight evenly between its planted feet
 * @param[in] dt The time since the last tick (s)
 * @note Each planted foot carrying more than its share is raised, and each carrying less lowered,
 *       at load_balance_gain per newton, with four or more feet down. Each change is stripped of
 *       any part that would raise or tilt the body (a plane fitted through the planted feet). Only
 *       while standing still; walking, the offsets are folded into each foot's terrain height.
*/
void GaitEngine::updateLoadBalance(Scalar dt) {
    constexpr Scalar MAX_RATE = S(0.02);  // m/s, fastest a foot is raised or lowered
    const Scalar maxStep = MAX_RATE * dt;

    const bool standing = std::abs(current_motion_intent.forward) < S(1e-3) &&
        std::abs(current_motion_intent.left) < S(1e-3) &&
        std::abs(current_motion_intent.yaw) < S(1e-3);  // tilt mode counts: its feet stay planted
    if (!standing && p.terrain_adaptation) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            terrain_offset[legNum] = std::clamp(terrain_offset[legNum] + foot_height_offset[legNum],
                -p.max_step_height, p.max_step_height);
            foot_height_offset[legNum] = 0;
        }
        return;
    }

    std::array<Scalar, kMaxLegs> change{};
    if (!p.load_balancing || !standing || sit_state != SitState::STANDING) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            change[legNum] = -foot_height_offset[legNum];
        }
    } else {
        int planted[kMaxLegs];
        int count = 0;
        Scalar total = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            if (!isSwinging(legNum)) {
                planted[count++] = legNum;
                total += foot_forces[legNum].z();
            }
        }
        if (!has_foot_forces || count < 4 || total <= 0) {
            return;
        }
        const Scalar share = total / count;
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kMaxLegs, 1> raise(count);
        Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, kMaxLegs, 3> plane(count, 3);
        for (int i = 0; i < count; ++i) {
            raise(i) = p.load_balance_gain * dt * (foot_forces[planted[i]].z() - share);
            const Vec3 foot = trajectoryGen.neutralFootInBody(planted[i]);
            plane.row(i) << 1, foot.x(), foot.y();
        }
        // Remove the best-fit plane through the planted feet
        raise -= plane * plane.colPivHouseholderQr().solve(raise);
        for (int i = 0; i < count; ++i) {
            change[planted[i]] = raise(i);
        }
    }
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        foot_height_offset[legNum] = std::clamp(
            foot_height_offset[legNum] + std::clamp(change[legNum], -maxStep, maxStep),
            -p.load_balance_max, p.load_balance_max);
    }
}

/**
 * @brief Update the leveling correction used while walking
 * @param[in] dt The time since the last tick (s)
 * @note The correction builds up the IMU's measured roll and pitch, so the body levels out on a
 *       slope, rotating about the ground point under the body. It changes at most 0.5 rad/s and
 *       only as far as every foot can still reach. Off it unwinds; without fresh IMU data it holds.
*/
void GaitEngine::updateWalkLeveling(Scalar dt) {
    constexpr Scalar MAX_RATE = S(0.5);  // rad/s
    const Scalar maxStep = MAX_RATE * dt;

    Scalar roll = walk_level_roll;
    Scalar pitch = walk_level_pitch;
    if (!p.walk_leveling || sit_state != SitState::STANDING) {
        roll = 0;
        pitch = 0;
    } else if (imuFresh()) {
        roll -= p.walk_level_gain * dt * body_roll;
        pitch -= p.walk_level_gain * dt * body_pitch;
    }
    roll = std::clamp(roll, -p.walk_max_angle, p.walk_max_angle);
    pitch = std::clamp(pitch, -p.walk_max_angle, p.walk_max_angle);
    roll = walk_level_roll + std::clamp(roll - walk_level_roll, -maxStep, maxStep);
    pitch = walk_level_pitch + std::clamp(pitch - walk_level_pitch, -maxStep, maxStep);

    // Scale back towards level until every foot is reachable (level always is)
    const Vec3 pivot(0, 0, config().currentHeight);
    auto fits = [&](Scalar scale) {
        const BodyPose pose = TrajectoryGenerator::rotationAbout(roll * scale, pitch * scale, pivot);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            if (!reachable(legNum, trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], pose))) {
                return false;
            }
        }
        return true;
    };
    if (!fits(1)) {
        Scalar low = 0, high = 1;
        for (int i = 0; i < 10; ++i) {
            const Scalar mid = S(0.5) * (low + high);
            (fits(mid) ? low : high) = mid;
        }
        roll *= low;
        pitch *= low;
    }
    walk_level_roll = roll;
    walk_level_pitch = pitch;
    walk_level = TrajectoryGenerator::rotationAbout(roll, pitch, pivot);
}

/**
 * @brief The feet on the ground this tick (body frame, x and y)
 * @param[out] feet Each planted foot
 * @param[in] staying true: only those that stay down when this phase's legs lift (for the lift gate).
 *            false: every foot not mid-swing (at a phase's start the next legs are still down)
 * @return How many
*/
int GaitEngine::plantedFeet(std::array<Vec2, kMaxLegs>& feet, bool staying) const {
    const bool contact = contactPolygon();
    int count = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        bool up = staying ? leg_swinging[legNum] : (leg_swinging[legNum] && phase_progress > 0);
        if (contact) {
            // A foot commanded down but hanging over a gap holds nothing up
            up = up || !foot_contact[legNum];
        }
        if (!up) {
            const Vec3& foot = contact ? measured_leg_positions[legNum] : current_leg_positions[legNum];
            feet[count++] = trajectoryGen.legToBody(legNum, foot).head<2>();
        }
    }
    return count;
}

/**
 * @brief Whether the support polygon is the feet in contact, rather than those commanded down
 * @return true with balance_feedback on, foot forces that can be trusted, and at least three of the
 *         feet commanded down in contact: fewer can't make a polygon, which would leave nothing the
 *         body could do, so it's more likely the force estimates than the robot balancing on two feet
*/
bool GaitEngine::contactPolygon() const {
    if (!p.balance_feedback || !has_foot_forces) {
        return false;
    }
    Scalar total = 0;
    int touching = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        total += std::max(S(0.0), foot_forces[legNum].z());
        const bool down = !(leg_swinging[legNum] && phase_progress > 0);
        touching += down && foot_contact[legNum] ? 1 : 0;
    }
    return total > 3 * p.contact_force_on && touching >= 3;
}

/**
 * @brief Where the feet are measured pushing from: each foot weighted by its vertical force
 * @param[out] cop The centre of pressure (body frame, x and y)
 * @return false without foot forces to trust
*/
bool GaitEngine::centreOfPressure(Vec2& cop) const {
    if (!contactPolygon()) {
        return false;
    }
    Vec2 moment = Vec2::Zero();
    Scalar total = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const Scalar force = std::max(S(0.0), foot_forces[legNum].z());
        moment += force * trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]).head<2>();
        total += force;
    }
    cop = moment / total;
    return true;
}

/**
 * @brief Walk at the commanded velocity no faster than the feet can push for
 * @param[in] dt The time since the last tick (s)
 * @note With dynamic_stability on, the velocity walked (applied_intent) moves towards the command,
 *       capped at the gait's top speed, by as much of the change as keeps the ZMP inside the planted
 *       feet: accelerating at a moves it a / omega^2 behind the centre of mass, so the most the body
 *       can take is omega^2 times the spare margin behind it. While a lift is held it slows to the
 *       speed the feet left down can stop it from, and while waiting for the body to sway it slows to a stop. The result replaces current_motion_intent, so idling, stride and phase length
 *       all follow the velocity actually walked. Off, it walks the command as before.
*/
void GaitEngine::limitAcceleration(Scalar dt) {
    requested_intent = current_motion_intent;
    // Capped at the gait's top speed, as walk() caps it
    MotionIntent wanted = current_motion_intent;
    if (p.dynamic_stability && sway_holding) {
        wanted = MotionIntent{};   // waiting for the body to sway over the feet: come to a stop
    } else if (p.dynamic_stability && lift_held) {
        // Down to the speed whose capture point the feet left down can take
        wanted.forward = applied_intent.forward * lift_speed_scale;
        wanted.left = applied_intent.left * lift_speed_scale;
        wanted.yaw = applied_intent.yaw * lift_speed_scale;
    }
    const Scalar footSpeed = std::hypot(wanted.forward, wanted.left) + std::abs(wanted.yaw) * trajectoryGen.maxBodyRadius();
    const Scalar maxSpeed = maxFootSpeed();
    if (footSpeed > maxSpeed) {
        const Scalar scale = maxSpeed / footSpeed;
        wanted.forward *= scale;
        wanted.left *= scale;
        wanted.yaw *= scale;
    }
    if (!p.dynamic_stability || !(dt > 0)) {
        applied_intent = wanted;   // only reported (balanceState): walk() caps the command itself
        return;
    }
    // Just back from tilt mode or sitting, the pendulum has no state yet: take it from here
    if (!(balance_controller.state().omega > 0)) {
        updateBalance(dt, 0);
    }
    const Scalar dForward = wanted.forward - applied_intent.forward;
    const Scalar dLeft = wanted.left - applied_intent.left;
    const Scalar dYaw = wanted.yaw - applied_intent.yaw;
    // The change at the centre of mass: turning faster carries it round the body origin
    const Vec2 c = com.head<2>() + sway + correction;
    const Vec2 acceleration(dForward - dYaw * c.y(), dLeft + dYaw * c.x());
    std::array<Vec2, kMaxLegs> feet;
    const int count = plantedFeet(feet, false);
    const Scalar scale = balance_controller.accelerationScale(acceleration / dt, feet.data(), count, p.min_stability_margin);
    applied_intent.forward += scale * dForward;
    applied_intent.left += scale * dLeft;
    applied_intent.yaw += scale * dYaw;
    current_motion_intent = applied_intent;
}

/**
 * @brief The lift gate: whether this phase's legs must wait before lifting
 * @param[in] dt The time since the last tick (s)
 * @return true to hold them down this tick
 * @note Only at a phase's start, with dynamic_stability on: the capture point and the ZMP must be
 *       min_stability_margin inside the feet that stay down (see BalanceController::canLift), so the
 *       polygon left after the lift can still hold the body's momentum. While held, the body slows
 *       (limitAcceleration) to the speed whose capture point fits those feet (velocityScale), no
 *       further, which brings both back over them. Gives up after a few phases' length, with a warning.
*/
bool GaitEngine::holdLift(Scalar dt) {
    if (!p.dynamic_stability || phase_progress > 0) {
        lift_hold_time = 0;
        return false;
    }
    std::array<Vec2, kMaxLegs> feet;
    const int count = plantedFeet(feet, true);
    if (balance_controller.canLift(feet.data(), count, p.min_stability_margin)) {
        lift_hold_time = 0;
        return false;
    }
    lift_speed_scale = balance_controller.velocityScale(feet.data(), count, p.min_stability_margin);
    lift_hold_time += dt;
    const Scalar maxHold = 3 * p.phase_duration;
    if (lift_hold_time > maxHold) {
        emitThrottled(LOG_LIFT_HOLD_TIMEOUT, 5, LogLevel::Warn,
            "Legs held down %.1f s waiting for the body's momentum to fit the feet that stay down "
            "(capture point %.0f mm, ZMP %.0f mm inside all the feet): lifting anyway",
            static_cast<double>(lift_hold_time), static_cast<double>(balance_controller.state().captureMargin * 1000),
            static_cast<double>(balance_controller.state().zmpMargin * 1000));
        return false;
    }
    return true;
}

/**
 * @brief Update the centre of mass and the inverted pendulum after a walking tick
 * @param[in] dt The time since the last tick (s)
 * @param[in] moveScale How much of this tick the body moved at applied_intent (moveDt / dt): 0
 *            while idle or holding for sway
 * @note Always runs while walking, for balanceState(); dynamic_stability only decides whether the
 *       gait acts on it.
*/
void GaitEngine::updateBalance(Scalar dt, Scalar moveScale) {
    com = liveCentreOfMass();
    std::array<Vec2, kMaxLegs> feet;
    const int count = plantedFeet(feet, false);
    Scalar footHeight = 0;
    int heights = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!(leg_swinging[legNum] && phase_progress > 0)) {
            footHeight += trajectoryGen.legToBody(legNum, current_leg_positions[legNum]).z();
            ++heights;
        }
    }
    footHeight = heights > 0 ? footHeight / static_cast<Scalar>(heights) : config().currentHeight;
    const Vec2 offset = sway + correction;
    const Vec3 centre(com.x() + offset.x(), com.y() + offset.y(), com.z());
    const Vec2 offsetRate = has_previous_com && dt > 0 ? Vec2((offset - previous_offset) / dt) : Vec2::Zero();
    const Vec2 drift = has_previous_com && dt > 0 ? Vec2((com - previous_com).head<2>() / dt) : Vec2::Zero();
    previous_com = com;
    previous_offset = offset;
    has_previous_com = true;
    // What the gait commanded the body to do over the ground: how the feet that stayed down moved
    // under the joints it sent, fitted as a rigid motion exactly as leg odometry does with the joints
    // reported, so the two only differ by what really happened. That counts whatever the velocity
    // alone doesn't: feet held at the edge of their stride, joint speed limits, a leg that can't reach
    // its target, and the sway and correction moving the body over the feet.
    Vec2 overGround = moveScale * Vec2(applied_intent.forward - applied_intent.yaw * centre.y(),
                                       applied_intent.left + applied_intent.yaw * centre.x()) + offsetRate;
    std::array<Vec2, kMaxLegs> before, after;
    int stayed = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const bool down = !(leg_swinging[legNum] && phase_progress > 0);
        const Vec3 bodyFoot = trajectoryGen.legToBody(legNum, kinematics(legNum).fk(output.joints[legNum]));
        const Vec2 foot = trajectoryGen.unposedBodyPoint(bodyFoot, walk_level).head<2>();
        if (down && previous_command_planted[legNum] && has_previous_command_feet) {
            before[stayed] = previous_command_feet[legNum];
            after[stayed] = foot;
            ++stayed;
        }
        previous_command_feet[legNum] = foot;
        previous_command_planted[legNum] = down;
    }
    has_previous_command_feet = true;
    BodyMotion motion;
    if (dt > 0 && LegOdometry::estimateMotion(before.data(), after.data(), static_cast<size_t>(stayed), motion)) {
        const Scalar yawRate = motion.dyaw / dt;
        overGround = Vec2(motion.dx / dt - yawRate * centre.y(), motion.dy / dt + yawRate * centre.x());
    }
    // The controller adds the centre of mass moving relative to the body origin (c includes the sway
    // and correction), so it's given the body's own motion without them
    balance_controller.update(dt, centre, overGround - offsetRate, feet.data(), count, footHeight, p.com_acceleration_filter);
    updateFeedback(dt, overGround, drift, feet, count);
}

/**
 * @brief Balance feedback, one walking tick: measure, correct the capture point, decide on a step
 * @param[in] dt The time since the last tick (s)
 * @param[in] commandedOverGround The body's commanded velocity over the ground at the centre of mass
 *            this tick, sway and correction included (m/s): what leg odometry would measure if the
 *            robot did exactly as told
 * @param[in] drift How fast the centre of mass moved within the body this tick (the legs, m/s)
 * @param[in] feet The support polygon this tick
 * @param[in] count How many feet
 * @note The measured c dot is the body's velocity over the ground (leg odometry, which sees the
 *       sway and the correction moving the body over its feet, blended with the IMU) plus the centre
 *       of mass moving within the body (the legs). The commanded c dot counts the same sway and
 *       correction, so the error is only what the feet didn't command: a push, a slip, tipping.
 *
 *       The correction velocity heads for -capture_gain x omega x error, plus a slow return to
 *       centre, changing no faster than the feet can push for (accelerationScale). The correction
 *       integrates it, up to correction_max. A step is needed when the predicted capture point is
 *       short of min_stability_margin inside the feet, or the correction can't go any further.
*/
void GaitEngine::updateFeedback(Scalar dt, const Vec2& commandedOverGround, const Vec2& drift,
                                const std::array<Vec2, kMaxLegs>& feet, int count) {
    constexpr Scalar RETURN_TIME = S(1.0);   // s, how fast an unneeded correction eases back
    if (!p.balance_feedback || !(dt > 0)) {
        velocity_estimator.reset();
        correction_velocity = Vec2::Zero();
        correction_saturated = false;
        step_needed = false;
        return;
    }

    // The body over the ground at the centre of mass. Leg odometry measures joint states, so it's
    // a command or more behind: what it adds is how far the body's motion then differed from what was
    // commanded then (a push, a slip), carried onto this tick's command. With no walking command
    // from then (just back from tilt mode or sitting, whose motion was commanded too), it waits.
    const Vec2 c = com.head<2>() + sway + correction;
    const Vec2& commandedNow = commandedOverGround;
    command_history[command_next] = CommandRecord{now_, commandedNow};
    command_next = (command_next + 1) % static_cast<int>(command_history.size());
    command_count = std::min(command_count + 1, static_cast<int>(command_history.size()));
    Vec2 reference = commandedNow;
    if (has_odom_velocity && toSeconds(now_ - odom_velocity_stamp) < 0.1) {
        const CommandRecord* then = nullptr;   // the latest command at or before the joint state
        for (int i = 0; i < command_count; ++i) {
            const CommandRecord& record = command_history[i];
            if (record.time <= odom_velocity_stamp && (then == nullptr || record.time > then->time)) {
                then = &record;
            }
        }
        if (then != nullptr) {
            const Vec2 measured = odom_velocity + odom_yaw_rate * Vec2(-c.y(), c.x());
            reference = commandedNow + (measured - then->velocity);
        }
    }
    BodyVelocityInputs in;
    in.dt = dt;
    in.reference = reference;
    // An accelerometer always reads about 1 g standing still: a reading without it (all zeros, from a
    // robot that doesn't pass it on) is no reading
    in.imuFresh = imuFresh() && body_linear_acceleration.norm() > BodyVelocityEstimator::GRAVITY / 2;
    in.roll = body_roll;
    in.pitch = body_pitch;
    in.angularVelocity = body_angular_velocity;
    in.linearAcceleration = body_linear_acceleration;
    in.comHeight = balance_controller.state().height;
    velocity_estimator.update(in, p.velocity_filter);

    Vec2 cop;
    const bool hasCop = centreOfPressure(cop);
    balance_controller.measure(velocity_estimator.velocity() + drift, hasCop ? &cop : nullptr, feet.data(), count,
                               p.balance_delay);

    // What counts as a push: an error beyond the deadband and beyond the gait's own usual wobble
    // (a trotting body rocks, soft servos lag), which the running RMS learns over a couple of seconds
    constexpr Scalar NOISE_TIME = S(2.0);   // s
    const BalanceState& s = balance_controller.state();
    const Scalar threshold = p.capture_deadband + 3 * std::sqrt(capture_noise);
    capture_noise += (dt / (NOISE_TIME + dt)) * (s.captureError.squaredNorm() - capture_noise);
    // Until it has watched the gait for that long it doesn't know its wobble: it only learns (a heavy
    // robot's joints lagging as it sets off would otherwise all look like a push)
    feedback_time += dt;
    const bool learning = feedback_time < NOISE_TIME;

    // Shift the body to put the capture point back. Only for a gait whose feet always make a polygon:
    // a trot's pair can't push from anywhere but their line, so a shift does nothing for it (the
    // moments its four feet are down don't count). Without area just now, it eases back too.
    const bool area = gaitHasSupportArea(config().currentGait) && s.supportArea && !learning;
    const Vec2 push = area ? balance_controller.correctionVelocity(p.capture_gain, threshold) : Vec2::Zero();
    const Vec2 wanted = push - correction / RETURN_TIME;
    const Vec2 change = wanted - correction_velocity;
    const Scalar scale = balance_controller.accelerationScale(change / dt, feet.data(), count, p.min_stability_margin);
    correction_velocity += scale * change;
    correction += correction_velocity * dt;
    correction_saturated = scale < 1;
    if (correction.norm() > p.correction_max) {
        correction *= p.correction_max / correction.norm();
        correction_velocity = Vec2::Zero();
        correction_saturated = true;
    }

    // Too far for the shift to catch: step where the capture point is heading. Not with feet in a
    // line, as a trot's: that gait balances by stepping already, and placing its feet from its
    // dynamics needs them planned (preview control).
    step_needed = p.capture_stepping && area && s.captureError.norm() > threshold &&
        (s.predictedCaptureMargin < p.min_stability_margin || correction_saturated);
    if (step_needed) {
        // The swing lands when this phase ends, or a whole (fast) phase from now if none has begun
        const Scalar remaining = phase_progress > 0 ? (1 - phase_progress) * current_phase_duration : p.min_phase_duration;
        Vec2 wantedStep = balance_controller.touchdownOffset(remaining);
        if (wantedStep.norm() > config().maxStride) {
            wantedStep *= config().maxStride / wantedStep.norm();
        }
        if (wantedStep.norm() > step_offset.norm()) {
            step_offset = wantedStep;   // the furthest it has needed this step
        }
    }
    const bool wasCatching = catching_step;
    catching_step = step_needed || (catching_step && phase_progress > 0);
    if (catching_step && !wasCatching) {
        emit(LogLevel::Info, "Capture point %.0f mm off: stepping %.0f mm to catch the robot",
            static_cast<double>(s.captureError.norm() * 1000), static_cast<double>(step_offset.norm() * 1000));
    }
}

/**
 * @brief Forget the walking motion: tilt mode and sitting start and end at rest
*/
void GaitEngine::resetBalance() {
    // The correction eases back like the sway does, rather than jumping the feet
    const Vec2 back = -correction;
    const Scalar maxStep = p.sway_speed * S(0.01);
    correction += back.norm() > maxStep ? Vec2(back * (maxStep / back.norm())) : back;
    correction_velocity = Vec2::Zero();
    correction_saturated = false;
    capture_noise = 0;
    feedback_time = 0;
    catching_step = false;
    step_needed = false;
    step_offset = Vec2::Zero();
    velocity_estimator.reset();
    has_previous_com = false;
    command_count = 0;
    command_next = 0;
    has_previous_command_feet = false;
    balance_controller.reset();
    applied_intent = MotionIntent{};
    requested_intent = MotionIntent{};
    lift_held = false;
    lift_hold_time = 0;
    lift_speed_scale = 1;
}

/**
 * @brief Tilt mode: hold the feet where they are and move the body to the requested pose
 * @param[in] dt The time since the last tick (s)
 * @param[in] exiting true once another mode is selected, to ease the body back to neutral
 * @note On entry the legs step back to their start positions first. The requested pose is rate
 *       limited, then the IMU leveling correction is added, so the body holds its angles in space
 *       when the ground beneath it tilts or turns. A pose the legs can't reach is cut back to the
 *       furthest safe pose towards it.
*/
void GaitEngine::tilt(Scalar dt, bool exiting) {
    current_motion_intent = MotionIntent{};

    if (!tilt_active) {
        // Plant every foot at its start position, so the body has the full range to move in
        if (!idleReturning && !feetAtStart()) {
            idleReturning = true;
            phase_progress = 0;
            leg_swinging.fill(false);
        }
        if (idleReturning) {
            returnToStart(dt);
            return;
        }
        // Take over from walk leveling: the feet as actually placed become tilt mode's neutral
        tilt_saved_roll = walk_level_roll;
        tilt_saved_pitch = walk_level_pitch;
        tilt_saved_feet = current_leg_positions;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            tilt_ground_feet[legNum] = trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], walkPose());
        }
        current_leg_positions = tilt_ground_feet;
        walk_level = BodyPose{};
        tilt_saved_sway = sway;   // the feet above were captured with it: tilt starts from there
        sway = Vec2::Zero();
        body_pose_request = BodyPose{};
        tilt_command = BodyPose{};
        tilt_correction = BodyPose{};
        tilt_applied = BodyPose{};
        tilt_yaw_reference = body_yaw;
        tilt_active = true;
        emit(LogLevel::Info, "Tilt mode: feet planted, body follows body_pose%s",
            p.tilt_leveling ? ", held in place by the IMU" : "");
    }

    // Move the command towards the request (or neutral when leaving) at a limited rate
    auto approach = [](Scalar& value, Scalar goal, Scalar maxStep) {
        value += std::clamp(goal - value, -maxStep, maxStep);
    };
    const BodyPose neutralPose{};
    const BodyPose& goal = exiting ? neutralPose : body_pose_request;
    const Scalar angleStep = p.tilt_angle_rate * dt;
    approach(tilt_command.roll, goal.roll, angleStep);
    approach(tilt_command.pitch, goal.pitch, angleStep);
    approach(tilt_command.yaw, goal.yaw, angleStep);
    for (int i = 0; i < 3; ++i) {
        approach(tilt_command.translation[i], goal.translation[i], p.tilt_translation_rate * dt);
    }

    // Leveling: integrate the error between the commanded and measured angles. Without fresh IMU
    // data the correction holds, and when leaving or with leveling off it unwinds.
    const BodyPose previousCorrection = tilt_correction;
    if (exiting || !p.tilt_leveling) {
        approach(tilt_correction.roll, 0, angleStep);
        approach(tilt_correction.pitch, 0, angleStep);
        approach(tilt_correction.yaw, 0, angleStep);
    } else if (imuFresh()) {
        const Scalar gain = p.tilt_level_gain * dt;
        tilt_correction.roll += gain * (tilt_command.roll - body_roll);
        tilt_correction.pitch += gain * (tilt_command.pitch - body_pitch);
        if (p.tilt_lock_yaw || controller_tilt_active) {
            tilt_correction.yaw += gain * std::remainder(tilt_command.yaw - (body_yaw - tilt_yaw_reference), 2 * kPi);
        }
    }
    // The command plus correction stays within tilt_max_angle
    tilt_correction.roll = std::clamp(tilt_correction.roll, -p.tilt_max_angle - tilt_command.roll, p.tilt_max_angle - tilt_command.roll);
    tilt_correction.pitch = std::clamp(tilt_correction.pitch, -p.tilt_max_angle - tilt_command.pitch, p.tilt_max_angle - tilt_command.pitch);
    tilt_correction.yaw = std::clamp(tilt_correction.yaw, -p.tilt_max_angle - tilt_command.yaw, p.tilt_max_angle - tilt_command.yaw);

    BodyPose desired;
    desired.roll = tilt_command.roll + tilt_correction.roll;
    desired.pitch = tilt_command.pitch + tilt_correction.pitch;
    desired.yaw = tilt_command.yaw + tilt_correction.yaw;
    desired.translation = tilt_command.translation;

    // Step from the last pose (always safe) towards the desired one, never jumping
    constexpr Scalar MAX_POSE_ANGLE_SPEED = S(2.0);   // rad/s
    constexpr Scalar MAX_POSE_SHIFT_SPEED = S(0.2);   // m/s
    const Scalar maxAngle = MAX_POSE_ANGLE_SPEED * dt;
    const Scalar maxShift = MAX_POSE_SHIFT_SPEED * dt;
    BodyPose step = tilt_applied;
    approach(step.roll, desired.roll, maxAngle);
    approach(step.pitch, desired.pitch, maxAngle);
    approach(step.yaw, desired.yaw, maxAngle);
    for (int i = 0; i < 3; ++i) {
        approach(step.translation[i], desired.translation[i], maxShift);
    }

    std::array<Vec3, kMaxLegs> targets;
    targets.fill(Vec3::Zero());
    BodyPose pose = step;
    if (!tiltFeasible(pose, targets)) {
        // Out of reach: ease yaw back towards zero instead, since turning swings the feet furthest
        BodyPose yawEased = step;
        yawEased.yaw = tilt_applied.yaw;
        approach(yawEased.yaw, 0, maxAngle);
        if (tiltFeasible(yawEased, targets)) {
            pose = yawEased;
            tilt_correction.yaw = previousCorrection.yaw;  // stop it winding up against the limit
        } else {
            // Go as far along the step as is safe, possibly nowhere
            Scalar low = 0, high = 1;
            for (int i = 0; i < 10; ++i) {
                const Scalar mid = S(0.5) * (low + high);
                BodyPose trial;
                trial.roll = tilt_applied.roll + mid * (step.roll - tilt_applied.roll);
                trial.pitch = tilt_applied.pitch + mid * (step.pitch - tilt_applied.pitch);
                trial.yaw = tilt_applied.yaw + mid * (step.yaw - tilt_applied.yaw);
                trial.translation = tilt_applied.translation + mid * (step.translation - tilt_applied.translation);
                (tiltFeasible(trial, targets) ? low : high) = mid;
            }
            pose.roll = tilt_applied.roll + low * (step.roll - tilt_applied.roll);
            pose.pitch = tilt_applied.pitch + low * (step.pitch - tilt_applied.pitch);
            pose.yaw = tilt_applied.yaw + low * (step.yaw - tilt_applied.yaw);
            pose.translation = tilt_applied.translation + low * (step.translation - tilt_applied.translation);
            tilt_correction = previousCorrection;
        }
        tiltFeasible(pose, targets);
        emitThrottled(LOG_TILT_LIMITED, 2, LogLevel::Warn, "Tilt pose limited by the legs' reach");
    }
    tilt_applied = pose;
    sendFootTargets(targets, dt);

    // Back at neutral after leaving tilt mode: hand over to walking
    const bool neutral = std::abs(pose.roll) < S(1e-4) && std::abs(pose.pitch) < S(1e-4) &&
        std::abs(pose.yaw) < S(1e-4) && pose.translation.norm() < S(1e-4);
    if (exiting && neutral) {
        tilt_active = false;
        tilt_applied = BodyPose{};
        // Hand back to walk leveling where it left off; the feet are where it had them
        walk_level_roll = tilt_saved_roll;
        walk_level_pitch = tilt_saved_pitch;
        walk_level = TrajectoryGenerator::rotationAbout(walk_level_roll, walk_level_pitch,
            Vec3(0, 0, config().currentHeight));
        current_leg_positions = tilt_saved_feet;
        sway = tilt_saved_sway;
        phase = 0;
        phase_progress = 0;
        emit(LogLevel::Info, "Tilt mode finished, body back to neutral");
    }
}

}  // namespace gait_core
