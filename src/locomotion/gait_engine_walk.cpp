// GaitEngine: stepping and walking. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

/**
 * @brief Move every leg along its swing or stance spline
 * @param[in] dt The time since the last tick (s)
 * @param[in] idle If true, the step doesn't advance
 * @param[in] handlePhaseTransition If false, phase transitions are left to the caller
*/
void GaitEngine::performLegStep(Scalar dt, bool idle, bool handlePhaseTransition) {
    Scalar ds = idle ? S(0.0) : dt / current_phase_duration;
    std::array<Vec3, kMaxLegs> targets = legTargets(ds);
    const Scalar scale = jointSpeedScale(targets, dt);
    if (scale < 1) {
        ds *= scale;
        targets = legTargets(ds);
    }

    // Advance the splines and deactivate any that have reached their end
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];
        if (spline.active) {
            spline.s = std::min(S(1.0), spline.s + ds);
            spline.active = spline.s < 1;
        } else if (stanceSpline.active) {
            stanceSpline.s = std::min(S(1.0), stanceSpline.s + ds);
            stanceSpline.active = stanceSpline.s < 1;
        }
    }

    sendFootTargets(targets, dt);

    phase_progress = std::min(S(1.0), phase_progress + ds);
    if (!handlePhaseTransition) return;

    if (phase_progress >= 1) {
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());
        phase_progress = 0;
    }
}

/**
 * @brief Switch to a requested gait, if there is one
 * @note Only call between phases, when no leg is mid-swing. Restarts at the new gait's first phase.
*/
void GaitEngine::applyPendingGait() {
    if (!config().gaitChangeRequested) {
        return;
    }
    config().gaitChangeRequested = false;
    if (config().pendingGait == config().currentGait) {
        return;
    }
    config().currentGait = config().pendingGait;
    trajectoryGen.gaitState.config = config().getGaitConfig(config().currentGait);
    phase = 0;
    phase_progress = 0;
    leg_swinging.fill(false);
    emit(LogLevel::Info, "Gait changed to %s", config().gaitName(config().currentGait));
}

bool GaitEngine::isSwinging(int legNum) const {
    return leg_swinging[legNum] || trajectoryGen.gaitState.swingSplines[legNum].active;
}

Vec3 GaitEngine::plantedStart(int legNum) {
    return config().startPosition.at(legNum) + Vec3(0, 0, terrain_offset[legNum]);
}

/**
 * @brief Whether every commanded foot is within 1 mm of its start position
*/
bool GaitEngine::feetAtStart() {
    constexpr Scalar POSITION_TOLERANCE = S(0.001);  // m
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if ((current_leg_positions[legNum] - plantedStart(legNum)).norm() > POSITION_TOLERANCE) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Step the feet back to their start positions, one gait group at a time
 * @param[in] dt The time since the last tick (s)
*/
void GaitEngine::returnToStart(Scalar dt) {
    // Standing up straight after startup comes here before the walking code has set up the gait's
    // groups, and with none the feet never stepped back in to the standing reach
    trajectoryGen.EnsureGaitConfig();

    if (phase >= trajectoryGen.gaitState.config.size()) {
        idleReturning = false;
        return_counter = 0;
        phase_progress = 0;
        return;
    }

    // Only between phases: mid-swing, a foot within the tolerance of its start is still in the air,
    // and stopping there would leave it hovering for the walk to drop in a single tick
    if (phase_progress == 0 && feetAtStart()) {
        idleReturning = false;
        return_counter = 0;
        phase_progress = 0;
        phase = 0;
        return;
    }

    if (phase_progress == 0) {
        current_phase_duration = p.phase_duration;
        // Swinging legs move to their start position, standing legs hold where they are
        std::array<Vec3, kMaxLegs> startPositions;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            startPositions[legNum] = plantedStart(legNum);
        }
        trajectoryGen.GenerateTrajectories(startPositions, current_leg_positions, current_leg_positions, phase);
    }

    // Stepping back in lifts legs too: a gait that sways waits for the body to be over the others
    planned_motion = PlannedMotion{0, 0, 0, current_phase_duration * static_cast<Scalar>(
        std::max<size_t>(1, trajectoryGen.gaitState.config.size() - 1))};
    performLegStep(dt, updateSway(dt, false), false);

    if (phase_progress >= 1) {
        return_counter++;
        phase_progress = 0;
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());

        // After all phases, finish return-to-start and handle gait change if requested
        if (return_counter >= static_cast<int>(trajectoryGen.gaitState.config.size())) {
            return_counter = 0;
            idleReturning = false;
            applyPendingGait();
        }
    }
}

/**
 * @brief Return the legs to their start positions after idle_return_delay seconds idle
 * @param[in] dt The time since the last tick (s)
 * @return true if the command is idle
*/
bool GaitEngine::handleIdleReturn(Scalar dt) {
    constexpr Scalar DEADZONE = S(0.01);

    // Idle once nothing is asked for and the body has stopped: slowed down by the gait (dynamic
    // stability), it isn't idle, nor is it while still slowing down after the command stops
    auto still = [DEADZONE](const MotionIntent& intent) {
        return std::abs(intent.forward) < DEADZONE && std::abs(intent.left) < DEADZONE && std::abs(intent.yaw) < DEADZONE;
    };
    // A step to catch the robot counts as moving too, so the feet step back to their stance after
    const bool idle = still(current_motion_intent) && still(requested_intent) && !catching_step;

    // Track if we've ever moved (received a non-zero command and started stepping)
    if (!idle && phase_progress > 0) {
        has_moved_from_start = true;
        return_completed = false;
    }

    if (idle) {
        idle_time += dt;
    } else {
        idle_time = 0;
    }

    if (idleReturning) {
        returnToStart(dt);
        if (!idleReturning) {
            return_completed = true;
            has_moved_from_start = false;  // don't re-trigger until the robot moves again
        }
        idle_time = 0;
    } else if (has_moved_from_start && !return_completed && idle_time > p.idle_return_delay) {
        idleReturning = true;
        phase_progress = 0;
        leg_swinging.fill(false);
        returnToStart(dt);
        idle_time = 0;
    }
    return idle;
}

/**
 * @brief Foot targets for one walking tick, without changing any state
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s)
 * @param[in] ds How far to advance the phase this tick (fraction of a phase)
 * @param[in] moveDt How much body motion to apply to the stance feet this tick (s)
 * @return The target foot position of each leg (leg frame)
*/
std::array<Vec3, kMaxLegs> GaitEngine::walkTargets(Scalar forward, Scalar left, Scalar yaw, Scalar ds, Scalar moveDt) {
    std::array<Vec3, kMaxLegs> targets;
    const Scalar s = std::min(S(1.0), phase_progress + ds);
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (leg_swinging[legNum]) {
            if (s >= 1) {
                targets[legNum] = swing_target[legNum];
                continue;
            }
            SwingSpline spline;
            trajectoryGen.genSwingSpline(spline, swing_start[legNum], swing_target[legNum],
                swing_lift[legNum] > 0 ? swing_lift[legNum] : trajectoryGen.swingLift);
            targets[legNum] = trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s);
            // A stumble lifts the foot higher, fading out over the last 30% so it still lands on target
            targets[legNum].z() += stumble_boost[legNum] * std::min(S(1.0), (1 - s) / S(0.3));
            targets[legNum] = limitSwingLift(legNum, targets[legNum],
                swing_start[legNum].z() + (swing_target[legNum].z() - swing_start[legNum].z()) * s);
        } else {
            const Vec3& current = current_leg_positions[legNum];
            targets[legNum] = trajectoryGen.stanceTarget(legNum, current,
                trajectoryGen.footDisplacementFrom(forward, left, yaw, legNum, current, moveDt));
            targets[legNum].z() = plantedStart(legNum).z();  // on its own ground
        }
    }
    return targets;
}

/**
 * @brief Walking, one tick
 * @param[in] dt The time since the last tick (s)
 * @note The command is re-read every tick: stance feet follow the current body velocity, and swing
 *       feet re-aim their landing target (at up to max_target_speed). The phase length also
 *       follows the command.
*/
void GaitEngine::walk(Scalar dt) {
    const bool velocityIdle = handleIdleReturn(dt);
    if (idleReturning) {
        // Stepping back to stance, without the balance update: start it afresh afterwards rather
        // than take the whole return for one tick's motion
        resetBalance();
        return;
    }

    trajectoryGen.EnsureGaitConfig();

    // Switch gait between phases, when every foot is planted
    if (phase_progress == 0) {
        applyPendingGait();
    }
    adaptToTerrain(dt);

    // Current command, capped at what this gait can do
    Scalar forward = current_motion_intent.forward;
    Scalar left = current_motion_intent.left;
    Scalar yaw = current_motion_intent.yaw;
    Scalar footSpeed = std::hypot(forward, left) + std::abs(yaw) * trajectoryGen.maxBodyRadius();
    const Scalar maxSpeed = maxFootSpeed();
    if (footSpeed > maxSpeed) {
        const Scalar scale = maxSpeed / footSpeed;
        forward *= scale;
        left *= scale;
        yaw *= scale;
        footSpeed = maxSpeed;
    }

    // Step faster once the stride would exceed maxStride
    const size_t groups = std::max<size_t>(2, trajectoryGen.gaitState.config.size());
    current_phase_duration = p.phase_duration;
    if (footSpeed > S(1e-6)) {
        const Scalar strideLimitedDuration = 2 * config().maxStride / ((groups - 1) * footSpeed);
        current_phase_duration = std::clamp(strideLimitedDuration, p.min_phase_duration, p.phase_duration);
    }
    if (catching_step) {
        current_phase_duration = std::min(current_phase_duration, p.min_phase_duration);   // catch it quickly
    }
    const Scalar stanceTime = (groups - 1) * current_phase_duration;

    // Start of a phase: the next group lifts off from where it stands
    if (phase_progress == 0) {
        stumble_target.fill(0);
        stumble_boost.fill(0);
        swing_lift.fill(0);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            leg_swinging[legNum] = trajectoryGen.gaitState.config.swingsIn(legNum, phase);
            swing_start[legNum] = current_leg_positions[legNum];
            swing_target[legNum] = current_leg_positions[legNum];
            trajectoryGen.gaitState.swingSplines[legNum].active = false;
            trajectoryGen.gaitState.stanceSplines[legNum].active = false;
        }
    }

    // Stumble lift ramps in at 0.5 m/s rather than jerking the foot up
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        stumble_boost[legNum] += std::clamp(stumble_target[legNum] - stumble_boost[legNum], -S(0.5) * dt, S(0.5) * dt);
    }

    // Swing landing targets follow the command, at a limited rate, slowed as much as the gait last
    // was (a target moving at full rate would carry the joints past their speed limits). Both ends
    // stay inside the stride circle, so every point in between does too.
    const Scalar maxTargetStep = p.max_target_speed * dt * walk_speed_scale;
    const bool useMap = p.terrain_map && p.terrain_adaptation && forcesValid() &&
        footholdPlanner != nullptr && footholdPlanner->ready();
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!leg_swinging[legNum]) continue;
        Vec3 displacement = trajectoryGen.footDisplacement(forward, left, yaw, legNum, stanceTime / 2);
        if (catching_step) {
            // Land where the capture point will be (still within the stride circle)
            displacement += AngleAxis(-trajectoryGen.legYaw(legNum), Vec3::UnitZ()) * Vec3(step_offset.x(), step_offset.y(), 0);
        }
        // A gait that balances by stepping lands where the body is really heading too
        const Vec2 placement = placementOffset();
        displacement += AngleAxis(-trajectoryGen.legYaw(legNum), Vec3::UnitZ()) * Vec3(placement.x(), placement.y(), 0);
        Vec3 desired = trajectoryGen.swingTarget(legNum, displacement);
        desired.z() = plantedStart(legNum).z();  // expect the ground where this foot last stood
        if (useMap) {
            footholdPlanner->plan(legNum, desired, swing_start[legNum], swing_lift[legNum]);
        }
        Vec3 step = desired - swing_target[legNum];
        if (step.norm() > maxTargetStep) {
            step *= maxTargetStep / step.norm();
        }
        swing_target[legNum] += step;
    }

    // A gait that sways waits at a phase's start, every foot down, until the body is over the feet
    // that stay down (planned ahead with preview control, which needs the command it walks with)
    planned_motion = PlannedMotion{forward, left, yaw, stanceTime};
    const bool swayHold = updateSway(dt, velocityIdle);

    // With dynamic stability, the legs also wait until the feet that stay down can hold the body's
    // momentum (the lift gate)
    lift_held = !velocityIdle && holdLift(dt);
    if (!lift_held) {
        lift_speed_scale = 1;
    }
    // Held for either, every foot stays down. Without dynamic stability the body stops where it is.
    // With it, the body keeps moving and slows (limitAcceleration) no harder than the feet can push
    // for, so the legs about to lift travel with the ground like the rest (they're picked again next tick).
    const bool held = swayHold || lift_held;
    const bool bodyStops = velocityIdle || (swayHold && !p.dynamic_stability);
    if (held && !bodyStops) {
        leg_swinging.fill(false);
    }

    // Advance, slowing the whole gait if a joint would move faster than its speed limit
    Scalar ds = (velocityIdle || held) ? S(0.0) : dt / current_phase_duration;
    Scalar moveDt = bodyStops ? S(0.0) : dt;
    auto targets = walkTargets(forward, left, yaw, ds, moveDt);
    const Scalar scale = jointSpeedScale(targets, dt);
    walk_speed_scale = scale;
    if (scale < 1) {
        ds *= scale;
        moveDt *= scale;
        targets = walkTargets(forward, left, yaw, ds, moveDt);
    }
    sendFootTargets(targets, dt);
    walk_advancing = ds > 0;
    updateBalance(dt, dt > 0 ? moveDt / dt : S(0.0));

    phase_progress = std::min(S(1.0), phase_progress + ds);
    if (phase_progress >= 1) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            // The foot is where its swing aimed: that's its ground now
            if (leg_swinging[legNum] && p.terrain_adaptation) {
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - config().startPosition.at(legNum).z(),
                    -p.max_step_height, p.max_step_height);
            }
            // A foot that has just landed takes a moment to load up: check again once the weight
            // has had time to come onto it
            if (leg_swinging[legNum] && p.terrain_adaptation && forcesValid() &&
                foot_forces[legNum].z() < p.contact_force_off) {
                landing_check[legNum] = true;
                unloaded_time[legNum] = 0;
            }
        }
        leg_swinging.fill(false);
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());
        phase_progress = 0;
        if (!step_needed) {
            step_offset = Vec2::Zero();   // that step has landed and none more is needed
        }
    }
}

/**
 * @brief Lower a swinging foot until every joint of its leg is inside its range
 * @param[in] legNum The leg
 * @param[in] target The swing's foot target (leg frame)
 * @param[in] groundZ Height of the swing's ground line (lift-off to landing) under the target
 * @return target, with its height above groundZ cut back as far as needed, but no lower than groundZ
*/
Vec3 GaitEngine::limitSwingLift(int legNum, const Vec3& target, Scalar groundZ) {
    auto fits = [&](const Vec3& foot) { return reachable(legNum, trajectoryGen.posedFoot(legNum, foot, walkPose())); };
    const Scalar lift = target.z() - groundZ;
    if (lift <= 0 || fits(target)) {
        return target;
    }
    Vec3 lowered = target;
    Scalar low = 0, high = 1;  // share of the lift kept
    for (int i = 0; i < 10; ++i) {
        const Scalar mid = S(0.5) * (low + high);
        lowered.z() = groundZ + lift * mid;
        (fits(lowered) ? low : high) = mid;
    }
    lowered.z() = groundZ + lift * low;
    emitThrottled(LOG_SWING_LOWERED, 1, LogLevel::Debug, "Leg %d swing lowered by %.0f mm to stay inside its joint limits",
        legNum + 1, static_cast<double>((1 - low) * lift * 1000));
    return lowered;
}

}  // namespace gait_core
