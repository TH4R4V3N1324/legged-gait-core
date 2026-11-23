// GaitEngine: commands. One class over several files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

CommandResult GaitEngine::setGait(uint8_t gait) {
    CommandResult result;
    // An index past the robot's list would give an empty schedule, and phase % 0
    if (gait >= config().numGaits()) {
        setMessage(result, "Gait %d is not one of this robot's %d gaits", gait, config().numGaits());
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    config().setGait(gait);
    emit(LogLevel::Info, "Gait %d (%s) requested", gait, config().gaitName(gait));
    result.success = true;
    setMessage(result, "Gait %d (%s) requested, applies at the next step", gait, config().gaitName(gait));
    return result;
}

CommandResult GaitEngine::setMode(uint8_t mode) {
    CommandResult result;
    if (mode >= NUM_MODES) {
        setMessage(result, "Mode %d is not a valid mode", mode);
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    if (mode == MODE_TILT && sit_state != SitState::STANDING) {
        setMessage(result, "Can't enter tilt mode while sitting");
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    config().setMode(static_cast<Mode>(mode));
    result.success = true;
    setMessage(result, "Mode changed to %d", mode);
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief Glide the body to a new height
 * @param[in] height The feet's z below the hips, so negative: -0.12 for a 12 cm body height
*/
CommandResult GaitEngine::setHeight(Scalar height) {
    CommandResult result;
    const double h = static_cast<double>(height);
    if (sit_state != SitState::STANDING) {
        setMessage(result, "can't change height while sitting");
        emit(LogLevel::Warn, "Refused height %.3f: %s", h, result.message);
        return result;
    }
    if (-height < p.min_height || -height > p.max_height) {
        setMessage(result, "height must be between %f and %f m (sent as negative z)",
            static_cast<double>(p.min_height), static_cast<double>(p.max_height));
        emit(LogLevel::Warn, "Refused height %.3f: %s", h, result.message);
        return result;
    }
    target_height = height;
    result.success = true;
    setMessage(result, "moving to %f m", -h);
    emit(LogLevel::Info, "Height target %.3f m", -h);
    return result;
}

/**
 * @brief Sit down (true) or stand up (false)
 * @note Returns straight away; the move carries on in update(). A request part way through
 *       reverses the move from wherever it has got to.
*/
CommandResult GaitEngine::sit(bool down) {
    CommandResult result;
    result.success = true;
    if (down) {
        switch (sit_state) {
            case SitState::STANDING:
                if (!everyLegReaches(Vec3(p.sit_reach, 0, -p.sit_height))) {
                    result.success = false;
                    setMessage(result, "sit_reach and sit_height put a foot out of reach");
                    emit(LogLevel::Warn, "Refused to sit: %s", result.message);
                    return result;
                }
                if (!everyLegReaches(Vec3(p.sit_reach, 0, stand_height))) {
                    // e.g. a leg with two joints, whose reach and height go together
                    result.success = false;
                    setMessage(result, "a foot can't step out to sit_reach at the standing height, as sitting starts by doing");
                    emit(LogLevel::Warn, "Refused to sit: %s", result.message);
                    return result;
                }
                stand_height = target_height;
                // Tilt mode eases the body back to neutral before the sit starts
                config().setMode(MODE_NORMAL);
                config().setReach(p.sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::GATHERING:
                config().setReach(p.sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::RAISING:
                target_height = -p.sit_height;
                sit_state = SitState::LOWERING;
                break;
            case SitState::SPREADING:
            case SitState::LOWERING:
            case SitState::SEATED:
                setMessage(result, "already sitting");
                return result;
        }
        setMessage(result, "sitting down");
    } else {
        // Standing up cancels a power down; unpowered, it powers the servos and startup takes it from there
        power_down_pending = false;
        const bool poweringUp = !servo_power;
        if (poweringUp) {
            servo_power = true;
            has_joint_states = false;   // wait for joint states from the powered servos
        }
        switch (sit_state) {
            case SitState::SEATED:
            case SitState::LOWERING:
                target_height = stand_height;
                sit_state = SitState::RAISING;
                if (poweringUp) {
                    setMessage(result, "powering the servos, then standing up");
                    emit(LogLevel::Info, "%s", result.message);
                    return result;   // seated() hasn't changed: startup reports it once it stands
                }
                break;
            case SitState::SPREADING:
                config().setReach(stand_reach);
                sit_state = SitState::GATHERING;
                break;
            case SitState::STANDING:
            case SitState::RAISING:
            case SitState::GATHERING:
                setMessage(result, "already standing");
                return result;
        }
        setMessage(result, "standing up");
    }
    if (seatedChanged) {
        seatedChanged(seated());
    }
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief Sit down, then turn the servo power off once seated
 * @note A stand command before it's off cancels it; one after powers the servos back up.
*/
CommandResult GaitEngine::powerDown() {
    CommandResult result;
    if (!servo_power) {
        result.success = true;
        setMessage(result, "servos already off");
        return result;
    }
    const CommandResult sat = sit(true);
    if (!sat.success) {
        return sat;
    }
    power_down_pending = true;
    result.success = true;
    setMessage(result, sit_state == SitState::SEATED && positions_initialized
        ? "turning the servos off" : "sitting down, then turning the servos off");
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief The servos lost power without being asked (e-stop, kill loop, brownout)
 * @note Turns servo power off, so nothing moves when the power comes back until a stand command.
 *       The engine notices by itself after servo_loss_timeout without joint states; call this when
 *       the hardware knows sooner.
*/
void GaitEngine::servoPowerLost() {
    if (!servo_power) {
        return;
    }
    emit(LogLevel::Warn, "Servo power lost: send a stand command to power up and stand again");
    cutServoPower();
}

/**
 * @brief Turn servo power off and go back to how the engine starts, so the next power up runs
 *        startup again from wherever the legs are
 * @note Keeps the settings, hooks, odometry, IMU, gait choice and standing height.
*/
void GaitEngine::cutServoPower() {
    const bool wasSeated = seated();
    GaitEngine fresh(p, robot);
    fresh.log = std::move(log);
    fresh.seatedChanged = std::move(seatedChanged);
    fresh.footholdPlanner = footholdPlanner;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        // Also sizes fresh.output.joints[legNum] for an override with a different joint count
        fresh.setLegKinematics(legNum, legKinematics[legNum]);
    }
    fresh.legOdometry = legOdometry;
    fresh.body_roll = body_roll;
    fresh.body_pitch = body_pitch;
    fresh.body_yaw = body_yaw;
    fresh.last_imu_time = last_imu_time;
    fresh.has_imu = has_imu;
    fresh.body_angular_velocity = body_angular_velocity;
    fresh.body_linear_acceleration = body_linear_acceleration;
    fresh.stand_height = stand_height;
    fresh.config().setGait(config().currentGait);
    fresh.last_throttled_log = last_throttled_log;
    fresh.throttled_log_sent = throttled_log_sent;
    fresh.now_ = now_;
    fresh.servo_power = false;
    *this = std::move(fresh);
    if (!wasSeated && seatedChanged) {
        seatedChanged(true);
    }
}

/**
 * @brief The body pose wanted in tilt mode, relative to level and the heading when tilt mode started
 * @note Each axis is clamped to tilt_max_translation or tilt_max_angle. Ignored outside tilt mode.
*/
void GaitEngine::setBodyPose(Scalar roll, Scalar pitch, Scalar yaw, const Vec3& translation) {
    body_pose_request.roll = std::clamp(roll, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.pitch = std::clamp(pitch, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.yaw = std::clamp(yaw, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.translation = translation.cwiseMax(-p.tilt_max_translation).cwiseMin(p.tilt_max_translation);
}

/**
 * @brief Fastest a foot can travel relative to the body in the current gait
 * @return The speed (m/s), which is also the gait's top walking speed
 * @note A stance foot sweeps up to 2 x maxStride over (groups - 1) phases, and phases can be no
 *       shorter than min_phase_duration
*/
Scalar GaitEngine::maxFootSpeed() const {
    size_t groups = std::max<size_t>(2, config().getGaitConfig(config().currentGait).size());
    return 2 * config().maxStride / ((groups - 1) * p.min_phase_duration);
}

}  // namespace gait_core
