// Private to the GaitEngine sources: shared helpers and log keys. Not installed.

#ifndef GAIT_CORE__SRC__LOCOMOTION__GAIT_ENGINE_INTERNAL_HPP
#define GAIT_CORE__SRC__LOCOMOTION__GAIT_ENGINE_INTERNAL_HPP

#include "gait_core/locomotion/gait_engine.hpp"
#include "gait_core/locomotion/support_polygon.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

namespace gait_core {
namespace engine_internal {

constexpr Scalar S(double value) {
    return static_cast<Scalar>(value);
}

/**
 * @brief printf into a command result's message
*/
inline void setMessage(CommandResult& result, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(result.message, sizeof(result.message), fmt, args);
    va_end(args);
}

// Keys for emitThrottled
constexpr int LOG_WAITING_FOR_JOINTS = 0;
constexpr int LOG_SWING_LOWERED = 1;
constexpr int LOG_TILT_LIMITED = 2;
constexpr int LOG_INVALID_MODE = 3;
constexpr int LOG_SERVOS_OFF = 4;
constexpr int LOG_UNSUPPORTED_ROBOT = 5;
constexpr int LOG_UNSTABLE_STANCE = 6;
constexpr int LOG_SWAY_TIMEOUT = 7;
constexpr int LOG_JOINT_SPEED_LIMITED = 8;
constexpr int LOG_LIFT_HOLD_TIMEOUT = 9;
constexpr int LOG_JOINT_RANGE_LIMITED = 10;   // last_throttled_log and throttled_log_sent hold 11 keys

}  // namespace engine_internal
}  // namespace gait_core

#endif  // GAIT_CORE__SRC__LOCOMOTION__GAIT_ENGINE_INTERNAL_HPP
