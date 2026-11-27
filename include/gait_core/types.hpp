#ifndef GAIT_CORE__TYPES_HPP
#define GAIT_CORE__TYPES_HPP

#include <cstdint>
#include <Eigen/Dense>

#ifndef GAIT_CORE_MAX_LEGS
#define GAIT_CORE_MAX_LEGS 6
#endif

static_assert(GAIT_CORE_MAX_LEGS > 0, "GAIT_CORE_MAX_LEGS must be a positive integer");

#ifndef GAIT_CORE_MAX_JOINTS
#define GAIT_CORE_MAX_JOINTS 6
#endif

static_assert(GAIT_CORE_MAX_JOINTS > 0, "GAIT_CORE_MAX_JOINTS must be a positive integer");

#ifndef GAIT_CORE_MAX_APPENDAGES
#define GAIT_CORE_MAX_APPENDAGES 2
#endif

static_assert(GAIT_CORE_MAX_APPENDAGES >= 0, "GAIT_CORE_MAX_APPENDAGES must be zero or a positive integer");

/**
 * Scalar type used throughout the core. Defaults to float, which the ESP32 has hardware for and
 * which is much cheaper than double on the RP2040 (no FPU at all). The ROS build sets
 * GAIT_CORE_USE_DOUBLE so its maths matches what it was before the split.
*/
#ifndef GAIT_CORE_SCALAR
#define GAIT_CORE_SCALAR float
#endif

namespace gait_core {

using Scalar = GAIT_CORE_SCALAR;

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using AngleAxis = Eigen::AngleAxis<Scalar>;
using Rotation2D = Eigen::Rotation2D<Scalar>;

// M_PI is not standard C++ (MSVC and some embedded toolchains hide it)
inline constexpr Scalar kPi = static_cast<Scalar>(3.14159265358979323846);

// The maximum number of joints per leg, which is used to size fixed-size Eigen matrices. The
// actual number of joints per leg is a runtime parameter, but it must be <= this value
inline constexpr int kMaxJointsPerLeg = GAIT_CORE_MAX_JOINTS;

// The maximum number of legs, which sizes every per-leg array. The robot's actual leg count
// (RobotDefinition::numLegs) is a runtime value, but it must be <= this value
inline constexpr int kMaxLegs = GAIT_CORE_MAX_LEGS;

// The most appendages (heads, arms: see AppendageDefinition) a robot can have, which sizes their arrays
inline constexpr int kMaxAppendages = GAIT_CORE_MAX_APPENDAGES;

// The most ground contact points a support polygon is built from: four corners for each foot with
// a sole (LegDefinition::soleMin, soleMax), one for each point foot
inline constexpr int kMaxContactPoints = 4 * kMaxLegs;

using JointVec = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::ColMajor, kMaxJointsPerLeg, 1>;
using LegJacobian = Eigen::Matrix<Scalar, 3, Eigen::Dynamic, Eigen::ColMajor, 3, kMaxJointsPerLeg>;
// The foot's linear (rows 0-2) and angular (rows 3-5) velocity for each joint: what a foot with a
// sole needs, as the ground can push on it with a moment as well as a force
using LegJacobian6 = Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Eigen::ColMajor, 6, kMaxJointsPerLeg>;

/**
 * @brief A point in time, in nanoseconds from any fixed origin (ROS time, or micros() * 1000)
*/
using Timestamp = int64_t;

/**
 * @brief Length of a time interval in seconds
*/
inline double toSeconds(Timestamp interval) {
    return static_cast<double>(interval) / 1e9;
}

/**
 * @brief How serious a log message is; each robot maps these onto its own logging
*/
enum class LogLevel : uint8_t { Debug, Info, Warn, Error };

/**
 * @brief Commanded body velocity, in the body frame
 * @note SI units, REP 103 body frame: +x forward, +y left, +z up, yaw positive anticlockwise.
 *       Replaces geometry_msgs/Twist inside the core; the ROS node copies Twist into it.
*/
struct VelocityCommand {
    Scalar vx = 0;    // m/s, forward
    Scalar vy = 0;    // m/s, left
    Scalar wz = 0;    // rad/s, anticlockwise
};

}  // namespace gait_core

#endif  // GAIT_CORE__TYPES_HPP
