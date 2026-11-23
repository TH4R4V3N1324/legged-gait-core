#ifndef GAIT_CORE__BODY_VELOCITY_HPP
#define GAIT_CORE__BODY_VELOCITY_HPP

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief One tick's measurements for BodyVelocityEstimator, in the body frame
*/
struct BodyVelocityInputs {
    Scalar dt = 0;                           // s since the last update
    Vec2 reference = Vec2::Zero();           // m/s, the slow, drift-free velocity: leg odometry (or the
                                             // command, without it) over the ground at the centre of mass
    bool imuFresh = false;                   // false: the estimate is the reference itself
    Scalar roll = 0;                         // rad, from the IMU
    Scalar pitch = 0;
    Vec3 angularVelocity = Vec3::Zero();     // rad/s
    Vec3 linearAcceleration = Vec3::Zero();  // m/s^2, gravity included, as an accelerometer reads
    Scalar comHeight = 0;                    // m, the centre of mass above the feet
};

/**
 * @brief The centre of mass's horizontal velocity over the ground, from leg odometry and the IMU
 * @note A complementary filter. The IMU is fast but drifts: its acceleration, with gravity taken
 *       out (using roll and pitch), is integrated each tick. Leg odometry doesn't drift but only sees
 *       the feet move relative to the body, so it misses the whole robot sliding or tipping. The
 *       estimate is pulled towards the reference (odometry, plus the centre of mass tipping about the
 *       feet, which the gyro measures as w x (0, 0, h)) with time constant `filter`: slower changes
 *       come from it, faster ones (pushes, slips) from the accelerometer. Allocates nothing.
*/
class BodyVelocityEstimator {
public:
    /**
     * @brief Update from one tick's measurements
     * @param[in] in The measurements; nothing happens unless dt is positive
     * @param[in] filter s, how long the accelerometer is trusted before the reference takes over
    */
    void update(const BodyVelocityInputs& in, Scalar filter);

    /**
     * @brief Start again: the next update takes its reference as the velocity
    */
    void reset();

    // m/s, body frame: the estimated velocity of the centre of mass over the ground
    const Vec2& velocity() const { return velocity_; }
    // m/s^2: the last tick's acceleration from the IMU, gravity removed (zero without fresh IMU data)
    const Vec2& acceleration() const { return acceleration_; }

    static constexpr Scalar GRAVITY = Scalar(9.81);   // m/s^2

private:
    Vec2 velocity_ = Vec2::Zero();
    Vec2 acceleration_ = Vec2::Zero();
    bool started_ = false;
};

}  // namespace gait_core

#endif  // GAIT_CORE__BODY_VELOCITY_HPP
