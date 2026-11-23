#ifndef GAIT_CORE__BALANCE_CONTROLLER_HPP
#define GAIT_CORE__BALANCE_CONTROLLER_HPP

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief The body as a linear inverted pendulum over its planted feet, one tick's worth
 * @note Body frame, x and y over the ground (the body's sway included). Margins are how far a point
 *       is inside the planted feet's polygon (negative outside), as stabilityMargin gives them.
*/
struct BalanceState {
    Vec3 com = Vec3::Zero();               // m, c: the whole robot's centre of mass
    Vec2 comVelocity = Vec2::Zero();       // m/s, c dot, relative to the planted feet
    Vec2 comAcceleration = Vec2::Zero();   // m/s^2, c double dot, filtered
    Scalar height = 0;                     // m, h: the centre of mass above the planted feet
    Scalar omega = 0;                      // 1/s, sqrt(g / h)
    Vec2 capturePoint = Vec2::Zero();      // m, xi = c + c dot / omega: where the body could come to rest
    Vec2 zmp = Vec2::Zero();               // m, c - c double dot / omega^2: where the feet must push from
    int plantedCount = 0;                  // feet on the ground
    Scalar comMargin = 0;                  // m
    Scalar captureMargin = 0;              // m
    Scalar zmpMargin = 0;                  // m
};

/**
 * @brief Capturability and the zero moment point for a legged robot, from what the gait commands
 * @note Phase A of dynamic balance: the velocity and acceleration are the gait's own command
 *       (what the body is being told to do), not measured. The centre of mass is the live one,
 *       with the legs where they are, so it moves as they swing.
 *
 *       Each tick the engine gives the centre of mass, the body's commanded velocity over the
 *       ground and the planted feet. c dot is that velocity plus how the centre of mass moved
 *       within the body (legs swinging, sway); c double dot is its rate of change, low-pass
 *       filtered. Then:
 *       - capture point xi = c + c dot / omega. Inside the feet, the robot can stop over them.
 *       - ZMP = c - c double dot / omega^2. Inside the feet, they can make this acceleration.
 *       canLift() checks both against the feet left down when a leg lifts, and
 *       accelerationScale() turns the ZMP round: accelerating by a moves the ZMP by a / omega^2,
 *       so the most the body can take is omega^2 times the spare margin behind it.
 *       Allocates nothing; a plain value type.
*/
class BalanceController {
public:
    /**
     * @brief Update from one tick of the gait
     * @param[in] dt s since the last update; nothing happens unless positive
     * @param[in] com The live centre of mass (body frame, x and y with the body's sway)
     * @param[in] bodyVelocity The body's commanded velocity over the ground at the centre of mass,
     *            this tick (m/s, body frame, turning included)
     * @param[in] feet The planted feet (body frame, x and y)
     * @param[in] count How many
     * @param[in] footHeight Their mean height (body frame z, m): h is com.z less this
     * @param[in] accelerationFilter s, time constant of the c double dot low-pass filter
    */
    void update(Scalar dt, const Vec3& com, const Vec2& bodyVelocity, const Vec2* feet, int count,
                Scalar footHeight, Scalar accelerationFilter);

    /**
     * @brief Whether the feet left down can hold the body's momentum: the capture point and the
     *        ZMP both inside them by margin
     * @param[in] feet The feet that stay down when the next legs lift (body frame, x and y)
     * @param[in] count How many
     * @param[in] margin m, how far inside they must be
     * @return true if both are at least margin inside, or no closer to the edge than the centre of
     *         mass itself is: holding a lift can only take away what the motion adds, so a body that
     *         is already short of margin standing still (whatever the reason) never waits for ever
    */
    bool canLift(const Vec2* feet, int count, Scalar margin) const;

    /**
     * @brief How much of an acceleration the feet can push for
     * @param[in] acceleration The centre of mass acceleration wanted (m/s^2, body frame)
     * @param[in] feet The planted feet (body frame, x and y)
     * @param[in] count How many
     * @param[in] margin m, how far inside them the ZMP must stay
     * @return s in [0, 1]: accelerating by s x acceleration keeps the ZMP, c - s a / omega^2, at
     *         least margin inside the feet (or, if c itself isn't, no closer to the edge than c)
    */
    Scalar accelerationScale(const Vec2& acceleration, const Vec2* feet, int count, Scalar margin) const;

    /**
     * @brief How much of the current velocity some feet can stop the body from
     * @param[in] feet The feet (body frame, x and y), e.g. those that stay down when the next legs lift
     * @param[in] count How many
     * @param[in] margin m, how far inside them the capture point must be
     * @return s in [0, 1]: at s x the current velocity, the capture point, c + s c dot / omega, is
     *         at least margin inside the feet (or, if c itself isn't, no closer to the edge than c)
    */
    Scalar velocityScale(const Vec2* feet, int count, Scalar margin) const;

    /**
     * @brief Back to rest: the next update starts the velocity and acceleration afresh
    */
    void reset();

    const BalanceState& state() const { return state_; }

    static constexpr Scalar GRAVITY = Scalar(9.81);    // m/s^2
    static constexpr Scalar MIN_HEIGHT = Scalar(0.01); // m, h is never taken as less (seated, or no feet)

private:
    Scalar scaleAlong(const Vec2& shift, const Vec2* feet, int count, Scalar margin) const;

    BalanceState state_;
    Vec2 previousCom_ = Vec2::Zero();
    Vec2 previousVelocity_ = Vec2::Zero();
    int history_ = 0;   // updates since a reset, up to 2: velocity needs one before, acceleration two
};

}  // namespace gait_core

#endif  // GAIT_CORE__BALANCE_CONTROLLER_HPP
