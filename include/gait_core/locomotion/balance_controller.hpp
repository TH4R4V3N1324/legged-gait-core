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
    int plantedCount = 0;                  // contact points on the ground: feet, or 4 per sole
    bool supportArea = false;              // whether they make a polygon with an inside (three or more, not in a line)
    Scalar comMargin = 0;                  // m
    Scalar captureMargin = 0;              // m
    Scalar zmpMargin = 0;                  // m

    // Measured (Phase B), from measure(); until it's called, the commanded values above
    Vec2 measuredVelocity = Vec2::Zero();       // m/s, c dot as measured (leg odometry and IMU)
    Vec2 measuredCapturePoint = Vec2::Zero();   // m, c + measured c dot / omega
    bool hasCentreOfPressure = false;           // whether foot forces gave one
    Vec2 centreOfPressure = Vec2::Zero();       // m, where the feet are measured pushing from
    Scalar copMargin = 0;                       // m, how far it is inside the feet
    Scalar delay = 0;                           // s, the loop delay the prediction covers
    Vec2 predictedCapturePoint = Vec2::Zero();  // m, the measured capture point once the delay has passed
    Scalar predictedCaptureMargin = 0;          // m, how far that is inside the feet
    Vec2 captureError = Vec2::Zero();           // m, the measured capture point less the commanded one, after the delay
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
 *
 *       Feet with no area between them (fewer than three, or in a line, like a trot's diagonal pair)
 *       have no inside to keep anything in, so those checks don't hold anything back: such a gait
 *       balances by its motion, which needs that motion planned (preview control), as the static
 *       checks leave it to allow_unstable_gaits. Allocates nothing; a plain value type.
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
     * @brief Add what was measured to this tick's update (Phase B)
     * @param[in] measuredVelocity c dot as measured: the body's velocity over the ground plus the
     *            centre of mass moving within it (m/s, body frame)
     * @param[in] centreOfPressure Where the feet are measured pushing from (body frame), or null
     * @param[in] feet The feet in contact (body frame, x and y)
     * @param[in] count How many
     * @param[in] delay s from measuring to the feet acting on it: sensing, filtering, the servos
     * @note The capture point runs away from where the feet push: xi(t + d) = p + (xi - p) e^(omega d),
     *       p being the centre of pressure if measured, otherwise the ZMP. So an error grows by
     *       e^(omega d) before anything can act on it (30 ms on explorer: about 30%). The error is
     *       the measured capture point less the commanded one, both carried over the delay the same way.
    */
    void measure(const Vec2& measuredVelocity, const Vec2* centreOfPressure, const Vec2* feet, int count, Scalar delay);

    /**
     * @brief The change in c dot that would put the capture point back where it's commanded
     * @param[in] gain Share of the error to correct (0..1): 1 cancels it at once
     * @param[in] deadband m, an error this small is left alone (and this much less of a larger one)
     * @return m/s, body frame: -gain x omega x error. Changing c dot by v moves xi by v / omega, so
     *         gain 1 cancels the error; the feet make it happen by pushing from past xi (the body
     *         shifted over them), which is what accelerationScale limits.
    */
    Vec2 correctionVelocity(Scalar gain, Scalar deadband) const;

    /**
     * @brief How far from its commanded place the capture point will be when a swing lands
     * @param[in] timeToTouchdown s until the swinging feet land
     * @return m, body frame: the capture point error carried on to touchdown, error x e^(omega t):
     *         landing that much further along puts the feet where the body is really heading
    */
    Vec2 touchdownOffset(Scalar timeToTouchdown) const;

    /**
     * @brief Back to rest: the next update starts the velocity and acceleration afresh
    */
    void reset();

    const BalanceState& state() const { return state_; }

    /**
     * @brief Whether some feet enclose an area: three or more, not in a line
     * @note Judged by the largest circle inside them (deepestSupportPoint), at least MIN_AREA_RADIUS
    */
    static bool hasArea(const Vec2* feet, int count);

    static constexpr Scalar GRAVITY = Scalar(9.81);    // m/s^2
    static constexpr Scalar MIN_HEIGHT = Scalar(0.01); // m, h is never taken as less (seated, or no feet)
    static constexpr Scalar MIN_AREA_RADIUS = Scalar(0.002);  // m, a polygon thinner than this counts as a line

private:
    Scalar scaleAlong(const Vec2& shift, const Vec2* feet, int count, Scalar margin) const;

    BalanceState state_;
    Vec2 previousCom_ = Vec2::Zero();
    Vec2 previousVelocity_ = Vec2::Zero();
    int history_ = 0;   // updates since a reset, up to 2: velocity needs one before, acceleration two
};

}  // namespace gait_core

#endif  // GAIT_CORE__BALANCE_CONTROLLER_HPP
