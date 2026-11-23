#ifndef GAIT_CORE__PREVIEW_CONTROLLER_HPP
#define GAIT_CORE__PREVIEW_CONTROLLER_HPP

#include <array>

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief ZMP preview control: the smoothest centre of mass path that keeps the ZMP on a planned track
 * @note Kajita's cart-table model, one copy per horizontal axis. The centre of mass is driven by its
 *       jerk u, with x = (c, c dot, c double dot) and
 *           x[k+1] = A x[k] + B u[k],    zmp[k] = c - c double dot / omega^2
 *       Because the capture point runs away, the body has to start moving before the support it's
 *       heading for arrives: the jerk is chosen from the ZMP reference over the next N ticks,
 *           u[k] = -Gi sum(zmp - ref) - Gx x[k] - sum_{j=1..N} Gp[j] ref[k + j]
 *       with the gains from the LQ problem that weighs the ZMP's error (integral action, Qe) against
 *       the jerk (R), solved once for a given tick and omega (a discrete Riccati equation, iterated
 *       in double precision so it converges on a float build too).
 *
 *       Allocates nothing; the preview is at most MAX_STEPS ticks.
*/
class PreviewController {
public:
    static constexpr int MAX_STEPS = 200;

    /**
     * @brief Work out the gains for a tick and pendulum, and a preview of `steps` ticks
     * @param[in] dt s per tick
     * @param[in] omega 1/s, sqrt(g / h)
     * @param[in] steps How many ticks ahead the reference is given, 1..MAX_STEPS (clamped)
     * @param[in] errorWeight Qe, the weight on the ZMP's tracking error
     * @param[in] jerkWeight R, the weight on the jerk: smaller tracks more tightly
     * @return false if the Riccati iteration didn't converge (the gains are then left unset)
     * @note Keeps the state: a new omega (the body raised or lowered) carries on from where it was.
    */
    bool configure(Scalar dt, Scalar omega, int steps, double errorWeight = 1.0, double jerkWeight = 1e-6);

    bool configured() const { return configured_; }
    Scalar dt() const { return dt_; }
    Scalar omega() const { return omega_; }
    int steps() const { return steps_; }

    /**
     * @brief Start from a centre of mass at rest (or moving), forgetting the integral of the error
    */
    void reset(const Vec2& position, const Vec2& velocity = Vec2::Zero(), const Vec2& acceleration = Vec2::Zero());

    /**
     * @brief Advance one tick
     * @param[in] reference The ZMP reference for the next steps() ticks, ref[k + 1] .. ref[k + steps()]
     *            (the first is one tick from now); same frame as the state
     * @note Does nothing until configured.
    */
    void step(const Vec2* reference);

    const Vec2& position() const { return position_; }
    const Vec2& velocity() const { return velocity_; }
    const Vec2& acceleration() const { return acceleration_; }
    // The ZMP the current state makes: c - c double dot / omega^2
    Vec2 zmp() const;

    // The gains, for tests and telemetry
    Scalar gainIntegral() const { return gi_; }
    const std::array<Scalar, 3>& gainState() const { return gx_; }
    Scalar gainPreview(int j) const { return (j >= 1 && j <= steps_) ? gp_[static_cast<size_t>(j - 1)] : Scalar(0); }

private:
    bool configured_ = false;
    Scalar dt_ = 0;
    Scalar omega_ = 0;
    int steps_ = 0;
    Scalar gi_ = 0;
    std::array<Scalar, 3> gx_{};
    std::array<Scalar, MAX_STEPS> gp_{};

    Vec2 origin_ = Vec2::Zero();     // where it was reset: the state and references are taken relative to it
    Vec2 position_ = Vec2::Zero();
    Vec2 velocity_ = Vec2::Zero();
    Vec2 acceleration_ = Vec2::Zero();
    Vec2 errorSum_ = Vec2::Zero();
    Vec2 currentReference_ = Vec2::Zero();   // where the ZMP should be this tick (last tick's ref[k + 1])
    bool hasCurrentReference_ = false;
};

}  // namespace gait_core

#endif  // GAIT_CORE__PREVIEW_CONTROLLER_HPP
