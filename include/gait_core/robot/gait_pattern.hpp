#ifndef GAIT_CORE__GAIT_PATTERN_HPP
#define GAIT_CORE__GAIT_PATTERN_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "gait_core/types.hpp"

namespace gait_core {

// Most gaits a robot can offer
inline constexpr int kMaxGaits = 8;

// Most phases in a gait's cycle: a swing for each leg, and a phase with every foot down after each
inline constexpr int kMaxGaitPhases = 2 * kMaxLegs;

/**
 * @brief Which legs swing in each phase of a gait, as the engine runs it
 * @note A gait is a cycle of phases; in each, one group of legs swings while the rest stand, or
 *       (a biped's double support, say) every foot stands. Every leg swings in exactly one phase per
 *       cycle. Derived from a GaitPattern.
*/
struct GaitSchedule {
    uint8_t phases = 0;                       // phases in a cycle, 0 if not set up
    std::array<uint8_t, kMaxLegs> phaseOf{};  // phase each leg swings in, indexed by leg number

    size_t size() const { return phases; }
    bool empty() const { return phases == 0; }
    /**
     * @brief Whether a leg swings in a phase
    */
    bool swingsIn(int legNum, size_t phase) const { return phaseOf[legNum] == phase; }
};

/**
 * @brief A gait as each leg's timing: what fraction of the cycle a foot is down (the duty factor),
 *        and when in the cycle each leg lifts off (its offset)
 * @note The engine steps in phases of one swing each, so it can run a pattern whose offsets all
 *       fall on a phase boundary (a multiple of 1 - dutyFactor): tripod, ripple and wave for six
 *       legs, trot and crawl for four. A phase where no leg lifts has every foot down for that long,
 *       as a biped's walk needs between steps to move its weight from one foot to the other.
 *       schedule() says whether it can run a pattern.
*/
struct GaitPattern {
    char name[16] = "";                     // for logs and controllers, e.g. "tripod"
    int numLegs = 0;                        // the legs it times, 0..numLegs - 1
    Scalar dutyFactor = 0;                  // fraction of the cycle each foot is down, 0..1
    std::array<Scalar, kMaxLegs> offset{};  // fraction of the cycle at which each leg lifts, 0..1

    /**
     * @brief Groups of legs that lift together, in turn, each swinging for an equal share
     * @param[in] name The gait's name (truncated to 15 characters)
     * @param[in] groups The legs that swing together, in the order they swing. An empty group is
     *            a phase with every foot down: {{0}, {}, {1}, {}} walks a biped with both feet
     *            down for a phase between its steps.
     * @return A pattern with dutyFactor (n - 1) / n and offsets group / n, for n groups
    */
    static GaitPattern fromGroups(const char* name, std::initializer_list<std::initializer_list<int>> groups);

    /**
     * @brief The phases the engine runs this pattern as
     * @param[in] robotLegs How many legs the robot has; the pattern must time exactly these
     * @param[out] out The schedule, if it can be run
     * @return false if not: a different number of legs, a duty factor that isn't (n - 1) / n for
     *         2..kMaxGaitPhases phases, or an offset off a phase boundary or outside the cycle
    */
    bool schedule(int robotLegs, GaitSchedule& out) const;
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_PATTERN_HPP
