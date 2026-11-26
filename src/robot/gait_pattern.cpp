#include "gait_core/robot/gait_pattern.hpp"

#include <cmath>
#include <cstring>

namespace gait_core {

GaitPattern GaitPattern::fromGroups(const char* name, std::initializer_list<std::initializer_list<int>> groups) {
    GaitPattern pattern;
    std::strncpy(pattern.name, name, sizeof(pattern.name) - 1);
    pattern.name[sizeof(pattern.name) - 1] = '\0';
    const Scalar n = static_cast<Scalar>(groups.size());
    pattern.dutyFactor = (n - 1) / n;
    int group = 0;
    for (const auto& legs : groups) {
        for (const int leg : legs) {
            if (leg >= 0 && leg < kMaxLegs) {
                pattern.offset[leg] = static_cast<Scalar>(group) / n;
                pattern.numLegs = leg + 1 > pattern.numLegs ? leg + 1 : pattern.numLegs;
            }
        }
        ++group;
    }
    return pattern;
}

bool GaitPattern::schedule(int robotLegs, GaitSchedule& out) const {
    constexpr Scalar TOLERANCE = static_cast<Scalar>(1e-3);  // of a phase, for 1/3 and the like in float
    if (numLegs != robotLegs || numLegs < 1 || numLegs > kMaxLegs) {
        return false;
    }
    // Each leg swings for one phase, so a phase is the swing's share of the cycle
    const Scalar swing = 1 - dutyFactor;
    if (!(swing > 0)) {
        return false;
    }
    const long phases = std::lround(1 / swing);
    if (phases < 2 || phases > kMaxGaitPhases || std::abs(static_cast<Scalar>(phases) * swing - 1) > TOLERANCE) {
        return false;
    }

    GaitSchedule result;
    result.phases = static_cast<uint8_t>(phases);
    for (int leg = 0; leg < numLegs; ++leg) {
        const Scalar position = offset[leg] / swing;   // in phases
        const long phase = std::lround(position);
        if (std::abs(position - static_cast<Scalar>(phase)) > TOLERANCE || phase < 0 || phase >= phases) {
            return false;
        }
        result.phaseOf[leg] = static_cast<uint8_t>(phase);
    }
    // A phase no leg swings in has every foot down (a biped's double support)
    out = result;
    return true;
}

}  // namespace gait_core
