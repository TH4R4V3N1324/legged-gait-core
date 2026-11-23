#include "gait_core/locomotion/balance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "gait_core/locomotion/support_polygon.hpp"

namespace gait_core {

void BalanceController::update(Scalar dt, const Vec3& com, const Vec2& bodyVelocity, const Vec2* feet, int count,
                               Scalar footHeight, Scalar accelerationFilter) {
    if (!(dt > 0)) {
        return;
    }
    const Vec2 c = com.head<2>();
    // c dot: the body carrying the centre of mass over the ground, plus it moving within the body
    const Vec2 velocity = history_ >= 1 ? Vec2(bodyVelocity + (c - previousCom_) / dt) : bodyVelocity;
    if (history_ >= 1) {
        const Vec2 raw = (velocity - previousVelocity_) / dt;
        const Scalar blend = accelerationFilter > 0 ? dt / (accelerationFilter + dt) : Scalar(1);
        state_.comAcceleration += blend * (raw - state_.comAcceleration);
    } else {
        state_.comAcceleration = Vec2::Zero();
    }
    previousCom_ = c;
    previousVelocity_ = velocity;
    history_ = std::min(history_ + 1, 2);

    state_.com = com;
    state_.comVelocity = velocity;
    state_.height = std::max(com.z() - footHeight, MIN_HEIGHT);
    state_.omega = std::sqrt(GRAVITY / state_.height);
    state_.capturePoint = c + velocity / state_.omega;
    state_.zmp = c - state_.comAcceleration / (state_.omega * state_.omega);
    state_.plantedCount = count;
    state_.supportArea = hasArea(feet, count);
    state_.comMargin = stabilityMargin(feet, count, c);
    state_.captureMargin = stabilityMargin(feet, count, state_.capturePoint);
    state_.zmpMargin = stabilityMargin(feet, count, state_.zmp);

    // Until measured, the measurement is the command
    state_.measuredVelocity = velocity;
    state_.measuredCapturePoint = state_.capturePoint;
    state_.hasCentreOfPressure = false;
    state_.centreOfPressure = state_.zmp;
    state_.copMargin = state_.zmpMargin;
    state_.delay = 0;
    state_.predictedCapturePoint = state_.capturePoint;
    state_.predictedCaptureMargin = state_.captureMargin;
    state_.captureError = Vec2::Zero();
}

void BalanceController::measure(const Vec2& measuredVelocity, const Vec2* centreOfPressure, const Vec2* feet, int count,
                                Scalar delay) {
    if (!(state_.omega > 0)) {
        return;
    }
    const Vec2 c = state_.com.head<2>();
    state_.measuredVelocity = measuredVelocity;
    state_.measuredCapturePoint = c + measuredVelocity / state_.omega;
    state_.hasCentreOfPressure = centreOfPressure != nullptr;
    state_.centreOfPressure = centreOfPressure != nullptr ? *centreOfPressure : state_.zmp;
    state_.copMargin = stabilityMargin(feet, count, state_.centreOfPressure);
    state_.delay = std::max(delay, Scalar(0));
    const Scalar growth = std::exp(state_.omega * state_.delay);
    const Vec2& pivot = state_.centreOfPressure;
    state_.predictedCapturePoint = pivot + (state_.measuredCapturePoint - pivot) * growth;
    state_.predictedCaptureMargin = stabilityMargin(feet, count, state_.predictedCapturePoint);
    // Both carried over the delay from the same pivot, so only their difference grows
    state_.captureError = (state_.measuredCapturePoint - state_.capturePoint) * growth;
}

Vec2 BalanceController::correctionVelocity(Scalar gain, Scalar deadband) const {
    const Scalar size = state_.captureError.norm();
    if (size <= deadband || !(state_.omega > 0)) {
        return Vec2::Zero();
    }
    const Vec2 beyond = state_.captureError * ((size - deadband) / size);
    return -gain * state_.omega * beyond;
}

Vec2 BalanceController::touchdownOffset(Scalar timeToTouchdown) const {
    return state_.captureError * std::exp(state_.omega * std::max(timeToTouchdown, Scalar(0)));
}

bool BalanceController::hasArea(const Vec2* feet, int count) {
    Vec2 centre;
    return count >= 3 && deepestSupportPoint(feet, count, centre) >= MIN_AREA_RADIUS;
}

bool BalanceController::canLift(const Vec2* feet, int count, Scalar margin) const {
    if (!hasArea(feet, count)) {
        return true;   // a line of feet holds nothing inside: nothing to wait for
    }
    const Scalar needed = std::min(margin, stabilityMargin(feet, count, state_.com.head<2>()));
    return stabilityMargin(feet, count, state_.capturePoint) >= needed &&
        stabilityMargin(feet, count, state_.zmp) >= needed;
}

Scalar BalanceController::accelerationScale(const Vec2& acceleration, const Vec2* feet, int count, Scalar margin) const {
    if (state_.omega <= 0) {
        return 1;
    }
    // Accelerating moves the ZMP back from the centre of mass by a / omega^2
    return scaleAlong(-acceleration / (state_.omega * state_.omega), feet, count, margin);
}

Scalar BalanceController::velocityScale(const Vec2* feet, int count, Scalar margin) const {
    if (state_.omega <= 0) {
        return 1;
    }
    // The capture point leads the centre of mass by c dot / omega
    return scaleAlong(state_.comVelocity / state_.omega, feet, count, margin);
}

/**
 * @brief The largest s in [0, 1] that keeps c + s x shift margin inside the feet
 * @note Or, if c itself isn't that far inside, no closer to the edge than c. The margin to a convex
 *       polygon is concave along a line, so the points that pass are one stretch from s = 0: bisect.
*/
Scalar BalanceController::scaleAlong(const Vec2& shift, const Vec2* feet, int count, Scalar margin) const {
    if (shift.squaredNorm() == 0 || !hasArea(feet, count)) {
        return 1;
    }
    const Vec2 c = state_.com.head<2>();
    const Scalar needed = std::min(margin, stabilityMargin(feet, count, c));
    auto fits = [&](Scalar s) { return stabilityMargin(feet, count, c + s * shift) >= needed; };
    if (fits(1)) {
        return 1;
    }
    Scalar low = 0, high = 1;
    for (int i = 0; i < 12; ++i) {
        const Scalar mid = Scalar(0.5) * (low + high);
        (fits(mid) ? low : high) = mid;
    }
    return low;
}

void BalanceController::reset() {
    *this = BalanceController{};
}

}  // namespace gait_core
