#include "gait_core/estimation/body_velocity.hpp"

#include <algorithm>

namespace gait_core {

void BodyVelocityEstimator::update(const BodyVelocityInputs& in, Scalar filter) {
    if (!(in.dt > 0)) {
        return;
    }
    // The centre of mass tipping about the feet: w x (0, 0, h). Pitching nose down (+y) carries it
    // forward, rolling right side down (+x) carries it right (-y). Odometry can't see it: the feet
    // don't move relative to the body.
    const Scalar h = std::max(in.comHeight, Scalar(0));
    const Vec2 tipping(h * in.angularVelocity.y(), -h * in.angularVelocity.x());
    const Vec2 reference = in.reference + (in.imuFresh ? tipping : Vec2::Zero());
    if (!started_) {
        velocity_ = reference;
        started_ = true;
    }
    const Scalar blend = filter > 0 ? in.dt / (filter + in.dt) : Scalar(1);

    if (in.imuFresh) {
        // The accelerometer into the level frame, less gravity: the body's own acceleration
        const Eigen::Quaternion<Scalar> tilt(AngleAxis(in.pitch, Vec3::UnitY()) * AngleAxis(in.roll, Vec3::UnitX()));
        acceleration_ = (tilt * in.linearAcceleration - Vec3(0, 0, GRAVITY)).head<2>();
        velocity_ = (1 - blend) * (velocity_ + acceleration_ * in.dt) + blend * reference;
    } else {
        // Nothing fast to blend in: the reference is already a measurement, and filtering it would
        // only make it lag
        acceleration_ = Vec2::Zero();
        velocity_ = reference;
    }
}

void BodyVelocityEstimator::reset() {
    *this = BodyVelocityEstimator{};
}

}  // namespace gait_core
