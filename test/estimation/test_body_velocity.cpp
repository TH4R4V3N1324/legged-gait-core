#include <gtest/gtest.h>

#include <cmath>

#include "gait_core/estimation/body_velocity.hpp"

using gait_core::BodyVelocityEstimator;
using gait_core::BodyVelocityInputs;
using gait_core::Scalar;
using gait_core::Vec2;
using gait_core::Vec3;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);       // s
constexpr Scalar FILTER = static_cast<Scalar>(0.2);    // s
constexpr Scalar HEIGHT = static_cast<Scalar>(0.14);   // m

/**
 * @brief Standing still and level: the accelerometer reads gravity, odometry nothing
*/
BodyVelocityInputs still() {
    BodyVelocityInputs in;
    in.dt = DT;
    in.imuFresh = true;
    in.linearAcceleration = Vec3(0, 0, BodyVelocityEstimator::GRAVITY);
    in.comHeight = HEIGHT;
    return in;
}

void run(BodyVelocityEstimator& estimator, const BodyVelocityInputs& in, double seconds) {
    const int ticks = static_cast<int>(std::lround(seconds / static_cast<double>(DT)));
    for (int i = 0; i < ticks; ++i) {
        estimator.update(in, FILTER);
    }
}

}  // namespace

TEST(BodyVelocity, StandingStillIsAtRest) {
    BodyVelocityEstimator estimator;
    run(estimator, still(), 2);
    EXPECT_NEAR(estimator.velocity().norm(), 0, 1e-6);
    EXPECT_NEAR(estimator.acceleration().norm(), 0, 1e-5);
}

TEST(BodyVelocity, GravityIsTakenOutOnASlope) {
    BodyVelocityEstimator estimator;
    BodyVelocityInputs in = still();
    in.roll = static_cast<Scalar>(0.2);
    in.pitch = static_cast<Scalar>(-0.15);
    const Eigen::Quaternion<Scalar> tilt(gait_core::AngleAxis(in.pitch, Vec3::UnitY()) * gait_core::AngleAxis(in.roll, Vec3::UnitX()));
    in.linearAcceleration = tilt.inverse() * Vec3(0, 0, BodyVelocityEstimator::GRAVITY);
    run(estimator, in, 2);
    EXPECT_NEAR(estimator.velocity().norm(), 0, 1e-4);
}

TEST(BodyVelocity, TheFirstUpdateTakesTheReference) {
    BodyVelocityEstimator estimator;
    BodyVelocityInputs in = still();
    in.reference = Vec2(static_cast<Scalar>(0.1), static_cast<Scalar>(-0.05));
    estimator.update(in, FILTER);
    EXPECT_NEAR((estimator.velocity() - in.reference).norm(), 0, 1e-6);
}

TEST(BodyVelocity, ItFollowsOdometry) {
    // Walking steadily: the accelerometer reads nothing extra, so odometry sets the velocity
    BodyVelocityEstimator estimator;
    run(estimator, still(), 0.1);
    BodyVelocityInputs in = still();
    in.reference = Vec2(static_cast<Scalar>(0.12), 0);
    run(estimator, in, 2);
    EXPECT_NEAR(estimator.velocity().x(), 0.12, 1e-3);
}

TEST(BodyVelocity, APushShowsUpBeforeOdometrySeesIt) {
    // Shoved forward with the feet holding still relative to the body (sliding or tipping): leg
    // odometry says nothing, the accelerometer says 3 m/s^2
    BodyVelocityEstimator estimator;
    run(estimator, still(), 0.1);
    BodyVelocityInputs push = still();
    push.linearAcceleration.x() = 3;
    run(estimator, push, 0.1);
    const Scalar during = estimator.velocity().x();
    EXPECT_GT(during, 0.15);   // most of 0.3 m/s: a complementary filter keeps the fast part
    EXPECT_LT(during, 0.3);

    // Afterwards it settles back to odometry's view
    run(estimator, still(), 2);
    EXPECT_NEAR(estimator.velocity().x(), 0, 1e-3);
}

TEST(BodyVelocity, TippingIsSeenByTheGyro) {
    BodyVelocityEstimator estimator;
    BodyVelocityInputs in = still();
    in.angularVelocity = Vec3(0, static_cast<Scalar>(0.5), 0);   // pitching nose down
    run(estimator, in, 2);
    EXPECT_NEAR(estimator.velocity().x(), HEIGHT * 0.5, 1e-4);
    EXPECT_NEAR(estimator.velocity().y(), 0, 1e-6);

    in.angularVelocity = Vec3(static_cast<Scalar>(0.5), 0, 0);   // rolling right side down
    run(estimator, in, 2);
    EXPECT_NEAR(estimator.velocity().y(), -HEIGHT * 0.5, 1e-4);
}

TEST(BodyVelocity, WithoutTheImuItIsTheReference) {
    // Leg odometry is a measurement already: with nothing faster to blend in, filtering it would lag
    BodyVelocityEstimator estimator;
    BodyVelocityInputs in;
    in.dt = DT;
    estimator.update(in, FILTER);   // starts at rest
    in.reference = Vec2(static_cast<Scalar>(0.2), 0);
    in.linearAcceleration = Vec3(50, 50, 0);   // ignored: not fresh
    estimator.update(in, FILTER);
    EXPECT_NEAR(estimator.velocity().x(), 0.2, 1e-6);
    EXPECT_EQ(estimator.acceleration(), Vec2::Zero());
}

TEST(BodyVelocity, ResetStartsAgain) {
    BodyVelocityEstimator estimator;
    BodyVelocityInputs push = still();
    push.linearAcceleration.x() = 3;
    run(estimator, push, 0.2);
    estimator.reset();
    EXPECT_EQ(estimator.velocity(), Vec2::Zero());
    BodyVelocityInputs in = still();
    in.reference = Vec2(0, static_cast<Scalar>(0.07));
    estimator.update(in, FILTER);
    EXPECT_NEAR((estimator.velocity() - in.reference).norm(), 0, 1e-6);
}
