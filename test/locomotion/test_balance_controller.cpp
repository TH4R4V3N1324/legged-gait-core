#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "gait_core/locomotion/balance_controller.hpp"

using gait_core::BalanceController;
using gait_core::BalanceState;
using gait_core::Scalar;
using gait_core::Vec2;
using gait_core::Vec3;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);        // s
constexpr Scalar HEIGHT = static_cast<Scalar>(0.134);   // m, explorer's standing height
constexpr Scalar FILTER = static_cast<Scalar>(0.05);    // s
constexpr double TOL = 1e-4;

/**
 * @brief A square of feet, half width w, centred on the body origin, h below the centre of mass
*/
struct Square {
    std::array<Vec2, 4> feet;
    explicit Square(Scalar w = static_cast<Scalar>(0.1))
        : feet{Vec2(w, w), Vec2(w, -w), Vec2(-w, -w), Vec2(-w, w)} {}
};

Scalar omega() {
    return std::sqrt(BalanceController::GRAVITY / HEIGHT);
}

/**
 * @brief Update with the centre of mass at (x, y, 0), h above feet at -h
*/
void step(BalanceController& balance, const Square& square, const Vec2& com, const Vec2& bodyVelocity) {
    balance.update(DT, Vec3(com.x(), com.y(), 0), bodyVelocity, square.feet.data(), 4, -HEIGHT, FILTER);
}

}  // namespace

TEST(BalanceController, StandingStillEverythingIsAtTheCentreOfMass) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 50; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    const BalanceState& s = balance.state();
    EXPECT_NEAR(s.height, HEIGHT, TOL);
    EXPECT_NEAR(s.omega, omega(), TOL);
    EXPECT_NEAR(s.comVelocity.norm(), 0, TOL);
    EXPECT_NEAR(s.capturePoint.norm(), 0, TOL);
    EXPECT_NEAR(s.zmp.norm(), 0, TOL);
    EXPECT_NEAR(s.comMargin, 0.1, TOL);
    EXPECT_NEAR(s.captureMargin, 0.1, TOL);
    EXPECT_NEAR(s.zmpMargin, 0.1, TOL);
    EXPECT_EQ(s.plantedCount, 4);
}

TEST(BalanceController, TheCapturePointLeadsByVelocityOverOmega) {
    BalanceController balance;
    const Square square;
    const Vec2 v(static_cast<Scalar>(0.2), static_cast<Scalar>(-0.1));
    for (int i = 0; i < 50; ++i) {
        step(balance, square, Vec2::Zero(), v);
    }
    const BalanceState& s = balance.state();
    EXPECT_NEAR((s.comVelocity - v).norm(), 0, TOL);
    EXPECT_NEAR((s.capturePoint - v / omega()).norm(), 0, TOL);
    EXPECT_NEAR(s.zmp.norm(), 0, TOL);   // steady: no acceleration
}

TEST(BalanceController, TheCentreOfMassMovingWithinTheBodyCounts) {
    // Legs swinging forward carry the centre of mass forward relative to the body
    BalanceController balance;
    const Square square;
    const Scalar drift = static_cast<Scalar>(0.05);   // m/s
    for (int i = 0; i < 50; ++i) {
        step(balance, square, Vec2(drift * DT * static_cast<Scalar>(i), 0), Vec2::Zero());
    }
    EXPECT_NEAR(balance.state().comVelocity.x(), drift, TOL);
}

TEST(BalanceController, AcceleratingPutsTheZmpBehind) {
    BalanceController balance;
    const Square square;
    const Scalar a = 2;   // m/s^2
    for (int i = 0; i < 100; ++i) {
        step(balance, square, Vec2::Zero(), Vec2(a * DT * static_cast<Scalar>(i), 0));
    }
    const BalanceState& s = balance.state();
    EXPECT_NEAR(s.comAcceleration.x(), a, 1e-2);
    EXPECT_NEAR(s.zmp.x(), -a / (omega() * omega()), 1e-3);
    EXPECT_LT(s.zmpMargin, s.comMargin);
}

TEST(BalanceController, BrakingPutsTheZmpAhead) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 100; ++i) {
        step(balance, square, Vec2::Zero(), Vec2(static_cast<Scalar>(0.3) - DT * static_cast<Scalar>(i), 0));
    }
    EXPECT_GT(balance.state().zmp.x(), 0);
}

TEST(BalanceController, TheAccelerationIsFiltered) {
    // A one-tick jump in velocity is a huge acceleration for one tick: filtered, the ZMP moves a bit
    BalanceController balance;
    const Square square;
    step(balance, square, Vec2::Zero(), Vec2::Zero());
    step(balance, square, Vec2::Zero(), Vec2(static_cast<Scalar>(0.1), 0));
    const Scalar raw = static_cast<Scalar>(0.1) / DT;
    EXPECT_NEAR(balance.state().comAcceleration.x(), raw * DT / (FILTER + DT), TOL);
}

TEST(BalanceController, HeightFollowsTheFeet) {
    BalanceController balance;
    const Square square;
    balance.update(DT, Vec3(0, 0, static_cast<Scalar>(0.01)), Vec2::Zero(), square.feet.data(), 4,
                   static_cast<Scalar>(-0.09), FILTER);
    EXPECT_NEAR(balance.state().height, 0.1, TOL);
    // Never less than MIN_HEIGHT, e.g. with the feet above the centre of mass
    balance.update(DT, Vec3::Zero(), Vec2::Zero(), square.feet.data(), 4, static_cast<Scalar>(0.05), FILTER);
    EXPECT_NEAR(balance.state().height, BalanceController::MIN_HEIGHT, TOL);
}

TEST(BalanceController, LiftingWaitsUntilTheMomentumFitsTheFeetLeft) {
    BalanceController balance;
    const Square square;
    // Three feet stay down: the front left lifts. Their triangle leaves the centre of mass on its edge
    // diagonal, so shift it towards the back right corner to give it room.
    const std::array<Vec2, 3> staying = {square.feet[1], square.feet[2], square.feet[3]};
    const Vec2 com(static_cast<Scalar>(-0.03), static_cast<Scalar>(-0.03));
    for (int i = 0; i < 50; ++i) {
        step(balance, square, com, Vec2::Zero());
    }
    EXPECT_TRUE(balance.canLift(staying.data(), 3, static_cast<Scalar>(0.01)));

    // Moving forward and left towards the lifting foot: the capture point leaves the triangle
    BalanceController moving;
    for (int i = 0; i < 50; ++i) {
        step(moving, square, com, Vec2(static_cast<Scalar>(0.3), static_cast<Scalar>(0.3)));
    }
    EXPECT_FALSE(moving.canLift(staying.data(), 3, static_cast<Scalar>(0.01)));
}

TEST(BalanceController, LiftingNeverWaitsForMoreThanStandingStillGives) {
    // The centre of mass is already outside the feet left: only the motion's part can hold the lift
    BalanceController balance;
    const Square square;
    const std::array<Vec2, 2> staying = {square.feet[0], square.feet[2]};   // a trot's diagonal pair
    for (int i = 0; i < 50; ++i) {
        step(balance, square, Vec2(0, static_cast<Scalar>(0.02)), Vec2::Zero());
    }
    EXPECT_TRUE(balance.canLift(staying.data(), 2, static_cast<Scalar>(0.01)));
}

TEST(BalanceController, TheMostAccelerationIsOmegaSquaredTimesTheSpareMargin) {
    BalanceController balance;
    const Square square;   // 0.1 m from the centre of mass to each edge
    for (int i = 0; i < 10; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    const Scalar margin = static_cast<Scalar>(0.01);
    const Scalar most = omega() * omega() * (static_cast<Scalar>(0.1) - margin);
    EXPECT_EQ(balance.accelerationScale(Vec2(most / 2, 0), square.feet.data(), 4, margin), 1);
    EXPECT_NEAR(balance.accelerationScale(Vec2(2 * most, 0), square.feet.data(), 4, margin), 0.5, 1e-3);
    EXPECT_NEAR(balance.accelerationScale(Vec2(0, -4 * most), square.feet.data(), 4, margin), 0.25, 1e-3);
}

TEST(BalanceController, AccelerationIsntLimitedBeforeTheFirstUpdate) {
    const BalanceController balance;
    const Square square;
    EXPECT_EQ(balance.accelerationScale(Vec2(100, 0), square.feet.data(), 4, static_cast<Scalar>(0.01)), 1);
}

TEST(BalanceController, ResetStartsAfresh) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), Vec2(DT * static_cast<Scalar>(i), 0));
    }
    balance.reset();
    EXPECT_EQ(balance.state().comAcceleration, Vec2::Zero());
    // The first update after a reset takes the velocity as given, with no acceleration from before
    step(balance, square, Vec2(static_cast<Scalar>(0.5), 0), Vec2(static_cast<Scalar>(0.1), 0));
    EXPECT_NEAR(balance.state().comVelocity.x(), 0.1, TOL);
    EXPECT_EQ(balance.state().comAcceleration, Vec2::Zero());
}

// ---- Measured (Phase B) -----------------------------------------------------------------------

TEST(BalanceController, UnmeasuredTheMeasurementIsTheCommand) {
    BalanceController balance;
    const Square square;
    const Vec2 v(static_cast<Scalar>(0.1), 0);
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), v);
    }
    const BalanceState& s = balance.state();
    EXPECT_EQ(s.measuredVelocity, s.comVelocity);
    EXPECT_EQ(s.predictedCapturePoint, s.capturePoint);
    EXPECT_EQ(s.captureError, Vec2::Zero());
    EXPECT_FALSE(s.hasCentreOfPressure);
}

TEST(BalanceController, MeasuringWhatWasCommandedIsNoError) {
    BalanceController balance;
    const Square square;
    const Vec2 v(static_cast<Scalar>(0.1), 0);
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), v);
    }
    balance.measure(balance.state().comVelocity, nullptr, square.feet.data(), 4, static_cast<Scalar>(0.03));
    EXPECT_NEAR(balance.state().captureError.norm(), 0, 1e-6);
    EXPECT_EQ(balance.correctionVelocity(1, 0), Vec2::Zero());
}

TEST(BalanceController, AnErrorGrowsOverTheDelay) {
    // Measured 0.1 m/s faster than commanded: xi is 0.1 / omega further on, and by the time the feet
    // can act, e^(omega d) times that
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    const Scalar delay = static_cast<Scalar>(0.03);
    balance.measure(Vec2(static_cast<Scalar>(0.1), 0), nullptr, square.feet.data(), 4, delay);
    const BalanceState& s = balance.state();
    const Scalar growth = std::exp(omega() * delay);
    EXPECT_NEAR(s.measuredCapturePoint.x(), 0.1 / omega(), TOL);
    EXPECT_NEAR(s.captureError.x(), 0.1 / omega() * growth, TOL);
    EXPECT_NEAR(s.predictedCapturePoint.x(), 0.1 / omega() * growth, TOL);   // the ZMP is at c here
    EXPECT_NEAR(s.predictedCaptureMargin, 0.1 - s.predictedCapturePoint.x(), TOL);
}

TEST(BalanceController, TheCapturePointRunsAwayFromTheCentreOfPressure) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    // Standing still, but the feet are pushing from 2 cm behind: xi drifts forward, away from it
    const Vec2 cop(static_cast<Scalar>(-0.02), 0);
    const Scalar delay = static_cast<Scalar>(0.05);
    balance.measure(Vec2::Zero(), &cop, square.feet.data(), 4, delay);
    const BalanceState& s = balance.state();
    EXPECT_TRUE(s.hasCentreOfPressure);
    EXPECT_NEAR(s.copMargin, 0.08, TOL);
    EXPECT_NEAR(s.predictedCapturePoint.x(), -0.02 + 0.02 * std::exp(omega() * delay), TOL);
    EXPECT_NEAR(s.captureError.norm(), 0, TOL);   // the same for the command: nothing to correct
}

TEST(BalanceController, TheCorrectionCancelsTheError) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    balance.measure(Vec2(0, static_cast<Scalar>(-0.2)), nullptr, square.feet.data(), 4, 0);
    // Gain 1: change c dot by -omega x error, which is exactly the velocity measured too much
    EXPECT_NEAR((balance.correctionVelocity(1, 0) - Vec2(0, static_cast<Scalar>(0.2))).norm(), 0, TOL);
    EXPECT_NEAR((balance.correctionVelocity(static_cast<Scalar>(0.5), 0) - Vec2(0, static_cast<Scalar>(0.1))).norm(), 0, TOL);
    // The deadband takes its size off the error
    const Scalar error = static_cast<Scalar>(0.2) / omega();
    const Scalar deadband = error / 2;
    EXPECT_NEAR(balance.correctionVelocity(1, deadband).y(), 0.1, TOL);
    EXPECT_EQ(balance.correctionVelocity(1, error * 2), Vec2::Zero());
}

TEST(BalanceController, TheTouchdownOffsetIsTheErrorCarriedOn) {
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2::Zero(), Vec2::Zero());
    }
    balance.measure(Vec2(static_cast<Scalar>(0.05), 0), nullptr, square.feet.data(), 4, 0);
    const Scalar t = static_cast<Scalar>(0.3);
    EXPECT_NEAR(balance.touchdownOffset(t).x(), 0.05 / omega() * std::exp(omega() * t), TOL);
    EXPECT_NEAR(balance.touchdownOffset(0).x(), 0.05 / omega(), TOL);
}

TEST(BalanceController, EachUpdateStartsUnmeasured) {
    BalanceController balance;
    const Square square;
    step(balance, square, Vec2::Zero(), Vec2::Zero());
    balance.measure(Vec2(1, 0), nullptr, square.feet.data(), 4, static_cast<Scalar>(0.03));
    step(balance, square, Vec2::Zero(), Vec2::Zero());
    EXPECT_EQ(balance.state().captureError, Vec2::Zero());
    EXPECT_EQ(balance.state().delay, 0);
}

TEST(BalanceController, FeetInALineHoldNothingBack) {
    // A trot's diagonal pair: no inside to keep the capture point or the ZMP in, so neither the lift
    // gate nor the acceleration limit has anything to judge, rather than stopping the robot dead
    BalanceController balance;
    const Square square;
    for (int i = 0; i < 20; ++i) {
        step(balance, square, Vec2(static_cast<Scalar>(0.02), 0), Vec2(static_cast<Scalar>(0.3), 0));
    }
    const std::array<Vec2, 2> pair = {square.feet[0], square.feet[2]};
    EXPECT_FALSE(BalanceController::hasArea(pair.data(), 2));
    EXPECT_TRUE(balance.canLift(pair.data(), 2, static_cast<Scalar>(0.01)));
    EXPECT_EQ(balance.accelerationScale(Vec2(50, 0), pair.data(), 2, static_cast<Scalar>(0.01)), 1);
    EXPECT_EQ(balance.velocityScale(pair.data(), 2, static_cast<Scalar>(0.01)), 1);
    // Three in a line are no better; three in a triangle are
    const std::array<Vec2, 3> line = {Vec2(-0.1f, 0), Vec2(0, 0), Vec2(0.1f, 0)};
    EXPECT_FALSE(BalanceController::hasArea(line.data(), 3));
    EXPECT_TRUE(BalanceController::hasArea(square.feet.data(), 3));
}
