// Turn feedback: the robot turns as fast as it's asked, whatever its feet do

#include <gtest/gtest.h>

#include <cmath>

#include "test_common.hpp"
#include "gait_core/locomotion/gait_engine.hpp"

using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::GaitParams;
using gait_core::ImuSample;
using gait_core::JointFeedback;
using gait_core::JointVec;
using gait_core::Scalar;
using gait_core::Timestamp;
using gait_core::Vec3;

namespace {

/**
 * @brief Explorer with perfect servos and an IMU whose gyro reports `slip` times the turn its feet
 *        make (leg odometry): a robot whose feet slip round as it turns, by that share
*/
struct SlippingRobot {
    explicit SlippingRobot(const GaitParams& params, double slipFactor, bool withImu = true)
        : engine(params, explorerRobot()), slip(slipFactor), imu(withImu) {
        for (int leg = 0; leg < gait_core::explorer::NUM_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = JointVec::Zero(3);
        }
        run(3.0);
        engine.sit(false);
        run(6.0);
    }
    // Runs, and returns the turn rate the IMU saw on average over the last `average` s (rad/s)
    double run(double seconds, double average = 0, double gyroNoise = 0) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        const int from = ticks - static_cast<int>(std::lround(average / 0.01));
        double sum = 0;
        int count = 0;
        for (int i = 0; i < ticks; ++i) {
            feedback.stamp = now;
            engine.onJointStates(feedback);
            const double yaw = engine.odometry().yaw();
            const double rate = slip * std::remainder(yaw - lastYaw, 2 * M_PI) / 0.01;
            lastYaw = yaw;
            if (imu) {
                ImuSample sample;
                sample.stamp = now;
                sample.linearAcceleration = Vec3(0, 0, static_cast<Scalar>(9.81));
                sample.angularVelocity = Vec3(0, 0, static_cast<Scalar>(rate + gyroNoise * std::sin(i * 1.7)));
                engine.setImu(sample);
            }
            now += 10'000'000;
            const GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            if (out.commanded) {
                for (int leg = 0; leg < gait_core::explorer::NUM_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
            if (i >= from) {
                sum += rate;
                ++count;
            }
        }
        return count > 0 ? sum / count : 0;
    }
    GaitEngine engine;
    JointFeedback feedback;
    Timestamp now = 1'000'000'000;
    double slip;
    bool imu;
    double lastYaw = 0;
};

GaitParams withFeedback(bool on = true) {
    GaitParams params;
    params.turn_feedback = on;
    return params;
}

}  // namespace

TEST(TurnFeedback, ARobotThatTurnsTooFarIsBroughtBackToTheRateAsked) {
    const Scalar asked = static_cast<Scalar>(0.3);
    // Without it, a quarter too far
    SlippingRobot open(withFeedback(false), 1.25);
    open.engine.setVelocity({0, 0, asked});
    EXPECT_NEAR(open.run(12.0, 6.0), 1.25 * asked, 0.02 * asked);
    EXPECT_EQ(open.engine.turnScale(), 1);
    // With it, as asked, the scale having learned the robot's 1 / 1.25
    SlippingRobot closed(withFeedback(), 1.25);
    closed.engine.setVelocity({0, 0, asked});
    EXPECT_NEAR(closed.run(12.0, 6.0), asked, 0.02 * asked);
    EXPECT_NEAR(closed.engine.turnScale(), 0.8, 0.02);
}

TEST(TurnFeedback, ARobotThatTurnsTooLittleIsSpedUpAndTheOtherWayRoundToo) {
    SlippingRobot robot(withFeedback(), 0.8);
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(-0.25)});
    EXPECT_NEAR(robot.run(12.0, 6.0), -0.25, 0.02 * 0.25);
    EXPECT_NEAR(robot.engine.turnScale(), 1.25, 0.03);
}

TEST(TurnFeedback, WhatItLearnedCarriesOverToOtherTurns) {
    SlippingRobot robot(withFeedback(), 1.25);
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(12.0);
    // Walking straight leaves it be; a new turn, on an arc, starts right
    robot.engine.setVelocity({static_cast<Scalar>(0.05), 0, 0});
    robot.run(4.0);
    EXPECT_NEAR(robot.engine.turnScale(), 0.8, 0.02);
    robot.engine.setVelocity({static_cast<Scalar>(0.05), 0, static_cast<Scalar>(0.2)});
    EXPECT_NEAR(robot.run(3.0, 2.0), 0.2, 0.03 * 0.2);
}

TEST(TurnFeedback, WalkingStraightANoisyGyroChangesNothing) {
    SlippingRobot robot(withFeedback(), 1.25);
    robot.engine.setVelocity({static_cast<Scalar>(0.08), 0, 0});
    robot.run(8.0, 0, 0.3);   // the gyro swinging +-0.3 rad/s
    EXPECT_EQ(robot.engine.turnScale(), 1);
}

TEST(TurnFeedback, WithoutAnImuNothingIsLearned) {
    SlippingRobot robot(withFeedback(), 1.25, false);
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(8.0);
    EXPECT_EQ(robot.engine.turnScale(), 1);
}

TEST(TurnFeedback, TheScaleStaysInItsRange) {
    SlippingRobot robot(withFeedback(), 3.0);   // three times too far: more than it may correct
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(12.0);
    EXPECT_NEAR(robot.engine.turnScale(), 0.5, 1e-6);
}

TEST(TurnFeedback, CuttingTheServosKeepsWhatItLearned) {
    SlippingRobot robot(withFeedback(), 1.25);
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(12.0);
    const Scalar learned = robot.engine.turnScale();
    robot.engine.setVelocity({0, 0, 0});
    robot.run(4.0);
    ASSERT_TRUE(robot.engine.powerDown().success);
    robot.run(9.0);
    EXPECT_EQ(robot.engine.turnScale(), learned);
}
