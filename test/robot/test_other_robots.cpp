#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "test_robots.hpp"

using gait_core::GaitParams;
using gait_core::Scalar;
using gait_core::SitState;
using test_robots::SimRobot;

namespace {

std::string joined(const std::vector<std::string>& lines) {
    std::string all;
    for (const std::string& line : lines) {
        all += "\n  " + line;
    }
    return all;
}

Scalar sc(double value) {
    return static_cast<Scalar>(value);
}

/**
 * @brief Everything the engine does, in order: stand, walk, turn, stop, tilt, sit, power down and
 *        back up. Checks each stage happened and every command along the way was sound.
 * @param[in] speed m/s to walk at, slow enough for the robot's gait
 * @param[in] canSit false for a robot that can't: then sitting and powering down must be refused
*/
void liveALife(SimRobot& robot, double speed, bool canSit = true) {
    ASSERT_TRUE(robot.engine.robotSupported()) << joined(robot.errors);
    robot.stage = "standing up";
    ASSERT_TRUE(robot.standUp()) << "never stood up" << joined(robot.warnings);

    // Walk forward, then turn on the spot
    robot.stage = "walking";
    robot.engine.setVelocity({sc(speed), 0, 0});
    robot.run(8.0);
    EXPECT_GT(robot.engine.odometry().x(), 0.4 * speed * 8.0) << "walked forward";
    EXPECT_NEAR(robot.engine.odometry().y(), 0.0, 0.03);
    robot.stage = "turning";
    robot.engine.setVelocity({0, 0, sc(0.15)});
    const Scalar yawBefore = robot.engine.odometry().yaw();
    robot.run(8.0);
    EXPECT_GT(robot.engine.odometry().yaw() - yawBefore, 0.3) << "turned anticlockwise";

    // Stop: the feet step back to their stance
    robot.stage = "stopping";
    robot.engine.setVelocity({0, 0, 0});
    robot.run(10.0);

    // Tilt mode: lean and shift, then come back
    robot.stage = "tilting";
    ASSERT_TRUE(robot.engine.setMode(gait_core::MODE_TILT).success);
    robot.run(1.0);
    robot.engine.setBodyPose(sc(0.08), sc(-0.05), 0, test_robots::v(0.01, 0, 0));
    robot.run(2.0);
    robot.engine.setMode(gait_core::MODE_NORMAL);
    robot.run(3.0);

    // Sit, power down, and stand again from where it was left
    if (!canSit) {
        robot.stage = "refusing to sit";
        const gait_core::CommandResult sit = robot.engine.sit(true);
        EXPECT_FALSE(sit.success);
        EXPECT_NE(std::string(sit.message).find("sit_reach"), std::string::npos) << sit.message;
        EXPECT_FALSE(robot.engine.powerDown().success) << "it would have to sit first";
        robot.run(2.0);
        EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
        EXPECT_TRUE(robot.servoPower);
        EXPECT_TRUE(robot.problem.empty()) << robot.problem;
        EXPECT_TRUE(robot.errors.empty()) << joined(robot.errors);
        return;
    }
    robot.stage = "sitting";
    ASSERT_TRUE(robot.engine.sit(true).success);
    EXPECT_TRUE(robot.runUntil([&] { return robot.engine.sitState() == SitState::SEATED; }, 30.0)) << "sat down";
    robot.stage = "powering down";
    ASSERT_TRUE(robot.engine.powerDown().success);
    EXPECT_TRUE(robot.runUntil([&] { return !robot.servoPower; }, 10.0)) << "servos off";
    robot.stage = "standing again";
    robot.engine.sit(false);
    EXPECT_TRUE(robot.runUntil([&] { return robot.engine.sitState() == SitState::STANDING; }, 40.0)) << "stood again";

    EXPECT_TRUE(robot.problem.empty()) << robot.problem;
    EXPECT_TRUE(robot.errors.empty()) << joined(robot.errors);
}

}  // namespace

// ---- Explorer, as the control -----------------------------------------------------------------

TEST(Explorer, LivesALife) {
    SimRobot robot{explorerRobot()};
    liveALife(robot, 0.08);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

// ---- A quadruped --------------------------------------------------------------------------------

TEST(Quadruped, LivesALife) {
    SimRobot robot{test_robots::quadruped()};
    liveALife(robot, 0.03);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

// ---- Four joints per leg --------------------------------------------------------------------------

TEST(FourJointLegs, LiveALife) {
    SimRobot robot{test_robots::fourJointHexapod()};
    ASSERT_EQ(robot.engine.legSolver(0), gait_core::LegSolver::Chain);
    liveALife(robot, 0.08);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

// ---- Two joints per leg ---------------------------------------------------------------------------

TEST(TwoJointLegs, LiveALife) {
    SimRobot robot{test_robots::twoJointHexapod()};
    // Its foot moves on a surface, so it walks and turns but can't do what needs a foot held at a
    // height of its choosing: the seated pose (it stands straight up instead), sitting, or a full tilt
    liveALife(robot, 0.05, false);
    for (const std::string& warning : robot.warnings) {
        const bool expected = warning.find("straight to the standing pose") != std::string::npos ||
            warning.find("Tilt pose limited") != std::string::npos || warning.find("Refused to sit") != std::string::npos;
        EXPECT_TRUE(expected) << warning;
    }
}

// ---- The same lives with dynamic stability on ------------------------------------------------------

namespace {

GaitParams dynamicStability() {
    GaitParams params;
    params.dynamic_stability = true;
    params.balance_feedback = true;   // and Phase B's on top
    params.preview_control = true;    // and Phase C's
    return params;
}

}  // namespace

TEST(Explorer, LivesALifeWithDynamicStability) {
    SimRobot robot{explorerRobot(), dynamicStability()};
    liveALife(robot, 0.08);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

TEST(Quadruped, LivesALifeWithDynamicStability) {
    SimRobot robot{test_robots::quadruped(), dynamicStability()};
    liveALife(robot, 0.03);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

TEST(FourJointLegs, LiveALifeWithDynamicStability) {
    SimRobot robot{test_robots::fourJointHexapod(), dynamicStability()};
    liveALife(robot, 0.08);
    EXPECT_TRUE(robot.warnings.empty()) << joined(robot.warnings);
}

TEST(TwoJointLegs, LiveALifeWithDynamicStability) {
    SimRobot robot{test_robots::twoJointHexapod(), dynamicStability()};
    liveALife(robot, 0.05, false);
    for (const std::string& warning : robot.warnings) {
        const bool expected = warning.find("straight to the standing pose") != std::string::npos ||
            warning.find("Tilt pose limited") != std::string::npos || warning.find("Refused to sit") != std::string::npos;
        EXPECT_TRUE(expected) << warning;
    }
}

// ---- And with sensors: an IMU and foot forces, so balance feedback measures rather than assumes ----
// Nothing pushes these robots, so feedback must never take a step to catch one. The simulated IMU
// always reads level, even while tilt mode leans the body, so tilt leveling winds up against the
// legs' reach and says so (with every feature off too): that one warning is the simulation's.

namespace {

void expectOnlySimulatedImuWarnings(const SimRobot& robot) {
    for (const std::string& warning : robot.warnings) {
        EXPECT_NE(warning.find("Tilt pose limited"), std::string::npos) << warning;
    }
}

}  // namespace

TEST(Explorer, LivesALifeWithSensorsAndFeedback) {
    SimRobot robot{explorerRobot(), dynamicStability()};
    robot.sensors = true;
    liveALife(robot, 0.08);
    expectOnlySimulatedImuWarnings(robot);
    EXPECT_EQ(robot.catchingTicks, 0);
}

TEST(Quadruped, LivesALifeWithSensorsAndFeedback) {
    SimRobot robot{test_robots::quadruped(), dynamicStability()};
    robot.sensors = true;
    liveALife(robot, 0.03);
    expectOnlySimulatedImuWarnings(robot);
    EXPECT_EQ(robot.catchingTicks, 0);
}

TEST(FourJointLegs, LiveALifeWithSensorsAndFeedback) {
    SimRobot robot{test_robots::fourJointHexapod(), dynamicStability()};
    robot.sensors = true;
    liveALife(robot, 0.08);
    expectOnlySimulatedImuWarnings(robot);
    EXPECT_EQ(robot.catchingTicks, 0);
}

TEST(TwoJointLegs, LiveALifeWithSensorsAndFeedback) {
    SimRobot robot{test_robots::twoJointHexapod(), dynamicStability()};
    robot.sensors = true;
    liveALife(robot, 0.05, false);
    for (const std::string& warning : robot.warnings) {
        const bool expected = warning.find("straight to the standing pose") != std::string::npos ||
            warning.find("Tilt pose limited") != std::string::npos || warning.find("Refused to sit") != std::string::npos;
        EXPECT_TRUE(expected) << warning;
    }
    EXPECT_EQ(robot.catchingTicks, 0);
}

TEST(FourJointLegs, StandBackInTheSamePoseWhereverTheyWalked) {
    // Four joints reach each foot position many ways. Walking and stepping back to the stance must
    // still end in the pose the legs started in, not wherever the path left the spare joint.
    SimRobot robot{test_robots::fourJointHexapod()};
    ASSERT_TRUE(robot.standUp());
    robot.run(2.0);
    const auto before = robot.feedback.angles;
    robot.engine.setVelocity({sc(0.08), 0, sc(0.2)});
    robot.run(6.0);
    robot.engine.setVelocity({sc(-0.08), sc(0.03), sc(-0.2)});
    robot.run(6.0);
    robot.engine.setVelocity({0, 0, 0});
    robot.run(10.0);
    for (int leg = 0; leg < 6; ++leg) {
        EXPECT_LT((robot.feedback.angles[leg] - before[leg]).cwiseAbs().maxCoeff(), 0.005) << "leg " << leg;
    }
    EXPECT_TRUE(robot.problem.empty()) << robot.problem;
}
