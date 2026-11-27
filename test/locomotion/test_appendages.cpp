// Appendages (heads, arms): their weight and what they hold count towards the robot's centre of mass

#include <gtest/gtest.h>

#include <cmath>

#include "test_robots.hpp"
#include "gait_core/kinematics/mass_properties.hpp"

using gait_core::AppendageDefinition;
using gait_core::GaitParams;
using gait_core::JointVec;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::Vec3;
using test_robots::SimRobot;

namespace {

constexpr int ARM = 0;
const Vec3 DOWN(0, 0, static_cast<Scalar>(-9.81));

JointVec armAngles(double turn, double shoulder) {
    JointVec q(2);
    q << static_cast<Scalar>(turn), static_cast<Scalar>(shoulder);
    return q;
}

// The whole robot's mass as its definition gives it: body, legs, the arm's links
Scalar definedMass(const RobotDefinition& robot) {
    Scalar mass = robot.bodyMass;
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        for (int j = 0; j < robot.legs[leg].numJoints; ++j) {
            mass += robot.legs[leg].joints[j].mass;
        }
    }
    for (int i = 0; i < robot.numAppendages; ++i) {
        for (int j = 0; j < robot.appendages[i].numJoints; ++j) {
            mass += robot.appendages[i].joints[j].mass;
        }
    }
    return mass;
}

// The arm's tip in the body frame: where a 1 kg load on a weightless copy of it sits
Vec3 tipInBody(const AppendageDefinition& arm, const JointVec& q) {
    AppendageDefinition weightless = arm;
    for (int j = 0; j < weightless.numJoints; ++j) {
        weightless.joints[j].mass = 0;
    }
    const Vec3 tip = gait_core::appendageMass(weightless, q, 1).centre();
    return arm.mount.translation_offset + Eigen::AngleAxis<Scalar>(arm.mount.yaw, Vec3::UnitZ()) * tip;
}

/**
 * @brief Explorer with its arm, standing, the arm reported at an angle every tick
*/
struct ArmRobot {
    explicit ArmRobot(const GaitParams& params = GaitParams{}, const RobotDefinition& definition = test_robots::explorerWithArm())
        : sim(definition, params) {
        point(armAngles(0, 0));
        EXPECT_TRUE(sim.standUp());
    }
    void point(const JointVec& q) {
        sim.feedback.appendagePresent[ARM] = true;
        sim.feedback.appendageAngles[ARM] = q;
        sim.feedback.appendageEffort[ARM] = JointVec::Zero(q.size());
    }
    // The efforts holding the arm still at its angles with a load at the tip (gravity straight down)
    void holding(Scalar kg) {
        sim.efforts = true;
        sim.feedback.appendageEffort[ARM] = gait_core::appendageHoldTorques(sim.definition.appendages[ARM],
            sim.feedback.appendageAngles[ARM], DOWN, kg);
    }
    SimRobot sim;
};

}  // namespace

TEST(Appendages, ARobotWithoutThemIsSizedForNone) {
    SimRobot robot(explorerRobot());
    robot.run(0.5);
    EXPECT_EQ(robot.engine.numAppendages(), 0);
    for (int i = 0; i < gait_core::kMaxAppendages; ++i) {
        EXPECT_EQ(robot.last.appendageTorques[i].size(), 0) << "appendage " << i;
    }
}

TEST(Appendages, TheArmsWeightMovesTheCentreOfMassWhereverItPoints) {
    ArmRobot robot;
    robot.sim.run(0.5);
    const Vec3 forward = robot.sim.engine.centreOfMass();
    // Swung round to point back: its 0.3 kg moves 0.15 m back (0.075 out either way)
    robot.point(armAngles(M_PI, 0));
    robot.sim.run(0.5);
    const Vec3 back = robot.sim.engine.centreOfMass();
    const Scalar mass = definedMass(robot.sim.definition);
    EXPECT_NEAR(forward.x() - back.x(), 0.3 * 0.15 / mass, 1e-4);
    EXPECT_NEAR(forward.y() - back.y(), 0, 1e-4);
    // Raised straight up: its weight 0.075 m over the shoulder, and over the mount sideways
    robot.point(armAngles(0, -M_PI / 2));
    robot.sim.run(0.5);
    const Vec3 up = robot.sim.engine.centreOfMass();
    EXPECT_NEAR(forward.x() - up.x(), 0.3 * 0.075 / mass, 1e-4);
    EXPECT_NEAR(up.z() - forward.z(), 0.3 * 0.075 / mass, 1e-4);
}

TEST(Appendages, ALoadWorkedOutFromTheEffortsShiftsTheWeight) {
    ArmRobot robot;
    robot.sim.run(0.5);
    const Vec3 empty = robot.sim.engine.centreOfMass();
    EXPECT_EQ(robot.sim.engine.appendagePayload(ARM), 0);

    // It picks up 0.5 kg: the shoulder works harder than the arm's own weight needs
    robot.holding(0.5f);
    robot.sim.run(3.0);   // six of payload_filter's time constants
    EXPECT_NEAR(robot.sim.engine.appendagePayload(ARM), 0.5, 0.01);
    const Scalar mass = definedMass(robot.sim.definition);
    const Vec3 tip = tipInBody(robot.sim.definition.appendages[ARM], armAngles(0, 0));
    const Vec3 expected = (mass * empty + 0.5f * tip) / (mass + 0.5f);
    EXPECT_NEAR((robot.sim.engine.centreOfMass() - expected).norm(), 0, 2e-4);

    // And puts it down
    robot.holding(0);
    robot.sim.run(3.0);
    EXPECT_NEAR(robot.sim.engine.appendagePayload(ARM), 0, 0.01);
    EXPECT_NEAR((robot.sim.engine.centreOfMass() - empty).norm(), 0, 2e-4);
}

TEST(Appendages, ALoadIsKeptBetweenNothingAndTheMost) {
    ArmRobot robot;
    robot.holding(3);   // more than its 1 kg
    robot.sim.run(3.0);
    EXPECT_NEAR(robot.sim.engine.appendagePayload(ARM), 1, 0.01);
    robot.holding(-1);  // something pushing the tip up: no load
    robot.sim.run(3.0);
    EXPECT_NEAR(robot.sim.engine.appendagePayload(ARM), 0, 0.01);
}

TEST(Appendages, AHeadNeverMakesUpALoad) {
    RobotDefinition definition = test_robots::explorerWithArm();
    definition.appendages[ARM].maxPayload = 0;   // a head: nothing to carry
    ArmRobot robot(GaitParams{}, definition);
    robot.holding(0.5f);
    robot.sim.run(2.0);
    EXPECT_EQ(robot.sim.engine.appendagePayload(ARM), 0);
}

TEST(Appendages, ALoadSetByTheRobotOverridesTheEfforts) {
    ArmRobot robot;
    robot.holding(0.5f);
    robot.sim.engine.setAppendagePayload(ARM, 0.25f);
    robot.sim.run(2.0);
    EXPECT_EQ(robot.sim.engine.appendagePayload(ARM), static_cast<Scalar>(0.25)) << "as set, whatever the efforts say";
    robot.sim.engine.setAppendagePayload(ARM, -1);   // back to working it out
    robot.sim.run(3.0);
    EXPECT_NEAR(robot.sim.engine.appendagePayload(ARM), 0.5, 0.01);
    robot.sim.engine.setAppendagePayload(7, 1);      // not an appendage: ignored
}

TEST(Appendages, TorqueOutputHoldsTheArmAndItsLoadUp) {
    GaitParams params;
    params.torque_output = true;
    ArmRobot robot(params);
    robot.point(armAngles(0.4, 0.3));
    robot.sim.engine.setAppendagePayload(ARM, 0.6f);
    robot.sim.run(0.5);
    const JointVec expected = gait_core::appendageHoldTorques(robot.sim.definition.appendages[ARM], armAngles(0.4, 0.3), DOWN, 0.6f);
    ASSERT_EQ(robot.sim.last.appendageTorques[ARM].size(), 2);
    EXPECT_NEAR((robot.sim.last.appendageTorques[ARM] - expected).norm(), 0, 1e-4);
    EXPECT_LT(robot.sim.last.appendageTorques[ARM](1), -1) << "the shoulder holds a 0.6 kg load out";

    // Without torque output, nothing (still sized)
    ArmRobot off;
    off.sim.run(0.2);
    ASSERT_EQ(off.sim.last.appendageTorques[ARM].size(), 2);
    EXPECT_EQ(off.sim.last.appendageTorques[ARM].norm(), 0);
}

TEST(Appendages, CuttingTheServosKeepsWhereTheArmIsAndWhatItHolds) {
    ArmRobot robot;
    robot.point(armAngles(1.0, -0.5));
    robot.sim.engine.setAppendagePayload(ARM, static_cast<Scalar>(0.4));
    robot.sim.run(0.3);
    ASSERT_TRUE(robot.sim.engine.powerDown().success);
    robot.sim.run(8.5);
    ASSERT_FALSE(robot.sim.servoPower);
    EXPECT_EQ(robot.sim.engine.appendagePayload(ARM), static_cast<Scalar>(0.4));
    EXPECT_EQ(robot.sim.engine.appendageAngles(ARM), armAngles(1.0, -0.5));
}

TEST(Appendages, ItWalksWhileTheArmSwingsAndCarries) {
    GaitParams params;
    params.torque_output = true;
    ArmRobot robot(params);
    robot.sim.engine.setVelocity({static_cast<Scalar>(0.05), 0, 0});
    Scalar lowest = 1, highest = -1;
    for (int i = 0; i < 300; ++i) {
        robot.point(armAngles(0.8 * std::sin(i * 0.05), 0.4 * std::sin(i * 0.03)));
        robot.holding(0.8f);
        robot.sim.run(0.01);
        lowest = std::min(lowest, robot.sim.engine.centreOfMass().y());
        highest = std::max(highest, robot.sim.engine.centreOfMass().y());
    }
    EXPECT_TRUE(robot.sim.problem.empty()) << robot.sim.problem;
    EXPECT_GT(robot.sim.engine.odometry().x(), 0.05) << "it walked";
    EXPECT_GT(highest - lowest, 0.005) << "the centre of mass followed the arm from side to side";
}
