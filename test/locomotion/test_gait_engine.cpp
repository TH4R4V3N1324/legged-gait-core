#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "test_common.hpp"
#include "gait_core/locomotion/gait_engine.hpp"
#include "gait_core/locomotion/support_polygon.hpp"

using gait_core::CommandResult;
using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::GaitParams;
using gait_core::ImuSample;
using gait_core::JointFeedback;
using gait_core::JointVec;
using gait_core::explorer::NUM_LEGS;
using gait_core::Scalar;
using gait_core::SitState;
using gait_core::Timestamp;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);  // s
constexpr Timestamp DT_NS = 10'000'000;

/**
 * @brief Runs an engine with perfect servos: every command comes straight back as the joint state
 * @note The servos only answer while the engine wants them powered and the e-stop is released,
 *       like the leg hubs' SERVO_EN and kill loop
*/
class Robot {
public:
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    Timestamp now = 1'000'000'000;
    JointFeedback feedback;
    int commandedTicks = 0;
    bool estop = false;
    bool servoPower;   // what the engine last asked for

    explicit Robot(const GaitParams& params = GaitParams{}, const gait_core::RobotDefinition& definition = explorerRobot())
        : engine(params, definition), servoPower(params.power_on_startup) {
        // Legs laid out flat at their zeros
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = JointVec::Zero(3);
        }
    }

    /**
     * @brief The robot slumps onto its belly with its legs at their zeros
    */
    void collapse() {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.angles[leg] = JointVec::Zero(3);
        }
    }

    void run(double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        for (int i = 0; i < ticks; ++i) {
            if (servoPower && !estop) {
                feedback.stamp = now;
                engine.onJointStates(feedback);
            }
            now += DT_NS;
            const GaitOutput out = engine.update(DT, now);
            servoPower = out.servoPower;
            if (out.commanded) {
                ++commandedTicks;
                for (int leg = 0; leg < NUM_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }
};

}  // namespace

TEST(GaitEngine, WaitsForEveryJointBeforeMoving) {
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    JointFeedback partial;
    for (int leg = 0; leg < gait_core::explorer::LEG_FRONT_LEFT; ++leg) {  // the front left leg missing
        partial.present[leg] = true;
    }
    engine.onJointStates(partial);
    for (int i = 0; i < 50; ++i) {
        EXPECT_FALSE(engine.update(DT, (i + 1) * DT_NS).commanded);
    }
    EXPECT_FALSE(engine.initialized());
}

TEST(GaitEngine, StartsSeatedThenStandsWhenAsked) {
    Robot robot;
    robot.run(3.0);
    EXPECT_TRUE(robot.engine.initialized());
    EXPECT_EQ(robot.engine.sitState(), SitState::SEATED);

    const CommandResult refused = robot.engine.setMode(gait_core::MODE_TILT);
    EXPECT_FALSE(refused.success) << "tilt mode needs the robot standing";

    EXPECT_TRUE(robot.engine.sit(false).success);
    robot.run(6.0);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_NEAR(robot.engine.gaitConfig().currentHeight, robot.engine.gaitConfig().startPos.z(), tol(1e-9));
}

TEST(GaitEngine, WalkingForwardMovesTheOdometryForward) {
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);

    // 0.1 m/s for 4 s: about 0.4 m, less the time the gait takes to get going
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(4.0);
    const auto& odom = robot.engine.odometry();
    EXPECT_GT(odom.x(), 0.25);
    EXPECT_LT(odom.x(), 0.45);
    EXPECT_NEAR(odom.y(), 0.0, 0.02);
    EXPECT_NEAR(odom.yaw(), 0.0, 0.02);
}

TEST(GaitEngine, TurningLeftTurnsTheOdometryAnticlockwise) {
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);

    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(4.0);
    EXPECT_GT(robot.engine.odometry().yaw(), 0.5);
    EXPECT_NEAR(robot.engine.odometry().x(), 0.0, 0.03);
}

/**
 * @brief A robot that's been through startup and is standing
*/
Robot standingRobot(const GaitParams& params = GaitParams{}) {
    Robot robot(params);
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    return robot;
}

TEST(GaitEngine, ControllerTiltLatchesNeutralAndRecentresOnRelease) {
    Robot robot = standingRobot();
    ASSERT_TRUE(robot.engine.setMode(gait_core::MODE_TILT).success);

    ImuSample controller;
    controller.stamp = robot.now;
    controller.roll = static_cast<Scalar>(0.4);
    robot.engine.setControllerImu(controller, true);
    EXPECT_TRUE(robot.engine.controllerTiltActive());

    controller.stamp = robot.now + DT_NS;
    controller.roll = static_cast<Scalar>(0.5);
    robot.engine.setControllerImu(controller, true);
    robot.engine.setControllerImu(controller, false);
    EXPECT_FALSE(robot.engine.controllerTiltActive());
}

namespace {

/**
 * @brief An IMU sample stamped now, with angular velocity and linear acceleration
*/
ImuSample imuWithRates(Timestamp now) {
    ImuSample sample;
    sample.stamp = now;
    sample.roll = static_cast<Scalar>(0.05);
    sample.angularVelocity = gait_core::Vec3(static_cast<Scalar>(0.1), static_cast<Scalar>(-0.2), static_cast<Scalar>(0.3));
    sample.linearAcceleration = gait_core::Vec3(static_cast<Scalar>(0.5), 0, static_cast<Scalar>(9.81));
    return sample;
}

}  // namespace

TEST(GaitEngine, ImuRatesAndAccelerationAreKept) {
    Robot robot;
    const ImuSample sample = imuWithRates(robot.now);
    robot.engine.setImu(sample);
    robot.run(0.01);
    EXPECT_EQ(robot.engine.imuAngularVelocity(), sample.angularVelocity);
    EXPECT_EQ(robot.engine.imuLinearAcceleration(), sample.linearAcceleration);
}

TEST(GaitEngine, PowerLossRebuildKeepsTheImuRates) {
    Robot robot = standingRobot();
    const ImuSample sample = imuWithRates(robot.now);
    robot.engine.setImu(sample);
    robot.engine.servoPowerLost();
    EXPECT_FALSE(robot.engine.servoPower());
    EXPECT_EQ(robot.engine.imuAngularVelocity(), sample.angularVelocity);
    EXPECT_EQ(robot.engine.imuLinearAcceleration(), sample.linearAcceleration);
}

TEST(GaitEngine, ServosStayOffUntilTheFirstStand) {
    GaitParams params;
    params.power_on_startup = false;
    Robot robot(params);
    robot.run(3.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_EQ(robot.commandedTicks, 0) << "nothing is commanded while the servos are off";
    EXPECT_FALSE(robot.engine.initialized());

    const CommandResult stand = robot.engine.sit(false);
    EXPECT_TRUE(stand.success);
    EXPECT_NE(std::string(stand.message).find("powering"), std::string::npos) << stand.message;
    robot.run(9.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_TRUE(robot.engine.initialized());
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING) << "one stand command: power up, seated pose, stand";
}

TEST(GaitEngine, PowerDownSitsThenTurnsTheServosOff) {
    Robot robot = standingRobot();
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_TRUE(robot.engine.powerDown().success);
    robot.run(0.5);
    EXPECT_TRUE(robot.servoPower) << "still powered while sitting down";
    robot.run(8.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_TRUE(robot.engine.seated());
    EXPECT_NEAR(gait_core::YawPitchPitchLeg(explorerRobot().legs[0]).fk(robot.feedback.angles[0]).z(), -robot.engine.params().sit_height, 1e-3)
        << "the servos went off with the robot seated";

    const int commanded = robot.commandedTicks;
    robot.run(2.0);
    EXPECT_EQ(robot.commandedTicks, commanded);
    EXPECT_STREQ(robot.engine.powerDown().message, "servos already off");

    // Standing powers back up
    robot.engine.sit(false);
    robot.run(10.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, StandingUpBeforeTheServosGoOffCancelsThePowerDown) {
    Robot robot = standingRobot();
    robot.engine.powerDown();
    robot.run(1.0);
    robot.engine.sit(false);
    robot.run(10.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, LosingTheServosWaitsForAStandBeforeMovingAgain) {
    Robot robot = standingRobot();
    int seatedReports = 0;
    robot.engine.seatedChanged = [&seatedReports](bool seated) { seatedReports += seated ? 1 : 0; };

    // E-stop: the servos stop answering and the robot collapses
    robot.estop = true;
    robot.run(1.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_TRUE(robot.engine.seated());
    EXPECT_EQ(seatedReports, 1);
    robot.collapse();

    // Releasing the e-stop moves nothing by itself
    robot.estop = false;
    const int commanded = robot.commandedTicks;
    robot.run(2.0);
    EXPECT_EQ(robot.commandedTicks, commanded);

    // A stand starts again from where the legs are now, through the seated pose
    robot.engine.sit(false);
    robot.run(0.3);
    EXPECT_TRUE(robot.engine.seated());
    robot.run(10.0);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, JointStatesThatKeepComingNeverCountAsAPowerLoss) {
    GaitParams params;
    params.servo_loss_timeout = static_cast<Scalar>(0.05);
    Robot robot = standingRobot(params);
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(3.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, EveryCommandIsReachable) {
    // Whatever the gait does, IK must never be asked for a pose outside the joint limits
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    robot.engine.setMode(gait_core::MODE_STRAFE);
    robot.engine.setVelocity({static_cast<Scalar>(0.08), static_cast<Scalar>(0.06), static_cast<Scalar>(0.3)});
    for (int i = 0; i < 400; ++i) {
        robot.run(0.01);
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const auto& q = robot.feedback.angles[leg];
            ASSERT_TRUE(q.allFinite()) << "leg " << leg;
            EXPECT_LE(std::abs(q(0)), 1.4 + 1e-6) << "leg " << leg;
            EXPECT_GE(q(1), -1.6 - 1e-6) << "leg " << leg;
            EXPECT_LE(q(1), 1.4 + 1e-6) << "leg " << leg;
        }
    }
}

// ---- Per-leg kinematics (JointVec) -----------------------------------------------------------

namespace {

/**
 * @brief The built in leg, counting how often the engine solves with it
*/
class CountingLeg : public gait_core::LegKinematics {
public:
    mutable int calls = 0;
    int numJoints() const override { return leg.numJoints(); }
    gait_core::Vec3 fk(const JointVec& q) const override { ++calls; return leg.fk(q); }
    bool ik(const gait_core::Vec3& target, const JointVec& seed, JointVec& q) const override {
        ++calls;
        return leg.ik(target, seed, q);
    }
    gait_core::LegJacobian jacobian(const JointVec& q) const override { return leg.jacobian(q); }
private:
    gait_core::YawPitchPitchLeg leg{explorerRobot().legs[0]};
};

}  // namespace

TEST(GaitEngine, JointVectorsStartSizedAndZeroNotEmpty) {
    // A default JointVec is empty, so a forgotten resize would hand callers zero-length vectors
    const JointFeedback feedback;
    const GaitOutput output;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(feedback.angles[leg], JointVec::Zero(3)) << "leg " << leg;
        EXPECT_EQ(feedback.effort[leg], JointVec::Zero(3)) << "leg " << leg;
        EXPECT_EQ(output.joints[leg], JointVec::Zero(3)) << "leg " << leg;
    }
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    const GaitOutput first = engine.update(DT, DT_NS);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(first.joints[leg], JointVec::Zero(3)) << "leg " << leg;
    }
}

TEST(GaitEngine, WrongSizedAnglesCountAsMissing) {
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    JointFeedback feedback;
    feedback.present.fill(true);
    feedback.angles[gait_core::explorer::LEG_REAR_LEFT] = JointVec::Zero(2);
    engine.onJointStates(feedback);
    for (int i = 0; i < 20; ++i) {
        EXPECT_FALSE(engine.update(DT, (i + 1) * DT_NS).commanded);
    }
}

TEST(GaitEngine, PowerLossRebuildKeepsLegKinematicsAndJointSizes) {
    const CountingLeg counting;   // outlives the robot, as setLegKinematics requires
    Robot robot = standingRobot();
    robot.engine.setLegKinematics(gait_core::explorer::LEG_FRONT_RIGHT, &counting);

    // Sit and cut the servos: cutServoPower rebuilds the engine and moves it into place
    ASSERT_TRUE(robot.engine.powerDown().success);
    robot.run(8.5);
    ASSERT_FALSE(robot.servoPower);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(robot.feedback.angles[leg].size(), 3) << "leg " << leg;
    }

    // Power up and stand: the override must still be in use, and every command finite and sized
    const int callsBefore = counting.calls;
    robot.engine.sit(false);
    for (int i = 0; i < 200 && robot.engine.sitState() != SitState::STANDING; ++i) {
        robot.run(0.1);   // up to 20 s to rise and gather the feet in
    }
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_GT(counting.calls, callsBefore) << "the rebuilt engine dropped the leg's kinematics";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        ASSERT_EQ(robot.feedback.angles[leg].size(), 3) << "leg " << leg;
        EXPECT_TRUE(robot.feedback.angles[leg].allFinite()) << "leg " << leg;
    }

    // Null goes back to the built in leg
    robot.engine.setLegKinematics(gait_core::explorer::LEG_FRONT_RIGHT, nullptr);
    const int callsAfterReset = counting.calls;
    robot.run(0.5);
    EXPECT_EQ(counting.calls, callsAfterReset);
}

TEST(GaitEngine, CopiedEngineDoesNotUseTheOriginalsKinematics) {
    // The built in leg is reached through a null entry, not a pointer to the engine's own member,
    // so a copy keeps working after the original is gone
    std::unique_ptr<Robot> original = std::make_unique<Robot>(standingRobot());
    Robot copy = *original;
    original.reset();
    copy.engine.setVelocity({static_cast<Scalar>(0.05), 0, 0});
    copy.run(2.0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_TRUE(copy.feedback.angles[leg].allFinite()) << "leg " << leg;
    }
}

TEST(GaitEngine, EachLegStandsAtTheSharedStancePlusItsOffset) {
    using gait_core::explorer::LEG_FRONT_RIGHT;
    using gait_core::explorer::LEG_REAR_LEFT;
    const gait_core::Vec3 offset(static_cast<Scalar>(0.015), static_cast<Scalar>(-0.01), static_cast<Scalar>(0.005));
    gait_core::RobotDefinition definition = explorerRobot();
    definition.legs[LEG_FRONT_RIGHT].stanceOffset = offset;

    // Kept through height and reach changes
    gait_core::GaitConfig config{definition};
    config.setHeight(static_cast<Scalar>(-0.1));
    config.setReach(static_cast<Scalar>(0.2));
    EXPECT_NEAR((config.startPosition[LEG_FRONT_RIGHT] - (config.startPos + offset)).norm(), 0.0, tol(1e-9));
    EXPECT_NEAR((config.startPosition[LEG_REAR_LEFT] - config.startPos).norm(), 0.0, tol(1e-9));

    // And the robot stands on them
    Robot robot{GaitParams{}, definition};
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    const gait_core::YawPitchPitchLeg leg{definition.legs[0]};
    const gait_core::Vec3 stance = robot.engine.gaitConfig().startPos;
    EXPECT_NEAR((leg.fk(robot.feedback.angles[LEG_FRONT_RIGHT]) - (stance + offset)).norm(), 0.0, 1e-3);
    EXPECT_NEAR((leg.fk(robot.feedback.angles[LEG_REAR_LEFT]) - stance).norm(), 0.0, 1e-3);
}

namespace {

/**
 * @brief A square quadruped with explorer's legs at the corners: legs 0..3 front right, rear right,
 *        rear left, front left. Gait 0 trots (diagonal pairs), gait 1 crawls (one leg at a time).
*/
gait_core::RobotDefinition quadruped() {
    using gait_core::kPi;
    gait_core::RobotDefinition robot = explorerRobot();
    const gait_core::LegDefinition leg = robot.legs[0];
    const Scalar c = static_cast<Scalar>(0.12);
    const Scalar yaws[4] = {-kPi / 4, -3 * kPi / 4, 3 * kPi / 4, kPi / 4};
    const Scalar xs[4] = {c, -c, -c, c};
    const Scalar ys[4] = {-c, -c, c, c};
    robot.numLegs = 4;
    for (int i = 0; i < 4; ++i) {
        robot.legs[i] = leg;
        robot.legs[i].mount = {yaws[i], gait_core::Vec3(xs[i], ys[i], 0)};
    }
    robot.numGaits = 2;
    robot.gaits[0] = gait_core::GaitPattern::fromGroups("trot", {{0, 2}, {1, 3}});
    robot.gaits[1] = gait_core::GaitPattern::fromGroups("crawl", {{1}, {0}, {2}, {3}});
    // Its weight exactly at the centre, which these tests' geometry relies on
    robot.bodyMass = 0;
    robot.centreOfMass = gait_core::Vec3::Zero();
    return robot;
}

/**
 * @brief How far the centre of mass is inside all of a robot's feet, from its commanded joints
*/
Scalar standingMargin(const Robot& robot, const gait_core::RobotDefinition& definition) {
    std::array<gait_core::Vec2, gait_core::kMaxLegs> feet;
    const gait_core::YawPitchPitchLeg leg{definition.legs[0]};
    for (int i = 0; i < definition.numLegs; ++i) {
        feet[i] = robot.engine.trajectory().legToBody(i, leg.fk(robot.feedback.angles[i])).head<2>();
    }
    return gait_core::stabilityMargin(feet.data(), definition.numLegs, definition.centreOfMass.head<2>());
}

}  // namespace

TEST(GaitEngine, ExplorersGaitsAreAllStaticallyStable) {
    const GaitEngine engine{GaitParams{}, explorerRobot()};
    for (uint8_t gait = 0; gait < gait_core::explorer::NUM_GAITS; ++gait) {
        // Even the tripod, the narrowest, keeps its centre of mass several centimetres in
        EXPECT_GT(engine.gaitStabilityMargin(gait), 0.03) << engine.gaitConfig().gaitName(gait);
    }
    EXPECT_EQ(engine.unstableGait(), -1);
    EXPECT_TRUE(engine.robotSupported());
}

TEST(GaitEngine, AQuadrupedsTrotIsRefusedUnlessAllowed) {
    const GaitEngine strict{GaitParams{}, quadruped()};
    const double stride = quadruped().maxStride;
    // Two diagonal feet: the centre of mass is on their line, and a stride takes it off. Swaying
    // can't help: two feet have no area to move over.
    EXPECT_NEAR(strict.gaitStabilityMargin(0), -stride, tol(1e-6)) << "trot";
    EXPECT_FALSE(strict.gaitSways(0));
    EXPECT_EQ(strict.unstableGait(), 0);
    EXPECT_FALSE(strict.robotSupported());

    GaitParams params;
    params.allow_unstable_gaits = true;
    const GaitEngine allowed{params, quadruped()};
    EXPECT_EQ(allowed.unstableGait(), -1);
    EXPECT_TRUE(allowed.robotSupported());
}

TEST(GaitEngine, AnUnstableRobotNeverMoves) {
    Robot robot{GaitParams{}, quadruped()};
    robot.run(1.0);
    EXPECT_EQ(robot.commandedTicks, 0);
}

TEST(GaitEngine, TiltShiftStopsWhereTheCentreOfMassNearsTheEdgeOfTheFeet) {
    // Centre of mass moved forward until it is 30 mm inside the standing feet: a 40 mm forward
    // shift would take it past min_stability_margin (10 mm), so the shift has to stop short
    gait_core::RobotDefinition definition = explorerRobot();
    definition.bodyMass = 0;   // the whole robot's centre of mass, fixed where this test puts it
    const GaitEngine neutral{GaitParams{}, definition};
    std::array<gait_core::Vec2, gait_core::kMaxLegs> feet;
    for (int i = 0; i < NUM_LEGS; ++i) {
        feet[i] = neutral.trajectory().legToBody(i, neutral.gaitConfig().startPosition[i]).head<2>();
    }
    Scalar low = 0, high = static_cast<Scalar>(0.3);
    for (int i = 0; i < 40; ++i) {
        const Scalar mid = (low + high) / 2;
        (gait_core::stabilityMargin(feet.data(), NUM_LEGS, gait_core::Vec2(mid, 0)) > static_cast<Scalar>(0.03) ? low : high) = mid;
    }
    definition.centreOfMass.x() = low;

    GaitParams params;
    params.allow_unstable_gaits = true;   // walking with it this far forward isn't safe; standing is
    params.body_sway = false;             // nor is swaying: this is about tilt
    params.tilt_leveling = false;
    Robot robot{params, definition};
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_NEAR(standingMargin(robot, definition), 0.03, 1e-3);

    ASSERT_TRUE(robot.engine.setMode(gait_core::MODE_TILT).success);
    robot.run(1.0);   // tilt mode starts from neutral, so pose it once it has
    robot.engine.setBodyPose(0, 0, 0, gait_core::Vec3(static_cast<Scalar>(0.04), 0, 0));
    robot.run(3.0);
    // Shifted forward, the feet are further back in the body frame, and the margin is what's left
    EXPECT_NEAR(standingMargin(robot, definition), 0.01, 1e-3);
}

TEST(GaitEngine, TiltShiftGoesAllTheWayWhenTheFeetAllowIt) {
    Robot robot = standingRobot();
    const Scalar before = standingMargin(robot, explorerRobot());
    ASSERT_TRUE(robot.engine.setMode(gait_core::MODE_TILT).success);
    robot.run(1.0);   // tilt mode starts from neutral, so pose it once it has
    robot.engine.setBodyPose(0, 0, 0, gait_core::Vec3(static_cast<Scalar>(0.04), 0, 0));
    robot.run(3.0);
    EXPECT_LT(standingMargin(robot, explorerRobot()), before - 0.03) << "most of the 40 mm shift happened";
}

TEST(GaitEngine, TrajectoriesFollowAHeightChange) {
    // The trajectory generator once kept its own copy of the stance, so it planned feet at the
    // robot's original height after setHeight
    Robot robot = standingRobot();
    const Scalar original = robot.engine.gaitConfig().currentHeight;
    ASSERT_TRUE(robot.engine.setHeight(static_cast<Scalar>(-0.09)).success);
    robot.run(4.0);
    const Scalar height = robot.engine.gaitConfig().currentHeight;
    ASSERT_GT(std::abs(height - original), 0.02) << "the body moved to its new height";
    EXPECT_NEAR(robot.engine.trajectory().swingTarget(0, gait_core::Vec3::Zero()).z(), height, tol(1e-9));
    EXPECT_NEAR(robot.engine.trajectory().config().startPosition[0].z(), height, tol(1e-9));
}

// ---- Body sway ----------------------------------------------------------------------------------

namespace {

/**
 * @brief quadruped() with only its crawl, so nothing else stops it walking
*/
gait_core::RobotDefinition crawlingQuadruped() {
    gait_core::RobotDefinition robot = quadruped();
    robot.numGaits = 1;
    robot.gaits[0] = robot.gaits[1];
    return robot;
}

/**
 * @brief Sit then stand, waiting as long as it takes (a swaying gait gathers its feet slowly)
*/
bool standUp(Robot& robot) {
    robot.run(3.0);
    robot.engine.sit(false);
    for (int i = 0; i < 200 && robot.engine.sitState() != SitState::STANDING; ++i) {
        robot.run(0.1);
    }
    return robot.engine.sitState() == SitState::STANDING;
}

/**
 * @brief Walk a robot and find how close its centre of mass came to the edge of its planted feet
 *        while a foot was in the air
 * @note From the commanded joints alone: a foot more than 1 cm above the lowest is in the air.
*/
Scalar leastMarginWhileStepping(Robot& robot, const gait_core::RobotDefinition& definition, double seconds) {
    const gait_core::YawPitchPitchLeg leg{definition.legs[0]};
    Scalar least = std::numeric_limits<Scalar>::infinity();
    const int ticks = static_cast<int>(std::lround(seconds / 0.01));
    for (int t = 0; t < ticks; ++t) {
        robot.run(0.01);
        std::array<gait_core::Vec3, gait_core::kMaxLegs> feet;
        Scalar lowest = std::numeric_limits<Scalar>::infinity();
        for (int i = 0; i < definition.numLegs; ++i) {
            feet[i] = robot.engine.trajectory().legToBody(i, leg.fk(robot.feedback.angles[i]));
            lowest = std::min(lowest, feet[i].z());
        }
        std::array<gait_core::Vec2, gait_core::kMaxLegs> planted;
        int count = 0;
        for (int i = 0; i < definition.numLegs; ++i) {
            if (feet[i].z() < lowest + static_cast<Scalar>(0.01)) {
                planted[count++] = feet[i].head<2>();
            }
        }
        if (count < definition.numLegs) {
            least = std::min(least, gait_core::stabilityMargin(planted.data(), count, definition.centreOfMass.head<2>()));
        }
    }
    return least;
}

}  // namespace

TEST(GaitEngine, AQuadrupedsCrawlSwaysToStayStable) {
    // Corners of the square at 0.12 + 0.177 / sqrt(2) from the centre: each crawl triangle is half
    // the square, whose largest circle has radius side * (2 - sqrt(2)) / 2. Less the stride.
    const double side = 2 * (0.12 + 0.177 / std::sqrt(2.0));
    const double stride = quadruped().maxStride;
    const GaitEngine engine{GaitParams{}, quadruped()};
    EXPECT_TRUE(engine.gaitSways(1));
    EXPECT_NEAR(engine.gaitStabilityMargin(1), side * (2 - std::sqrt(2.0)) / 2 - stride, tol(1e-6));

    GaitParams noSway;
    noSway.body_sway = false;
    const GaitEngine without{noSway, quadruped()};
    EXPECT_FALSE(without.gaitSways(1));
    EXPECT_NEAR(without.gaitStabilityMargin(1), -stride, tol(1e-6)) << "the centre is on the diagonal";
}

TEST(GaitEngine, ExplorerNeverSways) {
    Robot robot = standingRobot();
    for (uint8_t gait = 0; gait < gait_core::explorer::NUM_GAITS; ++gait) {
        EXPECT_FALSE(robot.engine.gaitSways(gait));
    }
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, static_cast<Scalar>(0.2)});
    robot.run(3.0);
    EXPECT_EQ(robot.engine.bodySway(), gait_core::Vec2::Zero());
}

TEST(GaitEngine, ACrawlingQuadrupedKeepsItsCentreOfMassOverItsFeet) {
    const gait_core::RobotDefinition definition = crawlingQuadruped();
    Robot robot{GaitParams{}, definition};
    ASSERT_TRUE(robot.engine.robotSupported());
    std::vector<std::string> warnings;   // e.g. a sway that timed out, or the robot about to tip
    robot.engine.log = [&](gait_core::LogLevel level, const char* message) {
        if (level >= gait_core::LogLevel::Warn) {
            warnings.emplace_back(message);
        }
    };
    ASSERT_TRUE(standUp(robot));

    robot.engine.setVelocity({static_cast<Scalar>(0.03), 0, 0});
    const Scalar least = leastMarginWhileStepping(robot, definition, 12.0);
    EXPECT_TRUE(warnings.empty()) << warnings.front();
    EXPECT_GT(least, 0.01 - 0.002) << "every lift waited for the body to sway over the other three feet";
    EXPECT_GT(robot.engine.odometry().x(), 0.05) << "and it still walked";
}

TEST(GaitEngine, WithoutSwayACrawlTipsTheCentreOfMassOffItsFeet) {
    // The same walk with sway off, allowed to run: lifting a leg leaves the centre of mass on or
    // past the diagonal of the other three
    const gait_core::RobotDefinition definition = crawlingQuadruped();
    GaitParams params;
    params.body_sway = false;
    params.allow_unstable_gaits = true;
    Robot robot{params, definition};
    ASSERT_TRUE(standUp(robot));
    robot.engine.setVelocity({static_cast<Scalar>(0.03), 0, 0});
    EXPECT_LT(leastMarginWhileStepping(robot, definition, 12.0), 0.005);
}

TEST(GaitEngine, SwayReturnsToCentreWhenTheRobotStops) {
    const gait_core::RobotDefinition definition = crawlingQuadruped();
    Robot robot{GaitParams{}, definition};
    ASSERT_TRUE(standUp(robot));
    robot.engine.setVelocity({static_cast<Scalar>(0.03), 0, 0});
    robot.run(4.0);
    EXPECT_GT(robot.engine.bodySway().norm(), 0.005) << "swaying while it crawls";
    robot.engine.setVelocity({0, 0, 0});
    robot.run(10.0);
    EXPECT_LT(robot.engine.bodySway().norm(), 1e-6);
}

TEST(GaitEngine, ATrotEasesOffTheSwayLeftByACrawl) {
    // The trot doesn't sway, but it must not keep walking off-centre by what the crawl left behind
    GaitParams params;
    params.allow_unstable_gaits = true;   // a trot on two feet always is
    Robot robot{params, quadruped()};
    ASSERT_FALSE(robot.engine.gaitSways(0));
    ASSERT_TRUE(robot.engine.gaitSways(1));
    ASSERT_TRUE(robot.engine.setGait(1).success);
    ASSERT_TRUE(standUp(robot));
    robot.engine.setVelocity({static_cast<Scalar>(0.03), 0, 0});
    robot.run(4.0);
    ASSERT_GT(robot.engine.bodySway().norm(), 0.005) << "swaying while it crawls";

    ASSERT_TRUE(robot.engine.setGait(0).success);
    robot.run(6.0);
    ASSERT_EQ(robot.engine.gaitConfig().currentGait, 0);
    EXPECT_LT(robot.engine.bodySway().norm(), 1e-6) << "still walking, now trotting";
}

TEST(GaitEngine, ATrotEasesOffTheSwayLeftByGathering) {
    // Standing up gathers the feet in a crawl, which sways; the trot then starts from wherever that left off
    gait_core::RobotDefinition definition = quadruped();   // the crawl first, so it gathers in it
    std::swap(definition.gaits[0], definition.gaits[1]);
    GaitParams params;
    params.allow_unstable_gaits = true;
    Robot robot{params, definition};
    ASSERT_TRUE(robot.engine.gaitSways(0));
    ASSERT_FALSE(robot.engine.gaitSways(1));
    robot.run(3.0);
    robot.engine.sit(false);
    Scalar most = 0;
    for (int i = 0; i < 3000 && robot.engine.sitState() != SitState::STANDING; ++i) {
        robot.run(0.01);
        most = std::max(most, robot.engine.bodySway().norm());
    }
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    ASSERT_GT(most, 0.005) << "gathering the feet swayed the body";

    ASSERT_TRUE(robot.engine.setGait(1).success);
    robot.engine.setVelocity({0, static_cast<Scalar>(0.03), 0});   // strafing, as when the Go2 fell
    robot.run(6.0);
    ASSERT_EQ(robot.engine.gaitConfig().currentGait, 1);
    EXPECT_LT(robot.engine.bodySway().norm(), 1e-6);
}

// ---- Per-joint speed limits ------------------------------------------------------------------

namespace {

/**
 * @brief Explorer with a speed limit on some of its joints
 * @param[in] joint Which joint of every leg to limit (0 coxa, 1 femur, 2 tibia), or -1 for all
 * @param[in] maxVelocity The limit (rad/s)
*/
gait_core::RobotDefinition explorerWithJointLimit(int joint, double maxVelocity) {
    gait_core::RobotDefinition def = explorerRobot();
    for (int leg = 0; leg < def.numLegs; ++leg) {
        for (int j = 0; j < def.legs[leg].numJoints; ++j) {
            if (joint < 0 || j == joint) {
                def.legs[leg].joints[j].max_velocity = static_cast<Scalar>(maxVelocity);
            }
        }
    }
    return def;
}

/**
 * @brief How fast each joint moved while walking
*/
struct JointSpeeds {
    std::vector<double> fastest = std::vector<double>(3, 0.0);   // rad/s, per joint, over every leg
    std::vector<int> ticksOver = std::vector<int>(3, 0);         // leg-ticks above the given limit
    int samples = 0;                                             // leg-ticks measured, per joint
};

/**
 * @brief Stands a robot up, then walks it forward, measuring each joint's speed while walking
 * @param[in] limit Speed to count the ticks above (rad/s)
*/
JointSpeeds walkAndMeasureJointSpeeds(Robot& robot, double seconds, double limit = 1e9) {
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    robot.engine.setVelocity({static_cast<Scalar>(0.15), 0, 0});
    JointSpeeds result;
    auto last = robot.feedback.angles;
    const int ticks = static_cast<int>(std::lround(seconds / 0.01));
    for (int i = 0; i < ticks; ++i) {
        robot.run(0.01);
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            for (int j = 0; j < 3; ++j) {
                const double speed = std::abs(robot.feedback.angles[leg](j) - last[leg](j)) / 0.01;
                result.fastest[j] = std::max(result.fastest[j], speed);
                result.ticksOver[j] += speed > limit ? 1 : 0;
            }
            ++result.samples;
        }
        last = robot.feedback.angles;
    }
    return result;
}

}  // namespace

TEST(GaitEngine, AJointsOwnSpeedLimitHoldsOnlyThatJointBack) {
    Robot unlimited;
    const JointSpeeds free = walkAndMeasureJointSpeeds(unlimited, 4.0);
    ASSERT_GT(free.fastest[1], 1.0) << "the femurs must outrun the limit for the test to mean anything";

    // e.g. a slower servo on the femurs than on the coxas and tibias
    const double limit = 0.5;
    Robot slowFemurs{GaitParams{}, explorerWithJointLimit(1, limit)};
    const JointSpeeds limited = walkAndMeasureJointSpeeds(slowFemurs, 4.0, limit * 1.05);
    // The limiter can let a leg overshoot for a single tick as a phase hands over from one gait
    // group to the next (as it does with max_joint_speed alone); otherwise the femurs hold to it
    EXPECT_LE(limited.ticksOver[1], limited.samples / 100) << "femur ticks over the limit";
    EXPECT_LT(limited.fastest[1], free.fastest[1]) << "femur";
    // The whole gait slows to keep the femurs in, so it covers less ground
    EXPECT_LT(slowFemurs.engine.odometry().x(), unlimited.engine.odometry().x() - 0.05);
}

TEST(GaitEngine, JointSpeedLimitsOverrideTheGlobalOne) {
    // A global limit far too slow to walk at, overridden on every joint by the default's 4 rad/s,
    // walks exactly as the default does
    // (explorer's own limits cleared, so the global ones apply where the test wants them)
    const double unset = std::numeric_limits<double>::infinity();
    GaitParams slow;
    slow.max_joint_speed = static_cast<Scalar>(0.3);
    Robot overridden{slow, explorerWithJointLimit(-1, 4.0)};
    Robot reference{GaitParams{}, explorerWithJointLimit(-1, unset)};
    walkAndMeasureJointSpeeds(overridden, 3.0);
    walkAndMeasureJointSpeeds(reference, 3.0);
    EXPECT_NEAR(overridden.engine.odometry().x(), reference.engine.odometry().x(), tol(1e-9));

    Robot globallySlow{slow, explorerWithJointLimit(-1, unset)};
    walkAndMeasureJointSpeeds(globallySlow, 3.0);
    EXPECT_LT(globallySlow.engine.odometry().x(), reference.engine.odometry().x() - 0.05);
}

TEST(GaitEngine, StandingUpPlantsEveryFoot) {
    // Stepping back in to the standing reach used to stop as soon as every foot was within 1 mm of
    // its start, leaving the last group hovering mid-swing for the first step to drop in one tick.
    // Slow joints make the swings slow enough for one to end a tick inside that 1 mm, which left
    // a foot about 0.96 mm up.
    GaitParams slow;
    slow.max_joint_speed = static_cast<Scalar>(0.5);
    const gait_core::RobotDefinition definition = explorerWithJointLimit(-1, std::numeric_limits<double>::infinity());
    Robot robot{slow, definition};
    robot.run(3.0);
    robot.engine.sit(false);
    // The moment it stands, before anything else can move the feet
    for (int i = 0; i < 3000 && robot.engine.sitState() != SitState::STANDING; ++i) {
        robot.run(0.01);
    }
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const gait_core::Vec3 foot = gait_core::YawPitchPitchLeg(definition.legs[leg]).fk(robot.feedback.angles[leg]);
        // Within a fraction of a millimetre: joints this slow trail the height glide a little
        EXPECT_NEAR(foot.z(), definition.standHeight, 2e-4) << "leg " << leg;
    }
}

TEST(GaitEngine, NoJointEverExceedsItsSpeedLimit) {
    // From power up, through standing, walking in every gait, reversing and stopping: the gait
    // slows for the joints, and whatever it can't foresee (a landing target swinging round when the
    // command reverses, say) the joint command itself is held to
    const double femurLimit = 1.0;
    const gait_core::RobotDefinition definition = explorerWithJointLimit(1, femurLimit);
    for (uint8_t gait = 0; gait < gait_core::explorer::NUM_GAITS; ++gait) {
        Robot robot{GaitParams{}, definition};
        std::vector<std::string> failures;
        auto last = robot.feedback.angles;
        auto run = [&](double seconds) {
            const int ticks = static_cast<int>(std::lround(seconds / 0.01));
            for (int i = 0; i < ticks; ++i) {
                robot.run(0.01);
                for (int leg = 0; leg < NUM_LEGS; ++leg) {
                    for (int j = 0; j < 3; ++j) {
                        const double limit = definition.legs[leg].joints[j].max_velocity;
                        const double step = std::abs(robot.feedback.angles[leg](j) - last[leg](j));
                        const double speed = step / 0.01;
                        // A float build's angles are only good to about tol()
                        if (step > limit * 0.01 + tol(1e-9) && failures.size() < 5) {
                            failures.push_back("leg " + std::to_string(leg) + " joint " + std::to_string(j) +
                                ": " + std::to_string(speed) + " rad/s");
                        }
                    }
                }
                last = robot.feedback.angles;
            }
        };
        run(3.0);
        robot.engine.sit(false);
        run(8.0);
        robot.engine.setGait(gait);
        run(3.0);
        robot.engine.setVelocity({static_cast<Scalar>(0.25), 0, 0});
        run(4.0);
        robot.engine.setVelocity({static_cast<Scalar>(-0.25), 0, 0});
        run(2.0);
        robot.engine.setMode(gait_core::MODE_STRAFE);
        robot.engine.setVelocity({static_cast<Scalar>(0.08), static_cast<Scalar>(0.06), static_cast<Scalar>(0.3)});
        run(3.0);
        robot.engine.setVelocity({0, 0, 0});
        run(5.0);
        EXPECT_TRUE(failures.empty()) << "gait " << int(gait) << ", first: " << (failures.empty() ? "" : failures[0]);
    }
}

// ---------------------------------------------------------------------------------------------
// Centre of mass and dynamic stability
// ---------------------------------------------------------------------------------------------

namespace {

/**
 * @brief Run a tick at a time, calling check after each
*/
template <typename Check>
void runChecking(Robot& robot, double seconds, Check check) {
    const int ticks = static_cast<int>(std::lround(seconds / 0.01));
    for (int i = 0; i < ticks; ++i) {
        robot.run(0.01);
        check();
    }
}

/**
 * @brief Params whose min_stability_margin sits just under explorer's tripod margin, so the
 *        dynamic limits have little to spare
*/
GaitParams tightMargins(bool dynamicStability) {
    const Scalar tripod = GaitEngine(GaitParams{}, explorerRobot()).gaitStabilityMargin(gait_core::explorer::GAIT_TRIPOD);
    GaitParams params;
    params.min_stability_margin = tripod - static_cast<Scalar>(0.005);
    params.dynamic_stability = dynamicStability;
    return params;
}

}  // namespace

TEST(GaitEngine, TheCentreOfMassMovesAsTheLegsSwing) {
    Robot robot = standingRobot();
    const gait_core::Vec3 standing = robot.engine.centreOfMass();
    // The legs are two thirds of explorer, so the whole robot's isn't the body's own
    EXPECT_GT((standing - explorerRobot().centreOfMass).norm(), 0.002);
    EXPECT_NEAR(standing.y(), 0, 0.002);

    Scalar lowest = standing.x(), highest = standing.x();
    robot.engine.setVelocity({static_cast<Scalar>(0.15), 0, 0});
    runChecking(robot, 3.0, [&] {
        lowest = std::min(lowest, robot.engine.centreOfMass().x());
        highest = std::max(highest, robot.engine.centreOfMass().x());
    });
    EXPECT_GT(highest - lowest, 0.003);
}

TEST(GaitEngine, WithoutMassesTheCentreOfMassIsFixed) {
    gait_core::RobotDefinition definition = explorerRobot();
    definition.bodyMass = 0;
    definition.centreOfMass = gait_core::Vec3(static_cast<Scalar>(0.01), 0, 0);
    Robot robot(GaitParams{}, definition);
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    robot.engine.setVelocity({static_cast<Scalar>(0.15), 0, 0});
    runChecking(robot, 2.0, [&] { ASSERT_EQ(robot.engine.centreOfMass(), definition.centreOfMass); });
}

TEST(GaitEngine, BalanceStateFollowsTheWalk) {
    Robot robot = standingRobot();
    const Scalar speed = static_cast<Scalar>(0.08);
    robot.engine.setVelocity({speed, 0, 0});
    robot.run(2.0);   // up to speed

    double sum = 0;
    int ticks = 0;
    runChecking(robot, 2.2, [&] {   // two tripod cycles, so the legs' own motion averages out
        const gait_core::BalanceState& s = robot.engine.balanceState();
        sum += s.comVelocity.x();
        ++ticks;
        EXPECT_NEAR(s.omega, std::sqrt(9.81 / s.height), 1e-3);
        EXPECT_NEAR((s.capturePoint - (s.com.head<2>() + s.comVelocity / s.omega)).norm(), 0, 1e-5);
        EXPECT_GT(s.captureMargin, 0);
        EXPECT_GE(s.plantedCount, 3);
    });
    EXPECT_NEAR(sum / ticks, speed, 0.015);
    // The centre of mass sits between the hips and the feet
    EXPECT_GT(robot.engine.balanceState().height, 0.08);
    EXPECT_LT(robot.engine.balanceState().height, 0.16);
}

TEST(GaitEngine, DynamicStabilityLimitsHowHardTheBodySpeedsUp) {
    // Asking for full speed at once: off, the body takes it in a tick; on, it ramps up
    auto velocityAfter = [](bool dynamicStability, double seconds) {
        Robot robot = standingRobot(tightMargins(dynamicStability));
        robot.engine.setVelocity({static_cast<Scalar>(0.3), 0, 0});
        robot.run(seconds);
        return robot.engine.balanceState().comVelocity.x();
    };
    const Scalar off = velocityAfter(false, 0.01);
    const Scalar on = velocityAfter(true, 0.01);
    EXPECT_GT(off, 0.2);
    EXPECT_LT(on, 0.6 * off);
    // It still gets there
    EXPECT_NEAR(velocityAfter(true, 1.0), velocityAfter(false, 1.0), 0.05);
}

namespace {

/**
 * @brief A quadruped crawling at full speed for 6 s, from standing
 * @param[out] worstZmp The ZMP's smallest margin inside the planted feet on the way (m)
 * @param[out] heldTicks How many ticks a lift was held
 * @return The distance walked (m)
*/
Scalar crawl(bool dynamicStability, Scalar& worstZmp, int& heldTicks) {
    GaitParams params;
    params.allow_unstable_gaits = true;   // its trot is, not its crawl
    params.dynamic_stability = dynamicStability;
    Robot robot(params, quadruped());
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.5);
    robot.engine.setGait(1);
    robot.engine.setVelocity({static_cast<Scalar>(0.3), 0, 0});
    worstZmp = 1;
    heldTicks = 0;
    runChecking(robot, 6.0, [&] {
        worstZmp = std::min(worstZmp, robot.engine.balanceState().zmpMargin);
        heldTicks += robot.engine.liftHeld() ? 1 : 0;
    });
    return robot.engine.odometry().x();
}

}  // namespace

TEST(GaitEngine, DynamicStabilityKeepsACrawlsZmpInsideItsFeet) {
    Scalar worstOff, worstOn;
    int heldOff, heldOn;
    const Scalar walkedOff = crawl(false, worstOff, heldOff);
    const Scalar walkedOn = crawl(true, worstOn, heldOn);
    // Off, a full speed crawl lifts legs the body's momentum can't do without: the ZMP leaves the feet
    EXPECT_LT(worstOff, 0);
    EXPECT_EQ(heldOff, 0);
    // On, lifts wait for it to fit, and it stays inside
    EXPECT_GT(heldOn, 0);
    EXPECT_GT(worstOn, 0.005);
    // At little cost in speed
    EXPECT_GT(walkedOn, 0.85 * walkedOff);
}

TEST(GaitEngine, DynamicStabilityChangesNothingForExplorersTripod) {
    // A wide hexapod's lifts always fit: its walk is the same with or without
    auto walk = [](bool dynamicStability) {
        GaitParams params;
        params.dynamic_stability = dynamicStability;
        Robot robot = standingRobot(params);
        robot.engine.setVelocity({static_cast<Scalar>(0.15), 0, 0});
        bool held = false;
        runChecking(robot, 4.0, [&] { held = held || robot.engine.liftHeld(); });
        EXPECT_FALSE(held);
        return robot.engine.odometry().x();
    };
    EXPECT_NEAR(walk(true), walk(false), 0.01);
}

TEST(GaitEngine, DynamicStabilityStillWalksAndTurns) {
    GaitParams params;
    params.dynamic_stability = true;
    Robot robot = standingRobot(params);
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(4.0);
    EXPECT_GT(robot.engine.odometry().x(), 0.25);
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(4.0);
    EXPECT_GT(robot.engine.odometry().yaw(), 0.5);
}
