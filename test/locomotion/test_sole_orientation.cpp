// Soles: the engine turns each one with the robot and holds it flat under the body's pose

#include <gtest/gtest.h>

#include <cmath>

#include "test_robots.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/locomotion/trajectory_generator.hpp"

using gait_core::GaitParams;
using gait_core::JointVec;
using gait_core::Mat3;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::Vec3;
using test_robots::SimRobot;

namespace {

/**
 * @brief A leg's own chain, solved as usual, that records the sole it's asked for
*/
class SoleSpy : public gait_core::LegKinematics {
public:
    explicit SoleSpy(const gait_core::LegDefinition& leg) : chain(leg) {}
    int numJoints() const override { return chain.numJoints(); }
    Vec3 fk(const JointVec& q) const override { return chain.fk(q); }
    bool ik(const Vec3& target, const JointVec& seed, JointVec& q) const override { return chain.ik(target, seed, q); }
    bool ikWithSole(const Vec3& target, const Mat3& s, const JointVec& seed, JointVec& q) const override {
        sole = s;
        ++calls;
        return chain.ik(target, seed, q);
    }
    gait_core::LegJacobian jacobian(const JointVec& q) const override { return chain.jacobian(q); }
    mutable Mat3 sole = Mat3::Identity();
    mutable int calls = 0;
private:
    gait_core::ChainKinematics chain;
};

// The heading of a sole's forward axis (rad, about z)
Scalar headingOf(const Mat3& sole) {
    return std::atan2(sole(1, 0), sole(0, 0));
}

/**
 * @brief The biped, one foot at a time with both down between, its legs spied on
*/
struct SpiedBiped {
    explicit SpiedBiped(const GaitParams& params = test_robots::bipedParams(false))
        : definition(test_robots::biped()), left(definition.legs[0]), right(definition.legs[1]), sim(definition, params) {
        sim.engine.setLegKinematics(0, &left);
        sim.engine.setLegKinematics(1, &right);
    }
    RobotDefinition definition;
    SoleSpy left, right;   // before sim: the engine points at them
    SimRobot sim;
};

}  // namespace

TEST(SoleOrientation, PointFeetAreLeftAlone) {
    SimRobot robot(explorerRobot());
    ASSERT_TRUE(robot.standUp());
    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(2.0);
    for (int leg = 0; leg < robot.definition.numLegs; ++leg) {
        EXPECT_TRUE(robot.last.soles[leg].isIdentity(1e-6f)) << "leg " << leg;
    }
}

TEST(SoleOrientation, StandingTheSolesFaceForwardFlat) {
    SpiedBiped robot;
    ASSERT_TRUE(robot.sim.standUp());
    robot.sim.run(1.0);
    EXPECT_GT(robot.left.calls, 0) << "the engine solves legs with soles through ikWithSole";
    for (int leg = 0; leg < 2; ++leg) {
        EXPECT_TRUE(robot.sim.last.soles[leg].isIdentity(1e-5f)) << "leg " << leg;
    }
    EXPECT_TRUE(robot.left.sole.isIdentity(1e-5f));
}

TEST(SoleOrientation, TurningAPlantedSoleKeepsItsHeadingOnTheGround) {
    SpiedBiped robot;
    ASSERT_TRUE(robot.sim.standUp());
    const Scalar rate = static_cast<Scalar>(0.3);
    robot.sim.engine.setVelocity({0, 0, rate});
    robot.sim.run(1.0);
    // Turning on the spot, a planted foot's position turns about the body's centre by just as much as
    // the body turns over it. Its sole has to turn by the same angle in the body, or it twists on the
    // ground: so over each planted stretch, the sole's heading less the angle of the foot's position
    // stays put. (Not checked against leg odometry, which can't see the body turn over a single foot.)
    double worstDrift = 0, widest = 0, footTurned = 0;
    int stretches = 0;
    std::array<bool, 2> wasDown{};
    std::array<double, 2> plantedOffset{}, plantedAngle{};
    for (int i = 0; i < 800; ++i) {
        robot.sim.run(0.01);
        const Scalar z0 = robot.sim.foot(0).z(), z1 = robot.sim.foot(1).z();
        const Scalar ground = std::min(z0, z1);
        for (int leg = 0; leg < 2; ++leg) {
            const bool down = (leg == 0 ? z0 : z1) < ground + static_cast<Scalar>(0.001);
            const double inBody = headingOf(robot.sim.last.soles[leg]);
            widest = std::max(widest, std::abs(inBody));
            const Vec3 foot = robot.sim.foot(leg);
            const double angle = std::atan2(foot.y(), foot.x());
            const double offset = std::remainder(inBody - angle, 2 * M_PI);
            if (down && !wasDown[leg]) {
                plantedOffset[leg] = offset;
                plantedAngle[leg] = angle;
                ++stretches;
            } else if (down) {
                worstDrift = std::max(worstDrift, std::abs(std::remainder(offset - plantedOffset[leg], 2 * M_PI)));
                footTurned = std::max(footTurned, std::abs(std::remainder(angle - plantedAngle[leg], 2 * M_PI)));
            }
            wasDown[leg] = down;
        }
    }
    EXPECT_GE(stretches, 6);
    EXPECT_GT(footTurned, 0.1) << "the planted feet turned under the body";
    EXPECT_LT(worstDrift, 0.002) << "a planted sole twisted on the ground";
    // Never further round than a swing takes it: the turn over half a stance (3 phases of 0.4 s)
    EXPECT_LT(widest, rate * 3 * 0.4 / 2 + 0.01);
    EXPECT_GT(widest, rate * 3 * 0.4 / 4) << "the soles did turn";
    EXPECT_TRUE(robot.sim.problem.empty()) << robot.sim.problem;
}

TEST(SoleOrientation, SteppingBackToStanceStraightensThem) {
    GaitParams params = test_robots::bipedParams(false);
    params.idle_return_delay = static_cast<Scalar>(0.5);
    SpiedBiped robot(params);
    ASSERT_TRUE(robot.sim.standUp());
    robot.sim.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.sim.run(3.0);
    robot.sim.engine.setVelocity({0, 0, 0});
    robot.sim.run(6.0);
    for (int leg = 0; leg < 2; ++leg) {
        EXPECT_NEAR(headingOf(robot.sim.last.soles[leg]), 0, 1e-4) << "leg " << leg;
    }
}

TEST(SoleOrientation, InTiltModeTheSolesStayFlatAndSquareToTheGround) {
    SpiedBiped robot;
    ASSERT_TRUE(robot.sim.standUp());
    ASSERT_TRUE(robot.sim.engine.setMode(gait_core::MODE_TILT).success);
    robot.sim.run(0.5);
    gait_core::BodyPose pose;
    pose.roll = static_cast<Scalar>(0.08);
    pose.pitch = static_cast<Scalar>(-0.06);
    pose.yaw = static_cast<Scalar>(0.1);
    robot.sim.engine.setBodyPose(pose.roll, pose.pitch, pose.yaw, Vec3::Zero());
    robot.sim.run(2.0);
    // Turned back through the body's pose, each sole is the identity again: flat, facing as it was put down
    const Mat3 rotation = gait_core::TrajectoryGenerator::rotationOf(pose);
    for (int leg = 0; leg < 2; ++leg) {
        const Mat3 onGround = rotation * robot.sim.last.soles[leg];
        EXPECT_TRUE(onGround.isIdentity(1e-4f)) << "leg " << leg << "\n" << onGround;
    }
    EXPECT_FALSE(robot.sim.last.soles[0].isIdentity(1e-3f)) << "relative to the tilted body they're turned";
}
