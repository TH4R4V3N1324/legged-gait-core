#include <gtest/gtest.h>

#include <cmath>
#include <type_traits>

#include "test_common.hpp"
#include "gait_core/kinematics/mass_properties.hpp"

using gait_core::JointVec;
using gait_core::LegDefinition;
using gait_core::MassMoment;
using gait_core::Scalar;
using gait_core::Vec3;

namespace {

constexpr double TOL = 1e-5;

/**
 * @brief Explorer's front right leg, masses and all
*/
LegDefinition explorerLeg() {
    return explorerRobot().legs[gait_core::explorer::LEG_FRONT_RIGHT];
}

JointVec angles(double coxa, double femur, double tibia) {
    JointVec q(3);
    q << static_cast<Scalar>(coxa), static_cast<Scalar>(femur), static_cast<Scalar>(tibia);
    return q;
}

void expectNear(const Vec3& actual, const Vec3& expected) {
    EXPECT_NEAR((actual - expected).norm(), 0, TOL) << "actual (" << actual.transpose() << "), expected ("
                                                      << expected.transpose() << ")";
}

}  // namespace

TEST(MassProperties, ALegWithoutMassesWeighsNothing) {
    LegDefinition leg = explorerLeg();
    for (auto& joint : leg.joints) {
        joint.mass = 0;
    }
    const MassMoment m = gait_core::legMass(leg, angles(0.3, -0.2, 1.0));
    EXPECT_EQ(m.mass, 0);
    EXPECT_EQ(m.centre(Vec3(1, 2, 3)), Vec3(1, 2, 3));
}

TEST(MassProperties, ExplorersLegWeighsItsThreeLinks) {
    const MassMoment m = gait_core::legMass(explorerLeg(), angles(0, 0, 0));
    EXPECT_NEAR(m.mass, 0.0934 + 0.1724 + 0.1034, TOL);
}

TEST(MassProperties, AtZeroEachLinkSitsOutAlongTheLeg) {
    const LegDefinition leg = explorerLeg();
    const MassMoment m = gait_core::legMass(leg, angles(0, 0, 0));
    // Coxa at the coxa joint, femur 0.063 m out, tibia 0.163 m out
    const Vec3 expected = (Scalar(0.0934) * Vec3(Scalar(0.02018), Scalar(-0.00006), Scalar(-0.00074)) +
                           Scalar(0.1724) * Vec3(Scalar(0.063 + 0.05), Scalar(-0.00002), 0) +
                           Scalar(0.1034) * Vec3(Scalar(0.163 + 0.07831), 0, Scalar(0.01279))) / m.mass;
    expectNear(m.centre(), expected);
}

TEST(MassProperties, TurningTheCoxaTurnsTheWholeLeg) {
    const LegDefinition leg = explorerLeg();
    const Vec3 straight = gait_core::legMass(leg, angles(0, 0.4, 0.9)).centre();
    const Vec3 turned = gait_core::legMass(leg, angles(0.5, 0.4, 0.9)).centre();
    expectNear(turned, gait_core::AngleAxis(Scalar(0.5), Vec3::UnitZ()) * straight);
}

TEST(MassProperties, EachLinkTurnsWithItsJoint) {
    // Lowering the femur by q1 and folding the tibia by q2: the tibia's centre is at the knee plus
    // its offset turned by both
    const LegDefinition leg = explorerLeg();
    const double q1 = 0.4, q2 = 0.9;  // rad
    const MassMoment m = gait_core::legMass(leg, angles(0, q1, q2));
    const auto turn = [](double angle) { return gait_core::AngleAxis(static_cast<Scalar>(angle), Vec3::UnitY()); };
    const Vec3 femurJoint(Scalar(0.063), 0, 0);
    const Vec3 knee = femurJoint + turn(q1) * Vec3(Scalar(0.1), 0, 0);
    const Vec3 expected = (Scalar(0.0934) * leg.joints[0].centreOfMass +
                           Scalar(0.1724) * (femurJoint + turn(q1) * leg.joints[1].centreOfMass) +
                           Scalar(0.1034) * (knee + turn(q1 + q2) * leg.joints[2].centreOfMass)) / m.mass;
    expectNear(m.centre(), expected);
}

TEST(MassProperties, AnglesOfTheWrongSizeGiveNothing) {
    JointVec q = JointVec::Zero(2);
    EXPECT_EQ(gait_core::legMass(explorerLeg(), q).mass, 0);
}

TEST(MassProperties, ExplorerWeighsWhatItsCadSays) {
    const gait_core::RobotDefinition robot = explorerRobot();
    MassMoment total;
    total.add(robot.bodyMass, robot.centreOfMass);
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        total += gait_core::legMass(robot.legs[leg], angles(0, 0, 0));
    }
    EXPECT_NEAR(total.mass, 3.323, 1e-3);
}

// ---------------------------------------------------------------------------------------------
// Appendages: a head or an arm's links, what it holds, and the torques that hold it up
// ---------------------------------------------------------------------------------------------

namespace {

/**
 * @brief A three-joint arm, each link weighted and offset, so every term of the maths is exercised
*/
gait_core::AppendageDefinition threeJointArm() {
    gait_core::AppendageDefinition arm;
    arm.numJoints = 3;
    const Vec3 axes[3] = {Vec3::UnitZ(), Vec3::UnitY(), Vec3::UnitY()};
    const Vec3 origins[3] = {Vec3(0.01f, 0, 0.02f), Vec3(0, 0.01f, 0.03f), Vec3(0.12f, 0, 0)};
    const Scalar masses[3] = {0.2f, 0.35f, 0.15f};
    const Vec3 centres[3] = {Vec3(0, 0, 0.01f), Vec3(0.06f, 0.005f, 0), Vec3(0.05f, 0, -0.01f)};
    for (int j = 0; j < 3; ++j) {
        arm.joints[j].origin = origins[j];
        arm.joints[j].axis = axes[j];
        arm.joints[j].mass = masses[j];
        arm.joints[j].centreOfMass = centres[j];
    }
    arm.tip = Vec3(0.1f, 0, 0);
    return arm;
}

JointVec armAngles(double a, double b, double c) {
    JointVec q(3);
    q << static_cast<Scalar>(a), static_cast<Scalar>(b), static_cast<Scalar>(c);
    return q;
}

}  // namespace

TEST(MassProperties, AnAppendageWeighsItsLinksAndWhatItHolds) {
    const gait_core::AppendageDefinition arm = threeJointArm();
    const MassMoment bare = gait_core::appendageMass(arm, armAngles(0, 0, 0));
    EXPECT_NEAR(bare.mass, 0.7, TOL);
    // Straight out along x at zero: the payload hangs at the tip, 0.01 + 0.12 + 0.1 out and 0.05 up
    const MassMoment loaded = gait_core::appendageMass(arm, armAngles(0, 0, 0), 0.5f);
    EXPECT_NEAR(loaded.mass, 1.2, TOL);
    expectNear(Vec3((loaded.moment - bare.moment) / 0.5f), Vec3(0.23f, 0.01f, 0.05f));
    EXPECT_EQ(gait_core::appendageMass(arm, JointVec::Zero(2)).mass, 0) << "angles of the wrong size give nothing";
}

TEST(MassProperties, HoldingALinkOutLevelTakesItsWeightTimesItsReach) {
    // One link, 1 kg at 0.1 m along x, on a pitch joint: gravity turns it nose down (+ about y), so
    // the joint pushes back with -m g r
    gait_core::AppendageDefinition arm;
    arm.numJoints = 1;
    arm.joints[0].axis = Vec3::UnitY();
    arm.joints[0].mass = 1;
    arm.joints[0].centreOfMass = Vec3(0.1f, 0, 0);
    arm.tip = Vec3(0.2f, 0, 0);
    const Vec3 down(0, 0, -9.81f);
    JointVec level(1);
    level << 0;
    EXPECT_NEAR(gait_core::appendageHoldTorques(arm, level, down)(0), -0.981, 1e-4);
    EXPECT_NEAR(gait_core::appendageHoldTorques(arm, level, down, 0.5f)(0), -0.981 - 0.5 * 9.81 * 0.2, 1e-4);
    JointVec up(1);
    up << static_cast<Scalar>(-M_PI / 2);   // pointing straight up: nothing to hold
    EXPECT_NEAR(gait_core::appendageHoldTorques(arm, up, down, 0.5f)(0), 0, 1e-5);
}

TEST(MassProperties, HoldTorquesAreHowTheWeightsEnergyChangesWithEachJoint) {
    // tau_j = dU/dq_j for U = -g . (sum m p): the virtual work check, at an awkward pose and gravity
    const gait_core::AppendageDefinition arm = threeJointArm();
    const Vec3 gravity(0.8f, -1.3f, -9.6f);
    const Scalar payload = 0.4f;
    const JointVec q = armAngles(0.7, -0.5, 1.1);
    const JointVec torques = gait_core::appendageHoldTorques(arm, q, gravity, payload);
    const double FD_TOL = std::is_same<Scalar, double>::value ? 1e-5 : 2e-3;   // a central difference, in float or double
    ASSERT_EQ(torques.size(), 3);
    for (int j = 0; j < 3; ++j) {
        const double h = 1e-3;
        JointVec plus = q, minus = q;
        plus(j) += static_cast<Scalar>(h);
        minus(j) -= static_cast<Scalar>(h);
        const double up = -gravity.dot(gait_core::appendageMass(arm, plus, payload).moment);
        const double down = -gravity.dot(gait_core::appendageMass(arm, minus, payload).moment);
        EXPECT_NEAR(torques(j), (up - down) / (2 * h), FD_TOL) << "joint " << j;
    }
}

TEST(MassProperties, PayloadTorquesAreTheHoldTorquesPerKilogram) {
    const gait_core::AppendageDefinition arm = threeJointArm();
    const Vec3 gravity(0, 0, -9.81f);
    const JointVec q = armAngles(-0.3, 0.4, 0.6);
    const JointVec perKg = gait_core::appendagePayloadTorques(arm, q, gravity);
    const JointVec difference = gait_core::appendageHoldTorques(arm, q, gravity, 2) - gait_core::appendageHoldTorques(arm, q, gravity);
    for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(perKg(j) * 2, difference(j), 1e-4) << "joint " << j;
    }
}
