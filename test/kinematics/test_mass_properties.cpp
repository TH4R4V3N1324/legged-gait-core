#include <gtest/gtest.h>

#include <cmath>

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
