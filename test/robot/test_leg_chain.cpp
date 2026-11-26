#include <gtest/gtest.h>

#include <cmath>

#include "test_common.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"
#include "gait_core/robot/robot_definition.hpp"

using gait_core::AngleAxis;
using gait_core::JointDefinition;
using gait_core::JointVec;
using gait_core::LegDefinition;
using gait_core::Mat3;
using gait_core::explorer::NUM_LEGS;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::YawPitchPitchLeg;
using Vector3d = gait_core::Vec3;

namespace {

/**
 * @brief Foot position from a leg's joint chain: at each joint step along its origin, then turn
 *        about its axis by its angle; finally step out to the foot
 * @param[in] leg The leg
 * @param[in] q One angle per joint (rad)
 * @return The foot position in the leg frame (m)
 * @note A reference for the chain definition, written plainly rather than fast. Axes and origins
 *       are given at zero angles, with every joint frame aligned to the leg frame there.
*/
Vector3d chainFoot(const LegDefinition& leg, const JointVec& q) {
    Mat3 rotation = Mat3::Identity();
    Vector3d position = Vector3d::Zero();
    for (int j = 0; j < leg.numJoints; ++j) {
        const JointDefinition& joint = leg.joints[j];
        position += rotation * joint.origin;
        rotation = rotation * AngleAxis(q(j), joint.axis.normalized()).toRotationMatrix();
    }
    return position + rotation * leg.foot;
}

JointVec angles(double coxa, double femur, double tibia) {
    JointVec q(3);
    q << static_cast<Scalar>(coxa), static_cast<Scalar>(femur), static_cast<Scalar>(tibia);
    return q;
}

}  // namespace

TEST(LegChain, ExplorerLegsAreCoxaFemurTibia) {
    const RobotDefinition robot = explorerRobot();
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const LegDefinition& def = robot.legs[leg];
        ASSERT_EQ(def.numJoints, 3) << "leg " << leg;
        for (int j = 0; j < def.numJoints; ++j) {
            EXPECT_NEAR(def.joints[j].axis.norm(), 1.0, tol(1e-12)) << "leg " << leg << " joint " << j;
            EXPECT_LT(def.joints[j].lower, def.joints[j].upper) << "leg " << leg << " joint " << j;
        }
    }
}

TEST(LegChain, StraightLegReachesTheSumOfTheLinks) {
    // All zeros: coxa, femur and tibia in a line straight out along +x
    const LegDefinition& leg = explorerRobot().legs[0];
    EXPECT_NEAR((chainFoot(leg, angles(0, 0, 0)) - Vector3d(0.349, 0, 0)).norm(), 0.0, tol(1e-12));
}

TEST(LegChain, HandWorkedPoses) {
    const LegDefinition& leg = explorerRobot().legs[0];
    constexpr double half = M_PI / 2;
    // Coxa turned 90 degrees: the straight leg points along +y
    EXPECT_NEAR((chainFoot(leg, angles(half, 0, 0)) - Vector3d(0, 0.349, 0)).norm(), 0.0, tol(1e-9));
    // Femur lowered 90 degrees: femur and tibia hang straight down from the end of the coxa
    EXPECT_NEAR((chainFoot(leg, angles(0, half, 0)) - Vector3d(0.063, 0, -0.286)).norm(), 0.0, tol(1e-9));
    // Knee folded 90 degrees: femur out level, tibia straight down
    EXPECT_NEAR((chainFoot(leg, angles(0, 0, half)) - Vector3d(0.163, 0, -0.186)).norm(), 0.0, tol(1e-9));
}

TEST(LegChain, MatchesTheAnalyticSolverAcrossTheJointRanges) {
    // Every leg, on a grid covering each joint's whole range
    const RobotDefinition robot = explorerRobot();
    const YawPitchPitchLeg solver(robot.legs[0]);  // every explorer leg has the same chain
    constexpr int STEPS = 12;
    double worst = 0;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const LegDefinition& def = robot.legs[leg];
        auto at = [&](int j, int i) {
            return def.joints[j].lower + (def.joints[j].upper - def.joints[j].lower) * i / STEPS;
        };
        for (int a = 0; a <= STEPS; ++a) {
            for (int b = 0; b <= STEPS; ++b) {
                for (int c = 0; c <= STEPS; ++c) {
                    const JointVec q = angles(at(0, a), at(1, b), at(2, c));
                    const Vector3d expected = solver.fk(q);
                    const double error = (chainFoot(def, q) - expected).norm();
                    worst = std::max(worst, error);
                    ASSERT_LT(error, tol(1e-12)) << "leg " << leg << " at " << q.transpose();
                }
            }
        }
    }
    RecordProperty("worst_error_m", std::to_string(worst));
}

TEST(LegChain, ExplorerMatchesTheUrdf) {
    // Link lengths and joint ranges as in hexapod-ros2's URDF
    const LegDefinition& leg = explorerRobot().legs[0];
    EXPECT_EQ(leg.joints[1].origin, Vector3d(static_cast<Scalar>(0.063), 0, 0));   // coxa link
    EXPECT_EQ(leg.joints[2].origin, Vector3d(static_cast<Scalar>(0.100), 0, 0));   // femur
    EXPECT_EQ(leg.foot, Vector3d(static_cast<Scalar>(0.186), 0, 0));               // tibia
    const Scalar ranges[3][2] = {{-1.4f, 1.4f}, {-1.6f, 1.4f}, {-0.78f, 2.63f}};
    for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(leg.joints[j].lower, ranges[j][0], 1e-6) << "joint " << j;
        EXPECT_NEAR(leg.joints[j].upper, ranges[j][1], 1e-6) << "joint " << j;
    }
}


TEST(LegChain, TheSolePointsWhereTheYawJointTurnsIt) {
    const RobotDefinition robot = explorerRobot();
    const LegDefinition& def = robot.legs[0];
    EXPECT_FALSE(def.hasSole()) << "Explorer has point feet";
    // The coxa turns the foot about z; the femur and tibia only tilt it
    EXPECT_NEAR(def.soleYaw(angles(0.3, 0.4, -0.5)), 0.3, tol(1e-6));
    EXPECT_NEAR(def.soleYaw(angles(-1.2, -0.9, 1.5)), -1.2, tol(1e-6));
    EXPECT_EQ(def.soleYaw(JointVec::Zero(2)), 0) << "the wrong number of angles";
}
