#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <type_traits>

#include "test_common.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"
#include "gait_core/robot/robot_definition.hpp"

using gait_core::ChainKinematics;
using gait_core::YawPitchPitchLeg;
using gait_core::JointDefinition;
using gait_core::JointVec;
using gait_core::LegDefinition;
using gait_core::LegJacobian;
using gait_core::explorer::NUM_LEGS;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using Vector3d = gait_core::Vec3;

namespace {

constexpr bool IS_DOUBLE = std::is_same<Scalar, double>::value;

JointVec angles(std::initializer_list<double> values) {
    JointVec q(static_cast<int>(values.size()));
    int i = 0;
    for (double v : values) {
        q(i++) = static_cast<Scalar>(v);
    }
    return q;
}

Vector3d v(double x, double y, double z) {
    return Vector3d(static_cast<Scalar>(x), static_cast<Scalar>(y), static_cast<Scalar>(z));
}

JointDefinition revolute(const Vector3d& origin, const Vector3d& axis, double lower, double upper) {
    return {origin, axis, static_cast<Scalar>(lower), static_cast<Scalar>(upper),
            std::numeric_limits<Scalar>::infinity()};
}

/**
 * @brief Calls f(q) for a grid of angles over explorer's joint ranges, short of the limits and
 *        the straight knee, where the IK seeds below would get clamped or start singular
*/
template <typename F>
void forExplorerPoses(F f) {
    constexpr int STEPS = 8;
    for (int a = 0; a <= STEPS; ++a) {
        for (int b = 0; b <= STEPS; ++b) {
            for (int c = 0; c <= STEPS; ++c) {
                f(angles({-1.2 + 2.4 * a / STEPS, -1.4 + 2.6 * b / STEPS, 0.3 + 2.1 * c / STEPS}));
            }
        }
    }
}

/**
 * @brief The Jacobian by central differences, to check the analytic one against
*/
LegJacobian finiteJacobian(const ChainKinematics& chain, const JointVec& q) {
    const Scalar h = static_cast<Scalar>(IS_DOUBLE ? 1e-6 : 1e-3);
    LegJacobian J(3, chain.numJoints());
    for (int j = 0; j < chain.numJoints(); ++j) {
        JointVec plus = q;
        JointVec minus = q;
        plus(j) += h;
        minus(j) -= h;
        J.col(j) = (chain.fk(plus) - chain.fk(minus)) / (2 * h);
    }
    return J;
}

// Finite differences in float lose most of their digits to rounding
const double JACOBIAN_TOLERANCE = IS_DOUBLE ? 1e-7 : 2e-3;

/**
 * @brief A planar two-joint leg: hip and knee both about +y, 0.1 m thigh and 0.12 m shin
*/
LegDefinition twoJointLeg() {
    LegDefinition leg{};
    leg.numJoints = 2;
    leg.joints[0] = revolute(v(0, 0, 0), v(0, 1, 0), -1.5, 1.5);
    leg.joints[1] = revolute(v(0.1, 0, 0), v(0, 1, 0), 0.0, 2.6);
    leg.foot = v(0.12, 0, 0);
    return leg;
}

/**
 * @brief Explorer's leg with a fourth joint: a 0.05 m foot link on an ankle about +y
*/
LegDefinition fourJointLeg() {
    LegDefinition leg = explorerRobot().legs[0];
    leg.numJoints = 4;
    leg.joints[3] = revolute(leg.foot, v(0, 1, 0), -1.0, 1.0);
    leg.foot = v(0.05, 0, 0);
    return leg;
}

}  // namespace

// ---- Against the analytic solver -------------------------------------------------------------

TEST(ChainKinematics, FkMatchesTheAnalyticLegOnEveryExplorerLeg) {
    const RobotDefinition robot = explorerRobot();
    const YawPitchPitchLeg analytic(robot.legs[0]);  // every explorer leg has the same chain
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const ChainKinematics chain(robot.legs[leg]);
        ASSERT_EQ(chain.numJoints(), 3);
        forExplorerPoses([&](const JointVec& q) {
            ASSERT_NEAR((chain.fk(q) - analytic.fk(q)).norm(), 0.0, tol(1e-12)) << "leg " << leg << " at " << q.transpose();
        });
    }
}

TEST(ChainKinematics, JacobianMatchesTheAnalyticLeg) {
    const RobotDefinition robot = explorerRobot();
    const YawPitchPitchLeg analytic(robot.legs[0]);  // every explorer leg has the same chain
    const ChainKinematics chain(robot.legs[0]);
    forExplorerPoses([&](const JointVec& q) {
        ASSERT_NEAR((chain.jacobian(q) - analytic.jacobian(q)).norm(), 0.0, tol(1e-12)) << "at " << q.transpose();
    });
}

TEST(ChainKinematics, JacobianMatchesFiniteDifferences) {
    const ChainKinematics chain(explorerRobot().legs[0]);
    forExplorerPoses([&](const JointVec& q) {
        ASSERT_NEAR((chain.jacobian(q) - finiteJacobian(chain, q)).norm(), 0.0, JACOBIAN_TOLERANCE) << "at " << q.transpose();
    });
}

TEST(ChainKinematics, GroundForceMatchesTheAnalyticLeg) {
    const RobotDefinition robot = explorerRobot();
    const YawPitchPitchLeg analytic(robot.legs[0]);  // every explorer leg has the same chain
    const ChainKinematics chain(robot.legs[0]);
    const JointVec q = angles({0.2, -0.4, 1.6});
    const JointVec torques = angles({0.4, -0.9, 0.3});
    Vector3d expected;
    Vector3d force;
    ASSERT_TRUE(analytic.groundForce(q, torques, expected));
    ASSERT_TRUE(chain.groundForce(q, torques, force));
    EXPECT_NEAR((force - expected).norm(), 0.0, tol(1e-9));
}

// ---- IK ---------------------------------------------------------------------------------------

TEST(ChainKinematics, IkRoundTripsFromANearbySeed) {
    // The usual case in the gait: last tick's angles are close to this tick's answer
    const ChainKinematics chain(explorerRobot().legs[0]);
    int solves = 0;
    int skipped = 0;
    forExplorerPoses([&](const JointVec& q) {
        const Vector3d target = chain.fk(q);
        // A foot folded back onto the coxa's vertical axis (under the hip, inside the body) is the
        // same point at every coxa angle, so there's no coxa angle to find
        if (target.head<2>().norm() < 0.02) {
            ++skipped;
            return;
        }
        JointVec seed = q;
        seed.array() += static_cast<Scalar>(0.1);
        JointVec solved;
        ASSERT_TRUE(chain.ik(target, seed, solved)) << "at " << q.transpose();
        ASSERT_EQ(solved.size(), 3);
        EXPECT_LT((chain.fk(solved) - target).norm(), ChainKinematics::IK_TOLERANCE) << "at " << q.transpose();
        EXPECT_NEAR((solved - q).norm(), 0.0, 1e-3) << "a different solution from the seed's at " << q.transpose();
        ++solves;
    });
    EXPECT_EQ(solves + skipped, 9 * 9 * 9);
    EXPECT_LE(skipped, 9 * 9) << "the singular poses should be a thin slice of the grid";
}

TEST(ChainKinematics, IkAgreesWithTheAnalyticSolve) {
    const RobotDefinition robot = explorerRobot();
    const YawPitchPitchLeg analytic(robot.legs[0]);  // every explorer leg has the same chain
    const ChainKinematics chain(robot.legs[0]);
    for (const Vector3d& foot : {v(0.17965, 0, -0.13353), v(0.24896, -0.05286, -0.07071), v(0.15, 0.08, -0.12)}) {
        JointVec expected;
        ASSERT_TRUE(analytic.ik(foot, JointVec::Zero(3), expected));
        JointVec seed = expected;
        seed.array() -= static_cast<Scalar>(0.15);
        JointVec solved;
        ASSERT_TRUE(chain.ik(foot, seed, solved)) << foot.transpose();
        // IK stops within IK_TOLERANCE of the foot, well under 1e-3 rad here; a servo step is ~1.5e-3 rad
        EXPECT_NEAR((solved - expected).norm(), 0.0, 1e-3) << foot.transpose();
        EXPECT_LT((analytic.fk(solved) - foot).norm(), ChainKinematics::IK_TOLERANCE) << foot.transpose();
    }
}

TEST(ChainKinematics, IkOutOfReachGivesTheClosestPoseWithinTheLimits) {
    const LegDefinition leg = explorerRobot().legs[0];
    const ChainKinematics chain(leg);
    const Vector3d farAway = v(1.0, 0.0, -0.1);
    const JointVec seed = angles({0, -0.5, 1.5});
    JointVec solved;
    EXPECT_FALSE(chain.ik(farAway, seed, solved));
    for (int j = 0; j < 3; ++j) {
        EXPECT_GE(solved(j), leg.joints[j].lower) << "joint " << j;
        EXPECT_LE(solved(j), leg.joints[j].upper) << "joint " << j;
    }
    // Reaching towards it: the leg ends up nearly straight, pointing at the target
    EXPECT_LT((chain.fk(solved) - farAway).norm(), (chain.fk(seed) - farAway).norm());
    EXPECT_GT(chain.fk(solved).norm(), 0.33);
}

TEST(ChainKinematics, IkStartsFromZerosForASeedOfTheWrongSize) {
    const ChainKinematics chain(explorerRobot().legs[0]);
    const Vector3d target = chain.fk(angles({0.3, -0.5, 1.4}));
    JointVec solved;
    EXPECT_TRUE(chain.ik(target, JointVec(), solved));
    EXPECT_EQ(solved.size(), 3);
    EXPECT_LT((chain.fk(solved) - target).norm(), ChainKinematics::IK_TOLERANCE);
}

// ---- Other joint counts -----------------------------------------------------------------------

TEST(ChainKinematics, TwoJointLeg) {
    const ChainKinematics chain(twoJointLeg());
    ASSERT_EQ(chain.numJoints(), 2);
    // Hip level, knee folded 90 degrees: thigh straight out, shin straight down
    EXPECT_NEAR((chain.fk(angles({0, M_PI / 2})) - v(0.1, 0, -0.12)).norm(), 0.0, tol(1e-9));
    const JointVec q = angles({0.4, 1.2});
    EXPECT_EQ(chain.jacobian(q).cols(), 2);
    EXPECT_NEAR((chain.jacobian(q) - finiteJacobian(chain, q)).norm(), 0.0, JACOBIAN_TOLERANCE);
    // Planar, so any target in the leg's plane within reach
    const Vector3d target = chain.fk(q);
    JointVec solved;
    ASSERT_TRUE(chain.ik(target, angles({0.2, 1.0}), solved));
    EXPECT_LT((chain.fk(solved) - target).norm(), ChainKinematics::IK_TOLERANCE);
}

TEST(ChainKinematics, TwoJointsNeverGiveAFootForce) {
    // Two joints can't resolve a 3D force: whatever the torques and pose, there's no answer, even
    // where rounding leaves J J^T's determinant just off zero
    const ChainKinematics chain(twoJointLeg());
    for (double hip = -1.0; hip <= 1.0; hip += 0.25) {
        for (double knee = 0.3; knee <= 2.0; knee += 0.25) {
            const JointVec q = angles({hip, knee});
            const JointVec torques = -chain.jacobian(q).transpose() * Vector3d(0, 0, 5);
            Vector3d force = Vector3d::Constant(42);
            EXPECT_FALSE(chain.groundForce(q, torques, force)) << "hip " << hip << " knee " << knee;
            EXPECT_EQ(force, Vector3d::Constant(42)) << "left untouched";
        }
    }
}

TEST(ChainKinematics, FourJointLeg) {
    const ChainKinematics chain(fourJointLeg());
    ASSERT_EQ(chain.numJoints(), 4);
    // Straight out: 0.063 + 0.1 + 0.186 + 0.05
    EXPECT_NEAR((chain.fk(JointVec::Zero(4)) - v(0.399, 0, 0)).norm(), 0.0, tol(1e-9));
    const JointVec q = angles({0.3, -0.4, 1.3, 0.5});
    EXPECT_EQ(chain.jacobian(q).cols(), 4);
    EXPECT_NEAR((chain.jacobian(q) - finiteJacobian(chain, q)).norm(), 0.0, JACOBIAN_TOLERANCE);
    // Redundant (four joints for three coordinates): any solution that reaches will do
    const Vector3d target = chain.fk(q);
    JointVec solved;
    ASSERT_TRUE(chain.ik(target, angles({0.2, -0.3, 1.2, 0.3}), solved));
    EXPECT_LT((chain.fk(solved) - target).norm(), ChainKinematics::IK_TOLERANCE);
}

TEST(ChainKinematics, AxesNeedNotBeUnitLength) {
    LegDefinition leg = explorerRobot().legs[0];
    leg.joints[1].axis *= 3;   // same direction, so the same leg
    const ChainKinematics scaled(leg);
    const ChainKinematics unit(explorerRobot().legs[0]);
    const JointVec q = angles({0.2, -0.4, 1.6});
    EXPECT_NEAR((scaled.fk(q) - unit.fk(q)).norm(), 0.0, tol(1e-12));
    EXPECT_NEAR((scaled.jacobian(q) - unit.jacobian(q)).norm(), 0.0, tol(1e-12));
}

TEST(ChainKinematics, ASpareJointSettlesTowardsMidRangeWithoutMovingTheFoot) {
    // Solving the same target again and again, as the engine does every tick for a planted foot:
    // the foot stays put while the pose eases towards the middle of every joint's range
    const LegDefinition leg = fourJointLeg();
    const ChainKinematics chain(leg);
    const JointVec start = angles({0.3, -0.4, 1.3, 0.5});
    const Vector3d target = chain.fk(start);
    JointVec middle(4);
    for (int j = 0; j < 4; ++j) {
        middle(j) = (leg.joints[j].lower + leg.joints[j].upper) / 2;
    }
    JointVec q = start;
    for (int i = 0; i < 300; ++i) {
        JointVec next;
        ASSERT_TRUE(chain.ik(target, q, next)) << "solve " << i;
        q = next;
        ASSERT_LT((chain.fk(q) - target).norm(), ChainKinematics::IK_TOLERANCE) << "solve " << i;
    }
    EXPECT_LT((q - middle).norm(), (start - middle).norm()) << "closer to mid-range";
    // Settled: one more solve barely moves it
    JointVec again;
    ASSERT_TRUE(chain.ik(target, q, again));
    EXPECT_LT((again - q).cwiseAbs().maxCoeff(), 1e-3);
}

TEST(ChainKinematics, AThreeJointLegKeepsTheSolutionNearestItsSeed) {
    // Three joints have no spare: settling must not apply, so an exact seed comes straight back
    const ChainKinematics chain(explorerRobot().legs[0]);
    const JointVec q = angles({0.2, -0.4, 1.6});
    JointVec solved;
    ASSERT_TRUE(chain.ik(chain.fk(q), q, solved));
    EXPECT_EQ(solved, q);
}

// ---- The foot's turning, for feet with soles ----------------------------------------------------

TEST(ChainKinematics, FullJacobianTurnsTheFootAsFiniteDifferencesDo) {
    // The biped's six-joint leg (yaw, roll, pitch, knee, ankle pitch, ankle roll): the angular rows
    // are the foot's angular velocity, which finite differences of its rotation give as R' R^T
    LegDefinition leg{};
    leg.numJoints = 6;
    leg.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), -1, 1);
    leg.joints[1] = revolute(v(0, 0, 0), v(1, 0, 0), -1, 1);
    leg.joints[2] = revolute(v(0, 0, 0), v(0, 1, 0), -2, 2);
    leg.joints[3] = revolute(v(0, 0.04, -0.09), v(0, 1, 0), -0.1, 2.5);
    leg.joints[4] = revolute(v(0.1, 0, 0), v(0, 1, 0), -1.5, 1.5);
    leg.joints[5] = revolute(v(0, 0, 0), v(1, 0, 0), -1, 1);
    leg.foot = v(0.04, 0, -0.05);
    const ChainKinematics chain(leg);
    const JointVec q = angles({0.2, -0.1, -0.6, 1.1, -0.4, 0.15});
    const gait_core::LegJacobian6 J = chain.fullJacobian(q);
    ASSERT_EQ(J.cols(), 6);
    EXPECT_NEAR((J.topRows<3>() - finiteJacobian(chain, q)).norm(), 0.0, JACOBIAN_TOLERANCE);
    EXPECT_NEAR((J.topRows<3>() - chain.jacobian(q)).norm(), 0.0, tol(1e-12));
    const Scalar h = static_cast<Scalar>(IS_DOUBLE ? 1e-6 : 1e-3);
    for (int j = 0; j < 6; ++j) {
        JointVec plus = q, minus = q;
        plus(j) += h;
        minus(j) -= h;
        const gait_core::Mat3 dR = (chain.footRotation(plus) - chain.footRotation(minus)) / (2 * h);
        const gait_core::Mat3 W = dR * chain.footRotation(q).transpose();   // skew(omega)
        const Vector3d omega(W(2, 1), W(0, 2), W(1, 0));
        EXPECT_NEAR((J.col(j).tail<3>() - omega).norm(), 0.0, JACOBIAN_TOLERANCE * 10) << "joint " << j;
    }
}
