#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_common.hpp"
#include "gait_core/locomotion/support_polygon.hpp"

using gait_core::Scalar;
using gait_core::stabilityMargin;
using gait_core::Vec2;

namespace {

Vec2 v(double x, double y) {
    return Vec2(static_cast<Scalar>(x), static_cast<Scalar>(y));
}

Scalar margin(const std::vector<Vec2>& feet, const Vec2& point) {
    return stabilityMargin(feet.data(), static_cast<int>(feet.size()), point);
}

}  // namespace

TEST(SupportPolygon, InsideASquareIsTheDistanceToTheNearestEdge) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    EXPECT_NEAR(margin(square, v(0, 0)), 0.1, tol(1e-9));
    EXPECT_NEAR(margin(square, v(0.07, 0.02)), 0.03, tol(1e-9));
}

TEST(SupportPolygon, OutsideIsMinusTheDistanceToThePolygon) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    EXPECT_NEAR(margin(square, v(0.15, 0)), -0.05, tol(1e-9));
    EXPECT_NEAR(margin(square, v(0.13, 0.14)), -0.05, tol(1e-9)) << "past a corner: distance to the corner";
    EXPECT_NEAR(margin(square, v(0.1, 0.05)), 0.0, tol(1e-9)) << "on an edge";
}

TEST(SupportPolygon, OrderRepeatsAndInnerFeetDoNotMatter) {
    const std::vector<Vec2> scrambled{v(0.1, 0.1), v(-0.1, -0.1), v(0, 0), v(-0.1, 0.1),
                                      v(0.1, -0.1), v(0.1, 0.1), v(0.05, 0.0)};
    EXPECT_NEAR(margin(scrambled, v(0, 0)), 0.1, tol(1e-9));
}

TEST(SupportPolygon, TwoFeetOrALineGiveNoArea) {
    // A trot's diagonal pair: the centre right on the line is only just balanced
    EXPECT_NEAR(margin({v(0.1, 0.1), v(-0.1, -0.1)}, v(0, 0)), 0.0, tol(1e-9));
    EXPECT_NEAR(margin({v(0.1, 0.1), v(-0.1, -0.1)}, v(0.1, -0.1)), -0.1 * std::sqrt(2.0), tol(1e-9));
    EXPECT_LE(margin({v(-0.1, 0), v(0, 0), v(0.1, 0)}, v(0, 0.01)), 0);
}

TEST(SupportPolygon, OneFootOrNone) {
    EXPECT_NEAR(margin({v(0.03, 0.04)}, v(0, 0)), -0.05, tol(1e-9));
    EXPECT_NEAR(margin({v(0.03, 0.04), v(0.03, 0.04)}, v(0, 0)), -0.05, tol(1e-9));
    EXPECT_TRUE(std::isinf(margin({}, v(0, 0))));
    EXPECT_LT(margin({}, v(0, 0)), 0);
}

TEST(SupportPolygon, ACrawlTriangleOfASquareRobotJustReachesItsCentre) {
    // Three corners of a square: the centre lies on the diagonal, so a crawl that lifts one leg at a
    // time without moving the body sideways is only just statically stable
    EXPECT_NEAR(margin({v(0.1, 0.1), v(0.1, -0.1), v(-0.1, -0.1)}, v(0, 0)), 0.0, tol(1e-9));
    EXPECT_GT(margin({v(0.1, 0.1), v(0.1, -0.1), v(-0.1, -0.1)}, v(0.02, -0.02)), 0);
}

TEST(SupportPolygon, TheDeepestPointOfASquareIsItsCentre) {
    Vec2 centre;
    EXPECT_NEAR(gait_core::deepestSupportPoint(std::vector<Vec2>{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)}.data(), 4, centre),
                0.1, tol(1e-9));
    EXPECT_NEAR((centre - v(0, 0)).norm(), 0.0, tol(1e-9));
}

TEST(SupportPolygon, TheDeepestPointOfATriangleIsItsIncentre) {
    // Right triangle with legs 0.3 and 0.4: inradius (a + b - c) / 2 = 0.1, centre 0.1 in from both legs
    const std::vector<Vec2> triangle{v(0, 0), v(0.4, 0), v(0, 0.3)};
    Vec2 centre;
    EXPECT_NEAR(gait_core::deepestSupportPoint(triangle.data(), 3, centre), 0.1, tol(1e-9));
    EXPECT_NEAR((centre - v(0.1, 0.1)).norm(), 0.0, tol(1e-9));
    EXPECT_NEAR(stabilityMargin(triangle.data(), 3, centre), 0.1, tol(1e-9));
}

TEST(SupportPolygon, TwoFeetHaveNoDepth) {
    Vec2 centre;
    EXPECT_NEAR(gait_core::deepestSupportPoint(std::vector<Vec2>{v(0.1, 0.1), v(-0.1, -0.1)}.data(), 2, centre), 0.0, tol(1e-9));
    EXPECT_NEAR(centre.norm(), 0.0, tol(1e-9));
}

TEST(SupportPolygon, NearestStablePointLeavesAStablePointAlone) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    Vec2 nearest;
    ASSERT_TRUE(gait_core::nearestStablePoint(square.data(), 4, v(0.03, 0.02), 0.05, nearest));
    EXPECT_NEAR((nearest - v(0.03, 0.02)).norm(), 0.0, tol(1e-12));
}

TEST(SupportPolygon, NearestStablePointMovesStraightInFromAnEdge) {
    // A crawl's support triangle with the centre of mass on its long edge: move it in by the margin,
    // at right angles to that edge
    const std::vector<Vec2> triangle{v(0.1, 0.1), v(0.1, -0.1), v(-0.1, -0.1)};
    Vec2 nearest;
    ASSERT_TRUE(gait_core::nearestStablePoint(triangle.data(), 3, v(0, 0), 0.02, nearest));
    EXPECT_NEAR(stabilityMargin(triangle.data(), 3, nearest), 0.02, tol(1e-6));
    EXPECT_NEAR((nearest - v(0.02 / std::sqrt(2.0), -0.02 / std::sqrt(2.0))).norm(), 0.0, tol(1e-9));
}

TEST(SupportPolygon, NearestStablePointFromOutsidePastACorner) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    Vec2 nearest;
    ASSERT_TRUE(gait_core::nearestStablePoint(square.data(), 4, v(0.3, 0.25), 0.02, nearest));
    EXPECT_NEAR((nearest - v(0.08, 0.08)).norm(), 0.0, tol(1e-9)) << "the shrunk square's corner";
}

TEST(SupportPolygon, NearestStablePointWithTooLargeAMarginGivesTheDeepestPoint) {
    const std::vector<Vec2> triangle{v(0, 0), v(0.4, 0), v(0, 0.3)};
    Vec2 nearest;
    EXPECT_FALSE(gait_core::nearestStablePoint(triangle.data(), 3, v(0.3, 0.2), 0.15, nearest));
    EXPECT_NEAR((nearest - v(0.1, 0.1)).norm(), 0.0, tol(1e-9));
}

// Soles: a foot with a sole stands on its four corners

namespace {

std::vector<Vec2> soles(const std::vector<Vec2>& feet, Scalar yaw, const Vec2& soleMin, const Vec2& soleMax) {
    std::vector<Vec2> points(gait_core::kMaxContactPoints);
    int count = 0;
    for (const Vec2& foot : feet) {
        count = gait_core::appendContactPoints(foot, yaw, soleMin, soleMax, points.data(), count);
    }
    points.resize(static_cast<size_t>(count));
    return points;
}

}  // namespace

TEST(SupportPolygon, APointFootIsOneContactPoint) {
    const std::vector<Vec2> points = soles({v(0.2, 0.1)}, 0.7f, v(0, 0), v(0, 0));
    ASSERT_EQ(points.size(), 1u);
    EXPECT_NEAR((points[0] - v(0.2, 0.1)).norm(), 0.0, tol(1e-9));
}

TEST(SupportPolygon, OneSoleHasAMarginAtItsCentre) {
    // 0.2 long, 0.1 wide, the foot point 0.05 behind its centre
    const std::vector<Vec2> points = soles({v(0, 0)}, 0, v(-0.05, -0.05), v(0.15, 0.05));
    ASSERT_EQ(points.size(), 4u);
    EXPECT_NEAR(margin(points, v(0.05, 0)), 0.05, tol(1e-6)) << "half its width";
    EXPECT_LT(margin(points, v(0.2, 0)), 0) << "past its toe";
}

TEST(SupportPolygon, ASoleTurnsWithItsFoot) {
    const std::vector<Vec2> points = soles({v(1, 0)}, static_cast<Scalar>(M_PI / 2), v(-0.05, -0.05), v(0.15, 0.05));
    EXPECT_NEAR(margin(points, v(1, 0.05)), 0.05, tol(1e-6)) << "the centre, turned to point along y";
    EXPECT_LT(margin(points, v(1.1, 0)), 0) << "where its toe was before turning";
}

TEST(SupportPolygon, TwoSolesSideBySideHaveAMarginBetweenThem) {
    // Two feet 0.2 apart, each sole 0.1 square: the hull is 0.3 wide and 0.1 long
    const std::vector<Vec2> points = soles({v(0, 0.1), v(0, -0.1)}, 0, v(-0.05, -0.05), v(0.05, 0.05));
    ASSERT_EQ(points.size(), 8u);
    EXPECT_NEAR(margin(points, v(0, 0)), 0.05, tol(1e-6));
    Vec2 centre;
    EXPECT_NEAR(gait_core::deepestSupportPoint(points.data(), static_cast<int>(points.size()), centre), 0.05, tol(1e-6));
    Vec2 nearest;
    EXPECT_TRUE(gait_core::nearestStablePoint(points.data(), static_cast<int>(points.size()), v(0.2, 0), 0.02, nearest));
    EXPECT_NEAR((nearest - v(0.03, 0)).norm(), 0.0, tol(1e-6));
}

TEST(SupportPolygon, EverySoleCornerCounts) {
    // More points than legs: none may be dropped (the last foot's corners make the hull)
    std::vector<Vec2> feet;
    for (int i = 0; i < gait_core::kMaxLegs; ++i) {
        feet.push_back(v(0, 0));
    }
    feet.back() = v(1, 0);
    const std::vector<Vec2> points = soles(feet, 0, v(-0.05, -0.05), v(0.05, 0.05));
    ASSERT_EQ(points.size(), static_cast<size_t>(gait_core::kMaxContactPoints));
    EXPECT_NEAR(margin(points, v(0.5, 0)), 0.05, tol(1e-6));
}
