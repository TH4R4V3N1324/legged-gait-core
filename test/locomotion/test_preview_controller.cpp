#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "gait_core/locomotion/preview_controller.hpp"

using gait_core::PreviewController;
using gait_core::Scalar;
using gait_core::Vec2;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);   // s
constexpr int STEPS = 160;                         // 1.6 s of preview
const Scalar OMEGA = std::sqrt(static_cast<Scalar>(9.81) / static_cast<Scalar>(0.134));   // explorer

PreviewController configured(int steps = STEPS) {
    PreviewController p;
    EXPECT_TRUE(p.configure(DT, OMEGA, steps));
    return p;
}

/**
 * @brief Trace a reference track (one point per tick) through the controller, recording what it does
*/
struct Trace {
    std::vector<Vec2> com, zmp;
};

Trace follow(PreviewController& p, const std::vector<Vec2>& track) {
    Trace Trace;
    std::vector<Vec2> window(static_cast<size_t>(p.steps()));
    for (size_t k = 0; k + 1 < track.size(); ++k) {
        for (int j = 0; j < p.steps(); ++j) {
            window[static_cast<size_t>(j)] = track[std::min(track.size() - 1, k + 1 + static_cast<size_t>(j))];
        }
        p.step(window.data());
        Trace.com.push_back(p.position());
        Trace.zmp.push_back(p.zmp());
    }
    return Trace;
}

}  // namespace

TEST(PreviewController, TheGainsConvergeAndThePreviewFadesOut) {
    const PreviewController p = configured();
    EXPECT_TRUE(p.configured());
    EXPECT_GT(p.gainIntegral(), 0);
    EXPECT_TRUE(std::isfinite(p.gainState()[0]) && std::isfinite(p.gainState()[1]) && std::isfinite(p.gainState()[2]));
    // What's a long way off matters less and less
    EXPECT_GT(std::abs(p.gainPreview(10)), 10 * std::abs(p.gainPreview(STEPS)));
    EXPECT_EQ(p.gainPreview(0), 0);
    EXPECT_EQ(p.gainPreview(STEPS + 1), 0);
}

TEST(PreviewController, NothingToDoStandsStill) {
    PreviewController p = configured();
    const Vec2 here(static_cast<Scalar>(0.03), static_cast<Scalar>(-0.02));
    p.reset(here);
    const Trace Trace = follow(p, std::vector<Vec2>(400, here));
    for (const Vec2& c : Trace.com) {
        ASSERT_NEAR((c - here).norm(), 0, 1e-6);
    }
}

TEST(PreviewController, ItStartsMovingBeforeTheStepArrives) {
    // The ZMP has to move 40 mm sideways at t = 1 s: the body sets off before then, since the
    // capture point runs away from wherever the ZMP is, and ends up over the new point
    PreviewController p = configured();
    p.reset(Vec2::Zero());
    std::vector<Vec2> track(400, Vec2::Zero());
    for (size_t k = 100; k < track.size(); ++k) {
        track[k] = Vec2(0, static_cast<Scalar>(0.04));
    }
    const Trace Trace = follow(p, track);
    EXPECT_GT(Trace.com[95].y(), 0.002) << "already on its way 50 ms before";
    EXPECT_LT(Trace.com[95].y(), 0.03);
    EXPECT_NEAR(Trace.com.back().y(), 0.04, 5e-4);
    EXPECT_NEAR(Trace.zmp.back().y(), 0.04, 5e-4);
    // Smoothly: the ZMP stays within the two points but for a small swing, and x never moves
    for (size_t k = 0; k < Trace.zmp.size(); ++k) {
        ASSERT_GT(Trace.zmp[k].y(), -0.01) << k;
        ASSERT_LT(Trace.zmp[k].y(), 0.05) << k;
        ASSERT_NEAR(Trace.com[k].x(), 0, 1e-6);
    }
}

TEST(PreviewController, ItTracksASwayingGait) {
    // A crawl-like ZMP track: 30 mm left, then right, every 0.6 s. The ZMP follows it closely once
    // going, which a body moving only when each phase began couldn't
    PreviewController p = configured();
    p.reset(Vec2::Zero());
    std::vector<Vec2> track(800);
    for (size_t k = 0; k < track.size(); ++k) {
        track[k] = Vec2(0, ((k / 60) % 2 == 0 ? 1 : -1) * static_cast<Scalar>(0.03));
    }
    const Trace Trace = follow(p, track);
    double worst = 0, sum = 0;
    int n = 0;
    for (size_t k = 200; k < 700; ++k) {
        const double error = std::abs(static_cast<double>(Trace.zmp[k].y() - track[k + 1].y()));
        worst = std::max(worst, error);
        sum += error;
        ++n;
    }
    EXPECT_LT(sum / n, 0.006) << "mean ZMP error";
    EXPECT_LT(worst, 0.035) << "the jumps themselves can't be tracked exactly";
    // The centre of mass sways less than the ZMP does, and smoothly
    double swing = 0;
    for (size_t k = 200; k < 700; ++k) {
        swing = std::max(swing, static_cast<double>(std::abs(Trace.com[k].y())));
    }
    EXPECT_LT(swing, 0.03);
    EXPECT_GT(swing, 0.005);
}

TEST(PreviewController, ItWorksFarFromTheOrigin) {
    PreviewController p = configured();
    const Vec2 far(static_cast<Scalar>(5), static_cast<Scalar>(-3));
    p.reset(far);
    const Trace Trace = follow(p, std::vector<Vec2>(300, far));
    EXPECT_NEAR((Trace.com.back() - far).norm(), 0, 1e-4);
}

TEST(PreviewController, AShortPreviewStillConverges) {
    PreviewController p = configured(20);
    p.reset(Vec2::Zero());
    const Trace Trace = follow(p, std::vector<Vec2>(500, Vec2(static_cast<Scalar>(0.02), 0)));
    EXPECT_NEAR(Trace.com.back().x(), 0.02, 1e-3);
}

TEST(PreviewController, AnewOmegaKeepsTheState) {
    PreviewController p = configured();
    p.reset(Vec2(static_cast<Scalar>(0.01), 0), Vec2(static_cast<Scalar>(0.1), 0));
    ASSERT_TRUE(p.configure(DT, OMEGA * static_cast<Scalar>(0.9), STEPS));
    EXPECT_EQ(p.position(), Vec2(static_cast<Scalar>(0.01), 0));
    EXPECT_EQ(p.velocity(), Vec2(static_cast<Scalar>(0.1), 0));
}

TEST(PreviewController, NonsenseIsRefused) {
    PreviewController p;
    EXPECT_FALSE(p.configure(0, OMEGA, STEPS));
    EXPECT_FALSE(p.configure(DT, 0, STEPS));
    EXPECT_FALSE(p.configured());
    p.step(nullptr);   // nothing happens
    EXPECT_EQ(p.position(), Vec2::Zero());
}
