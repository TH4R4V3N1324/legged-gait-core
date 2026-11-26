#include <gtest/gtest.h>

#include "test_common.hpp"

#include <cstring>

#include "gait_core/locomotion/gait_config.hpp"
#include "gait_core/robot/gait_pattern.hpp"
#include "gait_core/robot/robot_definition.hpp"

using gait_core::GaitConfig;
using gait_core::GaitPattern;
using gait_core::GaitSchedule;
using gait_core::kMaxGaits;
using gait_core::explorer::NUM_GAITS;
using gait_core::explorer::NUM_LEGS;
using gait_core::RobotDefinition;
using gait_core::Scalar;

namespace {

/**
 * @brief A pattern from a duty factor and explicit offsets
*/
GaitPattern pattern(const char* name, double duty, std::initializer_list<double> offsets) {
    GaitPattern p;
    std::strncpy(p.name, name, sizeof(p.name) - 1);
    p.dutyFactor = static_cast<Scalar>(duty);
    for (double offset : offsets) {
        p.offset[p.numLegs++] = static_cast<Scalar>(offset);
    }
    return p;
}

GaitSchedule scheduleOf(const GaitPattern& p, int legs) {
    GaitSchedule schedule;
    EXPECT_TRUE(p.schedule(legs, schedule)) << p.name;
    return schedule;
}

void expectPhases(const GaitSchedule& schedule, size_t phases, std::initializer_list<int> phaseOf) {
    ASSERT_EQ(schedule.size(), phases);
    int leg = 0;
    for (int phase : phaseOf) {
        EXPECT_EQ(schedule.phaseOf[leg], phase) << "leg " << leg;
        ++leg;
    }
}

}  // namespace

// ---- Explorer: the patterns must give exactly the schedules the engine has always run ---------

TEST(GaitSchedule, ExplorersGaitsAreTheSameSchedulesAsBefore) {
    // Legs FR, MR, RR, RL, ML, FL; each number is the phase that leg swings in
    const GaitConfig config{explorerRobot()};
    ASSERT_EQ(config.numGaits(), 3);
    expectPhases(config.getGaitConfig(gait_core::explorer::GAIT_TRIPOD), 2, {0, 1, 0, 1, 0, 1});
    expectPhases(config.getGaitConfig(gait_core::explorer::GAIT_RIPPLE), 3, {2, 1, 0, 1, 2, 0});
    expectPhases(config.getGaitConfig(gait_core::explorer::GAIT_WAVE), 6, {2, 1, 0, 3, 4, 5});
}

TEST(GaitSchedule, ExplorersGaitsAsDutyFactorsAndOffsets) {
    const RobotDefinition robot = explorerRobot();
    const GaitPattern& tripod = robot.gaits[gait_core::explorer::GAIT_TRIPOD];
    EXPECT_STREQ(tripod.name, "tripod");
    EXPECT_EQ(tripod.numLegs, 6);
    EXPECT_NEAR(tripod.dutyFactor, 0.5, 1e-6);
    EXPECT_NEAR(tripod.offset[gait_core::explorer::LEG_MIDDLE_RIGHT], 0.5, 1e-6);
    EXPECT_NEAR(robot.gaits[gait_core::explorer::GAIT_RIPPLE].dutyFactor, 2.0 / 3, 1e-6);
    EXPECT_NEAR(robot.gaits[gait_core::explorer::GAIT_WAVE].dutyFactor, 5.0 / 6, 1e-6);
    EXPECT_NEAR(robot.gaits[gait_core::explorer::GAIT_WAVE].offset[gait_core::explorer::LEG_FRONT_LEFT], 5.0 / 6, 1e-6);
}

TEST(GaitSchedule, EveryGaitSwingsEachLegExactlyOnce) {
    const GaitConfig config{explorerRobot()};
    for (int g = 0; g < config.numGaits(); ++g) {
        const GaitSchedule schedule = config.getGaitConfig(static_cast<uint8_t>(g));
        ASSERT_FALSE(schedule.empty()) << "gait " << g;
        int swings[NUM_LEGS] = {};
        for (size_t phase = 0; phase < schedule.size(); ++phase) {
            int inPhase = 0;
            for (int leg = 0; leg < NUM_LEGS; ++leg) {
                if (schedule.swingsIn(leg, phase)) {
                    ++swings[leg];
                    ++inPhase;
                }
            }
            EXPECT_GT(inPhase, 0) << "gait " << g << " phase " << phase << " swings no legs";
        }
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            EXPECT_EQ(swings[leg], 1) << "gait " << g << " leg " << leg;
        }
    }
}

TEST(GaitSchedule, ExplorersGaitsAreListedByName) {
    const GaitConfig config{explorerRobot()};
    const char* names[] = {"tripod", "ripple", "wave"};
    size_t i = 0;
    for (const auto& gait : config.getGaits()) {
        EXPECT_EQ(gait.id, i);
        EXPECT_STREQ(gait.name, names[i]);
        ++i;
    }
    EXPECT_EQ(i, static_cast<size_t>(NUM_GAITS));
    EXPECT_EQ(config.unrunnableGait(), -1);
}

TEST(GaitSchedule, UnknownGaitIsEmpty) {
    const GaitConfig config{explorerRobot()};
    EXPECT_TRUE(config.getGaitConfig(NUM_GAITS).empty());
    EXPECT_TRUE(config.getGaitConfig(255).empty());
}

// ---- Other robots -------------------------------------------------------------------------------

TEST(GaitSchedule, QuadrupedTrotAndCrawl) {
    // Legs FR, RR, RL, FL. Trot: diagonal pairs, half the cycle each
    expectPhases(scheduleOf(pattern("trot", 0.5, {0, 0.5, 0, 0.5}), 4), 2, {0, 1, 0, 1});
    // Crawl: one leg at a time, three feet always down
    expectPhases(scheduleOf(pattern("crawl", 0.75, {0, 0.5, 0.25, 0.75}), 4), 4, {0, 2, 1, 3});
}

TEST(GaitSchedule, FromGroupsWorksForAnyLegCount) {
    const GaitPattern trot = GaitPattern::fromGroups("trot", {{0, 2}, {1, 3}});
    EXPECT_EQ(trot.numLegs, 4);
    EXPECT_NEAR(trot.dutyFactor, 0.5, 1e-6);
    expectPhases(scheduleOf(trot, 4), 2, {0, 1, 0, 1});
}

TEST(GaitSchedule, PatternsTheEngineCantRunAreRejected) {
    GaitSchedule schedule;
    // Timed for six legs, on a quadruped
    EXPECT_FALSE(explorerRobot().gaits[0].schedule(4, schedule));
    // A duty factor that isn't (n - 1) / n: swings that overlap phases
    EXPECT_FALSE(pattern("trot60", 0.6, {0, 0.4, 0, 0.4}).schedule(4, schedule));
    // An offset between phase boundaries
    EXPECT_FALSE(pattern("odd", 0.5, {0, 0.3, 0, 0.5}).schedule(4, schedule));
    // Offsets outside the cycle
    EXPECT_FALSE(pattern("late", 0.5, {0, 1.0, 0, 0.5}).schedule(4, schedule));
    // A duty factor of 1 (never lifts) or less than a half (one phase)
    EXPECT_FALSE(pattern("stand", 1.0, {0, 0, 0, 0}).schedule(4, schedule));
    EXPECT_FALSE(pattern("hop", 0.0, {0, 0, 0, 0}).schedule(4, schedule));
    EXPECT_TRUE(schedule.empty()) << "a rejected pattern leaves the schedule alone";
}

TEST(GaitSchedule, PhasesWithEveryFootDownAreAllowed) {
    // A biped's walk: left, both down, right, both down
    const GaitPattern walk = GaitPattern::fromGroups("walk", {{0}, {}, {1}, {}});
    EXPECT_EQ(walk.numLegs, 2);
    EXPECT_NEAR(walk.dutyFactor, 0.75, 1e-6);
    expectPhases(scheduleOf(walk, 2), 4, {0, 2});
    // The same as offsets, and a trot that pauses with all four feet down after each pair
    expectPhases(scheduleOf(pattern("walk", 0.75, {0, 0.5}), 2), 4, {0, 2});
    expectPhases(scheduleOf(pattern("gap", 0.75, {0, 0.25, 0, 0.25}), 4), 4, {0, 1, 0, 1});
    // As many phases as kMaxGaitPhases, but no more
    GaitSchedule schedule;
    const Scalar most = static_cast<Scalar>(gait_core::kMaxGaitPhases);
    EXPECT_TRUE(pattern("slow", (most - 1) / most, {0, 0.5}).schedule(2, schedule));
    EXPECT_EQ(schedule.size(), static_cast<size_t>(gait_core::kMaxGaitPhases));
    EXPECT_FALSE(pattern("slower", most / (most + 1), {0, 0}).schedule(2, schedule));
}

TEST(GaitSchedule, LongNamesAreTruncated) {
    const GaitPattern p = GaitPattern::fromGroups("a very long gait name indeed", {{0}, {1}});
    EXPECT_EQ(std::strlen(p.name), sizeof(p.name) - 1);
}

TEST(GaitSchedule, TheRobotsGaitsAreCycledInTurn) {
    RobotDefinition robot = explorerRobot();
    robot.numLegs = 4;
    robot.numGaits = 2;
    robot.gaits[0] = GaitPattern::fromGroups("trot", {{0, 2}, {1, 3}});
    robot.gaits[1] = GaitPattern::fromGroups("crawl", {{0}, {2}, {1}, {3}});
    GaitConfig config(robot);
    EXPECT_EQ(config.unrunnableGait(), -1);
    config.cycleGait();
    EXPECT_EQ(config.pendingGait, 1);
    config.currentGait = 1;
    config.cycleGait();
    EXPECT_EQ(config.pendingGait, 0) << "wraps at the robot's gait count, not explorer's";
    static_assert(kMaxGaits >= NUM_GAITS, "explorer's gaits must fit");
}
