// Checks the engine and the behaviours never allocate once running, so they're safe on microcontrollers.
// Replaces the global operator new for this test binary only, counting every allocation.

#include <gtest/gtest.h>

#include "test_common.hpp"
#include "test_robots.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>
#include <vector>

#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/kinematics/mass_properties.hpp"
#include "gait_core/locomotion/gait_engine.hpp"
#include "gait_core/robot/robot_definition.hpp"
#include "gait_core/behaviour/battery_monitor.hpp"

namespace {
std::atomic<long> allocations{0};
}  // namespace

void* operator new(std::size_t size) {
    ++allocations;
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
    ++allocations;
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::ImuSample;
using gait_core::JointFeedback;
using gait_core::YawPitchPitchLeg;
using gait_core::JointVec;
using gait_core::LogLevel;
using gait_core::explorer::NUM_LEGS;
using gait_core::Scalar;
using gait_core::Timestamp;
using gait_core::Vec3;

namespace {

int logged = 0;

/**
 * @brief Perfect servos, with the lowest feet carrying the weight so contact and load balancing run
*/
class Robot {
public:
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    YawPitchPitchLeg ik{explorerRobot().legs[0]};
    Timestamp now = 1'000'000'000;
    JointFeedback feedback;
    bool imu = false;

    Robot() {
        engine.log = [](LogLevel, const char*) { ++logged; };
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = JointVec::Zero(3);
        }
        feedback.hasEffort = true;
    }

    void run(double seconds) {
        const int ticks = static_cast<int>(seconds / 0.01 + 0.5);
        for (int i = 0; i < ticks; ++i) {
            setEfforts();
            feedback.stamp = now;
            engine.onJointStates(feedback);
            if (imu) {
                ImuSample sample;
                sample.stamp = now;
                sample.roll = static_cast<Scalar>(0.05);
                sample.pitch = static_cast<Scalar>(-0.04);
                sample.angularVelocity = Vec3(static_cast<Scalar>(0.1), static_cast<Scalar>(-0.1), 0);
                sample.linearAcceleration = Vec3(0, 0, static_cast<Scalar>(9.81));
                engine.setImu(sample);
            }
            now += 10'000'000;
            const GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            if (out.commanded) {
                for (int leg = 0; leg < NUM_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }

private:
    void setEfforts() {
        Scalar footZ[NUM_LEGS];
        Scalar lowest = 1;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            footZ[leg] = ik.fk(feedback.angles[leg]).z();
            lowest = std::min(lowest, footZ[leg]);
        }
        int loaded = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            loaded += footZ[leg] <= lowest + static_cast<Scalar>(0.005) ? 1 : 0;
        }
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const Scalar fz = footZ[leg] <= lowest + static_cast<Scalar>(0.005) ? 25 / static_cast<Scalar>(loaded) : 0;
            feedback.effort[leg] = -ik.jacobian(feedback.angles[leg]).transpose() * Vec3(0, 0, fz);
        }
    }
};

}  // namespace

TEST(NoHeap, TheCounterSeesAllocations) {
    const long before = allocations.load();
    // A direct call: compilers may drop a new-expression whose result is never used
    void* p = ::operator new(16);
    ::operator delete(p);
    std::vector<int> v(10);
    EXPECT_EQ(allocations.load() - before, 2);
}

TEST(NoHeap, RunningTheGaitNeverAllocates) {
    Robot robot;
    robot.run(3.0);  // startup
    robot.engine.sit(false);
    robot.run(6.0);  // stand up

    const long before = allocations.load();
    logged = 0;

    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(3.0);
    robot.engine.setGait(gait_core::explorer::GAIT_RIPPLE);
    robot.run(3.0);
    robot.engine.setGait(gait_core::explorer::GAIT_WAVE);
    robot.engine.setMode(gait_core::MODE_STRAFE);
    robot.engine.setVelocity({static_cast<Scalar>(0.05), static_cast<Scalar>(0.05), static_cast<Scalar>(0.2)});
    robot.run(4.0);
    robot.engine.setGait(gait_core::explorer::GAIT_TRIPOD);
    robot.engine.setVelocity({0, 0, 0});
    robot.run(3.5);   // idle return, load balancing
    robot.imu = true;
    robot.engine.setMode(gait_core::MODE_NORMAL);
    robot.engine.setVelocity({static_cast<Scalar>(0.08), 0, 0});
    robot.run(2.0);   // walk leveling
    robot.engine.setVelocity({0, 0, 0});
    robot.engine.setHeight(static_cast<Scalar>(-0.11));
    robot.run(2.0);
    robot.engine.setMode(gait_core::MODE_TILT);
    robot.engine.setBodyPose(static_cast<Scalar>(0.1), static_cast<Scalar>(0.05), 0, Vec3(static_cast<Scalar>(0.02), 0, 0));
    robot.run(3.0);
    robot.engine.setMode(gait_core::MODE_NORMAL);
    robot.run(2.0);
    robot.engine.sit(true);
    robot.run(6.0);
    robot.engine.sit(false);
    robot.run(6.0);
    robot.engine.powerDown();   // sit, servos off (rebuilds the engine's state)
    robot.run(8.0);
    robot.engine.sit(false);    // power up, startup, stand
    robot.run(10.0);

    EXPECT_EQ(allocations.load() - before, 0) << "allocations while running the gait";
    EXPECT_GT(logged, 5) << "the log hook should have been called (gait changes, tilt, sitting)";
    EXPECT_GT(robot.engine.odometry().x(), 0.2) << "the robot should have walked";
}

TEST(NoHeap, TheBatteryMonitorNeverAllocates) {
    using gait_core::behaviour::BatteryLevel;
    gait_core::behaviour::BatteryMonitor battery(2);
    int logs = 0;
    battery.log = [&logs](LogLevel, const char*) { ++logs; };

    const long before = allocations.load();
    Timestamp now = 1'000'000'000;
    BatteryLevel worst = BatteryLevel::Ok;
    // Run the battery down through warning to critical, sit, then charge it back up
    for (int i = 0; i < 2000; ++i) {
        now += 100'000'000;
        const Scalar volts = i < 1000 ? Scalar(11.5) - Scalar(0.002) * static_cast<Scalar>(i) : Scalar(11.5);
        battery.onReading(0, volts, Scalar(-2), now);
        battery.onReading(1, volts + Scalar(0.1), Scalar(-2), now);
        const auto status = battery.update(now);
        worst = std::max(worst, status.level);
        if (status.sit) {
            battery.sitRequested();
        }
    }
    EXPECT_EQ(allocations.load() - before, 0) << "allocations while monitoring the battery";
    EXPECT_EQ(worst, BatteryLevel::Critical);
    EXPECT_EQ(battery.level(), BatteryLevel::Ok);
    EXPECT_GE(logs, 3);
}

TEST(NoHeap, ChainKinematicsNeverAllocates) {
    // Numerical IK on every tick of a leg with no analytic solver must stay off the heap too
    const gait_core::ChainKinematics chain(explorerRobot().legs[0]);
    JointVec q = JointVec::Zero(3);
    q << Scalar(0.1), Scalar(-0.5), Scalar(1.5);
    const long before = allocations.load();
    Scalar reached = 0;
    for (int i = 0; i < 200; ++i) {
        const Vec3 target = Vec3(Scalar(0.17), Scalar(0.0005) * static_cast<Scalar>(i - 100), Scalar(-0.12));
        JointVec next;
        chain.ik(target, q, next);
        q = next;
        reached += chain.fk(q).norm() + chain.jacobian(q).norm();
        Vec3 force;
        chain.groundForce(q, q, force);
    }
    EXPECT_EQ(allocations.load() - before, 0) << "allocations in ChainKinematics";
    EXPECT_GT(reached, 0);
}

namespace {

/**
 * @brief Run one of the other test robots through walking, turning, stopping, tilt and sitting
 * @return The allocations made after it stood up
*/
long allocationsRunning(const gait_core::RobotDefinition& robot, double speed, bool sits,
                        bool dynamicStability = false, bool balanceFeedback = false) {
    gait_core::GaitParams params;
    params.dynamic_stability = dynamicStability;
    params.balance_feedback = balanceFeedback;
    params.preview_control = balanceFeedback;   // Phase C with Phase B
    params.torque_output = balanceFeedback;
    GaitEngine engine{params, robot};
    engine.log = [](LogLevel, const char*) { ++logged; };
    JointFeedback feedback;
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        feedback.present[leg] = true;
        feedback.angles[leg] = JointVec::Zero(robot.legs[leg].numJoints);
    }
    Timestamp now = 1'000'000'000;
    auto run = [&](double seconds) {
        const int ticks = static_cast<int>(seconds / 0.01 + 0.5);
        for (int i = 0; i < ticks; ++i) {
            feedback.stamp = now;
            engine.onJointStates(feedback);
            now += 10'000'000;
            const GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            if (out.commanded) {
                for (int leg = 0; leg < robot.numLegs; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    };
    run(3.0);
    engine.sit(false);
    run(20.0);

    const long before = allocations.load();
    engine.setVelocity({static_cast<Scalar>(speed), 0, 0});
    run(5.0);
    engine.setVelocity({0, 0, static_cast<Scalar>(0.15)});
    run(5.0);
    engine.setVelocity({0, 0, 0});
    run(8.0);
    engine.setMode(gait_core::MODE_TILT);
    run(1.0);
    engine.setBodyPose(static_cast<Scalar>(0.06), 0, 0, Vec3(static_cast<Scalar>(0.01), 0, 0));
    run(2.0);
    engine.setMode(gait_core::MODE_NORMAL);
    run(3.0);
    if (sits) {
        engine.sit(true);
        run(15.0);
    }
    return allocations.load() - before;
}

}  // namespace

TEST(NoHeap, AQuadrupedSwayingNeverAllocates) {
    EXPECT_EQ(allocationsRunning(test_robots::quadruped(), 0.03, true), 0);
}

TEST(NoHeap, DynamicStabilityNeverAllocates) {
    EXPECT_EQ(allocationsRunning(test_robots::quadruped(), 0.1, true, true), 0);
    EXPECT_EQ(allocationsRunning(explorerRobot(), 0.15, true, true), 0);
}

TEST(NoHeap, BalanceFeedbackNeverAllocates) {
    EXPECT_EQ(allocationsRunning(test_robots::quadruped(), 0.1, true, true, true), 0);
    EXPECT_EQ(allocationsRunning(explorerRobot(), 0.15, true, true, true), 0);
}

TEST(NoHeap, FourJointLegsNeverAllocate) {
    EXPECT_EQ(allocationsRunning(test_robots::fourJointHexapod(), 0.08, true), 0);
}

TEST(NoHeap, TwoJointLegsNeverAllocate) {
    EXPECT_EQ(allocationsRunning(test_robots::twoJointHexapod(), 0.05, false), 0);
}

TEST(NoHeap, AnArmSwingingAndCarryingNeverAllocates) {
    // Explorer with an arm, walking with torque output, the arm reported moving and its efforts
    // holding a load: the payload estimate, the centre of mass and the hold torques every tick
    gait_core::GaitParams params;
    params.torque_output = true;
    const gait_core::RobotDefinition robot = test_robots::explorerWithArm();
    GaitEngine engine{params, robot};
    engine.log = [](LogLevel, const char*) { ++logged; };
    JointFeedback feedback;
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        feedback.present[leg] = true;
        feedback.angles[leg] = JointVec::Zero(robot.legs[leg].numJoints);
    }
    feedback.hasEffort = true;
    feedback.appendagePresent[0] = true;
    feedback.appendageAngles[0] = JointVec::Zero(2);
    const Vec3 down(0, 0, static_cast<Scalar>(-9.81));
    Timestamp now = 1'000'000'000;
    int tick = 0;
    auto run = [&](double seconds) {
        const int ticks = static_cast<int>(seconds / 0.01 + 0.5);
        for (int i = 0; i < ticks; ++i, ++tick) {
            feedback.appendageAngles[0](0) = static_cast<Scalar>(0.8 * std::sin(tick * 0.05));
            feedback.appendageAngles[0](1) = static_cast<Scalar>(0.4 * std::sin(tick * 0.03));
            feedback.appendageEffort[0] = gait_core::appendageHoldTorques(robot.appendages[0], feedback.appendageAngles[0],
                                                                          down, static_cast<Scalar>(0.5));
            feedback.stamp = now;
            engine.onJointStates(feedback);
            now += 10'000'000;
            const GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            if (out.commanded) {
                for (int leg = 0; leg < robot.numLegs; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    };
    run(3.0);
    engine.sit(false);
    run(8.0);

    const long before = allocations.load();
    engine.setVelocity({static_cast<Scalar>(0.05), 0, 0});
    run(5.0);
    engine.setVelocity({0, 0, 0});
    run(2.0);
    EXPECT_EQ(allocations.load() - before, 0) << "allocations while the arm moved and carried";
    EXPECT_GT(engine.appendagePayload(0), 0.4) << "it worked the load out";
}
