#ifndef GAIT_CORE__TEST__TEST_ROBOTS_HPP
#define GAIT_CORE__TEST__TEST_ROBOTS_HPP

// Robots that differ from explorer in one way each, to prove the engine isn't a hexapod engine:
// the leg count (a quadruped), and the joints per leg (four, and two). Only the tests use them.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "test_common.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/locomotion/gait_engine.hpp"

namespace test_robots {

using gait_core::GaitPattern;
using gait_core::JointDefinition;
using gait_core::LegDefinition;
using gait_core::LegSolver;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::Vec3;

inline Vec3 v(double x, double y, double z) {
    return Vec3(static_cast<Scalar>(x), static_cast<Scalar>(y), static_cast<Scalar>(z));
}

inline JointDefinition revolute(const Vec3& origin, const Vec3& axis, double lower, double upper) {
    return {origin, axis, static_cast<Scalar>(lower), static_cast<Scalar>(upper),
            std::numeric_limits<Scalar>::infinity()};
}

/**
 * @brief Explorer's legs on the corners of a square body, crawling one leg at a time
 * @note Legs 0..3 front right, rear right, rear left, front left. A crawl is only stable with the
 *       body swayed over the three feet that stay down, so the engine sways it.
*/
inline RobotDefinition quadruped() {
    using gait_core::kPi;
    RobotDefinition robot = explorerRobot();
    const LegDefinition leg = robot.legs[0];
    const Scalar c = static_cast<Scalar>(0.12);
    const Scalar yaws[4] = {-kPi / 4, -3 * kPi / 4, 3 * kPi / 4, kPi / 4};
    const Scalar xs[4] = {c, -c, -c, c};
    const Scalar ys[4] = {-c, -c, c, c};
    robot.numLegs = 4;
    for (int i = 0; i < 4; ++i) {
        robot.legs[i] = leg;
        robot.legs[i].mount = {yaws[i], Vec3(xs[i], ys[i], 0)};
    }
    robot.numGaits = 1;
    // Rear right, front right, rear left, front left: each lift is next to the last one's side
    robot.gaits[0] = GaitPattern::fromGroups("crawl", {{1}, {0}, {2}, {3}});
    return robot;
}

/**
 * @brief Explorer with an ankle: the tibia split into a 0.100 m shank and a 0.086 m foot link by a
 *        pitch joint, so each leg has four joints for three dimensions
 * @note Same stance and reach as explorer. Not yaw-pitch-pitch, so it runs on ChainKinematics.
*/
inline RobotDefinition fourJointHexapod() {
    RobotDefinition robot = explorerRobot();
    for (int i = 0; i < robot.numLegs; ++i) {
        LegDefinition& leg = robot.legs[i];
        leg.numJoints = 4;
        leg.joints[3] = revolute(v(0.100, 0, 0), v(0, 1, 0), -1.2, 1.2);   // ankle, + folds the foot down
        leg.foot = v(0.086, 0, 0);
        leg.solver = LegSolver::Chain;
    }
    return robot;
}

/**
 * @brief Explorer with two joints per leg: a coxa yaw, then one pitch joint and a straight 0.176 m
 *        leg to the foot
 * @note Its foot moves on a surface: for each distance out from the hip there is one height. The
 *       stance (0.177 m out, 0.134 m down) is on it; most other heights aren't.
*/
inline RobotDefinition twoJointHexapod() {
    RobotDefinition robot = explorerRobot();
    for (int i = 0; i < robot.numLegs; ++i) {
        LegDefinition& leg = robot.legs[i];
        leg.numJoints = 2;
        leg.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), -1.4, 1.4);        // coxa
        leg.joints[1] = revolute(v(0.063, 0, 0), v(0, 1, 0), -1.6, 1.4);    // hip, + lowers the foot
        leg.foot = v(std::hypot(0.177 - 0.063, 0.134), 0, 0);
        leg.solver = LegSolver::Chain;
    }
    return robot;
}

/**
 * @brief A small biped: two legs of hip yaw, hip roll, hip pitch and knee, each on a 0.10 x 0.05 m sole
 * @param[in] hipRoll rad, how far each hip rolls either way, which bounds how far the body can sway
 *            over a foot
 * @note Leg 0 left, leg 1 right, hips 0.06 m either side of the body's centre and two 0.1 m links
 *       to the foot, standing 0.17 m below the hips. It walks one foot at a time with both feet down
 *       for a phase between steps; with one foot down it can't stand still, so its gait is allowed
 *       only as unstable (bipedParams).
*/
inline RobotDefinition biped(double hipRoll = 0.35) {
    RobotDefinition robot{};
    robot.numLegs = 2;
    for (int i = 0; i < 2; ++i) {
        const double side = i == 0 ? 1 : -1;
        LegDefinition& leg = robot.legs[i];
        leg.mount = {0, v(0, side * 0.06, 0)};
        leg.numJoints = 4;
        leg.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), -0.5, 0.5);           // hip yaw
        leg.joints[1] = revolute(v(0, 0, 0), v(1, 0, 0), -hipRoll, hipRoll);   // hip roll
        leg.joints[2] = revolute(v(0, 0, 0), v(0, 1, 0), -1.5, 1.5);           // hip pitch
        leg.joints[3] = revolute(v(0, 0, -0.1), v(0, 1, 0), -0.05, 2.0);       // knee, + bends it
        leg.foot = v(0, 0, -0.1);
        leg.solver = LegSolver::Chain;
        leg.soleMin = gait_core::Vec2(static_cast<Scalar>(-0.05), static_cast<Scalar>(-0.025));
        leg.soleMax = gait_core::Vec2(static_cast<Scalar>(0.05), static_cast<Scalar>(0.025));
    }
    robot.standReach = 0;
    robot.standHeight = static_cast<Scalar>(-0.17);
    robot.maxStride = static_cast<Scalar>(0.03);
    robot.centreOfMass = v(0, 0, 0.03);
    robot.numGaits = 1;
    robot.gaits[0] = GaitPattern::fromGroups("walk", {{0}, {}, {1}, {}});
    return robot;
}

/**
 * @brief Parameters biped() runs with: seated with its feet where they stand, its one-footed gait
 *        allowed, and preview control planning the weight from foot to foot if dynamic
*/
inline gait_core::GaitParams bipedParams(bool dynamic = true) {
    gait_core::GaitParams p;
    p.allow_unstable_gaits = true;
    p.sit_reach = 0;
    p.sit_height = static_cast<Scalar>(0.13);
    p.stand_on_startup = true;
    p.max_height = static_cast<Scalar>(0.18);
    p.phase_duration = static_cast<Scalar>(0.4);
    p.min_phase_duration = static_cast<Scalar>(0.3);
    p.step_height = static_cast<Scalar>(0.03);
    p.preview_control = dynamic;
    p.preview_dynamic_gaits = dynamic;
    return p;
}

/**
 * @brief Runs an engine on any robot with perfect servos: every command comes straight back as the
 *        joint state, while the engine wants the servos powered
 * @note Checks every command as it goes (sized to the leg, finite, inside the joint limits) and
 *       keeps the first problem, plus every warning and error the engine logs.
*/
class SimRobot {
public:
    explicit SimRobot(const RobotDefinition& robot, const gait_core::GaitParams& params = gait_core::GaitParams{})
        : definition(robot), engine(params, robot), servoPower(params.power_on_startup) {
        for (int leg = 0; leg < robot.numLegs; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = gait_core::JointVec::Zero(robot.legs[leg].numJoints);
        }
        engine.log = [this](gait_core::LogLevel level, const char* message) {
            if (level == gait_core::LogLevel::Warn) {
                warnings.emplace_back(stage + ": " + message);
            } else if (level == gait_core::LogLevel::Error) {
                errors.emplace_back(stage + ": " + message);
            }
        };
    }
    SimRobot(const SimRobot&) = delete;   // the log hook points at this one
    SimRobot& operator=(const SimRobot&) = delete;

    void run(double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        for (int i = 0; i < ticks; ++i) {
            if (servoPower) {
                feedback.stamp = now;
                feedback.hasEffort = sensors || efforts;
                if (sensors || efforts) {
                    setEfforts();
                }
                engine.onJointStates(feedback);
            }
            if (sensors || imu) {
                gait_core::ImuSample sample;
                sample.stamp = now;
                sample.linearAcceleration = Vec3(0, 0, static_cast<Scalar>(9.81));
                engine.setImu(sample);
            }
            now += 10'000'000;
            const gait_core::GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            servoPower = out.servoPower;
            if (engine.catchingStep()) {
                if (catchingTicks == 0) {
                    firstCatch = stage + " at t=" + std::to_string(now / 1e9) + " s";
                }
                ++catchingTicks;
            }
            heldTicks += engine.liftHeld() ? 1 : 0;
            if (out.commanded) {
                for (int leg = 0; leg < definition.numLegs; ++leg) {
                    check(leg, out.joints[leg]);
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }

    /**
     * @brief Run until a condition holds, for at most some time
     * @return Whether it held
    */
    template <typename Condition>
    bool runUntil(Condition condition, double limit) {
        for (double t = 0; t < limit && !condition(); t += 0.1) {
            run(0.1);
        }
        return condition();
    }

    bool standUp() {
        run(3.0);
        engine.sit(false);
        return runUntil([this] { return engine.sitState() == gait_core::SitState::STANDING; }, 30.0);
    }

    // Where each foot is, from the last command (body frame)
    Vec3 foot(int leg) const {
        return engine.trajectory().legToBody(leg, gait_core::ChainKinematics(definition.legs[leg]).fk(feedback.angles[leg]));
    }

    RobotDefinition definition;
    gait_core::GaitEngine engine;
    gait_core::JointFeedback feedback;
    gait_core::Timestamp now = 1'000'000'000;
    bool servoPower;
    bool sensors = false;                // feed a level IMU, and joint efforts with the lowest feet carrying the weight
    bool imu = false;                    // just the IMU
    bool efforts = false;                // just the joint efforts
    int catchingTicks = 0;               // ticks spent stepping to catch the robot (balance_feedback)
    std::string firstCatch;              // where the first of them was
    int heldTicks = 0;                   // ticks a lift was held (dynamic_stability)
    std::string problem;                 // the first bad command, empty if none
    std::string stage = "start";      // what the robot is doing, for messages
    std::vector<std::string> warnings;
    std::vector<std::string> errors;

private:
    /**
     * @brief 25 N shared by the lowest feet (within 5 mm), as the joint torques that hold it
     * @note A leg too short of joints to resolve a force (two) still reports torques; the engine
     *       finds no force from them, as it would on the real robot
    */
    void setEfforts() {
        Scalar lowest = 1;
        std::array<Scalar, gait_core::kMaxLegs> z{};
        for (int leg = 0; leg < definition.numLegs; ++leg) {
            z[leg] = foot(leg).z();
            lowest = std::min(lowest, z[leg]);
        }
        int loaded = 0;
        for (int leg = 0; leg < definition.numLegs; ++leg) {
            loaded += z[leg] <= lowest + static_cast<Scalar>(0.005) ? 1 : 0;
        }
        for (int leg = 0; leg < definition.numLegs; ++leg) {
            const Scalar fz = z[leg] <= lowest + static_cast<Scalar>(0.005) ? 25 / static_cast<Scalar>(loaded) : 0;
            const gait_core::ChainKinematics chain(definition.legs[leg]);
            feedback.effort[leg] = -chain.jacobian(feedback.angles[leg]).transpose() * Vec3(0, 0, fz);
        }
    }

    void check(int leg, const gait_core::JointVec& q) {
        if (!problem.empty()) {
            return;
        }
        const LegDefinition& def = definition.legs[leg];
        const std::string where = "leg " + std::to_string(leg) + " at t=" + std::to_string(now / 1e9) + " s (" + stage + "): ";
        if (q.size() != def.numJoints) {
            problem = where + std::to_string(q.size()) + " joints commanded, not " + std::to_string(def.numJoints);
        } else if (!q.allFinite()) {
            problem = where + "not finite";
        } else {
            for (int j = 0; j < def.numJoints; ++j) {
                if (q(j) < def.joints[j].lower - 1e-6 || q(j) > def.joints[j].upper + 1e-6) {
                    problem = where + "joint " + std::to_string(j) + " at " + std::to_string(q(j)) + " rad, outside its limits";
                    return;
                }
            }
        }
    }
};

}  // namespace test_robots

#endif  // GAIT_CORE__TEST__TEST_ROBOTS_HPP
