#ifndef GAIT_CORE__GAIT_ENGINE_HPP
#define GAIT_CORE__GAIT_ENGINE_HPP

#include <array>
#include <cstdint>
#include <functional>
#include <optional>

#include "gait_core/locomotion/balance_controller.hpp"
#include "gait_core/locomotion/gait_config.hpp"
#include "gait_core/locomotion/preview_controller.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"
#include "gait_core/estimation/body_velocity.hpp"
#include "gait_core/estimation/leg_odometry.hpp"
#include "gait_core/robot/robot_definition.hpp"
#include "gait_core/locomotion/trajectory_generator.hpp"
#include "gait_core/types.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"

namespace gait_core {

/**
 * @brief Every setting of the gait, in SI units. Defaults are the hexapod-ros2 parameter defaults.
 * @note The ones marked "live" can be changed while running through GaitEngine::params().
*/
struct GaitParams {
    // Timing
    Scalar phase_duration = Scalar(1.0);        // s per gait phase at low speeds
    Scalar min_phase_duration = Scalar(0.55);   // s, shortest phase allowed, which sets each gait's top speed
    Scalar max_foot_speed = Scalar(1.5);        // m/s, cap on how fast a foot target can move
    Scalar max_joint_speed = Scalar(4.0);       // rad/s, phases slow down so no joint exceeds this; a
                                                // joint's own JointDefinition::max_velocity overrides it
    Scalar idle_return_delay = Scalar(2.0);     // s idle before the legs return to their start positions
    Scalar startup_duration = Scalar(1.0);      // s for the startup move to the start positions
    Scalar max_target_speed = Scalar(0.5);      // m/s, how fast a landing target can move when the command changes
    uint8_t initial_mode = MODE_NORMAL; // mode at startup

    // Tilt mode (live)
    Scalar tilt_max_angle = Scalar(0.3);        // rad, largest roll, pitch or yaw in total
    Scalar tilt_max_translation = Scalar(0.04); // m, largest body shift along each axis
    Scalar tilt_angle_rate = Scalar(1.0);       // rad/s, how fast the commanded angles can change
    Scalar tilt_translation_rate = Scalar(0.1); // m/s, how fast the commanded shift can change
    bool tilt_leveling = true;          // hold the body's angles in space using the IMU
    bool tilt_lock_yaw = false;         // also hold the heading
    Scalar tilt_level_gain = Scalar(8.0);       // 1/s, how fast the leveling correction responds

    // Foot forces and contact (live)
    Scalar contact_force_on = Scalar(2.0);      // N, vertical force above which a foot is in contact
    Scalar contact_force_off = Scalar(1.0);     // N, below which it no longer is
    Scalar foot_force_filter = Scalar(0.05);    // s, time constant of the force low-pass filter
    Scalar blocked_force_ratio = Scalar(0.3);   // net horizontal / vertical foot force above which the robot is blocked

    // Load balancing (live)
    bool load_balancing = true;         // adjust foot heights to spread the weight evenly
    Scalar load_balance_gain = Scalar(0.0001);  // m/(N s), how fast a foot moves per newton of extra load
    Scalar load_balance_max = Scalar(0.02);     // m, largest height adjustment of a foot

    // Static stability: the centre of mass over the planted feet, judged in the body frame (no IMU needed)
    Scalar min_stability_margin = Scalar(0.01); // m, how far inside the planted feet's polygon the centre of
                                                // mass must stay: in every gait's phases, and in tilt mode (live)
    bool allow_unstable_gaits = false;  // run gaits that fall short of it, e.g. a quadruped trot; when off,
                                        // a robot with one won't move
    // Body sway: a gait whose planted feet can't hold the centre of mass where the body stands, but
    // could with the body moved over them (e.g. a quadruped crawl), shifts the body sideways before
    // each leg lifts. Gaits stable without it (a hexapod's) never sway.
    bool body_sway = true;              // sway gaits that need it (startup); off, they count as unstable
    Scalar sway_speed = Scalar(0.05);           // m/s, fastest the body sways (live)
    Scalar sway_margin = Scalar(0.02);          // m, how far inside the feet swaying aims for, at least
                                                // min_stability_margin, which a leg waits for (live)

    // Uneven ground (live)
    bool terrain_adaptation = true;     // stop swings on early contact and reach down for missing ground
    Scalar touchdown_force = Scalar(3.0);       // N, vertical force that ends a swing in its second half
    Scalar max_step_height = Scalar(0.04);      // m, furthest a foot's ground can be above or below nominal
    Scalar seek_speed = Scalar(0.05);           // m/s, how fast a foot that didn't find the ground reaches down
    Scalar body_height_rate = Scalar(0.02);     // m/s, how fast the body follows the feet's average ground height
    Scalar touchdown_ratio = Scalar(1.5);       // a touchdown's vertical force must be this many times its horizontal force
    Scalar stumble_force = Scalar(3.0);         // N, horizontal force on a swinging foot that means it hit something
    Scalar stumble_lift = Scalar(0.03);         // m, extra height a stumbling foot is lifted by
    bool terrain_map = false;           // use the foothold planner (a height map) when there is one

    // Swing shape (live)
    Scalar step_height = Scalar(0.06);          // m, height of the swing's middle control points
    Scalar swing_easing = Scalar(0.5);          // 0 even pace along the swing, 1 smootherstep

    // Body height and sitting (height ones live)
    Scalar height_rate = Scalar(0.03);          // m/s, how fast the body moves to a new height
    Scalar min_height = Scalar(0.06);           // m, lowest body height setHeight accepts
    Scalar max_height = Scalar(0.16);           // m, highest body height setHeight accepts
    Scalar sit_reach = Scalar(0.22);            // m, how far out the feet stand while seated
    Scalar sit_height = Scalar(0.015);          // m, feet below the hips while seated (positive)
    bool stand_on_startup = false;      // stand up as soon as startup has seated the robot

    // Servo power (a robot that switches it, like the leg hubs' SERVO_EN)
    bool power_on_startup = true;       // servos powered from the start (simulation, or no power switch);
                                        // false keeps them off until the first stand command
    Scalar servo_loss_timeout = Scalar(0.5);    // s without a full joint state, once running, before the servos
                                                // count as unpowered (e-stop); 0 never

    // Leveling while walking (live)
    bool walk_leveling = true;          // hold the body level with the IMU while walking
    Scalar walk_level_gain = Scalar(2.0);       // 1/s, how fast the correction responds
    Scalar walk_max_angle = Scalar(0.15);       // rad, largest roll or pitch correction

    // Dynamic stability (live): the body as an inverted pendulum over its feet, from what the gait
    // commands (see BalanceController). On, the body speeds up or slows down no harder than the
    // planted feet can push for (the ZMP stays min_stability_margin inside them), and a leg only
    // lifts once the capture point and the ZMP are that far inside the feet that stay down, the body
    // slowing until they are. Off, the same quantities are still worked out for balanceState().
    bool dynamic_stability = false;
    Scalar com_acceleration_filter = Scalar(0.05);  // s, low-pass time constant of the centre of mass acceleration

    // Balance feedback (live): react to the motion the robot really has, not just what it was told.
    // The centre of mass velocity is measured (leg odometry blended with the IMU), the support
    // polygon is the feet the force estimates say are pushing, and the capture point is predicted
    // over the loop delay. Its error from the commanded one is corrected by shifting the body over
    // the feet (no harder than they can push for), and when that can't catch it, by landing the
    // swinging feet where it will be at touchdown, stepping from standing if need be.
    bool balance_feedback = false;
    Scalar velocity_filter = Scalar(0.2);       // s, how long the IMU's acceleration is trusted before leg odometry takes over
    Scalar balance_delay = Scalar(0.03);        // s, from measuring to the feet acting on it
    Scalar capture_gain = Scalar(0.5);          // share of the capture point error the body shift corrects, 0..1
    Scalar capture_deadband = Scalar(0.01);     // m, capture point error always left alone; feedback also
                                                // ignores what's within 3x the error's own RMS (its
                                                // gait's usual wobble, over ~2 s), so only a push acts
    Scalar correction_max = Scalar(0.03);       // m, furthest the body shifts over its feet to correct
    bool capture_stepping = true;               // step where the capture point is heading when the shift can't catch it

    // Preview control (live): for a gait that sways, plan the sway over the coming phases instead of
    // reacting at each. The ZMP reference runs through points inside both the feet down in one phase
    // and the next, predicted from the gait's schedule and the command, and the body's path is the
    // smoothest that tracks it (PreviewController), so it's over the next feet before they're all it
    // has. The sway hold at a phase's start stays, as a safety net. Gaits that don't sway are untouched.
    bool preview_control = false;
    Scalar preview_time = Scalar(1.0);          // s of the coming gait it plans over (at most 200 ticks)

    // Joint torques (live), for robots whose actuators take a torque as well as a position: each
    // planted leg's share of carrying the robot and accelerating it as planned, tau = -J^T f, in
    // GaitOutput::torques. Needs the robot's masses (RobotDefinition::bodyMass and the links').
    bool torque_output = false;

    // Foot placement (live): for a gait that balances by stepping (one whose feet down are ever just a
    // line, like a trot, pace or bound), land each swinging foot where the body is really heading as
    // well as where the command sends it: offset by the capture point of the velocity the body has
    // that the command didn't ask for, (measured - commanded) / omega, times foot_placement_gain.
    // Measures the body's velocity as balance_feedback does (leg odometry and the IMU).
    bool foot_placement = false;
    Scalar foot_placement_gain = Scalar(1.0);   // share of that capture point offset the feet step by
};

enum class SitState : uint8_t {
    STANDING,
    SPREADING,  // stepping the feet out to sit_reach
    LOWERING,   // lowering the body onto the ground
    SEATED,
    RAISING,    // lifting the body back to its standing height
    GATHERING   // stepping the feet back in to the standing reach
};

/**
 * @brief Measured joint state of every leg
 * @note Each leg's vectors start as YawPitchPitchLeg::kNumJoints (3) zeros, which suits explorer. For
 *       legs with another joint count, size them to that leg's LegDefinition::numJoints: a leg
 *       whose angles have a different number of entries is treated as not reported. (A default
 *       JointVec is empty, not zero, so they must never be left default-constructed.)
*/
struct JointFeedback {
    Timestamp stamp = 0;
    std::array<JointVec, kMaxLegs> angles;           // rad, indexed by leg number, body to foot
    std::array<bool, kMaxLegs> present{};            // which legs' joints were all reported
    bool hasEffort = false;                          // whether effort is filled in
    std::array<JointVec, kMaxLegs> effort;           // N m the servos apply, same order as angles
    // From foot contact sensors, for a robot that has them: whether each foot is touching the ground.
    // When given, it replaces the contact worked out from the efforts, everywhere the engine uses it,
    // and torque output loads only feet that really are down.
    bool hasContact = false;
    std::array<bool, kMaxLegs> contact{};

    JointFeedback() {
        angles.fill(JointVec::Zero(YawPitchPitchLeg::kNumJoints));
        effort.fill(JointVec::Zero(YawPitchPitchLeg::kNumJoints));
    }
};

/**
 * @brief An IMU reading: orientation, angular velocity and linear acceleration, and when it was received
 * @note Rates and acceleration are in the body frame (REP 103: +x forward, +y left, +z up), so an
 *       IMU mounted another way is rotated into it by the caller. Linear acceleration is what an
 *       accelerometer reads (as sensor_msgs/Imu carries it), so it includes gravity: about
 *       (0, 0, +9.81) with the body level and still. setControllerImu only uses the orientation.
*/
struct ImuSample {
    Timestamp stamp = 0;
    Scalar roll = Scalar(0);
    Scalar pitch = Scalar(0);
    Scalar yaw = Scalar(0);
    Vec3 angularVelocity = Vec3::Zero();      // rad/s, about the body's x (roll), y (pitch) and z (yaw) axes
    Vec3 linearAcceleration = Vec3::Zero();   // m/s^2, gravity included
};

/**
 * @brief Whether a command was accepted, and a message saying what happened or why not
*/
struct CommandResult {
    bool success = false;
    char message[96] = "";
};

/**
 * @brief What one update produced
*/
struct GaitOutput {
    bool commanded = false;                           // false while waiting for joint states or unpowered
    std::array<JointVec, kMaxLegs> joints;            // rad, indexed by leg number, body to foot
    bool servoPower = false;                          // whether the servos should be powered (drive SERVO_EN from this)
    // With GaitParams::torque_output: a feedforward torque for each joint (N m, same order as joints)
    // that carries the robot's weight and pushes it as planned, to add to the position control. Zero
    // for swinging legs. hasTorques is false when there are none (off, no masses, no foot down).
    bool hasTorques = false;
    std::array<JointVec, kMaxLegs> torques;

    // Zeros, never empty; the engine resizes each leg to its kinematics' numJoints()
    GaitOutput() {
        joints.fill(JointVec::Zero(YawPitchPitchLeg::kNumJoints));
        torques.fill(JointVec::Zero(YawPitchPitchLeg::kNumJoints));
    }
};

/**
 * @brief Optional hook for choosing footholds from a map of the ground (hexapod-ros2's height map)
*/
class FootholdPlanner {
public:
    virtual ~FootholdPlanner() = default;
    /**
     * @brief Called once per walking tick before any foothold is planned
     * @return false if the map can't be used this tick
    */
    virtual bool ready() = 0;
    /**
     * @brief Move a swinging foot's landing target and set its lift
     * @param[in] legNum The leg
     * @param[in,out] target The landing target (leg frame)
     * @param[in] swingStart Where the swing lifted off (leg frame)
     * @param[in,out] swingLift The swing's lift (m, 0 = the default step_height)
    */
    virtual void plan(int legNum, Vec3& target, const Vec3& swingStart, Scalar& swingLift) = 0;
};

/**
 * @brief The whole gait: startup, walking, sitting, tilt mode, leveling, load balancing, terrain
 *        adaptation and leg odometry, with no ROS or hardware dependency
 * @note Call update() at a steady rate, and feed it joint states, IMU orientation, controller
 *       orientation and velocity commands as they arrive. Every update() returns all 18 joint
 *       targets once the joint states have arrived. Optional inputs (efforts, IMU, controller
 *       orientation) can simply never be given: the features that need them stay off.
 *
 *       Servo power: GaitOutput::servoPower says whether the servos should be powered. Unpowered,
 *       the engine commands nothing; a stand command powers them, waits for fresh joint states, moves
 *       the legs from wherever they are to the seated pose, then stands. powerDown() sits and then
 *       turns them off. Losing them unasked (no full joint state for servo_loss_timeout, or
 *       servoPowerLost()) turns power off too, so releasing an e-stop never moves the robot by itself.
 *       While the servos are unpowered or not answering, the robot should report those joints as
 *       not present rather than repeat stale angles.
*/
class GaitEngine {
public:
    /**
     * @brief An engine for one robot
     * @param[in] params The gait settings
     * @param[in] geometry The robot: its legs, stance and gaits (e.g. gait_core::explorer::robot())
    */
    GaitEngine(const GaitParams& params, const RobotDefinition& geometry);

    // ---- Inputs -----------------------------------------------------------------------------
    /**
     * @brief The commanded body velocity, already smoothed (SI, body frame)
    */
    void setVelocity(const VelocityCommand& cmd) { velocity = cmd; }
    void setImu(const ImuSample& sample);
    /**
     * @brief The body's velocity over the ground from an estimator outside the engine (visual or
     *        inertial odometry, motion capture, a state estimator)
     * @param[in] velocity m/s, body frame (x forward, y left), at the body origin
     * @param[in] yawRate rad/s, anticlockwise
     * @param[in] stamp When it was measured
     * @note While it's fresh (under 0.1 s old) balance feedback and foot placement use it in place of
     *       their own estimate from leg odometry and the IMU.
    */
    void setBodyVelocity(const Vec2& velocity, Scalar yawRate, Timestamp stamp) {
        external_velocity = velocity;
        external_yaw_rate = yawRate;
        external_velocity_stamp = stamp;
        has_external_velocity = true;
    }
    /**
     * @brief Measured joint state, at whatever rate it arrives
     * @return true if the leg odometry moved on (see odometry())
    */
    bool onJointStates(const JointFeedback& feedback);
    void servoPowerLost();
    /**
     * @brief Use a controller's orientation as the requested tilt while active
     * @param[in] sample Controller roll, pitch and yaw (rad), and when received
     * @param[in] active True while the hold button is pressed
     * @note The first active sample is latched as neutral. Releasing or losing the
     *       controller recentres the requested pose.
    */
    void setControllerImu(const ImuSample& sample, bool active);

    // ---- Commands ---------------------------------------------------------------------------
    CommandResult setGait(uint8_t gait);
    CommandResult setMode(uint8_t mode);
    CommandResult setHeight(Scalar height);  // feet z below the hips, negative
    CommandResult sit(bool down);
    CommandResult powerDown();
    void setBodyPose(Scalar roll, Scalar pitch, Scalar yaw, const Vec3& translation);

    // ---- Tick -------------------------------------------------------------------------------
    /**
     * @brief Advance the gait
     * @param[in] dt Time since the last update (s), already clamped by the caller
     * @param[in] now The current time
    */
    GaitOutput update(Scalar dt, Timestamp now);

    // ---- State ------------------------------------------------------------------------------
    GaitParams& params() { return p; }
    const GaitParams& params() const { return p; }
    const GaitConfig& gaitConfig() const { return trajectoryGen.config(); }
    const TrajectoryGenerator& trajectory() const { return trajectoryGen; }
    const LegOdometry& odometry() const { return legOdometry; }
    SitState sitState() const { return sit_state; }
    bool initialized() const { return positions_initialized; }
    bool seated() const { return sit_state != SitState::STANDING; }
    bool servoPower() const { return servo_power; }
    bool blocked() const { return is_blocked; }
    const Vec3& measuredFoot(int legNum) const { return measured_leg_positions[legNum]; }
    const Vec3& footForce(int legNum) const { return foot_forces[legNum]; }
    bool footForceUpdated(int legNum) const { return foot_force_updated[legNum]; }
    Scalar maxFootSpeed() const;
    Scalar footprintHeight() const;
    Scalar floorRise() const;
    Scalar imuRoll() const { return body_roll; }
    Scalar imuPitch() const { return body_pitch; }
    const Vec3& imuAngularVelocity() const { return body_angular_velocity; }   // rad/s, body frame
    const Vec3& imuLinearAcceleration() const { return body_linear_acceleration; }   // m/s^2, body frame, gravity included
    bool controllerTiltActive() const { return controller_tilt_active; }
    // m, body frame: the whole robot's centre of mass with the legs where they were last commanded
    // (RobotDefinition::centreOfMass if the robot gives no masses)
    const Vec3& centreOfMass() const { return com; }
    // The body as an inverted pendulum while walking: centre of mass, its commanded velocity and
    // acceleration, height, capture point, ZMP and their margins (see dynamic_stability)
    const BalanceState& balanceState() const { return balance_controller.state(); }
    // Whether a leg is waiting to lift until the body's momentum fits the feet that stay down
    bool liftHeld() const { return lift_held; }
    // m, body frame: how far balance feedback has shifted the body over its feet (see balance_feedback)
    const Vec2& balanceCorrection() const { return correction; }
    // m, body frame: how far foot placement is moving the swinging feet's landing points
    Vec2 placementOffset() const;
    // Whether the gait is stepping where the capture point is heading, and by how far (m, body frame)
    bool catchingStep() const { return catching_step; }
    const Vec2& stepOffset() const { return step_offset; }

    // ---- Hooks ------------------------------------------------------------------------------
    std::function<void(LogLevel, const char*)> log;           // optional
    std::function<void(bool seated)> seatedChanged;         // optional: a sit or stand began or ended
    FootholdPlanner* footholdPlanner = nullptr;             // optional, not owned
    void setLegKinematics(int legNum, const LegKinematics* kinematics);  // not owned; null = built in
    int numLegs() const { return robot.numLegs; }
    // m, how far the centre of mass stays inside the planted feet through a gait's phases, at the
    // worst: at the feet's neutral stance, less maxStride for how far they sweep while walking.
    // For a gait that sways, how far it could be with the body moved over the feet.
    // -infinity for a gait that can't run on the robot's legs or an unknown one.
    Scalar gaitStabilityMargin(uint8_t gait) const;
    // Whether a gait sways the body to stay stable (see GaitParams::body_sway)
    bool gaitSways(uint8_t gait) const { return gait < config().numGaits() && gait_sways[gait]; }
    // Whether the feet a gait keeps down always enclose an area, in every phase: false for a trot,
    // whose diagonal pair is a line. Balance feedback only shifts the body and steps for gaits that do.
    bool gaitHasSupportArea(uint8_t gait) const { return gait < config().numGaits() && gait_area[gait]; }
    // m, the body's current sway (x, y), relative to where walking alone would put it
    const Vec2& bodySway() const { return sway; }
    // Preview control's planner (see preview_control), and the ZMP it was last asked to track next
    const PreviewController& preview() const { return preview_controller; }
    const Vec2& previewReference() const { return preview_reference[0]; }
    // The first of the robot's gaits whose margin is below min_stability_margin, or -1 if none is
    // (always -1 with allow_unstable_gaits)
    int unstableGait() const;
    // Whether the engine can run every one of the robot's gaits on its legs (see GaitPattern::schedule),
    // and stably unless allowed otherwise. If not, it never moves and keeps the servos off.
    bool robotSupported() const { return numLegs() > 0 && config().unrunnableGait() < 0 && unstableGait() < 0; }
    // The solver each leg uses (unless setLegKinematics overrides it): its definition's, except
    // that a leg asking for YawPitchPitch whose chain isn't that shape gets Chain
    LegSolver legSolver(int legNum) const { return leg_solver[legNum]; }

private:
    GaitParams p;
    RobotDefinition robot;   // kept so cutting servo power can rebuild the engine
    TrajectoryGenerator trajectoryGen;
    // The robot's gaits, stance and modes live in trajectoryGen, which steps the feet from them;
    // one copy, so a height or reach change reaches the trajectories too
    GaitConfig& config() { return trajectoryGen.config(); }
    const GaitConfig& config() const { return trajectoryGen.config(); }

    // Every leg's kinematics: an override from setLegKinematics (not owned) if set, otherwise the
    // leg's analytic solver or its chain, as leg_solver says, both built from its definition. The
    // built in ones are members rather than pointers to them, so copying or moving the engine
    // (cutServoPower does) can never leave a pointer into the old engine.
    std::array<std::optional<YawPitchPitchLeg>, kMaxLegs> analytic;   // set where leg_solver says so
    std::array<ChainKinematics, kMaxLegs> chains;
    std::array<LegSolver, kMaxLegs> leg_solver{};
    std::array<const LegKinematics*, kMaxLegs> legKinematics{};
    const LegKinematics& kinematics(int legNum) const {
        if (legKinematics[legNum] != nullptr) {
            return *legKinematics[legNum];
        }
        if (leg_solver[legNum] == LegSolver::YawPitchPitch) {
            return *analytic[legNum];
        }
        return chains[legNum];
    }

    VelocityCommand velocity{};
    Timestamp now_ = 0;
    GaitOutput output;

    uint8_t phase = 0;
    Scalar phase_progress = Scalar(0);    // fraction of the current phase completed, 0 to 1
    Scalar startup_progress = Scalar(0);  // fraction of the startup move completed, 0 to 1
    Scalar current_phase_duration = Scalar(1);  // s, duration chosen for the phase in progress
    Scalar walk_speed_scale = Scalar(1);  // last walk tick's jointSpeedScale, which slows the landing targets too

    // IMU
    Scalar body_roll = Scalar(0);
    Scalar body_pitch = Scalar(0);
    Scalar body_yaw = Scalar(0);
    Timestamp last_imu_time = 0;
    bool has_imu = false;
    Vec3 body_angular_velocity = Vec3::Zero();      // rad/s, body frame
    Vec3 body_linear_acceleration = Vec3::Zero();   // m/s^2, body frame, gravity included

    // Controller orientation following
    Scalar controller_reference_roll = Scalar(0);
    Scalar controller_reference_pitch = Scalar(0);
    Scalar controller_reference_yaw = Scalar(0);
    Timestamp last_controller_imu_time = 0;
    bool controller_tilt_active = false;

    bool has_joint_states = false;
    bool positions_initialized = false;

    // Servo power
    bool servo_power = true;
    bool power_down_pending = false;   // seat, then turn the servos off
    bool full_state_since_update = false;  // a full joint state arrived since the last update
    Timestamp last_full_state = 0;     // update time at which the latest full joint state had arrived
    bool idleReturning = false;

    struct MotionIntent {
        Scalar forward;  // m/s
        Scalar left;     // m/s
        Scalar yaw;      // rad/s, positive anticlockwise
    };
    MotionIntent current_motion_intent{};

    std::array<Vec3, kMaxLegs> current_leg_positions;   // commanded foot positions (leg frame)
    std::array<Vec3, kMaxLegs> measured_leg_positions;  // foot positions from joint state feedback (leg frame)
    std::array<JointVec, kMaxLegs> measured_angles;     // the joint angles they came from (rad)
    std::array<Vec3, kMaxLegs> startup_start_positions;
    bool startup_planned = false;

    // Walking
    std::array<Vec3, kMaxLegs> swing_start;   // lift-off position (leg frame)
    std::array<Vec3, kMaxLegs> swing_target;  // landing target (leg frame), rate limited
    std::array<bool, kMaxLegs> leg_swinging{};

    // Idle return and return to start (function statics in the ROS node)
    int return_counter = 0;
    Scalar idle_time = Scalar(0);
    bool has_moved_from_start = false;
    bool return_completed = false;

    // Tilt mode
    BodyPose body_pose_request;
    BodyPose tilt_command;
    BodyPose tilt_correction;
    BodyPose tilt_applied;
    std::array<Vec3, kMaxLegs> tilt_ground_feet;
    bool tilt_active = false;
    Scalar tilt_yaw_reference = Scalar(0);

    // Foot forces and contact
    std::array<Vec3, kMaxLegs> foot_forces{};
    std::array<bool, kMaxLegs> foot_contact{};
    bool contact_sensed = false;   // the latest joint state's contact came from sensors (JointFeedback::contact)
    std::array<bool, kMaxLegs> foot_force_updated{};
    bool has_foot_forces = false;
    Timestamp last_foot_force_stamp = 0;
    std::array<Scalar, kMaxLegs> foot_force_offset{};
    Scalar push_ratio = Scalar(0);
    bool is_blocked = false;

    // Body height and sitting
    Scalar target_height;
    SitState sit_state = SitState::SEATED;
    Scalar stand_reach;
    Scalar stand_height;

    // Uneven ground
    std::array<Scalar, kMaxLegs> terrain_offset{};
    std::array<bool, kMaxLegs> seeking{};
    std::array<bool, kMaxLegs> landing_check{};
    std::array<Scalar, kMaxLegs> unloaded_time{};
    std::array<int, kMaxLegs> touchdown_ticks{};
    std::array<int, kMaxLegs> stumble_ticks{};
    std::array<Scalar, kMaxLegs> stumble_target{};
    std::array<Scalar, kMaxLegs> stumble_boost{};
    std::array<Scalar, kMaxLegs> swing_lift{};
    Scalar ground_level = Scalar(0);

    // Load balancing
    std::array<Scalar, kMaxLegs> foot_height_offset{};

    // Leveling while walking
    Scalar walk_level_roll = Scalar(0);
    Scalar walk_level_pitch = Scalar(0);
    BodyPose walk_level;
    Scalar tilt_saved_roll = Scalar(0);
    Scalar tilt_saved_pitch = Scalar(0);
    std::array<Vec3, kMaxLegs> tilt_saved_feet;

    // Centre of mass and dynamic stability
    Vec3 com = Vec3::Zero();               // m, body frame: live, see centreOfMass()
    BalanceController balance_controller;
    MotionIntent applied_intent{};         // the velocity actually walked, acceleration limited
    MotionIntent requested_intent{};       // the command it heads for, before limiting: idle only when both are
    bool lift_held = false;                // the phase's legs are waiting to lift
    std::array<Vec2, kMaxLegs> previous_command_feet{};    // last tick's commanded feet (body frame x, y)
    std::array<bool, kMaxLegs> previous_command_planted{}; // and which were down
    bool has_previous_command_feet = false;
    Scalar lift_speed_scale = 1;           // while held, the share of the command the capture point fits
    Scalar lift_hold_time = 0;             // s they have waited
    Vec3 liveCentreOfMass() const;
    void limitAcceleration(Scalar dt);
    bool holdLift(Scalar dt);
    void updateBalance(Scalar dt, Scalar moveScale);
    void resetBalance();

    // Balance feedback
    Vec2 odom_velocity = Vec2::Zero();       // m/s, body frame: the body over the planted feet, from leg odometry
    Scalar odom_yaw_rate = 0;                // rad/s
    Timestamp odom_velocity_stamp = 0;       // when it was measured
    bool has_odom_velocity = false;
    BodyVelocityEstimator velocity_estimator;
    Vec2 external_velocity = Vec2::Zero();   // m/s, from setBodyVelocity
    Scalar external_yaw_rate = 0;
    Timestamp external_velocity_stamp = 0;
    bool has_external_velocity = false;
    // What recent walking ticks commanded the body to do over its feet, so leg odometry (which
    // measures joint states, so a command or more behind) is compared with the command it measured
    struct CommandRecord {
        Timestamp time = 0;
        Vec2 velocity = Vec2::Zero();   // m/s, at the centre of mass, sway and correction included
    };
    std::array<CommandRecord, 16> command_history{};
    int command_count = 0;
    int command_next = 0;
    Vec3 previous_com = Vec3::Zero();        // last tick's centre of mass in the body (legs only moving it)
    Vec2 previous_offset = Vec2::Zero();     // last tick's sway plus correction
    bool has_previous_com = false;
    Vec2 correction = Vec2::Zero();          // m, the body's shift over its feet to correct the capture point
    Vec2 correction_velocity = Vec2::Zero(); // m/s, how fast that shift is moving
    bool correction_saturated = false;       // the shift is at correction_max or limited by the feet
    Scalar capture_noise = 0;                // m^2, the capture point error's running mean square (~2 s)
    Scalar feedback_time = 0;                // s it has been measuring since it last started (it learns before acting)
    Vec2 placement_error = Vec2::Zero();     // m/s, the body's velocity less the command's, smoothed over ~a phase
    bool catching_step = false;              // stepping to where the capture point is heading
    bool step_needed = false;                // the capture point needs a step this tick
    Vec2 step_offset = Vec2::Zero();         // m, body frame: how far the swinging feet land from the gait's targets
    void updateFeedback(Scalar dt, const Vec2& commandedOverGround, const Vec2& drift,
                        const std::array<Vec2, kMaxLegs>& feet, int count);
    bool centreOfPressure(Vec2& cop) const;
    bool contactPolygon() const;
    int plantedFeet(std::array<Vec2, kMaxLegs>& feet, bool staying) const;

    // Leg odometry
    LegOdometry legOdometry;
    std::array<Vec2, kMaxLegs> previous_feet;
    std::array<Scalar, kMaxLegs> previous_feet_z{};
    std::array<Scalar, kMaxLegs> planted_time{};
    static constexpr Scalar CLIMB_SETTLE_TIME = static_cast<Scalar>(0.1);
    static constexpr Scalar CLIMB_ANCHOR_TIME = static_cast<Scalar>(0.5);
    bool climb_anchored = false;
    Scalar odom_start_height = Scalar(0);
    std::array<bool, kMaxLegs> previous_contact{};
    Timestamp previous_feet_stamp = 0;
    bool has_previous_feet = false;

    // Throttled log messages: when each was last sent
    std::array<Timestamp, 10> last_throttled_log{};
    std::array<bool, 10> throttled_log_sent{};

    std::array<Scalar, kMaxGaits> gait_margin{};   // gaitStabilityMargin, worked out once
    std::array<bool, kMaxGaits> gait_sways{};      // gaitSways
    std::array<bool, kMaxGaits> gait_area{};       // gaitHasSupportArea
    Scalar computeGaitMargin(uint8_t gait, bool sway) const;
    Scalar plantedMargin(const std::array<Vec3, kMaxLegs>& feet, const Vec2& bodySway) const;

    // Body sway: the body's shift over the ground (x, y) on top of walking, added to walk_level
    // when the feet are commanded. Stays zero for gaits that don't sway.
    Vec2 sway = Vec2::Zero();
    Vec2 tilt_saved_sway = Vec2::Zero();
    Scalar sway_hold_time = 0;   // s the current phase has waited for the sway
    bool sway_holding = false;   // waiting at a phase's start for the body to sway over the feet
    BodyPose walkPose() const;
    void easeSway(const Vec2& goal, Scalar dt);
    bool updateSway(Scalar dt, bool idle);
    bool holdForSway(Scalar dt, bool idle, const std::array<Vec2, kMaxLegs>& feet, int count, const Vec2& goal);

    // Preview control
    struct PlannedMotion {
        Scalar forward = 0, left = 0, yaw = 0;   // m/s, m/s, rad/s: the body's commanded velocity
        Scalar stanceTime = 0;                   // s each foot stays down, which sets where swings land
    };
    PlannedMotion planned_motion;                // set by walk() (or zero, stepping back to stance)
    PreviewController preview_controller;
    bool preview_started = false;                // whether it has run at all
    Timestamp preview_last_tick = 0;             // when it last ran: a tick missed restarts it from the body
    Vec2 preview_from = Vec2::Zero();            // where the reference starts this phase: the last handover
    Vec2 preview_handover = Vec2::Zero();        // where it hands over to the next phase, as last worked out
    size_t preview_phase = 0;                    // the phase those are for
    bool preview_has_phase = false;
    std::array<Vec2, PreviewController::MAX_STEPS> preview_reference{};
    Vec2 previewSway(Scalar dt, bool idle, const std::array<Vec2, kMaxLegs>& staying, int stayingCount);
    Vec2 predictedFoot(int legNum, int segment, const std::array<Scalar, 16>& segmentEnd, Scalar t) const;   // 16: the phases a preview reaches into

    void cutServoPower();
    GaitOutput advance(Scalar dt, Timestamp now);
    void computeTorques();
    std::array<Scalar, kMaxLegs> torque_down_time{};   // s each foot has been down, for ramping its share in
    Scalar torque_dt = 0;                              // the tick the torques are for
    bool walk_advancing = false;                       // the last walking tick moved the gait on
    Scalar totalMass() const;

    // Log a printf-style message; nothing is formatted unless log is set (and, throttled, due)
    void emit(LogLevel level, const char* format, ...);
    void emitThrottled(int key, Scalar period, LogLevel level, const char* format, ...);
    bool throttleDue(int key, Scalar period);

    void updateFootForces(const JointFeedback& feedback);
    bool updateLegOdometry(Timestamp stamp);
    Eigen::Quaternion<Scalar> imuTilt() const;
    bool imuFresh() const;
    bool controllerImuFresh() const;
    bool feetAtStart();
    Vec3 plantedStart(int legNum);
    bool forcesValid();
    void adaptToTerrain(Scalar dt);
    void updateLoadBalance(Scalar dt);
    bool reachable(int legNum, const Vec3& target) const;
    bool everyLegReaches(const Vec3& target) const;
    Vec3 limitSwingLift(int legNum, const Vec3& target, Scalar groundZ);
    void updateWalkLeveling(Scalar dt);
    bool tiltFeasible(const BodyPose& pose, std::array<Vec3, kMaxLegs>& targets);
    void tilt(Scalar dt, bool exiting);
    void sitTick(Scalar dt);
    bool startup(Scalar dt);
    std::array<Vec3, kMaxLegs> legTargets(Scalar ds);
    std::array<Vec3, kMaxLegs> walkTargets(Scalar forward, Scalar left, Scalar yaw, Scalar ds, Scalar moveDt);
    void sendFootTargets(const std::array<Vec3, kMaxLegs>& targets, Scalar dt);
    void commandFoot(const Vec3& target, int legNum, Scalar dt);
    void applyPendingGait();
    bool isSwinging(int legNum) const;
    Scalar jointSpeedScale(const std::array<Vec3, kMaxLegs>& targets, Scalar dt);
    void performLegStep(Scalar dt, bool idle, bool handlePhaseTransition);
    void returnToStart(Scalar dt);
    bool handleIdleReturn(Scalar dt);
    void walk(Scalar dt);
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_ENGINE_HPP
