# legged-gait-core

A legged robot's gait and behaviours as plain C++17 libraries with no ROS, Arduino or Pico SDK
dependency, so every robot walks and reacts to its sensors the same way. A robot is described as
data (its legs as chains of joints, its stance and its gaits), so the same engine runs a hexapod or
a quadruped. It also holds what every stack has to agree on to be driven the same way: the
controller packets and the gamepad layout.

How this fits with the robots and the other repositories is documented in
[hexapod-docs](https://github.com/TH4R4V3N1324/hexapod-docs).

## Layout

Everything is under one include root and one namespace, `gait_core`, in a folder per area. `src/`
and `test/` mirror the same folders.

| Folder (`include/gait_core/…`) | Namespace | CMake target | What |
| --- | --- | --- | --- |
| `types.hpp` | `gait_core` | `gait_core::gait_core` | Scalars, vectors, `JointVec`, time, logging |
| `robot/` | `gait_core` | `gait_core::gait_core` | What a robot is: legs, joint chains, stance, gaits (data only) |
| `kinematics/` | `gait_core` | `gait_core::gait_core` | Solving one leg: the `LegKinematics` interface and its solvers |
| `locomotion/` | `gait_core` | `gait_core::gait_core` | Making the robot walk: the engine, gait timing, trajectories, frames |
| `estimation/` | `gait_core` | `gait_core::gait_core` | What the robot measures: leg odometry |
| `behaviour/` | `gait_core::behaviour` | `gait_core::behaviour` | Decisions from sensors, on top of the gait's types |
| `robots/` | `gait_core::explorer`, … | `gait_core::robots` | Real robots' definitions. The core has none built in |
| `protocol/` | (C) | `gait_core::protocol` | Controller ↔ robot packets. Header-only, no dependencies |
| `teleop/` | `gait_core::teleop` | `gait_core::teleop` | The shared gamepad layout. Header-only, no dependencies |

The gait doesn't depend on the behaviours or the robots, so a robot can use the gait alone with its
own definition. The protocol and teleop headers need nothing at all (not even Eigen), so
controllers can use them too.

### Robot description (`robot/`)

| Header | Contents |
| --- | --- |
| `robot_definition.hpp` | `RobotDefinition`: the legs (`numLegs`, up to `GAIT_CORE_MAX_LEGS`), neutral stance, max stride, centre of mass, body mass and gaits. With masses given (`bodyMass` > 0), `centreOfMass` is the body's own and the engine adds the legs' links where the joints put them; without, it's the whole robot's and fixed. Also its appendages (`numAppendages`, up to `GAIT_CORE_MAX_APPENDAGES`) |
| `appendage_definition.hpp` | `AppendageDefinition`: a head, arm or mast on the body that isn't a leg: its mount, joint chain (with masses), tip and `maxPayload`. The engine doesn't move it but follows it: given its angles in `JointFeedback::appendageAngles`, its links count towards the centre of mass and weight wherever they are, and so does what it holds at its tip, set with `setAppendagePayload` or worked out from `JointFeedback::appendageEffort` (the effort beyond what its own links need, up to `maxPayload`; 0, as for a head, never estimates one). With `torque_output`, `GaitOutput::appendageTorques` holds it up |
| `leg_definition.hpp` | `LegDefinition`: where the leg is mounted, its joint chain, the foot, which solver to use (`LegSolver`), and its own `stanceOffset` from the shared stance |
| `joint_definition.hpp` | `JointDefinition`: a revolute joint's origin, axis, limits and speed limit, and the mass and centre of mass of the link it turns (as a URDF `<inertial>` gives them) |
| `gait_pattern.hpp` | `GaitPattern`: a gait as a duty factor and per-leg offsets (`fromGroups` for groups that take turns); `GaitSchedule`, the phases the engine runs it as |

### Kinematics (`kinematics/`)

| Header | Contents |
| --- | --- |
| `leg_kinematics.hpp` | `LegKinematics`: FK, IK from a seed, Jacobian, and foot force from joint torques, for any number of joints |
| `chain_kinematics.hpp` | `ChainKinematics`: any `LegDefinition`, with exact FK/Jacobian and numerical IK (damped least squares) |
| `LegKinematics::ikWithSole` | For legs with a sole (`LegDefinition::hasSole`), the engine solves through `ikWithSole(target, sole, seed, q)`: `sole` is how the sole should be turned in the leg frame, flat on the ground and facing the way it was put down, under tilt mode's pose and as the robot turns (not walk leveling's, which comes from the IMU: turning planted soles by it closes a loop through the ground) (a planted sole turns back with the body; a swinging one turns to land at the turn over half a stance, as its landing point does). Kinematics that can turn the foot (a hip yaw, an ankle) override it; by default it's plain `ik`. `GaitOutput::soles` reports the same for robots that solve their own legs |
| Turn feedback | With `turn_feedback` on, the engine makes the robot turn as fast as it's asked: the turn it plans is scaled by `turnScale()`, learned from the IMU's yaw rate while walking a turn (by `turn_feedback_gain` x (asked - measured) / asked per second, within 1 +- `turn_feedback_range`). For a robot whose feet slip as it turns: in MuJoCo a biped's soles slip round as its weight passes from foot to foot, turning it ~20% further than planned at any rate |
| `mass_properties.hpp` | `legMass`: a leg's links' total mass and first moment for some joint angles; `MassMoment` adds them up into a centre of mass. `appendageMass`, `appendageHoldTorques` and `appendagePayloadTorques` do the same for an appendage, with a load at its tip, and the torques that hold it up |
| `yaw_pitch_pitch_leg.hpp` | `YawPitchPitchLeg`: exact analytic kinematics for a yaw joint then two pitch joints (coxa/femur/tibia) |

### Locomotion (`locomotion/`)

| Header | Contents |
| --- | --- |
| `gait_engine.hpp` | `GaitEngine`: the whole gait (startup, walking, sit/stand, tilt mode, leveling, load balancing, terrain adaptation, leg odometry) for the `RobotDefinition` it's given. Feed it joint states, IMU (`ImuSample`: orientation, body-frame angular velocity and linear acceleration) and velocity; call `update(dt, now)` each tick for every leg's joint targets, and `servoPower` to drive a servo power switch (unpowered until the first stand with `power_on_startup` off; `powerDown()` sits then switches off; losing the servos switches off too). Settings in `GaitParams` |
| `gait_config.hpp` | The robot's gaits and the current one, modes, neutral stance, height/reach |
| `trajectory_generator.hpp` | Swing Béziers, stance lines, foot displacement, body pose |
| `preview_controller.hpp` | `PreviewController`: ZMP preview control (Kajita's cart-table model). Given the ZMP reference for the next N ticks, the smoothest centre of mass path that tracks it, so the body sets off before the support it needs arrives. Gains from an LQ problem with integral action, solved once per tick length and ω (in double precision), no allocations |
| `support_polygon.hpp` | `stabilityMargin`: how far a point (the centre of mass) is inside the convex hull of some planted feet. `deepestSupportPoint` and `nearestStablePoint` for where to move it. The engine refuses a robot whose gaits fall below `min_stability_margin` (unless `allow_unstable_gaits`), stops tilt-mode body shifts at it, and warns when walking leaves it. A gait that is only stable with the body moved over its feet, such as a quadruped crawl, sways the body before each leg lifts (`body_sway`); gaits stable without it, such as explorer's, never sway. Judged in the body frame, so no IMU is needed |
| `balance_controller.hpp` | `BalanceController`: the body as a linear inverted pendulum over its planted feet, from what the gait commands. Each walking tick it takes the live centre of mass c and works out its velocity and acceleration, the height h and ω = √(g/h), the capture point ξ = c + ċ/ω and the ZMP c − c̈/ω², with how far each is inside the feet. `canLift` checks ξ and the ZMP against the feet left down when a leg lifts; `accelerationScale` caps acceleration at ω² × the spare margin; `velocityScale` gives the speed whose ξ fits some feet. With `dynamic_stability` on, the engine uses all three: the body speeds up and slows down no harder than the feet can push for, and a leg only lifts once the body's momentum fits the feet that stay down, the body slowing just enough until it does. Off (the default), the same state is still worked out for `GaitEngine::balanceState()`. On explorer's wide tripod nothing changes; a quadruped's crawl keeps its ZMP within 2 mm of its feet, against 4 mm outside without (the rest is its stance feet braking at the edge of their stride, which needs preview control). With `balance_feedback` on as well (Phase B), it also works from what's measured: `measure` takes the measured centre of mass velocity and centre of pressure and predicts the capture point over the loop delay (ξ grows away from where the feet push by e^(ωδ)); its error from the commanded one gives `correctionVelocity`, which the engine makes by shifting the body over its feet (no harder than `accelerationScale` allows, up to `correction_max`), and `touchdownOffset`, where swinging feet land when the shift can't catch it: a shove the shift can't absorb makes the robot step, from standing too. The support polygon is then the feet the force estimates say are pushing (at least three), and leg odometry is compared with the command it actually measured, so the gait's own motion is never taken for a push. `balanceCorrection()`, `catchingStep()` and `stepOffset()` report what it's doing. Feedback only acts on an error that stands out from the gait's own wobble (beyond `capture_deadband` plus three times the error's running RMS), so a robot's normal rocking or soft servos aren't taken for pushes. Feet with no area between them, like a trot's diagonal pair, have no inside to keep anything in: the lift gate and acceleration limit don't hold such a gait back, and feedback neither shifts the body nor steps for a gait that has such phases (`gaitHasSupportArea`), since it balances by its motion, which needs preview control. Tested on MuJoCo dogs, a hexapod, pentapod, octopod and a four-joint spider: a trot walks as it does without, statically stable gaits walk unchanged and brake 3-18% harder after a shove. Feedback learns a robot's usual wobble for 2 s after it starts before it acts. With `preview_control` on (Phase C), a gait that sways plans its sway over the coming phases: the ZMP reference runs through points inside both one phase's feet and the next's, predicted from the schedule and the command, and the body's path is the one that tracks it; when consecutive feet share too little, it waits with every foot down while the body moves over. A quadruped's crawl walks 15% further in the same time with more margin and half the sway acceleration, and ANYmal's crawl in MuJoCo, which falls without it, completes its demo. With `torque_output` on, `GaitOutput::torques` gives each planted leg's share of carrying the robot (and accelerating it along a planned path) as joint torques, tau = -J^T f at the measured joint angles, the shares putting the centre of pressure where it should be and ramped in and out over 50 ms as feet land and lift; `JointFeedback::contact` takes foot contact sensors, which it then loads by. In MuJoCo these torques walk the dogs at least as well as the harness's own weight feedforward. With `foot_placement` on, a gait that balances by stepping (a trot, pace or bound) lands each swinging foot further by the capture point of the velocity the body has beyond the command, `foot_placement_gain` x (measured - commanded) / omega, smoothed over about a phase (`placementOffset()`). It needs a good velocity: `setBodyVelocity` takes one from a state estimator outside the engine (visual-inertial odometry, motion capture). On MuJoCo dogs with their true velocity it helps a little (straighter heading, slightly more turn) and never hurts; with only leg odometry and the IMU, which can't see a trot's feet slip, it can make them worse, so leave it off without one |
| `frame_converter.hpp` | Leg frame ↔ body frame |

### Estimation (`estimation/`)

| Header | Contents |
| --- | --- |
| `leg_odometry.hpp` | Body motion and climb from planted feet |
| `body_velocity.hpp` | `BodyVelocityEstimator`: the centre of mass's velocity over the ground, leg odometry (no drift) blended with the IMU (fast: gravity-free acceleration, and the gyro's tipping velocity, which odometry can't see) in a complementary filter. Without a usable accelerometer it's the odometry itself |

### Robots (`robots/`)

| Header | Contents |
| --- | --- |
| `explorer.hpp` | Explorer, a hexapod: `gait_core::explorer::robot()` (with its CAD masses: 3.32 kg, two thirds of it in the legs), and names for its legs (`LEG_FRONT_RIGHT` …) and gaits (`GAIT_TRIPOD`, `GAIT_RIPPLE`, `GAIT_WAVE`) |

#### Other robots

Explorer is the only robot shipped, but the engine isn't specific to it. The tests run three
others through everything it does (stand, walk, turn, stop, tilt, sit, power down and up), with
every command checked against its leg's joint count and limits (`test/test_robots.hpp`,
`test/robot/test_other_robots.cpp`):

- **A quadruped** (explorer's legs on a square body, crawling): works, swaying its body over the
  three planted feet before each lift. A trot is refused unless `allow_unstable_gaits` is set,
  since two feet can't hold the body up and no sway helps.
- **Four joints per leg** (an ankle added): works on `ChainKinematics`. The spare joint is eased
  towards the middle of its range without moving the foot, so a stance comes back in the same pose
  whatever path the legs took.
- **Two joints per leg** (a coxa and one pitch joint): walks and turns, but its foot moves on a
  surface, where each distance from the hip has one height. Planted feet differ in height by up to
  about 10 mm as their reach changes through a stride. It can't take the seated pose (startup
  stands it straight up, with a warning), sit or power down (both refused: sitting steps the feet
  out before lowering), or tilt far (limited by reach, with a warning). Foot forces aren't
  estimated: two joints can't resolve a 3D force.

### Behaviours

Each behaviour takes sensor readings and returns what to do; the robot does it with its own
hardware (a service call or `GaitEngine::sit`, a controller rumble, a beep, an LED). A robot
without a sensor just never feeds it, and that behaviour stays idle.

| Header | Contents |
| --- | --- |
| `battery_monitor.hpp` | `BatteryMonitor`: filtered voltage from up to 4 power monitors, OK / warning / critical with hysteresis, alert when it gets worse, sit once at critical. Settings in `BatteryParams` |

### Protocol

The packets between a handheld controller or gamepad receiver and a microcontroller robot:
ESP-NOW for hexapod-esp32, I²C for hexapod-servo2040, both sent by hexapod-controller. Valid C as
well as C++, since some controller firmware is C.

| Header | Contents |
| --- | --- |
| `packets.h` | `ControlPacket` (sticks, tilt yaw and height, optional controller orientation hold), `HexPacket` (height, gait, mode, seated / servo power / feature flags, how many gaits the robot has, calibration offsets for up to `HEXAPOD_MAX_JOINTS_PER_LEG` joints), `Command`, `Gait`, `Mode` and `GaitFeature`. Both start with `HEXAPOD_PROTOCOL_VERSION` (4); a receiver drops any other version |

A gait is an index into the robot's gait list (`HexPacket.numGaits` of them); the `Gait` names are
explorer's, and `Mode` equals the engine's values (both checked in the tests). To change the
layout, bump the version and the size checks, and append rather than reorder enum values.

### Teleop

| Header | Contents |
| --- | --- |
| `gamepad_teleop.hpp` | `GamepadTeleop`: the gamepad layout. Feed it a normalised `GamepadState` and the robot's `RobotState` (seated, tilt mode); it returns velocities and a tilt pose as fractions of the robot's limits, plus events: next mode/gait, feature toggles, sit/stand, power down, height steps. `stepHeight()` picks the next body height. Settings in `TeleopParams` |

The layout, used by hexapod-ros2's `controller_teleop` and hexapod-controller's receiver:

| Input | Walking (normal / strafe) | Tilt mode |
| --- | --- | --- |
| Left stick | Forward/back, left/right | Shift the body forward/back, left/right |
| Right stick X | Turn | Roll |
| Right stick Y | — | Pitch (forward lowers the nose) |
| LT / RT [L2 / R2] | — | Yaw left / right |
| D-pad up / down | Body height up / down a step, repeating while held (not while seated) | Raise / lower the body |
| RB [R1] | On release: sit down if standing, otherwise stand up. Held 2 s: power down instead | The same |
| LB [L1] | Hold to follow the controller IMU orientation, release to recenter | The same |
| Start [Options] | Next mode | Next mode |
| Back [Create] | Next gait | Next gait |
| A, B, X, Y [Cross, Circle, Square, Triangle] | Terrain adaptation, walk leveling, load balancing, tilt leveling on/off | The same |

## API conventions

- SI units: metres, radians, seconds.
- Body frame as in [Coordinate frames](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/frames.md).
  Each leg frame has +X pointing out along the leg from its coxa joint.
- Legs are numbered `0 .. numLegs - 1` in the robot's definition, and per-leg arrays have room
  for `kMaxLegs` (`GAIT_CORE_MAX_LEGS`). Joint angles are a `JointVec` per leg, body to foot, with
  room for `kMaxJointsPerLeg` (`GAIT_CORE_MAX_JOINTS`); nothing allocates on the heap.
- Explorer's legs run clockwise from `explorer::LEG_FRONT_RIGHT` (0) to `explorer::LEG_FRONT_LEFT`
  (5). The hardware and URDF names count from 1
  (see [Legs](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/concepts/legs.md)),
  so leg 0 is `leg1`.
- Callers convert to and from other units or frames at their own edges, never inside the core.

## Building and testing

```sh
git clone --recursive https://github.com/TH4R4V3N1324/legged-gait-core.git
cd legged-gait-core
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

GoogleTest is used from the system if installed, otherwise downloaded.

### CMake options

| Option | Default | Effect |
| --- | --- | --- |
| `GAIT_CORE_USE_DOUBLE` | `OFF` | Make `Scalar` `double` instead of `float` |
| `GAIT_CORE_BUNDLED_EIGEN` | `OFF` | Always use `third_party/eigen`, even if a system Eigen is found. Always on when cross-compiling |
| `GAIT_CORE_BUILD_TESTS` | `ON` when top-level | Build the test suite |
| `GAIT_CORE_MAX_LEGS` | `6` | Most legs a robot can have (sizes every per-leg array). Explorer needs 6 |
| `GAIT_CORE_MAX_JOINTS` | `6` | Most joints in one leg (sizes `JointVec`). The protocol carries up to 6 |
| `GAIT_CORE_MAX_APPENDAGES` | `2` | Most appendages (heads, arms) a robot can have (sizes their arrays); 0 for none |

`Scalar` defaults to `float` because the microcontroller targets have no or single-precision
FPUs. CI runs the tests with both `float` and `double`, and cross-compiles for Cortex-M0+.

## Linking

- **CMake subproject:** `add_subdirectory(path/to/gait_core)` then link `gait_core::gait_core`,
  `gait_core::behaviour`, `gait_core::robots`, `gait_core::protocol` and/or `gait_core::teleop`.
- **Installed package (e.g. colcon):** `find_package(gait_core REQUIRED)` then link the same
  targets. `package.xml` lets colcon build it as a plain CMake package.
- **PlatformIO:** `library.json` is provided, so it works from `lib/` or `lib_deps`.
- **Just the protocol or teleop headers** (a controller that doesn't need the gait): add
  `include/` to the include path. They need nothing else, so the submodule can be cloned without
  its own Eigen submodule.

## Versioning

Releases are tagged `vMAJOR.MINOR.PATCH`. MINOR changes when the API changes; PATCH is for
fixes that keep the API.

## License

Apache-2.0. See [LICENSE](LICENSE).
