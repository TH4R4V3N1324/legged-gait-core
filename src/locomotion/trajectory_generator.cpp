#include "gait_core/locomotion/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace gait_core {

namespace {

/**
 * @brief Rotation matrix for a yaw-pitch-roll body pose (Z * Y * X)
*/
Mat3 poseRotation(const BodyPose& pose) {
    return (AngleAxis(pose.yaw, Vec3::UnitZ())
        * AngleAxis(pose.pitch, Vec3::UnitY())
        * AngleAxis(pose.roll, Vec3::UnitX())).toRotationMatrix();
}

}  // namespace

/**
 * @brief Ensure the gait configuration is set up correctly
*/
void TrajectoryGenerator::EnsureGaitConfig() {
    if (gaitState.config.empty()) {
        gaitState.config = gaitConfig.getGaitConfig(gaitConfig.currentGait);
    }
}

/**
 * @brief Generate swing and stance trajectories for the legs
 * @param[in] swingTargets Where each leg in the swinging phase should land (others are ignored)
 * @param[in] stanceTargets Where each standing leg should end up (swinging legs are ignored)
 * @param[in] currentPositions The current position of each leg
 * @param[in] currentPhase The gait phase whose legs swing
*/
void TrajectoryGenerator::GenerateTrajectories(
    const std::array<Vec3, kMaxLegs>& swingTargets,
    const std::array<Vec3, kMaxLegs>& stanceTargets,
    const std::array<Vec3, kMaxLegs>& currentPositions,
    uint8_t currentPhase
) {
    for (int legNum = 0; legNum < gaitConfig.numLegs(); ++legNum) {
        gaitState.swingSplines[legNum].active = false;
        gaitState.stanceSplines[legNum].active = false;
    }

    // Swing: Bezier from the current foot position to the target
    for (int legNum = 0; legNum < gaitConfig.numLegs(); ++legNum) {
        if (!gaitState.config.swingsIn(legNum, currentPhase)) continue;
        SwingSpline& spline = gaitState.swingSplines[legNum];
        const Vec3 start = currentPositions[legNum];
        const Vec3 end = swingTargets[legNum];

        if ((end - start).norm() < Scalar(1e-6)) {
            spline.s = 0;
            spline.P0 = start;
            spline.P1 = start;
            spline.P2 = start;
            spline.P3 = start;
            spline.P4 = end;
            spline.active = false;
            continue;
        }

        genSwingSpline(spline, start, end);
    }

    // Stance: straight line from the current foot position to the target
    for (int legNum = 0; legNum < gaitConfig.numLegs(); ++legNum) {
        if (gaitState.config.swingsIn(legNum, currentPhase)) continue;
        LineSpline& spline = gaitState.stanceSplines[legNum];
        spline.P0 = currentPositions[legNum];
        spline.P1 = stanceTargets[legNum];
        spline.s = 0;
        spline.active = true;
    }
}

/**
 * @brief Yaw of a leg's frame relative to the body frame
 * @param[in] legNum The leg
 * @return The yaw (rad), matching the coxa joint origins in the URDF
*/
Scalar TrajectoryGenerator::legYaw(int legNum) const {
    return converter.legConfigs.at(legNum).yaw;
}

/**
 * @brief Convert a position in a leg's frame to the body frame
 * @param[in] legNum The leg
 * @param[in] position The position in the leg frame
 * @return The position in the body frame
*/
Vec3 TrajectoryGenerator::legToBody(int legNum, const Vec3& position) const {
    return converter.legToBodyFrame(position, legNum);
}

/**
 * @brief Convert a position from the body frame to a leg's frame
 * @param[in] legNum The leg
 * @param[in] position The position (body frame)
 * @return The position (leg frame)
*/
Vec3 TrajectoryGenerator::bodyToLeg(int legNum, const Vec3& position) const {
    return converter.bodyToLegFrame(position, legNum);
}

/**
 * @brief Where a planted foot sits relative to its leg once the body takes up a pose
 * @param[in] legNum The leg
 * @param[in] groundFoot The foot's position with the body in its neutral pose (leg frame)
 * @param[in] pose The body pose relative to the neutral pose, rotating about the body origin
 * @return The foot's position (leg frame)
 * @note The foot stays where it is on the ground, so it moves by the inverse of the body's pose
*/
Vec3 TrajectoryGenerator::posedFoot(int legNum, const Vec3& groundFoot, const BodyPose& pose) const {
    const Vec3 foot = legToBody(legNum, groundFoot);
    return bodyToLeg(legNum, poseRotation(pose).transpose() * (foot - pose.translation));
}

/**
 * @brief Undo a body pose for a point in the body frame, the inverse of posedFoot
 * @param[in] bodyPoint A point in the posed body's frame
 * @param[in] pose The body pose
 * @return The point in the neutral body frame
*/
Vec3 TrajectoryGenerator::unposedBodyPoint(const Vec3& bodyPoint, const BodyPose& pose) const {
    return poseRotation(pose) * bodyPoint + pose.translation;
}

Mat3 TrajectoryGenerator::rotationOf(const BodyPose& pose) {
    return poseRotation(pose);
}

/**
 * @brief Body pose that rolls and pitches the body about a fixed point rather than its origin
 * @param[in] roll Roll (rad)
 * @param[in] pitch Pitch (rad)
 * @param[in] pivot The point that stays put (neutral body frame)
 * @return The pose, with the translation that keeps the pivot in place
*/
BodyPose TrajectoryGenerator::rotationAbout(Scalar roll, Scalar pitch, const Vec3& pivot) {
    BodyPose pose;
    pose.roll = roll;
    pose.pitch = pitch;
    const Mat3 rotation = (AngleAxis(pitch, Vec3::UnitY())
        * AngleAxis(roll, Vec3::UnitX())).toRotationMatrix();
    pose.translation = pivot - rotation * pivot;
    return pose;
}

/**
 * @brief Position of a leg's neutral foot in the body frame
 * @param[in] legNum The leg
 * @return The foot position relative to the body centre (body frame, z = 0)
*/
Vec3 TrajectoryGenerator::neutralFootInBody(int legNum) const {
    const Vec3& neutral = gaitConfig.startPosition.at(legNum);
    Vec3 foot = converter.legToBodyFrame(Vec3(neutral.x(), neutral.y(), 0), legNum);
    foot.z() = 0;
    return foot;
}

/**
 * @brief Foot displacement in a leg's frame for a body velocity held for a duration
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s, positive anticlockwise)
 * @param[in] legNum The leg
 * @param[in] duration How long the velocity is held (s)
 * @return The displacement (leg frame, z = 0), in the direction the foot swings
 * @note Worked out in the body frame (+x forward, +y left), then rotated into the leg frame.
 *       Yaw rotates the neutral foot about the body centre. A stance foot moves by the negative of this.
*/
Vec3 TrajectoryGenerator::footDisplacement(Scalar forward, Scalar left, Scalar yaw, int legNum, Scalar duration) const {
    return footDisplacementFrom(forward, left, yaw, legNum, gaitConfig.startPosition.at(legNum), duration);
}

/**
 * @brief Foot displacement in a leg's frame for a commanded body velocity held for a duration
 * @param[in] cmd The commanded body velocity
 * @param[in] legNum The leg
 * @param[in] duration How long the velocity is held (s)
 * @return The displacement (leg frame, z = 0), in the direction the foot swings
*/
Vec3 TrajectoryGenerator::footDisplacement(const VelocityCommand& cmd, int legNum, Scalar duration) const {
    return footDisplacement(cmd.vx, cmd.vy, cmd.wz, legNum, duration);
}

/**
 * @brief Foot displacement in a leg's frame for a body velocity, turning about the body centre from a given foot position
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s, positive anticlockwise)
 * @param[in] legNum The leg
 * @param[in] footPos The foot position the rotation starts from (leg frame)
 * @param[in] duration How long the velocity is held (s)
 * @return The displacement (leg frame, z = 0), in the direction the foot swings
*/
Vec3 TrajectoryGenerator::footDisplacementFrom(Scalar forward, Scalar left, Scalar yaw, int legNum, const Vec3& footPos, Scalar duration) const {
    Vec3 foot = legToBody(legNum, Vec3(footPos.x(), footPos.y(), 0));
    foot.z() = 0;
    const Vec3 linear(forward * duration, left * duration, 0);
    const Vec3 rotation = AngleAxis(yaw * duration, Vec3::UnitZ()) * foot - foot;
    return AngleAxis(-legYaw(legNum), Vec3::UnitZ()) * (linear + rotation);
}

/**
 * @brief Distance from the body centre to a leg's neutral foot position
 * @param[in] legNum The leg
 * @return The horizontal distance (m)
*/
Scalar TrajectoryGenerator::bodyRadius(int legNum) const {
    return neutralFootInBody(legNum).norm();
}

/**
 * @brief Largest bodyRadius of any leg
 * @return The distance (m)
*/
Scalar TrajectoryGenerator::maxBodyRadius() const {
    Scalar radius = 0;
    for (int legNum = 0; legNum < gaitConfig.numLegs(); ++legNum) {
        radius = std::max(radius, bodyRadius(legNum));
    }
    return radius;
}

/**
 * @brief Where a swinging foot should land
 * @param[in] legNum The leg
 * @param[in] displacement The foot displacement ahead of neutral (leg frame)
 * @return The target foot position (leg frame), at most maxStride from neutral
*/
Vec3 TrajectoryGenerator::swingTarget(int legNum, const Vec3& displacement) const {
    return clampToStride(legNum, gaitConfig.startPosition.at(legNum) + displacement);
}

/**
 * @brief Where a stance foot should be at the end of the phase
 * @param[in] legNum The leg
 * @param[in] currentPos The foot's current position (leg frame)
 * @param[in] displacement The body's movement over the phase, as a foot displacement (leg frame)
 * @return The target foot position (leg frame), at most maxStride from neutral
*/
Vec3 TrajectoryGenerator::stanceTarget(int legNum, const Vec3& currentPos, const Vec3& displacement) const {
    return clampToStride(legNum, currentPos - displacement);
}

/**
 * @brief Keep a foot target within maxStride of neutral and on the ground
 * @param[in] legNum The leg
 * @param[in] target The foot target (leg frame)
 * @return The clamped target, with z at the current body height
*/
Vec3 TrajectoryGenerator::clampToStride(int legNum, const Vec3& target) const {
    const Vec3& neutralPos = gaitConfig.startPosition.at(legNum);
    const Scalar maxStride = gaitConfig.maxStride;
    Scalar dx = target.x() - neutralPos.x();
    Scalar dy = target.y() - neutralPos.y();
    const Scalar distance = std::hypot(dx, dy);
    if (distance > maxStride) {
        dx *= maxStride / distance;
        dy *= maxStride / distance;
    }
    return {neutralPos.x() + dx, neutralPos.y() + dy, gaitConfig.currentHeight};
}

/**
 * @brief Set up the control points of a swing-phase Bezier spline
 * @param[out] spline The spline to set up
 * @param[in] start The starting position
 * @param[in] end The ending position
 * @note Resets the spline progress and marks the spline active
*/
void TrajectoryGenerator::genSwingSpline(SwingSpline& spline, const Vec3& start, const Vec3& end) const {
    genSwingSpline(spline, start, end, swingLift);
}

/**
 * @brief Set up the control points of a swing-phase Bezier spline with a given lift
 * @param[out] spline The spline to set up
 * @param[in] start The starting position
 * @param[in] end The ending position
 * @param[in] lift Height of the middle control points above the path (m); the foot peaks at ~0.88x this
 * @note The middle three points are all at the same height, so the foot rises most of the way
 *       before moving forward and comes down nearly vertically: at 7% of the way across it is
 *       already at half the lift, so it clears edges it used to clip.
*/
void TrajectoryGenerator::genSwingSpline(SwingSpline& spline, const Vec3& start, const Vec3& end, Scalar lift) const {
    spline.P0 = start;
    spline.P4 = end;

    spline.P1 = spline.P0;
    spline.P1.z() += lift;

    spline.P2 = Scalar(0.5) * (start + end);
    spline.P2.z() = std::max(start.z(), end.z()) + lift;

    spline.P3 = spline.P4;
    spline.P3.z() += lift;

    spline.s = 0;
    spline.active = true;
}

/**
 * @brief Evaluate a quartic Bezier curve with smoothstep easing, blended with even progress by swingEasing
 * @param[in] P0 The start point
 * @param[in] P1 The first control point
 * @param[in] P2 The second control point
 * @param[in] P3 The third control point
 * @param[in] P4 The end point
 * @param[in] s Normalised progress along the curve [0, 1]
 * @return The point on the curve
*/
Vec3 TrajectoryGenerator::evalBezier(const Vec3& P0, const Vec3& P1, const Vec3& P2, const Vec3& P3, const Vec3& P4, Scalar s) const {
    // Smootherstep starts and ends the swing at rest, which holds the foot nearly still around
    // every lift-off and touchdown; blending in even progress keeps it moving through them
    const Scalar eased = s * s * s * (s * (6 * s - 15) + 10);
    const Scalar st = swingEasing * eased + (1 - swingEasing) * s;
    const Scalar u = 1 - st;

    return P0 * (u * u * u * u) +
           P1 * (4 * u * u * u * st) +
           P2 * (6 * u * u * st * st) +
           P3 * (4 * u * st * st * st) +
           P4 * (st * st * st * st);
}

}  // namespace gait_core
