#ifndef GAIT_CORE__SUPPORT_POLYGON_HPP
#define GAIT_CORE__SUPPORT_POLYGON_HPP

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief How far a point is inside the support polygon of some planted feet
 * @param[in] feet The planted feet's ground contact points (x, y), in any order, as
 *            appendContactPoints makes them: one per point foot, a sole's four corners. Points
 *            inside the convex hull and repeats are fine.
 * @param[in] count How many feet, 0..kMaxContactPoints (more are ignored)
 * @param[in] point The point to judge, e.g. the centre of mass projected onto the ground
 * @return m, the distance from point to the nearest edge of the feet's convex hull: positive
 *         inside, zero on an edge, negative outside. With fewer than three feet not in a line
 *         there is no area to stand in, so the result is never positive: minus the distance to the
 *         line, the one foot, or -infinity with no feet.
 * @note Static stability only: a robot holding still is stable when its centre of mass is above
 *       the polygon, and the margin is how far it could be pushed before it tips.
*/
Scalar stabilityMargin(const Vec2* feet, int count, const Vec2& point);

/**
 * @brief The most stable point over some feet: the centre of the largest circle inside their polygon
 * @param[in] feet The planted feet (x, y), as for stabilityMargin
 * @param[in] count How many feet, 0..kMaxContactPoints
 * @param[out] centre That point (unchanged with no feet)
 * @return Its margin, the circle's radius: 0 for fewer than three feet not in a line, -infinity
 *         with no feet
*/
Scalar deepestSupportPoint(const Vec2* feet, int count, Vec2& centre);

/**
 * @brief The point nearest to another that is at least a margin inside some feet
 * @param[in] feet The planted feet (x, y), as for stabilityMargin
 * @param[in] count How many feet, 0..kMaxContactPoints
 * @param[in] point Where the centre of mass is now
 * @param[in] margin m, how far inside the feet it should be
 * @param[out] nearest point itself if it already has the margin, otherwise the closest point that
 *             does; with no such point (the feet are too close together), the deepest point
 * @return Whether a point with the margin exists
*/
bool nearestStablePoint(const Vec2* feet, int count, const Vec2& point, Scalar margin, Vec2& nearest);

/**
 * @brief Add one planted foot's ground contact points to a support polygon's
 * @param[in] foot Where the foot stands (x, y)
 * @param[in] yaw rad, which way its sole points, in the same frame as foot
 * @param[in] soleMin, soleMax The sole's opposite corners in the foot frame (LegDefinition::soleMin,
 *            soleMax); equal for a point foot
 * @param[in,out] points The contact points so far, room for kMaxContactPoints
 * @param[in] count How many there are so far
 * @return How many there are now: one more for a point foot, four more (the sole's corners, turned
 *         by yaw) for a sole; never more than kMaxContactPoints
*/
int appendContactPoints(const Vec2& foot, Scalar yaw, const Vec2& soleMin, const Vec2& soleMax,
                        Vec2* points, int count);

}  // namespace gait_core

#endif  // GAIT_CORE__SUPPORT_POLYGON_HPP
