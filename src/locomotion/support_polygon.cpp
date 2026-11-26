#include "gait_core/locomotion/support_polygon.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace gait_core {

namespace {

constexpr Scalar kInfinity = std::numeric_limits<Scalar>::infinity();

// z of (b - a) x (c - a): positive when a, b, c turn anticlockwise
Scalar cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

// A convex hull: the chain holds one more point than it has corners while it's built
using Hull = std::array<Vec2, kMaxContactPoints + 1>;

Scalar distanceToSegment(const Vec2& point, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Scalar length2 = ab.squaredNorm();
    const Scalar t = length2 > 0 ? std::clamp((point - a).dot(ab) / length2, Scalar(0), Scalar(1)) : Scalar(0);
    return (point - (a + t * ab)).norm();
}

/**
 * @brief Convex hull of some feet, anticlockwise (Andrew's monotone chain)
 * @return How many points it has: 1 for one foot or repeats of it, 2 for feet in a line
 * @note Points in a line or repeated are dropped. count must be 1..kMaxContactPoints.
*/
int convexHull(const Vec2* feet, int count, Hull& hull) {
    count = std::min(count, kMaxContactPoints);
    // By x then y. An insertion sort: at most kMaxContactPoints points, and std::sort here made GCC
    // warn about indices it can't rule out
    std::array<Vec2, kMaxContactPoints> sorted;
    for (int i = 0; i < count; ++i) {
        const Vec2 point = feet[i];
        int j = i;
        while (j > 0 && (point.x() < sorted[j - 1].x() ||
                         (point.x() == sorted[j - 1].x() && point.y() < sorted[j - 1].y()))) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = point;
    }
    int size = 0;
    for (int i = 0; i < count; ++i) {   // lower hull
        while (size >= 2 && cross(hull[size - 2], hull[size - 1], sorted[i]) <= 0) {
            --size;
        }
        hull[size++] = sorted[i];
    }
    for (int i = count - 2, lower = size + 1; i >= 0; --i) {   // upper hull
        while (size >= lower && cross(hull[size - 2], hull[size - 1], sorted[i]) <= 0) {
            --size;
        }
        hull[size++] = sorted[i];
    }
    return size > 1 ? size - 1 : size;   // the last point repeats the first
}

/**
 * @brief A hull's edges as inward half-planes: a point x is inside edge i by normal[i].x - offset[i]
*/
struct Edges {
    std::array<Vec2, kMaxContactPoints> normal;
    std::array<Scalar, kMaxContactPoints> offset;
    int count = 0;

    Edges(const Hull& hull, int size) {
        for (int i = 0; i < size; ++i) {
            const Vec2 along = hull[(i + 1) % size] - hull[i];
            normal[i] = Vec2(-along.y(), along.x()).normalized();   // anticlockwise: inward is left
            offset[i] = normal[i].dot(hull[i]);
        }
        count = size;
    }

    // The point's distance inside the nearest edge's line (negative past it)
    Scalar depth(const Vec2& point) const {
        Scalar least = kInfinity;
        for (int i = 0; i < count; ++i) {
            least = std::min(least, normal[i].dot(point) - offset[i]);
        }
        return least;
    }
};

}  // namespace

Scalar stabilityMargin(const Vec2* feet, int count, const Vec2& point) {
    count = std::clamp(count, 0, kMaxContactPoints);
    if (count == 0) {
        return -kInfinity;
    }
    Hull hull;
    const int size = convexHull(feet, count, hull);
    if (size < 3) {
        // A foot or a line: nowhere to balance, at best on the edge
        return -distanceToSegment(point, hull[0], hull[size - 1]);
    }
    bool inside = true;
    Scalar nearestEdge = kInfinity;
    for (int i = 0; i < size; ++i) {
        const Vec2& a = hull[i];
        const Vec2& b = hull[(i + 1) % size];
        if (cross(a, b, point) < 0) {
            inside = false;
        }
        nearestEdge = std::min(nearestEdge, distanceToSegment(point, a, b));
    }
    return inside ? nearestEdge : -nearestEdge;
}

Scalar deepestSupportPoint(const Vec2* feet, int count, Vec2& centre) {
    count = std::clamp(count, 0, kMaxContactPoints);
    if (count == 0) {
        return -kInfinity;
    }
    Hull hull;
    const int size = convexHull(feet, count, hull);
    if (size < 3) {
        centre = (hull[0] + hull[size - 1]) / 2;
        return 0;
    }
    // The largest inscribed circle touches three edges: try every three, keep the deepest centre
    // that is inside all of them (6 point feet: at most 6 edges, so 20 tries)
    const Edges edges(hull, size);
    Scalar best = -kInfinity;
    for (int i = 0; i < size; ++i) {
        for (int j = i + 1; j < size; ++j) {
            for (int k = j + 1; k < size; ++k) {
                // normal . x - r = offset for each of the three
                Eigen::Matrix<Scalar, 3, 3> a;
                Eigen::Matrix<Scalar, 3, 1> b;
                const int rows[3] = {i, j, k};
                for (int r = 0; r < 3; ++r) {
                    a.row(r) << edges.normal[rows[r]].x(), edges.normal[rows[r]].y(), -1;
                    b(r) = edges.offset[rows[r]];
                }
                if (std::abs(a.determinant()) < Scalar(1e-9)) {
                    continue;
                }
                const Eigen::Matrix<Scalar, 3, 1> solution = a.partialPivLu().solve(b);
                const Vec2 candidate = solution.head<2>();
                const Scalar depth = edges.depth(candidate);
                if (depth > best) {
                    best = depth;
                    centre = candidate;
                }
            }
        }
    }
    return best;
}

bool nearestStablePoint(const Vec2* feet, int count, const Vec2& point, Scalar margin, Vec2& nearest) {
    count = std::clamp(count, 0, kMaxContactPoints);
    if (count == 0) {
        nearest = point;
        return false;
    }
    Hull hull;
    const int size = convexHull(feet, count, hull);
    if (size < 3) {
        deepestSupportPoint(feet, count, nearest);
        return margin <= 0 && stabilityMargin(feet, count, nearest) >= margin;
    }
    // The closest point of the polygon shrunk by margin: point itself, its projection onto one
    // shrunk edge, or a corner where two meet. Try each and keep the nearest that is inside.
    const Edges edges(hull, size);
    constexpr Scalar SLACK = Scalar(1e-6);
    auto inside = [&](const Vec2& x) { return edges.depth(x) >= margin - SLACK; };
    if (inside(point)) {
        nearest = point;
        return true;
    }
    Scalar bestDistance = kInfinity;
    auto consider = [&](const Vec2& x) {
        const Scalar distance = (x - point).norm();
        if (distance < bestDistance && inside(x)) {
            bestDistance = distance;
            nearest = x;
        }
    };
    for (int i = 0; i < size; ++i) {
        const Vec2& n = edges.normal[i];
        consider(point - (n.dot(point) - edges.offset[i] - margin) * n);
        for (int j = i + 1; j < size; ++j) {
            const Vec2& m = edges.normal[j];
            const Scalar det = n.x() * m.y() - n.y() * m.x();
            if (std::abs(det) < Scalar(1e-9)) {
                continue;
            }
            const Scalar c1 = edges.offset[i] + margin;
            const Scalar c2 = edges.offset[j] + margin;
            consider(Vec2((c1 * m.y() - n.y() * c2) / det, (n.x() * c2 - c1 * m.x()) / det));
        }
    }
    if (bestDistance < kInfinity) {
        return true;
    }
    deepestSupportPoint(feet, count, nearest);
    return false;
}

int appendContactPoints(const Vec2& foot, Scalar yaw, const Vec2& soleMin, const Vec2& soleMax,
                        Vec2* points, int count) {
    count = std::clamp(count, 0, kMaxContactPoints);
    if (soleMin == soleMax) {
        if (count < kMaxContactPoints) {
            points[count++] = foot;
        }
        return count;
    }
    const Rotation2D turn(yaw);
    const Vec2 corners[4] = {soleMin, Vec2(soleMax.x(), soleMin.y()), soleMax, Vec2(soleMin.x(), soleMax.y())};
    for (const Vec2& corner : corners) {
        if (count < kMaxContactPoints) {
            points[count++] = foot + turn * corner;
        }
    }
    return count;
}

}  // namespace gait_core
