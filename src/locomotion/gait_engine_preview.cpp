// GaitEngine: preview control, the body's sway planned over the coming phases. One class over several
// files; see gait_engine.cpp.

#include "gait_engine_internal.hpp"

namespace gait_core {

using namespace engine_internal;

namespace {

constexpr int MAX_SEGMENTS = 16;   // phases a preview can reach into

}  // namespace

/**
 * @brief Where a foot will be (body frame, x and y) at a time in the preview
 * @param[in] legNum The leg
 * @param[in] segment The phase that time falls in: 0 the current one, 1 the next, ...
 * @param[in] segmentEnd When each phase ends (s from now)
 * @param[in] t The time (s from now)
 * @note A foot down stays put on the ground, so in the body frame it moves against the commanded
 *       motion. A foot that swings in an earlier phase of the preview lands where the gait would aim
 *       it (its swing target for the command), then does the same from there.
*/
Vec2 GaitEngine::predictedFoot(int legNum, int segment, const std::array<Scalar, MAX_SEGMENTS>& segmentEnd, Scalar t) const {
    const GaitSchedule& schedule = trajectoryGen.gaitState.config;
    const size_t phases = schedule.size();
    int swung = -1;   // the last phase of the preview before this one in which the leg swings
    for (int i = segment - 1; i >= 0; --i) {
        if (schedule.swingsIn(legNum, (phase + static_cast<size_t>(i)) % phases)) {
            swung = i;
            break;
        }
    }
    Vec2 foot;
    Scalar since;
    if (swung < 0) {
        foot = trajectoryGen.legToBody(legNum, current_leg_positions[legNum]).head<2>();
        since = t;
    } else {
        const PlannedMotion& m = planned_motion;
        const Vec3 landing = trajectoryGen.swingTarget(legNum,
            trajectoryGen.footDisplacement(m.forward, m.left, m.yaw, legNum, m.stanceTime / 2));
        foot = trajectoryGen.legToBody(legNum, landing).head<2>();
        since = t - segmentEnd[static_cast<size_t>(swung)];
    }
    // The body moves on: the planted foot turns back and slides back under it
    const Vec2 turned = Rotation2D(-planned_motion.yaw * since) * foot;
    return turned - Vec2(planned_motion.forward, planned_motion.left) * since;
}

/**
 * @brief Preview control, one tick: plan the centre of mass over the coming phases and sway to it
 * @param[in] dt The time since the last tick (s), also the preview's tick
 * @param[in] idle No velocity command: standing, the plan holds the centre of mass over the feet
 * @param[in] staying The feet that stay down this phase (body frame), for the reference while held
 * @param[in] stayingCount How many
 * @return The sway it's heading for (m, body frame), which it has also taken this tick
 * @note The ZMP reference runs through each coming phase from a point inside both its feet and the
 *       last phase's (where it starts) to one inside both its feet and the next's (where it ends),
 *       each the nearest such point to the centre of mass at least sway_margin inside them. A ZMP
 *       can't jump, and a crawl swaps one triangle of feet for the next in an instant, so the
 *       reference doesn't either: at each swap it's somewhere both triangles can push from. The stance
 *       feet slide back as the body walks, so each point is worked out where the feet will be then.
*/
Vec2 GaitEngine::previewSway(Scalar dt, bool idle, const ContactPoints& staying, int stayingCount) {
    const Vec2 centre = com.head<2>();
    if (!(dt > 0)) {
        return sway;
    }
    // The pendulum as it stands now; a new height or tick asks for new gains
    const Scalar height = balance_controller.state().height > 0
        ? balance_controller.state().height
        : std::max(com.z() - config().currentHeight, BalanceController::MIN_HEIGHT);
    const Scalar omega = std::sqrt(BalanceController::GRAVITY / height);
    const int steps = std::clamp(static_cast<int>(std::lround(p.preview_time / dt)), 1, PreviewController::MAX_STEPS);
    PreviewController& planner = preview_controller;
    if (!planner.configured() || planner.steps() != steps || std::abs(planner.dt() - dt) > S(0.1) * dt ||
        std::abs(planner.omega() - omega) > S(0.03) * omega) {
        if (!planner.configure(dt, omega, steps)) {
            easeSway(Vec2::Zero(), dt);
            return Vec2::Zero();
        }
    }
    // Restart from where the body is if the plan missed a tick (tilt mode, lowering or raising the
    // body, a gait that doesn't sway): what it planned then is stale. Not every tick of sitting or
    // standing up, whose steps in and out it plans too.
    if (preview_started && toSeconds(now_ - preview_last_tick) > 1.5 * static_cast<double>(dt)) {
        preview_started = false;
    }
    preview_last_tick = now_;
    if (!preview_started) {
        planner.reset(centre + sway);
        preview_started = true;
        preview_has_phase = false;
        preview_leading = false;
    }

    const Scalar margin = std::max(p.sway_margin, p.min_stability_margin);
    const GaitSchedule& schedule = trajectoryGen.gaitState.config;
    const size_t phases = schedule.size();
    // At a phase's start, every foot still down, with the body not yet over the feet that will stay
    // down (two consecutive triangles of a crawl may share too little to hand over between): those
    // feet are where to go, and the sway hold waits until the body is there. Not for a gait that
    // can't stand over its feet (preview_dynamic_gaits): it has to be on its way to the next ones
    // when the legs lift, so the plan of the coming phases runs on, its clock held at the phase's
    // start, while the hold waits for its ZMP.
    if (!idle && phase_progress == 0 && gaitSways(config().currentGait) && !overStayingFeet(staying, stayingCount)) {
        Vec2 target = centre;
        nearestStablePoint(staying.data(), stayingCount, centre, margin, target);
        // Ramped there from where the body is rather than jumped to: a ZMP asked to jump swings the
        // other way first (the pendulum has to be tipped towards where it's going)
        constexpr Scalar RAMP = S(0.3);   // s
        const Vec2 here = planner.position();
        for (int j = 0; j < steps; ++j) {
            const Scalar along = std::min(S(1.0), static_cast<Scalar>(j + 1) * dt / RAMP);
            preview_reference[static_cast<size_t>(j)] = here + (target - here) * along;
        }
        preview_has_phase = false;   // the phase's plan starts from here once the legs can lift
        planner.step(preview_reference.data());
        followPlan(centre, false);
        return reachableSway(target - centre);
    }
    if (idle && phase_progress == 0) {
        // Standing: keep the centre of mass where it is, over every foot down
        ContactPoints feet;
        int count = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            const Vec2 foot = trajectoryGen.legToBody(legNum, current_leg_positions[legNum]).head<2>();
            count = addContactPoints(legNum, foot, feet, count);
        }
        Vec2 target = centre;
        nearestStablePoint(feet.data(), count, centre, margin, target);
        std::fill(preview_reference.begin(), preview_reference.begin() + steps, target);
    } else {
        // The coming phases: the rest of this one, then whole ones
        const Scalar T = std::max(current_phase_duration, S(0.01));
        // A gait that can't stand over its feet, held at a phase's start with the ZMP not yet over the
        // feet that stay down (setting off from standing, or knocked off its plan), first gets a lead-in:
        // the ZMP eased from where it was into those feet over a phase, then the coming phases after
        // it. Asked to be there at once instead, it would swing the other way and then overshoot, and
        // the body would arrive with nothing of the motion the steps after need.
        // Once begun it runs to its end, the hold waiting (see overStayingFeet).
        if (!gaitSways(config().currentGait) && !idle && phase_progress == 0) {
            if (preview_leading) {
                preview_lead = std::max(S(0.0), preview_lead - dt);
            } else if (!overStayingFeet(staying, stayingCount)) {
                preview_leading = true;
                preview_lead = T;
                preview_lead_from = planner.zmp();
            }
        } else {
            preview_leading = false;
        }
        const Scalar lead = preview_leading ? preview_lead : S(0.0);
        std::array<Scalar, MAX_SEGMENTS> segmentEnd{};
        for (int k = 0; k < MAX_SEGMENTS; ++k) {
            segmentEnd[static_cast<size_t>(k)] = lead + (1 - phase_progress) * T + static_cast<Scalar>(k) * T;
        }
        // The feet down in a phase of the preview, where they'll be at time t
        auto feetIn = [&](int segment, Scalar t, ContactPoints& feet) {
            int count = 0;
            const size_t phaseThen = (phase + static_cast<size_t>(segment)) % phases;
            for (int legNum = 0; legNum < numLegs(); ++legNum) {
                if (!schedule.swingsIn(legNum, phaseThen)) {
                    count = addContactPoints(legNum, predictedFoot(legNum, segment, segmentEnd, t), feet, count);
                }
            }
            return count;
        };
        // Near the centre of mass, inside both one phase's feet and the next's at the moment one gives
        // way to the other: alternating projections onto the two converge into their overlap. If they
        // hardly overlap, it settles inside the earlier one, the feet the robot is standing on until
        // then; the hold at the next phase's start waits, every foot down, while the body moves over.
        auto handover = [&](int segment) {
            const Scalar t = segmentEnd[static_cast<size_t>(segment)];
            ContactPoints a, b;
            const int na = feetIn(segment, t, a);
            const int nb = feetIn(segment + 1, t, b);
            // (nearestStablePoint's answer can't be the point it's given: it writes it while reading)
            Vec2 point = centre, inA, inB;
            for (int i = 0; i < 6; ++i) {
                nearestStablePoint(b.data(), nb, point, margin, inB);
                nearestStablePoint(a.data(), na, inB, margin, inA);
                point = inA;
            }
            return point;
        };
        // This phase runs from where the last one handed over (or, just started, from the point nearest
        // the centre of mass inside its feet) to where it hands over to the next. Its start is in the
        // past by however far through it the gait is.
        if (!preview_has_phase || phase != preview_phase) {
            if (preview_has_phase) {
                preview_from = preview_handover;
            } else {
                // Just started, or let go by the hold: from where the body is, inside these feet
                ContactPoints now;
                const int nowCount = feetIn(0, 0, now);
                nearestStablePoint(now.data(), nowCount, centre + sway, margin, preview_from);
            }
            preview_phase = phase;
            preview_has_phase = true;
        }
        int segment = 0;
        Scalar start = lead - phase_progress * T;
        Vec2 from = preview_from;
        Vec2 to = handover(0);
        preview_handover = to;
        for (int j = 0; j < steps; ++j) {
            const Scalar t = static_cast<Scalar>(j + 1) * dt;
            if (t < lead) {
                // The lead-in, T long, of which T - lead has gone
                const Scalar along = std::clamp((T - lead + t) / T, S(0.0), S(1.0));
                preview_reference[static_cast<size_t>(j)] = preview_lead_from + (preview_from - preview_lead_from) * along;
                continue;
            }
            while (segment + 1 < MAX_SEGMENTS && t >= segmentEnd[static_cast<size_t>(segment)]) {
                start = segmentEnd[static_cast<size_t>(segment)];
                ++segment;
                from = to;
                to = handover(segment);
            }
            const Scalar length = segmentEnd[static_cast<size_t>(segment)] - start;
            const Scalar along = length > 0 ? std::clamp((t - start) / length, S(0.0), S(1.0)) : S(0.0);
            preview_reference[static_cast<size_t>(j)] = from + (to - from) * along;
        }
    }

    planner.step(preview_reference.data());
    followPlan(centre, gaitSways(config().currentGait));
    // Leading in, the hold waits for the body to get over the feet, however long that takes
    return preview_leading ? reachableSway(preview_from - centre) : sway;
}

/**
 * @brief Sway the body to where the planner has put the centre of mass, as far as the legs allow
 * @param[in] centre The centre of mass in the body (body frame, x and y)
 * @param[in] capAtStride Never further than a stride, which is all the feet of a gait that sways
 *            statically can keep up with. A gait only preview control plans (preview_dynamic_gaits)
 *            has to get its weight from foot to foot, however far apart: only the legs limit it.
 * @note Out of reach, the planner is moved to where the body really is, so it plans from there
*/
void GaitEngine::followPlan(const Vec2& centre, bool capAtStride) {
    PreviewController& planner = preview_controller;
    Vec2 next = planner.position() - centre;
    if (capAtStride && next.norm() > config().maxStride) {
        next *= config().maxStride / next.norm();
    }
    const Vec2 limited = reachableSway(next);
    if (limited != next) {
        planner.reset(centre + limited, planner.velocity(), planner.acceleration());
    }
    sway = limited;
}

}  // namespace gait_core
