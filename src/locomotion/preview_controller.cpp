#include "gait_core/locomotion/preview_controller.hpp"

#include <algorithm>
#include <cmath>

namespace gait_core {

bool PreviewController::configure(Scalar dt, Scalar omega, int steps, double errorWeight, double jerkWeight) {
    if (!(dt > 0) || !(omega > 0) || !(jerkWeight > 0)) {
        return false;
    }
    steps = std::clamp(steps, 1, MAX_STEPS);
    using Mat4 = Eigen::Matrix4d;
    using V4 = Eigen::Vector4d;
    const double T = static_cast<double>(dt);
    const double w2 = static_cast<double>(omega) * static_cast<double>(omega);

    // The cart-table model, x = (c, c dot, c double dot), u = jerk, zmp = C x
    Eigen::Matrix3d A;
    A << 1, T, T * T / 2,
         0, 1, T,
         0, 0, 1;
    const Eigen::Vector3d B(T * T * T / 6, T * T / 2, T);
    const Eigen::RowVector3d C(1, 0, -1 / w2);

    // Augmented with the ZMP's tracking error, whose sum the controller also drives to zero:
    // (e, dx), e[k+1] = e[k] + C A dx + C B du
    Mat4 Aa = Mat4::Zero();
    Aa(0, 0) = 1;
    Aa.block<1, 3>(0, 1) = C * A;
    Aa.block<3, 3>(1, 1) = A;
    V4 Ba;
    Ba(0) = (C * B)(0);
    Ba.tail<3>() = B;
    Mat4 Q = Mat4::Zero();
    Q(0, 0) = errorWeight;
    const double R = jerkWeight;

    // P = Q + Aa' P Aa - Aa' P Ba (R + Ba' P Ba)^-1 Ba' P Aa, iterated to its fixed point
    Mat4 P = Q;
    bool converged = false;
    for (int i = 0; i < 20000; ++i) {
        const double s = R + Ba.dot(P * Ba);
        const Mat4 next = Q + Aa.transpose() * P * Aa - (Aa.transpose() * P * Ba) * (Ba.transpose() * P * Aa) / s;
        const double change = (next - P).cwiseAbs().maxCoeff();
        P = next;
        if (change < 1e-12 * std::max(1.0, P.cwiseAbs().maxCoeff())) {
            converged = true;
            break;
        }
    }
    if (!converged || !P.allFinite()) {
        return false;
    }
    const double s = R + Ba.dot(P * Ba);
    const Eigen::RowVector4d K = (Ba.transpose() * P * Aa) / s;
    gi_ = static_cast<Scalar>(K(0));
    for (int i = 0; i < 3; ++i) {
        gx_[static_cast<size_t>(i)] = static_cast<Scalar>(K(i + 1));
    }
    // The preview gains: Gp[1] = -Gi, Gp[j] = (R + Ba' P Ba)^-1 Ba' X[j - 1], X[1] = -Ac' P I,
    // X[j] = Ac' X[j - 1], with Ac = Aa - Ba K the closed loop
    const Mat4 Ac = Aa - Ba * K;
    const V4 I(1, 0, 0, 0);
    V4 X = -Ac.transpose() * P * I;
    gp_[0] = -gi_;
    for (int j = 1; j < steps; ++j) {
        gp_[static_cast<size_t>(j)] = static_cast<Scalar>(Ba.dot(X) / s);
        X = Ac.transpose() * X;
    }
    dt_ = dt;
    omega_ = omega;
    steps_ = steps;
    configured_ = true;
    return true;
}

void PreviewController::reset(const Vec2& position, const Vec2& velocity, const Vec2& acceleration) {
    // Worked relative to here: with a finite preview the gains aren't quite translation invariant
    // (sum Gp isn't exactly -Gx[0]), so far from the origin the integral would have a start to make up
    origin_ = position;
    position_ = position;
    velocity_ = velocity;
    acceleration_ = acceleration;
    errorSum_ = Vec2::Zero();
    hasCurrentReference_ = false;
}

Vec2 PreviewController::zmp() const {
    return omega_ > 0 ? Vec2(position_ - acceleration_ / (omega_ * omega_)) : position_;
}

void PreviewController::step(const Vec2* reference) {
    if (!configured_ || reference == nullptr) {
        return;
    }
    // The error is against where the ZMP should be now: the first of last tick's preview (on the
    // first tick, this one's). Its sum is the LQ problem's integral action.
    errorSum_ += zmp() - (hasCurrentReference_ ? currentReference_ : reference[0]);
    currentReference_ = reference[0];
    hasCurrentReference_ = true;
    Vec2 preview = Vec2::Zero();
    for (int j = 0; j < steps_; ++j) {
        preview += gp_[static_cast<size_t>(j)] * (reference[j] - origin_);
    }
    const Vec2 jerk = -gi_ * errorSum_ - gx_[0] * (position_ - origin_) - gx_[1] * velocity_ - gx_[2] * acceleration_ - preview;
    const Scalar T = dt_;
    position_ += velocity_ * T + acceleration_ * (T * T / 2) + jerk * (T * T * T / 6);
    velocity_ += acceleration_ * T + jerk * (T * T / 2);
    acceleration_ += jerk * T;
}

}  // namespace gait_core
