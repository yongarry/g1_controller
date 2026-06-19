// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-environment C++ port of
//   tocabi_3d_footstep/mdp/preview_controller/preview_controller.py
//
// The discrete algebraic Riccati equation (DARE) for the preview controller is
// solved on-device at construction by iterating the Riccati recursion, so no
// offline gain export / scipy dependency is required. The gains depend only on
// (zc, dt, NL), which are fixed for a given policy.

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include "isaaclab/utils/math_utils.h"

namespace isaaclab
{

class PreviewController
{
public:
    PreviewController(float time_horizon, float dt, float zc)
    : dt_(dt), zc_(zc)
    {
        const int hz = static_cast<int>(1.0f / dt);
        NL_ = static_cast<int>(time_horizon * hz);
        const float g = 9.81f;

        A_ << 1.f, dt, dt * dt / 2.f,
              0.f, 1.f, dt,
              0.f, 0.f, 1.f;
        B_ << dt * dt * dt / 6.f, dt * dt / 2.f, dt;
        C_ << 1.f, 0.f, -zc / g;

        // augmented system (double precision for the Riccati solve)
        Eigen::Matrix4d A_bar = Eigen::Matrix4d::Zero();
        Eigen::Vector4d B_bar;
        Eigen::Matrix4d Q_bar = Eigen::Matrix4d::Zero();
        const double R = 1e-6;

        Eigen::Matrix3d Ad = A_.cast<double>();
        Eigen::Vector3d Bd = B_.cast<double>();
        Eigen::RowVector3d Cd = C_.cast<double>();

        // A_bar = [I_bar | F_bar],  F_bar = [C A; A]
        A_bar(0, 0) = 1.0;
        A_bar.block<1, 3>(0, 1) = Cd * Ad;
        A_bar.block<3, 3>(1, 1) = Ad;
        // B_bar = [C B; B]
        B_bar(0) = (Cd * Bd)(0, 0);
        B_bar.segment<3>(1) = Bd;
        Q_bar(0, 0) = 1.0; // Q_e

        Eigen::Matrix4d K = solve_dare(A_bar, B_bar, Q_bar, R);

        const double inv_term = 1.0 / (R + (B_bar.transpose() * K * B_bar)(0, 0));
        Eigen::RowVector4d G = inv_term * (B_bar.transpose() * K * A_bar); // 1x4
        G_i_ = static_cast<float>(G(0));
        G_x_ << static_cast<float>(G(1)), static_cast<float>(G(2)), static_cast<float>(G(3));

        Eigen::Matrix4d Ac_bar_T = (A_bar - B_bar * inv_term * B_bar.transpose() * K * A_bar).transpose();
        Eigen::RowVector4d RBT = inv_term * B_bar.transpose();
        Eigen::Vector4d I_bar(1.0, 0.0, 0.0, 0.0);

        G_d_.assign(NL_, 0.0f);
        std::vector<Eigen::Vector4d> X(NL_);
        X[0] = -Ac_bar_T * K * I_bar;
        for (int l = 1; l < NL_; ++l) X[l] = Ac_bar_T * X[l - 1];
        G_d_[0] = -G_i_;
        for (int l = 1; l < NL_; ++l) G_d_[l] = static_cast<float>((RBT * X[l - 1])(0));

        state_.setZero();
        error_integral_.setZero();
    }

    int NL() const { return NL_; }

    // state_: rows = [pos; vel; acc], cols = [x, y, z]
    void set_state(const Eigen::Matrix3f& s) { state_ = s; }
    const Eigen::Matrix3f& state() const { return state_; }
    void reset_error_integral() { error_integral_.setZero(); }
    void set_error_integral(const Eigen::Vector3f& e) { error_integral_ = e; }

    // vrp_ref: NL_ entries, each xyz. Returns the next preview state (3x3).
    Eigen::Matrix3f compute_target_state(const std::vector<math::Vec3>& vrp_ref)
    {
        // C @ state -> (1x3) per-axis cart output; vrp_ref[0] is current ref
        Eigen::RowVector3f cart = C_ * state_;
        error_integral_ += (cart.transpose() - vrp_ref[0]);

        Eigen::RowVector3f gx_state = G_x_ * state_; // (1x3)
        Eigen::Vector3f preview_sum = Eigen::Vector3f::Zero();
        for (int l = 1; l < NL_; ++l) preview_sum += G_d_[l] * vrp_ref[l];

        Eigen::RowVector3f input;
        input = (-G_i_ * error_integral_ - gx_state.transpose() - preview_sum).transpose();

        Eigen::Matrix3f next = A_ * state_ + B_ * input;
        return next;
    }

    void update_state(const Eigen::Matrix3f& next) { state_ = next; }

private:
    static Eigen::Matrix4d solve_dare(const Eigen::Matrix4d& A, const Eigen::Vector4d& B,
                                      const Eigen::Matrix4d& Q, double R,
                                      int max_iter = 100000, double tol = 1e-12)
    {
        Eigen::Matrix4d P = Q;
        for (int i = 0; i < max_iter; ++i)
        {
            const double denom = R + (B.transpose() * P * B)(0, 0);
            Eigen::Matrix4d Pn = A.transpose() * P * A
                - (A.transpose() * P * B) * (B.transpose() * P * A) / denom + Q;
            const double diff = (Pn - P).cwiseAbs().maxCoeff();
            P = Pn;
            if (diff < tol) break;
        }
        return P;
    }

    float dt_, zc_;
    int NL_;
    Eigen::Matrix3f A_;
    Eigen::Vector3f B_;
    Eigen::RowVector3f C_;

    float G_i_;
    Eigen::RowVector3f G_x_;
    std::vector<float> G_d_;

    Eigen::Matrix3f state_;
    Eigen::Vector3f error_integral_;
};

} // namespace isaaclab
