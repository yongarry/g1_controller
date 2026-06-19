// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-environment C++ port of the deploy-relevant subset of
//   tocabi_3d_footstep/mdp/commands.py :: OnlineFootCommand
// for the G1-2d footstep policy (G12DFootEnvCfg).
//
// Differences vs. the training command term (all intentional for deployment):
//   * num_envs == 1 (scalar / Eigen instead of batched tensors).
//   * Full-body link states (CoM, feet world poses) are obtained from the
//     Kinematics helper (Pinocchio FK) instead of the simulator.
//   * The "global" frame is re-anchored at the pelvis every control tick. This
//     is valid because every quantity carried across ticks (preview state,
//     foot command buffer, walking tick) lives in a relative (stance-foot or
//     command) frame; only quantities measured within a single tick are mixed.
//   * Foot-step commands come from the operator (joystick) and are clamped to
//     the trained ranges, instead of being randomly resampled.
//   * Reference-trajectory velocities are not computed: the differential IK
//     (computeDiffIK) consumes pose targets only.

#pragma once

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "isaaclab/utils/math_utils.h"
#include "isaaclab/utils/kinematics.h"
#include "isaaclab/envs/mdp/commands/vrp_generator.h"
#include "isaaclab/envs/mdp/commands/preview_controller.h"
#include "isaaclab/envs/mdp/commands/foot_command_source.h" // FootCommandInput

namespace isaaclab
{

struct FrameState
{
    math::Vec3 pos = math::Vec3::Zero();
    math::Quat quat = math::Quat::Identity();
    math::Vec3 lin = math::Vec3::Zero();
    math::Vec3 ang = math::Vec3::Zero();
};

// FootCommandInput is defined in foot_command_source.h (shared with the
// command-source abstraction).

class FootstepCommand
{
public:
    struct Config
    {
        int future_foot_step_num = 2;
        float step_dt = 0.02f;
        float vrp_height = 0.6607f;
        float pelv_com_offset = 0.0761f;
        float vrp_horizon_length = 5.0f;
        float preview_horizon_length = 2.0f;
        float swing_up_timing = 0.4f;
        float swing_down_timing = 0.5f;
        int ik_iters = 50;
        float ik_lambda = 0.05f;
        float ik_pos_tol = 1e-4f;
        // command ranges (used to clamp joystick-driven input)
        float foot_pos_x_min = -0.2f, foot_pos_x_max = 0.2f;
        float foot_pos_y_min = 0.2f, foot_pos_y_max = 0.4f;
        float foot_rot_y_min = -0.2f, foot_rot_y_max = 0.2f;
        float foot_ssp_min = 0.5f, foot_ssp_max = 1.0f;
        float foot_dsp_min = 0.0f, foot_dsp_max = 0.3f;
        float foot_height_min = 0.05f, foot_height_max = 0.10f;
        int start_phase_indicator = 0; // 0: right swing first, 1: left swing first
    };

    FootstepCommand(const Config& cfg, std::shared_ptr<Kinematics> kin)
    : cfg_(cfg), kin_(std::move(kin)),
      vrp_(cfg.vrp_horizon_length, cfg.step_dt, cfg.future_foot_step_num, cfg.vrp_height),
      preview_(cfg.preview_horizon_length, cfg.step_dt, cfg.vrp_height)
    {
        const int LA = cfg_.future_foot_step_num;
        foot_command_.assign(LA, {0,0,0,0,0,0,0,0,0});
        com_z_command_.assign(LA + 1, 0.0f);
        phase_indicator_.assign(LA, 0);
        target_joint_pos_.setZero(12);
        command_vec_.fill(0.0f);
    }

    // The operator updates this each tick; sampled at step boundaries.
    void set_input(const FootCommandInput& in) { input_ = in; }

    // --- public API mirroring CommandTerm ---
    void reset()
    {
        command_counter_ = 0;
        resample_command_();
        update_link_states_();
        generate_vrp_ref_trajectory_();
        preview_.reset_error_integral();
        Eigen::Matrix3f s = Eigen::Matrix3f::Zero();
        s.row(0) = com_pos_stance_.transpose();
        preview_.set_state(s);
        compute_command_vec_();
    }

    void compute()
    {
        const float dt = cfg_.step_dt;
        update_link_states_();
        generate_ref_trajectory_();
        time_left_ -= dt;
        walking_tick_ += 1;
        step_completed_ = (time_left_ <= 0.0f);
        if (step_completed_)
        {
            update_command_();
            update_link_states_();
            generate_vrp_ref_trajectory_();
            generate_ref_trajectory_();
        }
        compute_command_vec_();
    }

    // 23-dim command: [ik_target(12), phase_cos, phase_sin, foot_command0(9)]
    const std::array<float, 23>& command() const { return command_vec_; }

    // True for the tick on which a footstep just completed (a new step was
    // appended). Used to advance an external FootCommandSource (e.g. CSV).
    bool step_completed() const { return step_completed_; }

private:
    Kinematics::Side stance_side_() const
    {
        return (phase_indicator_[0] == 0) ? Kinematics::LEFT : Kinematics::RIGHT;
    }
    Kinematics::Side swing_side_() const
    {
        return (phase_indicator_[0] == 0) ? Kinematics::RIGHT : Kinematics::LEFT;
    }

    static float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

    // Build the 9-dim foot command for `step` from the operator input + phase.
    std::array<float, 9> build_foot_command_(int step)
    {
        std::array<float, 9> fc{};
        fc[0] = clampf(input_.step_x, cfg_.foot_pos_x_min, cfg_.foot_pos_x_max);
        fc[1] = clampf(input_.step_y, cfg_.foot_pos_y_min, cfg_.foot_pos_y_max); // positive base
        fc[2] = input_.step_z;
        fc[3] = 0.0f;
        fc[4] = 0.0f;
        fc[5] = clampf(input_.step_yaw, cfg_.foot_rot_y_min, cfg_.foot_rot_y_max);
        fc[6] = clampf(input_.ssp_t, cfg_.foot_ssp_min, cfg_.foot_ssp_max);
        fc[7] = clampf(input_.dsp_t, cfg_.foot_dsp_min, cfg_.foot_dsp_max);
        fc[8] = clampf(input_.height, cfg_.foot_height_min, cfg_.foot_height_max);
        // right-swing steps (phase indicator 0) place the foot on the -y side
        if (phase_indicator_[step] == 0) fc[1] *= -1.0f;
        return fc;
    }

    void resample_command_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        phase_indicator_[0] = cfg_.start_phase_indicator;
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i + 1] = 1 - phase_indicator_[i];
        for (int s = 0; s < LA; ++s) foot_command_[s] = build_foot_command_(s);
        for (int i = 0; i < LA; ++i) com_z_command_[i] = input_.com_z;
        com_z_command_[LA] = input_.com_z;
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
    }

    void update_command_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        for (int s = 0; s < LA - 1; ++s) foot_command_[s] = foot_command_[s + 1];
        for (int i = 0; i < LA - 1; ++i) com_z_command_[i] = com_z_command_[i + 1];
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i] = phase_indicator_[i + 1];
        phase_indicator_[LA - 1] = 1 - phase_indicator_[LA - 2];
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
        foot_command_[LA - 1] = build_foot_command_(LA - 1);
        com_z_command_[LA] = input_.com_z;
    }

    void update_link_states_()
    {
        const math::Quat R = robot_quat_w_; // pelvis orientation from IMU
        // CoM in global (pelvis anchored at origin)
        com_pos_global_ = math::quat_apply(R, kin_->com_pos());
        com_vel_global_ = math::quat_apply(R, kin_->com_vel());

        auto fill = [&](Kinematics::Side s, FrameState& fs) {
            fs.pos = math::quat_apply(R, kin_->foot_pos(s));
            fs.quat = math::quat_mul(R, kin_->foot_quat(s));
            fs.lin = math::quat_apply(R, kin_->foot_lin_vel(s));
            fs.ang = math::quat_apply(R, kin_->foot_ang_vel(s));
        };
        fill(stance_side_(), stance_foot_global_);
        fill(swing_side_(), swing_foot_global_);

        const math::Quat stance_yaw = math::yaw_quat(stance_foot_global_.quat);
        math::subtract_frame_transforms(stance_foot_global_.pos, stance_yaw,
                                        swing_foot_global_.pos, swing_foot_global_.quat,
                                        swing_foot_stance_pos_, swing_foot_stance_quat_);
        com_pos_stance_ = math::subtract_frame_transforms_pos(stance_foot_global_.pos, stance_yaw, com_pos_global_);
    }

    void generate_vrp_ref_trajectory_()
    {
        const int LA = cfg_.future_foot_step_num;
        // init vrp state (zmp x,y; com z) in stance frame
        math::Vec3 vrp_state_stance;
        vrp_state_stance[0] = swing_foot_stance_pos_[0] / 2.0f;
        vrp_state_stance[1] = swing_foot_stance_pos_[1] / 2.0f;
        vrp_state_stance[2] = com_pos_stance_[2];
        const float swing_yaw = math::wrap_to_pi(math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);

        vrp_.generate_vrp_online(foot_command_, vrp_state_stance, swing_yaw / 2.0f,
                                 swing_foot_stance_pos_, com_z_command_);
        for (int s = 0; s < LA; ++s) foot_command_[s][2] = vrp_.step_z[s];

        swing_foot_start_stance_pos_ = swing_foot_stance_pos_;
        swing_foot_start_stance_quat_ = swing_foot_stance_quat_;
        swing_foot_end_stance_pos_ = math::Vec3(foot_command_[0][0], foot_command_[0][1], foot_command_[0][2]);
        swing_foot_end_stance_quat_ = math::quat_from_euler_xyz(foot_command_[0][3], foot_command_[0][4], foot_command_[0][5]);
        stance_foot_start_global_ = stance_foot_global_;

        // update preview state in the (updated) stance foot frame
        const math::Quat stance_yaw = math::yaw_quat(stance_foot_global_.quat);
        Eigen::Matrix3f s;
        s.row(0) = math::quat_apply_inverse(stance_yaw, target_com_global_pos_ - stance_foot_global_.pos).transpose();
        s.row(1) = math::quat_apply_inverse(stance_yaw, target_com_global_vel_ - stance_foot_global_.lin).transpose();
        s.row(2) = math::quat_apply_inverse(stance_yaw, target_com_global_acc_).transpose();
        preview_.set_state(s);
    }

    void generate_ref_trajectory_()
    {
        const int previewNL = preview_.NL();
        // gather vrp reference window [walking_tick, walking_tick + previewNL)
        std::vector<math::Vec3> vrp_ref(previewNL);
        for (int l = 0; l < previewNL; ++l)
        {
            int idx = static_cast<int>(walking_tick_) + l;
            if (idx >= vrp_.NL()) idx = vrp_.NL() - 1;
            vrp_ref[l] = vrp_.vrp_ref_firststance[idx];
        }
        Eigen::Matrix3f next = preview_.compute_target_state(vrp_ref);
        preview_.update_state(next);

        const math::Vec3 com_stance_pos = next.row(0).transpose();
        const math::Vec3 com_stance_vel = next.row(1).transpose();
        const math::Vec3 com_stance_acc = next.row(2).transpose();

        const math::Quat stance_yaw = math::yaw_quat(stance_foot_global_.quat);
        target_com_global_pos_ = math::combine_frame_transforms_pos(stance_foot_global_.pos, stance_yaw, com_stance_pos);
        target_com_global_vel_ = math::quat_apply(stance_yaw, com_stance_vel);
        target_com_global_acc_ = math::quat_apply(stance_yaw, com_stance_acc);

        int yaw_idx = static_cast<int>(walking_tick_) + 1;
        if (yaw_idx >= vrp_.NL()) yaw_idx = vrp_.NL() - 1;
        const float com_yaw_ref = vrp_.CoM_yaw_ref_firststance[yaw_idx];

        // target pelvis in stance frame
        math::Vec3 pelv_pos_stance = com_stance_pos;
        pelv_pos_stance[2] += cfg_.pelv_com_offset;
        math::Quat pelv_quat_stance = math::quat_from_euler_xyz(0.0f, 0.0f, com_yaw_ref);

        // target swing foot in stance frame (pose only)
        generate_feet_ref_trajectory_();

        // stance foot reference pose in stance frame = origin / identity
        const math::Vec3 stance_pose_pos = math::Vec3::Zero();
        const math::Quat stance_pose_quat = math::Quat::Identity();

        // assign left / right targets in stance frame
        math::Vec3 l_pos_stance, r_pos_stance;
        math::Quat l_quat_stance, r_quat_stance;
        if (phase_indicator_[0] == 0) { // left is stance
            l_pos_stance = stance_pose_pos;  l_quat_stance = stance_pose_quat;
            r_pos_stance = target_swing_stance_pos_; r_quat_stance = target_swing_stance_quat_;
        } else { // right is stance
            r_pos_stance = stance_pose_pos;  r_quat_stance = stance_pose_quat;
            l_pos_stance = target_swing_stance_pos_; l_quat_stance = target_swing_stance_quat_;
        }

        // express foot targets in the pelvis (base) frame at desired pelvis pose
        const math::Quat pelv_quat_inv = pelv_quat_stance.conjugate();
        const math::Vec3 l_pos_b = math::quat_apply(pelv_quat_inv, l_pos_stance - pelv_pos_stance);
        const math::Quat l_quat_b = math::quat_mul(pelv_quat_inv, l_quat_stance);
        const math::Vec3 r_pos_b = math::quat_apply(pelv_quat_inv, r_pos_stance - pelv_pos_stance);
        const math::Quat r_quat_b = math::quat_mul(pelv_quat_inv, r_quat_stance);

        Eigen::VectorXf ql = kin_->diff_ik_leg(Kinematics::LEFT, l_pos_b, l_quat_b,
                                               kin_->leg_q(Kinematics::LEFT),
                                               cfg_.ik_iters, cfg_.ik_lambda, cfg_.ik_pos_tol);
        Eigen::VectorXf qr = kin_->diff_ik_leg(Kinematics::RIGHT, r_pos_b, r_quat_b,
                                               kin_->leg_q(Kinematics::RIGHT),
                                               cfg_.ik_iters, cfg_.ik_lambda, cfg_.ik_pos_tol);
        target_joint_pos_.head(6) = ql;
        target_joint_pos_.tail(6) = qr;
    }

    void generate_feet_ref_trajectory_()
    {
        const float dt = cfg_.step_dt;
        const float ssp_t = foot_command_[0][6];
        const float dsp_t = foot_command_[0][7];
        const float height = foot_command_[0][8];
        const long tick = walking_tick_;
        const int dsp1_end = static_cast<int>(dsp_t / dt);
        const int ssp_end = static_cast<int>((ssp_t + dsp_t) / dt);

        math::Vec3 pos;
        math::Quat quat;

        if (tick < dsp1_end)
        {
            pos = swing_foot_start_stance_pos_;
            quat = swing_foot_start_stance_quat_;
        }
        else if (tick < ssp_end)
        {
            const float ct = (tick - dsp1_end) * dt;
            // x, y
            pos[0] = math::cubic(swing_foot_start_stance_pos_[0], 0.f, swing_foot_end_stance_pos_[0], 0.f, ssp_t, ct);
            pos[1] = math::cubic(swing_foot_start_stance_pos_[1], 0.f, swing_foot_end_stance_pos_[1], 0.f, ssp_t, ct);
            // orientation (interp euler r,p,y)
            math::Vec3 e0 = math::euler_xyz_from_quat(swing_foot_start_stance_quat_);
            math::Vec3 e1 = math::euler_xyz_from_quat(swing_foot_end_stance_quat_);
            math::Vec3 et;
            for (int i = 0; i < 3; ++i)
                et[i] = math::cubic(math::wrap_to_pi(e0[i]), 0.f, math::wrap_to_pi(e1[i]), 0.f, ssp_t, ct);
            quat = math::quat_from_euler_xyz(et[0], et[1], et[2]);
            // z: lift up / maintain / down
            const float lift = std::max(swing_foot_start_stance_pos_[2], swing_foot_end_stance_pos_[2]) + height;
            const int up_end = static_cast<int>((dsp_t + ssp_t * cfg_.swing_up_timing) / dt);
            const int down_start = static_cast<int>((dsp_t + ssp_t * cfg_.swing_down_timing) / dt);
            if (tick < up_end)
            {
                const float t = (tick - dsp1_end) * dt;
                pos[2] = math::cubic(swing_foot_start_stance_pos_[2], 0.f, lift, 0.f, ssp_t * cfg_.swing_up_timing, t);
            }
            else if (tick < down_start)
            {
                pos[2] = lift;
            }
            else
            {
                const float t = (tick - down_start) * dt;
                pos[2] = math::cubic(lift, 0.f, swing_foot_end_stance_pos_[2], 0.f,
                                     ssp_t * (1.0f - cfg_.swing_down_timing), t);
            }
        }
        else
        {
            pos = swing_foot_end_stance_pos_;
            quat = swing_foot_end_stance_quat_;
        }
        target_swing_stance_pos_ = pos;
        target_swing_stance_quat_ = quat;
    }

    void compute_command_vec_()
    {
        const float total_phase = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
        const float ph = (total_phase > 1e-6f) ? (walking_tick_ * cfg_.step_dt / total_phase) : 0.0f;
        const float c = std::cos(2.0f * static_cast<float>(M_PI) * ph);
        const float s = std::sin(2.0f * static_cast<float>(M_PI) * ph);
        for (int i = 0; i < 12; ++i) command_vec_[i] = target_joint_pos_(i);
        command_vec_[12] = c;
        command_vec_[13] = s;
        for (int i = 0; i < 9; ++i) command_vec_[14 + i] = foot_command_[0][i];
    }

public:
    // pelvis orientation (world) is supplied by State_Footstep each tick.
    math::Quat robot_quat_w_ = math::Quat::Identity();

private:
    Config cfg_;
    std::shared_ptr<Kinematics> kin_;
    VrpGenerator vrp_;
    PreviewController preview_;
    FootCommandInput input_;

    long command_counter_ = 0;
    long walking_tick_ = 0;
    float time_left_ = 0.0f;
    bool step_completed_ = false;

    std::vector<std::array<float, 9>> foot_command_;
    std::vector<float> com_z_command_;
    std::vector<int> phase_indicator_;

    // measured state (per tick)
    math::Vec3 com_pos_global_ = math::Vec3::Zero();
    math::Vec3 com_vel_global_ = math::Vec3::Zero();
    math::Vec3 com_pos_stance_ = math::Vec3::Zero();
    FrameState stance_foot_global_, swing_foot_global_;
    math::Vec3 swing_foot_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_stance_quat_ = math::Quat::Identity();

    // step-boundary state
    math::Vec3 swing_foot_start_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_start_stance_quat_ = math::Quat::Identity();
    math::Vec3 swing_foot_end_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_end_stance_quat_ = math::Quat::Identity();
    FrameState stance_foot_start_global_;

    // preview/com targets (global)
    math::Vec3 target_com_global_pos_ = math::Vec3::Zero();
    math::Vec3 target_com_global_vel_ = math::Vec3::Zero();
    math::Vec3 target_com_global_acc_ = math::Vec3::Zero();

    // feet ref (stance frame)
    math::Vec3 target_swing_stance_pos_ = math::Vec3::Zero();
    math::Quat target_swing_stance_quat_ = math::Quat::Identity();

    Eigen::VectorXf target_joint_pos_;
    std::array<float, 23> command_vec_;
};

} // namespace isaaclab
