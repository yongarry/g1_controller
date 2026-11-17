// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-env port of yonglab rise/mdp/commands.py :: OnlineFootCommand (the RISE reference: VRP, preview control,
// IK) with the footsteps of JoystickFootCommand: nominal footsteps from the joystick velocity (stage 1) or picked by
// the stage-2 planner from depth (footstep_planner.h). The "global" frame is re-anchored at the pelvis every tick;
// valid because everything carried across ticks lives in a relative (stance-foot or command) frame.

#pragma once

#include <array>
#include <vector>
#include <map>
#include <limits>
#include <cmath>
#include <algorithm>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "isaaclab/utils/math_utils.h"
#include "isaaclab/utils/kinematics.h"
#include "isaaclab/envs/mdp/commands/vrp_generator.h"
#include "isaaclab/envs/mdp/commands/preview_controller.h"
#include "isaaclab/envs/mdp/commands/footstep_planner.h"

namespace isaaclab
{

// FootCommandCfg.com_generate_type: must match what the policy was trained with
// (PREV for the shipped policies; HEURI / HEURI2 are the G13DFootEnvCfg_heuri1 / _heuri2 ablations).
enum class ComGenerateType
{
    PREV,   // "prev"   : preview control on the VRP reference
    HEURI,  // "heuri"  : phase-based CoM, hold - move - hold
    HEURI2, // "heuri2" : phase-based CoM, always moving, sways toward the stance foot
};

enum class FootStateSource
{
    FK_ODOMETRY, // default: accumulate FK landing measurements (stance_world_)
    SIM_ODOM,    // MuJoCo sim: rt/odommodestate base position + FK foot offset
};

struct FrameState
{
    math::Vec3 pos = math::Vec3::Zero();
    math::Quat quat = math::Quat::Identity();
    math::Vec3 lin = math::Vec3::Zero();
};

struct WorldPose
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;
};

class FootstepCommand
{
public:
    struct Config
    {
        int future_foot_step_num = 2;
        float step_dt = 0.02f;
        float vrp_height = 0.6607f;
        float pelv_com_offset = 0.0761f;
        float vrpx_offset = 0.03f;   // VRP forward bias in foot yaw frame [m]
        float vrpy_offset = 0.02f;   // VRP lateral bias toward stance [m]
        float vrp_horizon_length = 5.0f;
        float preview_horizon_length = 2.0f;
        // HEURI / HEURI2 ignore the preview controller entirely.
        ComGenerateType com_generate_type = ComGenerateType::PREV;
        float swing_up_timing = 0.4f;
        float swing_down_timing = 0.5f;
        int ik_iters = 50;
        float ik_lambda = 0.05f;
        float ik_pos_tol = 1e-4f;
        // footstep ranges (JoystickFootCommandCfg.sample_ranges): the planner's candidate area and the clamps
        float foot_pos_x_min = -0.3f, foot_pos_x_max = 0.4f;
        float foot_pos_y_min = 0.2f, foot_pos_y_max = 0.4f;
        float foot_pos_z_min = -0.2f, foot_pos_z_max = 0.2f;
        float foot_rot_y_min = -0.8f, foot_rot_y_max = 0.8f;
        int start_phase_indicator = 0; // 0: right swing first, 1: left swing first
    };

    FootstepCommand(const Config& cfg, std::shared_ptr<Kinematics> kin)
    : cfg_(cfg), kin_(std::move(kin)),
      vrp_(cfg.vrp_horizon_length, cfg.step_dt, cfg.future_foot_step_num, cfg.vrp_height,
           cfg.vrpx_offset, cfg.vrpy_offset),
      preview_(cfg.preview_horizon_length, cfg.step_dt, cfg.vrp_height)
    {
        const int LA = cfg_.future_foot_step_num;
        foot_command_.assign(LA, {0,0,0,0,0,0,0,0,0});
        com_z_command_.assign(LA + 1, 0.0f);
        phase_indicator_.assign(LA, 0);
        target_joint_pos_.setZero(12);
        command_vec_.fill(0.0f);
    }

    // Markers for the simulator: soles in the yaw frame of `frame` (origin at that body), [x, y, z, yaw] with z = sole
    // bottom; plan_seq() changes whenever they are replanned.
    struct MarkerConfig
    {
        std::string frame = "torso_link";
        float sole_x = 0.20f, sole_y = 0.08f; // drawn sole size [m]
        float sole_offset_x = 0.035f;          // sole centre ahead of the ankle [m]
        float ankle_height = 0.035f;           // ankle above the sole [m]
    };
    // The joystick's nominal footstep.
    const std::vector<std::array<float, 4>>& nominal_soles() const { return nominal_soles_; }
    int plan_seq() const { return plan_seq_; }
    // Stage 2: sole of the footstep the planner picked; empty in stage 1.
    const std::vector<std::array<float, 4>>& picked_soles() const { return picked_soles_; }
    // Stage 2: the score map of the last pick, one tile per candidate cell (all of them):
    // [x, y, z, bin] = sole centre at the predicted landing height (same frame as the soles) and the score bin:
    // SCORE_BINS - 1 = best ... 0 = SCORE_SHOWN or more below the best. A tile is never drawn below the lowest
    // of the cells within SCORE_SHOWN of the best: the height of a bad cell (a gap) is not learned, and the tile
    // would sink out of sight. score_tiles_yaw(): yaw of the cell grid in that frame.
    static constexpr float SCORE_SHOWN = 2.5f;
    static constexpr int SCORE_BINS = 5;
    const std::vector<std::array<float, 4>>& score_tiles() const { return score_tiles_; }
    float score_tiles_yaw() const { return score_tiles_yaw_; }
    float cell_size() const { return pcfg_.res; }
    const MarkerConfig& marker_config() const { return mcfg_; }

    // The joystick velocity gives the nominal footstep and its timing (JoystickFootCommand). Stage 2 (`onnx_path` not
    // empty): the first footstep and both height steps are picked from the depth patch at every support transition
    // (DeployedPlanner). Stage 1 (`onnx_path` empty): the nominal footsteps are walked with z = 0 (flat ground only).
    void enable_planner_mode(const FootstepPlannerConfig& pc, const std::string& onnx_path)
    {
        planner_mode_ = true;
        pcfg_ = pc;
        cam_fid_ = kin_->frame_index(pc.camera_parent);
        patch_ = std::make_unique<PlannerPatch>(pc, cfg_.foot_pos_x_min, cfg_.foot_pos_x_max,
                                                cfg_.foot_pos_y_min, cfg_.foot_pos_y_max);
        // markers for the simulator: soles in the yaw frame of the camera's parent body
        mcfg_.frame = pc.camera_parent;
        mcfg_.sole_offset_x = pc.sole_offset_x;
        mcfg_.ankle_height = -pc.sole_z;
        if (onnx_path.empty()) return;
        net_ = std::make_unique<PlannerNet>(onnx_path);
        if (net_->H() != patch_->H() || net_->W() != patch_->W() || net_->proprio_dim() != 37)
            throw std::runtime_error("FootstepCommand: planner.onnx expects patch " + std::to_string(net_->H()) + "x" +
                                     std::to_string(net_->W()) + " / proprio " + std::to_string(net_->proprio_dim()) +
                                     ", the controller builds " + std::to_string(patch_->H()) + "x" +
                                     std::to_string(patch_->W()) + " / 37 (check footstep.ranges).");
    }
    bool planner_mode() const { return planner_mode_; }
    // Joystick velocity command [m/s, m/s, rad/s]; read at support transitions.
    void set_joystick_velocity(float vx, float vy, float wz) { joy_v_ = {vx, vy, wz}; }
    // [base ang vel (3), projected gravity (3), leg q - default (12), leg qd (12), waist q - default (3)], each tick.
    void set_planner_proprio(const std::array<float, 33>& p) { planner_proprio_ = p; }
    // Call after compute() / hold_standby() of every tick with the newest depth image (nullptr: none); every
    // `every`-th call keeps it, placed with the camera pose of this tick.
    void planner_observe(const float* depth)
    {
        if (!planner_mode_) return;
        const bool due = (planner_tick_++ % pcfg_.every) == 0;
        if (!due || !depth) return;
        Eigen::Vector3f p;
        Eigen::Matrix3f R;
        camera_world_(p, R);
        patch_->add_frame(depth, p, R);
    }
    // Candidate cell of the last pick (cells = in-place step, -1 = nominal footstep without a network); changes of
    // planner_pick_seq() mark a new pick.
    int planner_pick() const { return planner_pick_; }
    // Inputs and scores of the last pick (patch / seen: H x W row-major; score: cells + 1), and the stance pose used.
    const std::vector<float>& planner_patch() const { return patch_buf_; }
    const std::vector<float>& planner_seen() const { return seen_buf_; }
    const std::vector<float>& planner_proprio() const { return proprio_buf_; }
    const std::vector<float>& planner_score() const { return score_buf_; }
    const WorldPose& stance_world() const { return stance_world_; }
    int planner_pick_seq() const { return planner_pick_seq_; }


    void set_foot_state_source(FootStateSource src) { foot_state_source_ = src; }
    FootStateSource foot_state_source() const { return foot_state_source_; }

    // Sim only: pelvis world position from rt/odommodestate (set each control tick).
    void set_base_pos_world(const math::Vec3& p) { base_pos_world_ = p; }
    const math::Vec3& base_pos_world() const { return base_pos_world_; }

    // Walk start: two in-place footsteps, anchored on the state the robot is in now.
    void reset()
    {
        step_counter_ = 0;
        // Next hold_standby must re-solve IK from the pose we are standing in.
        standby_ik_latched_ = false;
        // A fresh resample owns the world anchor; don't track the IMU until it has set one.
        world_yaw_latched_ = false;
        resample_command_planner_();
        if (foot_state_source_ == FootStateSource::FK_ODOMETRY)
        {
            world_yaw_offset_ = math::wrap_to_pi(stance_world_.yaw - foot_yaw_imu_(stance_side_()));
            world_yaw_latched_ = true;
        }
        update_link_states_();
        if (uses_absolute_foot_())
            init_stance_world_from_abs_();
        generate_vrp_ref_trajectory_();
        preview_.reset_error_integral();
        Eigen::Matrix3f s = Eigen::Matrix3f::Zero();
        s.row(0) = com_pos_stance_.transpose();
        preview_.set_state(s);
        target_com_stance_ = com_pos_stance_;
        // env->reset() seeds the obs history from command(), so the IK target must be valid now.
        generate_feet_ref_trajectory_();
        math::Vec3 pelv_pos_stance = com_pos_stance_;
        pelv_pos_stance[2] += cfg_.pelv_com_offset;
        const float com_yaw_ref = vrp_.CoM_yaw_ref_firststance[std::min(1, vrp_.NL() - 1)];
        solve_leg_ik_(pelv_pos_stance, math::quat_from_euler_xyz(0.0f, 0.0f, com_yaw_ref),
                      target_swing_stance_pos_, target_swing_stance_quat_);
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
            // Captured before the command buffer shifts.
            const float sw_yaw = math::wrap_to_pi(math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);
            last_step_error_[0] = foot_command_[0][0] - swing_foot_stance_pos_[0];
            last_step_error_[1] = foot_command_[0][1] - swing_foot_stance_pos_[1];
            last_step_error_[2] = foot_command_[0][2] - swing_foot_stance_pos_[2];
            last_step_yaw_error_ = math::wrap_to_pi(foot_command_[0][5] - sw_yaw);
            last_step_total_time_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
            last_step_cmd_ = math::Vec3(foot_command_[0][0], foot_command_[0][1], foot_command_[0][2]);
            last_step_cmd_yaw_ = foot_command_[0][5];
            last_step_meas_ = math::Vec3(swing_foot_stance_pos_[0], swing_foot_stance_pos_[1],
                                         swing_foot_stance_pos_[2]);
            last_step_meas_yaw_ = sw_yaw;
            last_step_timing_ = math::Vec3(foot_command_[0][6], foot_command_[0][7], foot_command_[0][8]);
            last_step_swing_right_ = (phase_indicator_[0] == 0);
            ++step_counter_;

            update_command_planner_();
            update_link_states_();
            generate_vrp_ref_trajectory_();
            generate_ref_trajectory_();
        }
        compute_command_vec_();
    }

    // State re-entry: the robot may have moved since the standby IK was latched.
    void clear_standby() { standby_ik_latched_ = false; }

    // Stand-still command used instead of compute(). IK is latched on the first call (or com_z change):
    // re-solving every tick from measured feet would let sway chase the IK observation.
    void hold_standby(float com_z)
    {
        update_link_states_(); // keep measured state / IMU yaw fresh for resume

        const bool need_ik = !standby_ik_latched_ || (com_z != standby_ik_com_z_);
        if (need_ik)
        {
            math::Vec3 pelv_pos_stance(0.5f * swing_foot_stance_pos_[0],
                                       0.5f * swing_foot_stance_pos_[1],
                                       cfg_.vrp_height + com_z + cfg_.pelv_com_offset);
            // heading settles at the mean of the two feet, as it does during DSP
            const float mid_yaw = 0.5f * math::wrap_to_pi(
                math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);
            const math::Quat pelv_quat_stance = math::quat_from_euler_xyz(0.0f, 0.0f, mid_yaw);

            solve_leg_ik_(pelv_pos_stance, pelv_quat_stance,
                          swing_foot_stance_pos_, swing_foot_stance_quat_);
            for (int i = 0; i < 12; ++i) standby_ik_joints_[i] = target_joint_pos_(i);
            standby_ik_com_z_ = com_z;
            standby_ik_latched_ = true;
        }

        for (int i = 0; i < 12; ++i) command_vec_[i] = standby_ik_joints_[i];
        command_vec_[12] = 1.0f; // cos(0)
        command_vec_[13] = 0.0f; // sin(0)
        const std::array<float, 9> fc = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.7f, 0.15f, 0.08f};
        for (int i = 0; i < 9; ++i) command_vec_[14 + i] = fc[i];
        command_vec_[23] = com_z;
    }

    // 24-dim command: [ik_target(12), phase_cos, phase_sin, foot_command0(9), com_z(1)]
    const std::array<float, 24>& command() const { return command_vec_; }

    // True in the tick a footstep ends.
    bool step_completed() const { return step_completed_; }

    // Commanded minus measured swing-foot landing, stance frame. Valid only when step_completed().
    const math::Vec3& last_step_error() const { return last_step_error_; }
    float last_step_yaw_error() const { return last_step_yaw_error_; }
    // ssp + 2*dsp
    float last_step_total_time() const { return last_step_total_time_; }
    const math::Vec3& last_step_command() const { return last_step_cmd_; }
    float last_step_command_yaw() const { return last_step_cmd_yaw_; }
    const math::Vec3& last_step_measured() const { return last_step_meas_; }
    float last_step_measured_yaw() const { return last_step_meas_yaw_; }
    // [ssp_t, dsp_t, height]; foot_command0() has already shifted by the time step_completed() is seen.
    const math::Vec3& last_step_timing() const { return last_step_timing_; }
    bool last_step_swing_right() const { return last_step_swing_right_; }
    long step_counter() const { return step_counter_; }

    // Current foot command buffer slot 0 (stance frame): [x, y, z, r, p, yaw, ssp, dsp, height].
    const std::array<float, 9>& foot_command0() const { return foot_command_[0]; }

    // Logging accessors (tocabi cc.cpp writeFile columns). "global" = pelvis-anchored frame.
    // Reference ZMP (VRP), stance frame.
    math::Vec3 ref_zmp() const
    {
        int idx = static_cast<int>(walking_tick_);
        if (idx < 0) idx = 0;
        if (idx >= vrp_.NL()) idx = vrp_.NL() - 1;
        return vrp_.vrp_ref_firststance[idx];
    }
    // Heuristic reference instead of preview output when com_generate_type != PREV.
    math::Vec3 target_com_stance() const { return target_com_stance_; }
    math::Vec3 com_stance() const { return com_pos_stance_; }
    // Swing foot in the stance frame: measured, and the reference of this tick.
    math::Vec3 swing_stance() const { return swing_foot_stance_pos_; }
    math::Vec3 target_swing_stance() const { return target_swing_stance_pos_; }
    math::Vec3 com_global() const { return com_pos_global_; }
    math::Vec3 target_com_global() const { return target_com_global_pos_; }

    // Accumulated world frame (stance_world_ odometry), same as left_foot_pos()/right_foot_pos().
    math::Vec3 swing_target_world() const { return stance_to_world_(swing_foot_end_stance_pos_); }
    math::Vec3 ref_vrp_world() const { return stance_to_world_(ref_zmp()); }
    math::Vec3 target_com_world() const { return stance_to_world_(target_com_stance()); }
    math::Vec3 com_world() const { return stance_to_world_(com_pos_stance_); }
    math::Vec3 stance_foot_world() const { return stance_foot_world_pos_(); }
    math::Vec3 swing_foot_world() const
    {
        if (uses_absolute_foot_())
            return foot_world_pos_abs_(swing_side_());
        return swing_foot_world_pos_();
    }
    int phase_indicator0() const { return phase_indicator_.empty() ? 0 : phase_indicator_[0]; }
    math::Vec3 left_foot_pos() const
    {
        if (uses_absolute_foot_())
            return foot_world_pos_abs_(Kinematics::LEFT);
        return (phase_indicator_[0] == 0) ? stance_foot_world_pos_() : swing_foot_world_pos_();
    }
    math::Vec3 right_foot_pos() const
    {
        if (uses_absolute_foot_())
            return foot_world_pos_abs_(Kinematics::RIGHT);
        return (phase_indicator_[0] == 0) ? swing_foot_world_pos_() : stance_foot_world_pos_();
    }
    // left 6 + right 6, SDK 0..11 order
    const Eigen::VectorXf& target_joint_pos() const { return target_joint_pos_; }
    // Resets to 0 at each footstep.
    long walking_tick() const { return walking_tick_; }

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

    math::Vec3 stance_to_world_(const math::Vec3& p_stance) const
    {
        const float c = std::cos(stance_world_.yaw);
        const float s = std::sin(stance_world_.yaw);
        return math::Vec3(stance_world_.x + c * p_stance[0] - s * p_stance[1],
                          stance_world_.y + s * p_stance[0] + c * p_stance[1],
                          stance_world_.z + p_stance[2]);
    }
    math::Vec3 stance_foot_world_pos_() const
    {
        return math::Vec3(stance_world_.x, stance_world_.y, stance_world_.z);
    }
    math::Vec3 swing_foot_world_pos_() const
    {
        const float c = std::cos(stance_world_.yaw);
        const float s = std::sin(stance_world_.yaw);
        const math::Vec3& sw = swing_foot_stance_pos_;
        return math::Vec3(stance_world_.x + c * sw[0] - s * sw[1],
                          stance_world_.y + s * sw[0] + c * sw[1],
                          stance_world_.z + sw[2]);
    }

    // Valid while phase_indicator_[0] still matches the measured link states.
    WorldPose swing_world_pose_() const
    {
        if (uses_absolute_foot_())
            return world_pose_from_foot_abs_(swing_side_());
        const math::Vec3 p = swing_foot_world_pos_();
        const float yaw = math::wrap_to_pi(
            stance_world_.yaw + math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);
        return WorldPose{p[0], p[1], p[2], yaw};
    }

    // Inverse of build_global_step_.
    static WorldPose apply_step_(const WorldPose& from, const std::array<float, 9>& fc)
    {
        const float c = std::cos(from.yaw);
        const float s = std::sin(from.yaw);
        WorldPose w;
        w.x = from.x + c * fc[0] - s * fc[1];
        w.y = from.y + s * fc[0] + c * fc[1];
        w.z = from.z + fc[2];
        w.yaw = math::wrap_to_pi(from.yaw + fc[5]);
        return w;
    }

    // fk_odometry has no absolute position reference, but the IMU IS an absolute heading reference.
    float foot_yaw_imu_(Kinematics::Side s) const
    {
        return math::wrap_to_pi(math::euler_xyz_from_quat(
            math::quat_mul(robot_quat_w_, kin_->foot_quat(s)))[2]);
    }

    bool uses_absolute_foot_() const
    {
        return foot_state_source_ == FootStateSource::SIM_ODOM;
    }

    math::Vec3 foot_world_pos_sim_(Kinematics::Side s) const
    {
        return base_pos_world_ + math::quat_apply(robot_quat_w_, kin_->foot_pos(s));
    }
    WorldPose world_pose_from_foot_sim_(Kinematics::Side s) const
    {
        const math::Vec3 pos = foot_world_pos_sim_(s);
        const math::Quat q = math::quat_mul(robot_quat_w_, kin_->foot_quat(s));
        const float yaw = math::wrap_to_pi(math::euler_xyz_from_quat(q)[2]);
        return WorldPose{pos[0], pos[1], pos[2], yaw};
    }

    math::Vec3 foot_world_pos_abs_(Kinematics::Side s) const { return foot_world_pos_sim_(s); }
    WorldPose world_pose_from_foot_abs_(Kinematics::Side s) const { return world_pose_from_foot_sim_(s); }

    void init_stance_world_from_abs_()
    {
        stance_world_ = world_pose_from_foot_abs_(stance_side_());
    }

    // The foot that just landed becomes the stance at its measured pose, correcting commanded-vs-achieved drift.
    void accumulate_stance_world_()
    {
        if (uses_absolute_foot_())
        {
            stance_world_ = world_pose_from_foot_abs_(swing_side_());
            return;
        }
        const math::Vec3 sw = swing_foot_stance_pos_;
        const float c = std::cos(stance_world_.yaw);
        const float s = std::sin(stance_world_.yaw);
        WorldPose next;
        next.x = stance_world_.x + c * sw[0] - s * sw[1];
        next.y = stance_world_.y + s * sw[0] + c * sw[1];
        next.z = stance_world_.z + sw[2];
        // Heading is read from the IMU, not integrated (see update_link_states_).
        next.yaw = math::wrap_to_pi(foot_yaw_imu_(swing_side_()) + world_yaw_offset_);
        stance_world_ = next;
    }

    // ---- footsteps (JoystickFootCommand) ----
    // After a reset: two in-place footsteps, as the training env (FootCommandCfg.ranges); nothing seen yet.
    void resample_command_planner_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        stance_world_ = WorldPose{};
        phase_indicator_[0] = cfg_.start_phase_indicator;
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i + 1] = 1 - phase_indicator_[i];
        for (int s = 0; s < LA; ++s)
        {
            const float sign = (phase_indicator_[s] == 0) ? -1.0f : 1.0f;
            foot_command_[s] = {0.f, sign * pcfg_.stance_width, 0.f, 0.f, 0.f, 0.f,
                                pcfg_.reset_ssp, pcfg_.reset_dsp, pcfg_.foot_height};
        }
        for (int i = 0; i <= LA; ++i) com_z_command_[i] = 0.0f;
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
        patch_->clear();
        planner_tick_ = 0;
    }

    void update_command_planner_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        accumulate_stance_world_();
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i] = phase_indicator_[i + 1];
        phase_indicator_[LA - 1] = 1 - phase_indicator_[LA - 2];
        update_link_states_(); // the foot that just landed is now the stance foot
        fill_planner_buffer_();
    }

    // JoystickFootCommand.candidates + DeployedPlanner.pick for one env.
    void fill_planner_buffer_()
    {
        const int LA = cfg_.future_foot_step_num;
        float u = 0.0f;
        for (int i = 0; i < 3; ++i) u = std::max(u, std::abs(joy_v_[i]) / pcfg_.joystick_max[i]);
        const float T = pcfg_.step_time_slow - (pcfg_.step_time_slow - pcfg_.step_time_fast) * std::min(u, 1.0f);
        for (int s = 0; s < LA; ++s)
        {
            const float sign = (phase_indicator_[s] == 0) ? -1.0f : 1.0f;
            auto& fc = foot_command_[s];
            fc = {};
            fc[0] = clampf(joy_v_[0] * T, cfg_.foot_pos_x_min, cfg_.foot_pos_x_max);
            fc[1] = sign * clampf(pcfg_.stance_width + sign * joy_v_[1] * T, cfg_.foot_pos_y_min, cfg_.foot_pos_y_max);
            fc[5] = clampf(joy_v_[2] * T, cfg_.foot_rot_y_min, cfg_.foot_rot_y_max);
            fc[6] = pcfg_.ssp_ratio * T;
            fc[7] = 0.5f * (1.0f - pcfg_.ssp_ratio) * T;
            fc[8] = pcfg_.foot_height;
        }
        for (int i = 0; i <= LA; ++i) com_z_command_[i] = 0.0f;
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
        planner_pick_ = -1;
        ++planner_pick_seq_;
        const float nom_x = foot_command_[0][0], nom_y = foot_command_[0][1];
        picked_soles_.clear();
        score_tiles_.clear();
        nominal_soles_.assign(1, planner_sole_(nom_x, nom_y, 0.0f, foot_command_[0][5]));
        ++plan_seq_;
        if (!net_) return;

        const float side = (phase_indicator_[0] == 0) ? -1.0f : 1.0f;
        patch_->build(stance_world_.x, stance_world_.y, stance_world_.z, stance_world_.yaw, side, patch_buf_, seen_buf_);
        // planner_proprio: the joystick in shares of joystick_max, mirrored like the patch for a right swing
        proprio_buf_.assign(planner_proprio_.begin(), planner_proprio_.end());
        proprio_buf_.push_back(side);
        proprio_buf_.push_back(joy_v_[0] / pcfg_.joystick_max[0]);
        proprio_buf_.push_back(side * joy_v_[1] / pcfg_.joystick_max[1]);
        proprio_buf_.push_back(side * joy_v_[2] / pcfg_.joystick_max[2]);
        net_->run(patch_buf_, proprio_buf_, seen_buf_, score_buf_, z_buf_);
        const int best = (int)(std::max_element(score_buf_.begin(), score_buf_.end()) - score_buf_.begin());
        planner_pick_ = best;
        if (best < patch_->cells())
        {
            foot_command_[0][0] = patch_->cell_x(best);
            foot_command_[0][1] = side * patch_->cell_y(best);
        }
        else // in-place step: nothing else scored better
        {
            for (int s = 0; s < LA; ++s)
            {
                foot_command_[s][0] = 0.0f;
                foot_command_[s][5] = 0.0f;
                foot_command_[s][1] = ((phase_indicator_[s] == 0) ? -1.0f : 1.0f) * cfg_.foot_pos_y_min;
            }
        }
        for (int s = 0; s < std::min(LA, 2); ++s)
            foot_command_[s][2] = clampf(z_buf_[2 * best + s], cfg_.foot_pos_z_min, cfg_.foot_pos_z_max);
        // both at the predicted landing height: the nominal footstep has none of its own
        nominal_soles_[0] = planner_sole_(nom_x, nom_y, foot_command_[0][2], foot_command_[0][5]);
        picked_soles_.assign(1, planner_sole_(foot_command_[0][0], foot_command_[0][1], foot_command_[0][2], foot_command_[0][5]));
        float z_floor = cfg_.foot_pos_z_max;
        for (int i = 0; i < patch_->cells(); ++i)
            if (score_buf_[i] - score_buf_[best] > -SCORE_SHOWN)
                z_floor = std::min(z_floor, clampf(z_buf_[2 * i], cfg_.foot_pos_z_min, cfg_.foot_pos_z_max));
        for (int i = 0; i < patch_->cells(); ++i)
        {
            const float rel = std::max(score_buf_[i] - score_buf_[best], -SCORE_SHOWN);
            const float z = std::max(clampf(z_buf_[2 * i], cfg_.foot_pos_z_min, cfg_.foot_pos_z_max), z_floor);
            auto tile = planner_sole_(patch_->cell_x(i), side * patch_->cell_y(i), z, 0.0f);
            score_tiles_yaw_ = tile[3];
            tile[3] = (float)std::min(SCORE_BINS - 1, (int)((rel + SCORE_SHOWN) / SCORE_SHOWN * SCORE_BINS));
            score_tiles_.push_back(tile);
        }
        float seen = 0.0f;
        for (float v : seen_buf_) seen += v;
        const WorldPose land = apply_step_(stance_world_, foot_command_[0]); // ankle of the landing, odometry world frame
        spdlog::info("[FootPlanner] {} x={:.3f} y={:.3f} z={:.3f} yaw={:.3f} (nominal x={:.3f} y={:.3f}; next z={:.3f}; "
                     "{} images, {:.0f}% of the patch seen; lands at world {:.3f} {:.3f} {:.3f} yaw {:.3f}){}",
                     side > 0 ? "L" : "R", foot_command_[0][0], foot_command_[0][1], foot_command_[0][2], foot_command_[0][5],
                     nom_x, nom_y, foot_command_[1][2], patch_->num_frames(), 100.0f * seen / seen_buf_.size(),
                     land.x, land.y, land.z, land.yaw, best < patch_->cells() ? "" : " in-place");
    }

    // Sole centre [x, y, z, yaw] (z = sole bottom) of a first footstep (x, y, z, yaw: stance-foot frame) in the yaw
    // frame of the marker body (origin at the body).
    std::array<float, 4> planner_sole_(float x, float y, float z, float yaw) const
    {
        const math::Vec3 body = math::quat_apply(robot_quat_w_, kin_->frame_pos(cam_fid_));
        const float body_yaw = math::euler_xyz_from_quat(math::quat_mul(robot_quat_w_, kin_->frame_quat(cam_fid_)))[2];
        const float cb = std::cos(body_yaw), sb = std::sin(body_yaw);
        const math::Vec3 d = stance_foot_global_.pos - body;
        const float pyaw = math::wrap_to_pi(math::euler_xyz_from_quat(stance_foot_global_.quat)[2] - body_yaw);
        const float c = std::cos(pyaw), sn = std::sin(pyaw), syaw = pyaw + yaw;
        return {cb * d[0] + sb * d[1] + c * x - sn * y + std::cos(syaw) * mcfg_.sole_offset_x,
                -sb * d[0] + cb * d[1] + sn * x + c * y + std::sin(syaw) * mcfg_.sole_offset_x,
                d[2] + z - mcfg_.ankle_height, syaw};
    }

    // World pose of the depth camera (rotation columns: optical axes). sim_odom: the true base position; otherwise
    // carried by the stance foot (leg odometry), as the foot world poses.
    void camera_world_(Eigen::Vector3f& p, Eigen::Matrix3f& R) const
    {
        Eigen::Vector3f pp;
        Eigen::Matrix3f Rp;
        planner_camera_in_pelvis(*kin_, cam_fid_, pcfg_, pp, Rp);
        const Eigen::Matrix3f Rw = math::matrix_from_quat(robot_quat_w_);
        const Eigen::Vector3f pg = Rw * pp;
        if (foot_state_source_ == FootStateSource::SIM_ODOM)
        {
            p = base_pos_world_ + pg;
            R = Rw * Rp;
            return;
        }
        const float dyaw = stance_world_.yaw - math::euler_xyz_from_quat(stance_foot_global_.quat)[2];
        const Eigen::Matrix3f Rz = Eigen::AngleAxisf(dyaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        p = Eigen::Vector3f(stance_world_.x, stance_world_.y, stance_world_.z) + Rz * (pg - stance_foot_global_.pos);
        R = Rz * Rw * Rp;
    }

    void update_link_states_()
    {
        const math::Quat R = robot_quat_w_; // pelvis orientation from IMU
        com_pos_global_ = math::quat_apply(R, kin_->com_pos());

        auto fill = [&](Kinematics::Side s, FrameState& fs) {
            fs.pos = math::quat_apply(R, kin_->foot_pos(s));
            fs.quat = math::quat_mul(R, kin_->foot_quat(s));
        };
        fill(stance_side_(), stance_foot_global_);
        fill(swing_side_(), swing_foot_global_);
        // World velocity of the stance foot, read when the preview state is moved to a new stance frame. The foot is
        // planted, so it is zero. The fixed-base FK velocity is the foot's velocity relative to the pelvis (about minus
        // the walking speed): using it doubled the CoM velocity of the preview state at every footstep, and the CoM
        // reference was 3 cm (up to 8 cm) off the training's (test/planner_replay_test.cpp, 2026-11-16).
        stance_foot_global_.lin = math::Vec3::Zero();

        const math::Quat stance_yaw = math::yaw_quat(stance_foot_global_.quat);
        math::subtract_frame_transforms(stance_foot_global_.pos, stance_yaw,
                                        swing_foot_global_.pos, swing_foot_global_.quat,
                                        swing_foot_stance_pos_, swing_foot_stance_quat_);
        com_pos_stance_ = math::subtract_frame_transforms_pos(stance_foot_global_.pos, stance_yaw, com_pos_global_);

        // fk_odometry: read the heading from the IMU instead of integrating per-step foot yaw, which
        // accumulates landing error and misses turns made while not stepping (standby, stop at a goal).
        if (foot_state_source_ == FootStateSource::FK_ODOMETRY && world_yaw_latched_)
            stance_world_.yaw = math::wrap_to_pi(foot_yaw_imu_(stance_side_()) + world_yaw_offset_);
    }

    void generate_vrp_ref_trajectory_()
    {
        // init vrp state (zmp x,y; com z) in stance frame
        math::Vec3 vrp_state_stance;
        vrp_state_stance[0] = swing_foot_stance_pos_[0] / 2.0f;
        vrp_state_stance[1] = swing_foot_stance_pos_[1] / 2.0f;
        vrp_state_stance[2] = com_pos_stance_[2];
        const float swing_yaw = math::wrap_to_pi(math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);

        vrp_.generate_vrp_online(foot_command_, vrp_state_stance, swing_yaw / 2.0f,
                                 swing_foot_stance_pos_, com_z_command_);

        if (cfg_.com_generate_type == ComGenerateType::HEURI)
            vrp_.generate_com_heuristic_online(vrp_state_stance);
        else if (cfg_.com_generate_type == ComGenerateType::HEURI2)
            vrp_.generate_com_heuristic_online2(vrp_state_stance);

        swing_foot_start_stance_pos_ = swing_foot_stance_pos_;
        swing_foot_start_stance_quat_ = swing_foot_stance_quat_;
        swing_foot_end_stance_pos_ = math::Vec3(foot_command_[0][0], foot_command_[0][1], foot_command_[0][2]);
        swing_foot_end_stance_quat_ = math::quat_from_euler_xyz(foot_command_[0][3], foot_command_[0][4], foot_command_[0][5]);

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
        math::Vec3 com_stance_pos, com_stance_vel, com_stance_acc;
        if (cfg_.com_generate_type != ComGenerateType::PREV)
        {
            // Same +1 tick lead as the yaw reference below.
            int idx = static_cast<int>(walking_tick_) + 1;
            if (idx >= vrp_.NL()) idx = vrp_.NL() - 1;
            com_stance_pos = vrp_.com_ref_firststance_heuristic[idx];
            com_stance_vel = vrp_.com_refvel_firststance_heuristic[idx];
            com_stance_acc = math::Vec3::Zero();
        }
        else
        {
            const int previewNL = preview_.NL();
            std::vector<math::Vec3>& vrp_ref = vrp_ref_scratch_; // reused: no per-tick allocation
            vrp_ref.resize(previewNL);
            for (int l = 0; l < previewNL; ++l)
            {
                int idx = static_cast<int>(walking_tick_) + l;
                if (idx >= vrp_.NL()) idx = vrp_.NL() - 1;
                vrp_ref[l] = vrp_.vrp_ref_firststance[idx];
            }
            Eigen::Matrix3f next = preview_.compute_target_state(vrp_ref);
            preview_.update_state(next);

            com_stance_pos = next.row(0).transpose();
            com_stance_vel = next.row(1).transpose();
            com_stance_acc = next.row(2).transpose();
        }
        target_com_stance_ = com_stance_pos;

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

        generate_feet_ref_trajectory_();

        solve_leg_ik_(pelv_pos_stance, pelv_quat_stance,
                      target_swing_stance_pos_, target_swing_stance_quat_);
    }

    // Inputs in the stance-foot frame (stance foot = origin).
    void solve_leg_ik_(const math::Vec3& pelv_pos_stance, const math::Quat& pelv_quat_stance,
                       const math::Vec3& swing_pos_stance, const math::Quat& swing_quat_stance)
    {
        const math::Vec3 stance_pose_pos = math::Vec3::Zero();
        const math::Quat stance_pose_quat = math::Quat::Identity();

        math::Vec3 l_pos_stance, r_pos_stance;
        math::Quat l_quat_stance, r_quat_stance;
        if (phase_indicator_[0] == 0) { // left is stance
            l_pos_stance = stance_pose_pos;  l_quat_stance = stance_pose_quat;
            r_pos_stance = swing_pos_stance; r_quat_stance = swing_quat_stance;
        } else { // right is stance
            r_pos_stance = stance_pose_pos;  r_quat_stance = stance_pose_quat;
            l_pos_stance = swing_pos_stance; l_quat_stance = swing_quat_stance;
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
            pos[0] = math::cubic(swing_foot_start_stance_pos_[0], 0.f, swing_foot_end_stance_pos_[0], 0.f, ssp_t, ct);
            pos[1] = math::cubic(swing_foot_start_stance_pos_[1], 0.f, swing_foot_end_stance_pos_[1], 0.f, ssp_t, ct);
            math::Vec3 e0 = math::euler_xyz_from_quat(swing_foot_start_stance_quat_);
            math::Vec3 e1 = math::euler_xyz_from_quat(swing_foot_end_stance_quat_);
            math::Vec3 et;
            for (int i = 0; i < 3; ++i)
                et[i] = math::cubic(math::wrap_to_pi(e0[i]), 0.f, math::wrap_to_pi(e1[i]), 0.f, ssp_t, ct);
            quat = math::quat_from_euler_xyz(et[0], et[1], et[2]);
            // z: lift up / maintain / down
            const float lift = std::max(0.0f, std::max(swing_foot_start_stance_pos_[2], swing_foot_end_stance_pos_[2])) + height;
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
        command_vec_[23] = com_z_command_[0];
    }

public:
    // Set by State_Footstep each tick.
    math::Quat robot_quat_w_ = math::Quat::Identity();

private:
    Config cfg_;
    std::shared_ptr<Kinematics> kin_;
    VrpGenerator vrp_;
    PreviewController preview_;
    std::vector<math::Vec3> vrp_ref_scratch_;

    long walking_tick_ = 0;
    float time_left_ = 0.0f;
    bool step_completed_ = false;
    math::Vec3 last_step_error_ = math::Vec3::Zero();
    float last_step_yaw_error_ = 0.0f;
    math::Vec3 last_step_cmd_ = math::Vec3::Zero();
    float last_step_cmd_yaw_ = 0.0f;
    math::Vec3 last_step_meas_ = math::Vec3::Zero();
    float last_step_meas_yaw_ = 0.0f;
    math::Vec3 last_step_timing_ = math::Vec3::Zero();
    bool last_step_swing_right_ = true;
    long step_counter_ = 0;
    float last_step_total_time_ = 0.0f;

    std::vector<std::array<float, 9>> foot_command_;
    std::vector<float> com_z_command_;
    std::vector<int> phase_indicator_;

    MarkerConfig mcfg_;
    std::vector<std::array<float, 4>> nominal_soles_;
    std::vector<std::array<float, 4>> picked_soles_;
    std::vector<std::array<float, 4>> score_tiles_;
    float score_tiles_yaw_ = 0.0f;
    int plan_seq_ = 0;

    bool planner_mode_ = false;
    FootstepPlannerConfig pcfg_;
    pinocchio::FrameIndex cam_fid_ = 0;
    std::unique_ptr<PlannerPatch> patch_;
    std::unique_ptr<PlannerNet> net_;
    std::array<float, 3> joy_v_{};
    std::array<float, 33> planner_proprio_{};
    std::vector<float> patch_buf_, seen_buf_, proprio_buf_, score_buf_, z_buf_;
    long planner_tick_ = 0;
    int planner_pick_ = -1;
    int planner_pick_seq_ = 0;

    // World pose of the stance foot (odometry frame: the initial stance foot, or the simulator's world with sim_odom).
    WorldPose stance_world_;

    FootStateSource foot_state_source_ = FootStateSource::FK_ODOMETRY;
    math::Vec3 base_pos_world_ = math::Vec3::Zero(); // pelvis world pos (sim_odom)
    // fk_odometry: stance_world_.yaw == IMU foot yaw + this offset, latched at reset().
    float world_yaw_offset_ = 0.0f;
    bool world_yaw_latched_ = false;

    // measured state (per tick)
    math::Vec3 com_pos_global_ = math::Vec3::Zero();
    math::Vec3 com_pos_stance_ = math::Vec3::Zero();
    FrameState stance_foot_global_, swing_foot_global_;
    math::Vec3 swing_foot_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_stance_quat_ = math::Quat::Identity();

    // step-boundary state
    math::Vec3 swing_foot_start_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_start_stance_quat_ = math::Quat::Identity();
    math::Vec3 swing_foot_end_stance_pos_ = math::Vec3::Zero();
    math::Quat swing_foot_end_stance_quat_ = math::Quat::Identity();

    math::Vec3 target_com_stance_ = math::Vec3::Zero(); // stance frame (logging)
    math::Vec3 target_com_global_pos_ = math::Vec3::Zero();
    math::Vec3 target_com_global_vel_ = math::Vec3::Zero();
    math::Vec3 target_com_global_acc_ = math::Vec3::Zero();

    math::Vec3 target_swing_stance_pos_ = math::Vec3::Zero();
    math::Quat target_swing_stance_quat_ = math::Quat::Identity();

    Eigen::VectorXf target_joint_pos_;
    std::array<float, 24> command_vec_;

    // Standby IK, solved once per stop.
    bool standby_ik_latched_ = false;
    float standby_ik_com_z_ = 0.0f;
    std::array<float, 12> standby_ik_joints_{};
};

// FootstepCommand::Config of a deploy.yaml (start_phase_indicator is set by the caller).
inline FootstepCommand::Config footstep_config_from_yaml(const YAML::Node& deploy)
{
    FootstepCommand::Config fcfg;
    const YAML::Node fs = deploy["footstep"];
    fcfg.future_foot_step_num = fs["future_foot_step_num"].as<int>(2);
    fcfg.step_dt = deploy["step_dt"].as<float>();
    fcfg.vrp_height = fs["vrp_height"].as<float>();
    fcfg.pelv_com_offset = fs["pelv_com_offset"].as<float>();
    fcfg.vrpx_offset = fs["vrpx_offset"].as<float>(0.03f);
    fcfg.vrpy_offset = fs["vrpy_offset"].as<float>(0.02f);
    fcfg.vrp_horizon_length = fs["vrp_horizon_length"].as<float>(5.0f);
    fcfg.preview_horizon_length = fs["preview_horizon_length"].as<float>(2.0f);
    // Must match the com_generate_type the policy was trained with (G13DFootEnvCfg / _heuri1 / _heuri2).
    {
        const std::string cgt = fs["com_generate_type"].as<std::string>(std::string("prev"));
        if (cgt == "prev") fcfg.com_generate_type = ComGenerateType::PREV;
        else if (cgt == "heuri") fcfg.com_generate_type = ComGenerateType::HEURI;
        else if (cgt == "heuri2") fcfg.com_generate_type = ComGenerateType::HEURI2;
        else throw std::runtime_error("footstep.com_generate_type must be one of "
                                      "'prev', 'heuri', 'heuri2', got: " + cgt);
        spdlog::info("[Footstep] com_generate_type: {}", cgt);
    }
    fcfg.swing_up_timing = fs["swing_up_timing"].as<float>(0.4f);
    fcfg.swing_down_timing = fs["swing_down_timing"].as<float>(0.5f);
    fcfg.ik_iters = fs["ik_iters"].as<int>(50);
    fcfg.ik_lambda = fs["ik_lambda"].as<float>(0.05f);
    fcfg.ik_pos_tol = fs["ik_pos_tol"].as<float>(1e-4f);
    if (fs["ranges"])
    {
        auto r = fs["ranges"];
        auto pair = [&](const std::string& k, float& lo, float& hi) {
            if (r[k]) { lo = r[k][0].as<float>(); hi = r[k][1].as<float>(); }
        };
        pair("foot_pos_x", fcfg.foot_pos_x_min, fcfg.foot_pos_x_max);
        pair("foot_pos_y", fcfg.foot_pos_y_min, fcfg.foot_pos_y_max);
        pair("foot_pos_z", fcfg.foot_pos_z_min, fcfg.foot_pos_z_max);
        pair("foot_rot_y", fcfg.foot_rot_y_min, fcfg.foot_rot_y_max);
    }
    return fcfg;
}

} // namespace isaaclab
