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
#include "isaaclab/envs/mdp/commands/foot_command_source.h" // FootCommandInput
#include "isaaclab/envs/mdp/commands/vision_foot_target_source.h" // VisionTargetPelvis

namespace isaaclab
{

// How world-frame foot positions are obtained (logging & global-plan stance tracking).
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
    math::Vec3 ang = math::Vec3::Zero();
};

// Planar (x, y, z, yaw) world pose used by the "global" command mode to track
// the accumulated stance-foot pose across footsteps. See set_global_plan().
struct WorldPose
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;
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
        float foot_pos_z_min = -0.15f, foot_pos_z_max = 0.15f;
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

    // Enable "global" command mode: instead of per-tick local operator input,
    // the planner follows a fixed list of absolute (world-frame) swing-foot
    // targets. The world frame is anchored at the initial stance foot, and the
    // achieved stance pose is accumulated from the measured swing-foot landing
    // each step, so the local command for step 0 is recomputed from the current
    // (drift-corrected) stance foot to the indexed global target.
    // `init_stance` is the world pose of the initial stance foot, i.e. the
    // anchor of the global plan frame. It MUST match the frame the plan CSV was
    // generated in (see cmd/convert_footcommand_2_global.py), otherwise the
    // recomputed local commands are offset by the spawn pose. Defaults to the
    // origin (plan frame == initial-stance-foot frame).
    void set_global_plan(std::vector<GlobalFootTarget> plan, WorldPose init_stance = WorldPose{})
    {
        global_plan_ = std::move(plan);
        global_init_stance_ = init_stance;
        global_mode_ = !global_plan_.empty();
        if (global_mode_) cfg_.start_phase_indicator = global_plan_.front().phase;
    }
    bool global_mode() const { return global_mode_; }

    // --- vision command mode (ArUco footstep targets) ------------------------
    // Targets are measured by cmd/aruco_footstep_perception.py in the PELVIS
    // frame and fed in every tick via set_vision_targets(). They are converted
    // to the accumulated world frame (anchored at the initial stance foot) and
    // kept in a short id-keyed memory, so a target stays plannable while it is
    // temporarily outside the camera view. At every step boundary the planner
    // picks the two nearest feasible targets (slot 0 on the upcoming swing-foot
    // side, slot 1 on the opposite side relative to slot 0) and rebuilds the
    // 2-step command buffer; with no feasible target it steps in place.
    struct VisionConfig
    {
        float memory_s = 10.0f;       // [s] world-frame target memory
        float max_range = 1.5f;       // [m] ignore targets farther than this
        float min_forward = -0.10f;   // [m] stance-frame x cutoff
        float exclude_radius = 0.12f; // [m] targets under either foot are skipped
        float side_margin = 0.04f;    // [m] swing-side feasibility margin
    };

    void enable_vision_mode(const VisionConfig& vc)
    {
        vision_mode_ = true;
        vcfg_ = vc;
    }
    bool vision_mode() const { return vision_mode_; }

    // Monotonic clock for the target memory (call every control tick).
    void set_vision_clock(double now_s) { vision_now_ = now_s; }

    // Latest perception frame (pelvis frame); ingested inside compute() where
    // the stance-foot state of the same tick is available.
    void set_vision_targets(const std::vector<VisionTargetPelvis>& ts)
    {
        pending_vision_ = ts;
        vision_pending_ = true;
    }

    // Board target ids planned into buffer slots 0/1 (-1: station-keep step).
    const std::array<int, 2>& planned_vision_ids() const { return planned_ids_; }

    // Foot world-position source (see deploy.yaml foot_state_source).
    void set_foot_state_source(FootStateSource src) { foot_state_source_ = src; }
    FootStateSource foot_state_source() const { return foot_state_source_; }

    // Sim only: pelvis world position from rt/odommodestate (set each control tick).
    void set_base_pos_world(const math::Vec3& p) { base_pos_world_ = p; }
    const math::Vec3& base_pos_world() const { return base_pos_world_; }

    // --- public API mirroring CommandTerm ---
    void reset()
    {
        command_counter_ = 0;
        resample_command_();
        update_link_states_();
        if (foot_state_source_ == FootStateSource::SIM_ODOM && !global_mode_)
            init_stance_world_from_sim_();
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
        if (vision_mode_ && vision_pending_)
        {
            ingest_vision_targets_();
            vision_pending_ = false;
        }
        generate_ref_trajectory_();
        time_left_ -= dt;
        walking_tick_ += 1;
        step_completed_ = (time_left_ <= 0.0f);
        if (step_completed_)
        {
            // Landing error of the completed step (commanded minus measured,
            // stance frame), captured before the command buffer shifts.
            const float sw_yaw = math::wrap_to_pi(math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);
            last_step_error_[0] = foot_command_[0][0] - swing_foot_stance_pos_[0];
            last_step_error_[1] = foot_command_[0][1] - swing_foot_stance_pos_[1];
            last_step_error_[2] = math::wrap_to_pi(foot_command_[0][5] - sw_yaw);
            last_step_total_time_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;

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

    // Landing error of the footstep that just completed: commanded step minus
    // the measured swing-foot landing, in the stance frame. Layout: [x, y, yaw].
    // Only meaningful on ticks where step_completed() is true.
    const math::Vec3& last_step_error() const { return last_step_error_; }
    // Total duration (ssp + 2*dsp) of that completed step.
    float last_step_total_time() const { return last_step_total_time_; }

    // Current foot command buffer slot 0 (stance frame): [x, y, z, r, p, yaw, ssp, dsp, height].
    const std::array<float, 9>& foot_command0() const { return foot_command_[0]; }

    // --- logging accessors (mirror the tocabi cc.cpp writeFile columns) ------
    // Positions marked "stance frame" are expressed in the current stance foot
    // yaw frame. CoM quantities marked "global" are in the pelvis-anchored
    // frame (the pelvis is re-anchored at the origin every control tick, so
    // the pelvis position itself is always zero and is not logged). Foot
    // positions are in the accumulated world frame (see stance_world_), which
    // persists across footsteps.

    // Reference ZMP (VRP) at the current walking tick, in the stance frame.
    math::Vec3 ref_zmp() const
    {
        int idx = static_cast<int>(walking_tick_);
        if (idx < 0) idx = 0;
        if (idx >= vrp_.NL()) idx = vrp_.NL() - 1;
        return vrp_.vrp_ref_firststance[idx];
    }
    // Target CoM (preview controller output) in the stance frame.
    math::Vec3 target_com_stance() const { return preview_.state().row(0).transpose(); }
    // Measured CoM in the stance frame.
    math::Vec3 com_stance() const { return com_pos_stance_; }
    // Measured CoM in the (pelvis-anchored) global frame.
    math::Vec3 com_global() const { return com_pos_global_; }
    // Target CoM in the (pelvis-anchored) global frame.
    math::Vec3 target_com_global() const { return target_com_global_pos_; }
    // Foot positions in the world frame, resolved to L/R.
    // fk_odometry: accumulated stance_world_ + per-tick FK swing measurement.
    // sim_odom:    odom base position + IMU-rotated FK foot offset (each tick).
    math::Vec3 left_foot_pos() const
    {
        if (foot_state_source_ == FootStateSource::SIM_ODOM)
            return foot_world_pos_sim_(Kinematics::LEFT);
        return (phase_indicator_[0] == 0) ? stance_foot_world_pos_() : swing_foot_world_pos_();
    }
    math::Vec3 right_foot_pos() const
    {
        if (foot_state_source_ == FootStateSource::SIM_ODOM)
            return foot_world_pos_sim_(Kinematics::RIGHT);
        return (phase_indicator_[0] == 0) ? swing_foot_world_pos_() : stance_foot_world_pos_();
    }
    // Desired leg joint angles (12: left 6 + right 6, SDK 0..11 order).
    const Eigen::VectorXf& target_joint_pos() const { return target_joint_pos_; }
    // Per-step walking tick (resets to 0 at each footstep).
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

    // --- accumulated world frame (for logging & the global command mode) -----
    // The current stance foot sits at stance_world_; the swing foot is the
    // per-tick FK measurement (stance frame) expressed in that world frame.
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

    // World foot pose from sim odom base + FK (pelvis frame).
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

    // Initialize stance_world_ from the current stance foot (sim_odom, local mode).
    void init_stance_world_from_sim_()
    {
        stance_world_ = world_pose_from_foot_sim_(stance_side_());
    }

    // Accumulate the achieved stance pose at a step boundary: the foot that
    // just swung (its landing measured relative to the old stance frame)
    // becomes the new stance. This corrects drift between commanded and
    // achieved steps.
    void accumulate_stance_world_()
    {
        if (foot_state_source_ == FootStateSource::SIM_ODOM)
        {
            // Absolute world pose of the foot that just landed (swing side).
            stance_world_ = world_pose_from_foot_sim_(swing_side_());
            return;
        }
        const math::Vec3 sw = swing_foot_stance_pos_;
        const float sw_yaw = math::wrap_to_pi(math::euler_xyz_from_quat(swing_foot_stance_quat_)[2]);
        const float c = std::cos(stance_world_.yaw);
        const float s = std::sin(stance_world_.yaw);
        WorldPose next;
        next.x = stance_world_.x + c * sw[0] - s * sw[1];
        next.y = stance_world_.y + s * sw[0] + c * sw[1];
        next.z = stance_world_.z + sw[2];
        next.yaw = math::wrap_to_pi(stance_world_.yaw + sw_yaw);
        stance_world_ = next;
    }

    // Build the 9-dim foot command for `step` from the operator input + phase.
    std::array<float, 9> build_foot_command_(int step)
    {
        std::array<float, 9> fc{};
        fc[0] = clampf(input_.step_x, cfg_.foot_pos_x_min, cfg_.foot_pos_x_max);
        // Nominal width is symmetric; lateral_bias shifts net CoM sideways by
        // widening the left-swing step and narrowing the right-swing step.
        const float y_mag = (phase_indicator_[step] == 1)
            ? clampf(input_.step_y + input_.lateral_bias, cfg_.foot_pos_y_min, cfg_.foot_pos_y_max)
            : clampf(input_.step_y - input_.lateral_bias, cfg_.foot_pos_y_min, cfg_.foot_pos_y_max);
        fc[1] = (phase_indicator_[step] == 0) ? -y_mag : y_mag;
        fc[2] = clampf(input_.step_z, cfg_.foot_pos_z_min, cfg_.foot_pos_z_max);
        fc[3] = 0.0f;
        fc[4] = 0.0f;
        fc[5] = clampf(input_.step_yaw, cfg_.foot_rot_y_min, cfg_.foot_rot_y_max);
        fc[6] = clampf(input_.ssp_t, cfg_.foot_ssp_min, cfg_.foot_ssp_max);
        fc[7] = clampf(input_.dsp_t, cfg_.foot_dsp_min, cfg_.foot_dsp_max);
        fc[8] = clampf(input_.height, cfg_.foot_height_min, cfg_.foot_height_max);
        return fc;
    }

    void resample_command_()
    {
        if (global_mode_) { resample_command_global_(); return; }
        if (vision_mode_) { resample_command_vision_(); return; }
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        stance_world_ = WorldPose{}; // world frame anchored at the initial stance foot
        phase_indicator_[0] = cfg_.start_phase_indicator;
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i + 1] = 1 - phase_indicator_[i];
        for (int s = 0; s < LA; ++s) foot_command_[s] = build_foot_command_(s);
        for (int i = 0; i < LA; ++i) com_z_command_[i] = input_.com_z;
        com_z_command_[LA] = input_.com_z;
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
    }

    void update_command_()
    {
        if (global_mode_) { update_command_global_(); return; }
        if (vision_mode_) { update_command_vision_(); return; }
        accumulate_stance_world_(); // keep the world-frame foot poses tracking
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

    // --- global command mode -------------------------------------------------
    // Phase for the step at plan index `idx`. Feet strictly alternate, so the
    // phase is fixed by the parity of `idx` relative to the start phase. This
    // matches the plan's foot labels while it lasts and keeps alternating past
    // the plan end (instead of getting stuck on one foot).
    int phase_for_(int idx) const
    {
        const int start_phase = global_plan_.front().phase;
        return (idx & 1) ? (1 - start_phase) : start_phase;
    }

    // Local 9-dim foot command for a swing from world pose `from` to target `tgt`.
    std::array<float, 9> build_global_step_(const WorldPose& from, const GlobalFootTarget& tgt) const
    {
        std::array<float, 9> fc{};
        const float c = std::cos(-from.yaw);
        const float s = std::sin(-from.yaw);
        const float dx = tgt.x - from.x;
        const float dy = tgt.y - from.y;
        fc[0] = c * dx - s * dy;
        fc[1] = s * dx + c * dy;
        fc[2] = tgt.z - from.z;
        fc[3] = 0.0f;
        fc[4] = 0.0f;
        fc[5] = math::wrap_to_pi(tgt.yaw - from.yaw);
        fc[6] = tgt.ssp_t;
        fc[7] = tgt.dsp_t;
        fc[8] = tgt.height;
        return fc;
    }

    // World pose used as the swing origin for buffer slot `s` (>=1): the planned
    // landing of the previous footstep, or the accumulated stance for s==0.
    WorldPose plan_pose_(int idx) const
    {
        const GlobalFootTarget& t = global_plan_[std::min(idx, (int)global_plan_.size() - 1)];
        return WorldPose{t.x, t.y, t.z, t.yaw};
    }

    // World pose the foot swinging at (virtual) plan index `idx` should HOLD
    // once the plan is exhausted: its own last planned landing (feet alternate,
    // so this is one of the last two plan entries), or the initial stance pose
    // if the plan never moved that foot. Anchoring the in-place steps to these
    // fixed world poses keeps them drift-corrected; a purely relative in-place
    // step would accumulate the per-step landing error into stance_world_.
    WorldPose held_pose_(int idx) const
    {
        const int N = (int)global_plan_.size();
        const bool same_foot_as_last = (((idx - (N - 1)) % 2) == 0);
        if (!same_foot_as_last && N < 2) return global_init_stance_;
        const GlobalFootTarget& t = global_plan_[same_foot_as_last ? N - 1 : N - 2];
        return WorldPose{t.x, t.y, t.z, t.yaw};
    }

    void fill_global_buffer_()
    {
        const int LA = cfg_.future_foot_step_num;
        const int N = (int)global_plan_.size();
        for (int s = 0; s < LA; ++s)
        {
            const int idx = planner_index_ + s;
            phase_indicator_[s] = phase_for_(idx);

            WorldPose from;
            if (s == 0) from = stance_world_;
            else if (idx - 1 < N) from = plan_pose_(idx - 1);
            else from = held_pose_(idx - 1);

            if (idx < N)
            {
                foot_command_[s] = build_global_step_(from, global_plan_[idx]);
                com_z_command_[s] = global_plan_[idx].com_z;
            }
            else
            {
                // Past the plan end: station-keep on the final plan poses
                // (timing/height/com_z carried over from the last plan entry).
                GlobalFootTarget tgt = global_plan_.back();
                const WorldPose hp = held_pose_(idx);
                tgt.x = hp.x; tgt.y = hp.y; tgt.z = hp.z; tgt.yaw = hp.yaw;
                foot_command_[s] = build_global_step_(from, tgt);
                com_z_command_[s] = global_plan_.back().com_z;
            }
        }
        com_z_command_[LA] = (planner_index_ + LA - 1 < N)
                                 ? global_plan_[planner_index_ + LA - 1].com_z
                                 : global_plan_.back().com_z;
        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
    }

    void resample_command_global_()
    {
        walking_tick_ = 0;
        planner_index_ = 0;
        stance_world_ = global_init_stance_; // anchor of the global plan frame
        fill_global_buffer_();
    }

    void update_command_global_()
    {
        walking_tick_ = 0;
        accumulate_stance_world_();
        ++planner_index_; // past plan end, fill_global_buffer_ emits station-keeping steps
        fill_global_buffer_();
    }

    // --- vision command mode ---------------------------------------------
    // Convert the pending pelvis-frame targets to the accumulated world frame
    // and upsert them into the id-keyed memory. Uses this tick's stance-foot
    // state (call after update_link_states_()).
    void ingest_vision_targets_()
    {
        const math::Quat stance_yaw = math::yaw_quat(stance_foot_global_.quat);
        const float cw = std::cos(stance_world_.yaw);
        const float sw = std::sin(stance_world_.yaw);
        for (const auto& t : pending_vision_)
        {
            // pelvis frame -> gravity-aligned pelvis-anchored global frame
            const math::Vec3 pg = math::quat_apply(robot_quat_w_, t.pos);
            const math::Quat qg = math::quat_mul(robot_quat_w_, t.quat);
            // -> stance-foot (yaw) frame
            math::Vec3 p_st;
            math::Quat q_st;
            math::subtract_frame_transforms(stance_foot_global_.pos, stance_yaw,
                                            pg, qg, p_st, q_st);
            const float yaw_st = math::wrap_to_pi(math::euler_xyz_from_quat(q_st)[2]);
            // -> accumulated world frame
            WorldPose w;
            w.x = stance_world_.x + cw * p_st[0] - sw * p_st[1];
            w.y = stance_world_.y + sw * p_st[0] + cw * p_st[1];
            w.z = stance_world_.z + p_st[2];
            w.yaw = math::wrap_to_pi(stance_world_.yaw + yaw_st);
            vision_memory_[t.id] = VisionMemEntry{w, vision_now_};
        }
    }

    // Plan the command buffer from the target memory: slot 0 = nearest
    // feasible target on the upcoming swing-foot side, slot 1 = nearest
    // feasible target for the following (opposite) foot relative to slot 0.
    // Any slot without a feasible target falls back to a station-keeping step
    // (build_foot_command_ with the default operator input, step_x = 0).
    void fill_vision_buffer_()
    {
        const int LA = cfg_.future_foot_step_num;
        planned_ids_ = {-1, -1};

        // default: step in place
        for (int s = 0; s < LA; ++s) foot_command_[s] = build_foot_command_(s);
        for (int i = 0; i <= LA; ++i) com_z_command_[i] = input_.com_z;

        // purge expired memory
        for (auto it = vision_memory_.begin(); it != vision_memory_.end();)
        {
            if (vision_now_ - it->second.stamp > vcfg_.memory_s)
                it = vision_memory_.erase(it);
            else
                ++it;
        }

        // candidates in the current stance-foot frame
        struct Cand { int id; float x, y, z, yaw, d; };
        std::vector<Cand> cands;
        const float cw = std::cos(stance_world_.yaw);
        const float sw = std::sin(stance_world_.yaw);
        for (const auto& [id, e] : vision_memory_)
        {
            const float dx = e.pose.x - stance_world_.x;
            const float dy = e.pose.y - stance_world_.y;
            Cand cd;
            cd.id = id;
            cd.x = cw * dx + sw * dy;
            cd.y = -sw * dx + cw * dy;
            cd.z = e.pose.z - stance_world_.z;
            cd.yaw = math::wrap_to_pi(e.pose.yaw - stance_world_.yaw);
            cd.d = std::hypot(cd.x, cd.y);
            if (cd.d > vcfg_.max_range) continue;
            if (cd.x < vcfg_.min_forward) continue;
            if (cd.d < vcfg_.exclude_radius) continue; // under the stance foot
            if (std::hypot(cd.x - swing_foot_stance_pos_[0],
                           cd.y - swing_foot_stance_pos_[1]) < vcfg_.exclude_radius)
                continue; // under the swing foot
            cands.push_back(cd);
        }
        if (cands.empty())
        {
            time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
            return;
        }

        // slot 0: upcoming swing foot (phase 0 -> right swings, y must be
        // on the right (-) side of the stance foot; phase 1 mirrored)
        const bool right_swings = (phase_indicator_[0] == 0);
        const Cand* t0 = nullptr;
        for (const auto& cd : cands)
        {
            if (right_swings ? (cd.y > -vcfg_.side_margin)
                             : (cd.y < vcfg_.side_margin)) continue;
            if (!t0 || cd.d < t0->d) t0 = &cd;
        }
        if (!t0)
        {
            time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
            return;
        }

        auto timing = [&](std::array<float, 9>& fc) {
            fc[6] = clampf(input_.ssp_t, cfg_.foot_ssp_min, cfg_.foot_ssp_max);
            fc[7] = clampf(input_.dsp_t, cfg_.foot_dsp_min, cfg_.foot_dsp_max);
            fc[8] = clampf(input_.height, cfg_.foot_height_min, cfg_.foot_height_max);
        };

        planned_ids_[0] = t0->id;
        std::array<float, 9> fc0{};
        fc0[0] = t0->x; fc0[1] = t0->y; fc0[2] = t0->z;
        fc0[5] = t0->yaw;
        timing(fc0);
        foot_command_[0] = fc0;

        // slot 1: opposite foot, relative to the slot-0 landing frame
        if (LA > 1)
        {
            const float c0 = std::cos(t0->yaw);
            const float s0 = std::sin(t0->yaw);
            const Cand* t1 = nullptr;
            std::array<float, 9> fc1{};
            float best = std::numeric_limits<float>::max();
            for (const auto& cd : cands)
            {
                if (cd.id == t0->id) continue;
                const float rx = c0 * (cd.x - t0->x) + s0 * (cd.y - t0->y);
                const float ry = -s0 * (cd.x - t0->x) + c0 * (cd.y - t0->y);
                // the foot after the swing lands on the opposite side
                if (right_swings ? (ry < vcfg_.side_margin)
                                 : (ry > -vcfg_.side_margin)) continue;
                if (rx < vcfg_.min_forward) continue;
                const float dd = std::hypot(rx, ry);
                if (dd < vcfg_.exclude_radius || dd >= best) continue;
                best = dd;
                t1 = &cd;
                fc1.fill(0.0f);
                fc1[0] = rx; fc1[1] = ry; fc1[2] = cd.z - t0->z;
                fc1[5] = math::wrap_to_pi(cd.yaw - t0->yaw);
                timing(fc1);
            }
            if (t1)
            {
                planned_ids_[1] = t1->id;
                foot_command_[1] = fc1;
            }
            // else: keep the station-keeping step beside the slot-0 landing
        }

        time_left_ = foot_command_[0][6] + foot_command_[0][7] * 2.0f;
    }

    void resample_command_vision_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        stance_world_ = WorldPose{};
        vision_memory_.clear();
        phase_indicator_[0] = cfg_.start_phase_indicator;
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i + 1] = 1 - phase_indicator_[i];

        // Prime the FIRST step from a target already visible at entry. Ingestion
        // otherwise only happens inside compute() (i.e. during the first step),
        // so foot_command_[0] would be a station-keep step and the recognized
        // command would take effect only from the SECOND step. Compute the
        // current stance/swing state and world anchor here (reset() repeats
        // these harmlessly for a static pose), then ingest the pending frame
        // fed by State_Footstep just before reset() and plan the buffer now.
        update_link_states_();
        if (foot_state_source_ == FootStateSource::SIM_ODOM)
            init_stance_world_from_sim_();
        if (vision_pending_) { ingest_vision_targets_(); vision_pending_ = false; }
        fill_vision_buffer_();
        spdlog::info("[FootVision] first step primed: slot0={} slot1={} "
                     "(memory={}, cmd0: x={:.3f} y={:.3f} z={:.3f} yaw={:.3f})",
                     planned_ids_[0], planned_ids_[1], vision_memory_.size(),
                     foot_command_[0][0], foot_command_[0][1],
                     foot_command_[0][2], foot_command_[0][5]);
    }

    void update_command_vision_()
    {
        const int LA = cfg_.future_foot_step_num;
        walking_tick_ = 0;
        accumulate_stance_world_();
        for (int i = 0; i < LA - 1; ++i) phase_indicator_[i] = phase_indicator_[i + 1];
        phase_indicator_[LA - 1] = 1 - phase_indicator_[LA - 2];
        fill_vision_buffer_();
        spdlog::info("[FootVision] planned targets: slot0={} slot1={} "
                     "(memory={}, cmd0: x={:.3f} y={:.3f} z={:.3f} yaw={:.3f})",
                     planned_ids_[0], planned_ids_[1], vision_memory_.size(),
                     foot_command_[0][0], foot_command_[0][1],
                     foot_command_[0][2], foot_command_[0][5]);
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
            const float lift = std::max(0.0f, std::max(swing_foot_start_stance_pos_[2], swing_foot_end_stance_pos_[2])) + height;
            // const float lift = height;
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
    // landing error of the last completed footstep (stance frame): [x, y, yaw]
    math::Vec3 last_step_error_ = math::Vec3::Zero();
    float last_step_total_time_ = 0.0f;

    std::vector<std::array<float, 9>> foot_command_;
    std::vector<float> com_z_command_;
    std::vector<int> phase_indicator_;

    // vision command mode (ArUco footstep targets from the perception node)
    bool vision_mode_ = false;
    VisionConfig vcfg_;
    std::vector<VisionTargetPelvis> pending_vision_; // latest frame (pelvis)
    bool vision_pending_ = false;
    double vision_now_ = 0.0; // monotonic clock fed by State_Footstep
    struct VisionMemEntry { WorldPose pose; double stamp; };
    std::map<int, VisionMemEntry> vision_memory_; // id -> world pose memory
    std::array<int, 2> planned_ids_ = {-1, -1};   // slots 0/1 (-1: station-keep)

    // global command mode (absolute world-frame foot targets)
    bool global_mode_ = false;
    std::vector<GlobalFootTarget> global_plan_;
    WorldPose global_init_stance_; // world pose of the initial stance foot (plan anchor)
    int planner_index_ = 0;        // plan index of the step at buffer slot 0
    // Accumulated world pose of the current stance foot. Maintained in every
    // command mode (drives the planner in global mode; used by the world-frame
    // logging accessors in all modes). Anchored at the initial stance foot
    // (or global_init_stance_ in global mode).
    WorldPose stance_world_;

    FootStateSource foot_state_source_ = FootStateSource::FK_ODOMETRY;
    math::Vec3 base_pos_world_ = math::Vec3::Zero(); // pelvis world pos (sim_odom)

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
