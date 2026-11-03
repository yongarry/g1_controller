// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// FSM state that deploys the G1-2d footstep policy (G12DFootEnvCfg); a dedicated
// thread runs the policy at env.step_dt.

#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <string>
#include <memory>
#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/utils/kinematics.h"
#include "isaaclab/envs/mdp/commands/footstep_command.h"
#include "isaaclab/envs/mdp/commands/vision_foot_target_source.h"
#include "unitree/dds_wrapper/robots/go2/go2_sub.h"

class State_Footstep : public FSMState
{
public:
    State_Footstep(int state_mode, std::string state_string);

    void enter();
    void run();
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) policy_thread.join();
        // Opened once in the constructor, so keep it open for the next entry.
        if (log_file_.is_open()) log_file_.flush();
        if (eval_file_.is_open()) eval_file_.close();
    }

    // Static so the command-backed observation terms can read it.
    static isaaclab::FootstepCommand* command;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<isaaclab::Kinematics> kin_;
    std::unique_ptr<isaaclab::FootstepCommand> command_;
    std::unique_ptr<isaaclab::FootCommandSource> command_source_;
    // The node publishes camera-optical-frame poses; vision_cam_tf_ maps them to the pelvis (waist FK).
    isaaclab::VisionFootTargetSubscriber::SharedPtr vision_sub_;
    isaaclab::D435PelvisCamTransform vision_cam_tf_;

    // upper-body joints held at a pose (SDK indices + targets/gains)
    std::vector<int> upper_ids_;
    std::vector<float> upper_default_;
    std::vector<float> upper_kp_;
    std::vector<float> upper_kd_;

    // Per-goal stop: the interpolation runs in run() (1 kHz); the policy thread only
    // picks the pose and waits for upper_motion_done_.
    void update_upper_target();
    std::vector<std::vector<float>> stop_poses_; // per goal (may be shorter/empty)
    float stop_move_time_ = 1.5f;                // [s] upper-body motion time
    float stop_hold_time_ = 1.0f;                // [s] extra pause after it
    std::atomic<int> upper_pose_index_{-1};      // -1: upper_default_
    std::atomic<bool> upper_motion_done_{true};
    std::vector<float> upper_target_;
    std::vector<float> upper_from_;
    int upper_pose_active_ = -1;
    std::chrono::steady_clock::time_point upper_t0_;

    isaaclab::FootCommandInput default_input_;
    float joy_x_scale_ = 0.2f;
    float joy_y_scale_ = 0.1f;
    float joy_yaw_scale_ = 0.2f;

    std::thread policy_thread;
    std::atomic<bool> policy_thread_running{false};
    std::atomic<bool> policy_ready_{false}; // first env->step() done since enter()

    // Set by run() on the Y button; the policy thread stands still (standby)
    // until it flips, then resets the planner and starts walking.
    std::atomic<bool> walk_started_{false};

    bool use_sim_odom_ = false;   // base pos + FK foot
    bool use_mujoco_foot_ = false; // ankle-roll body xpos/yaw
    std::shared_ptr<unitree::robot::go2::subscription::SportModeState> odom_sub_;

    void open_log_file(const std::string& path);
    void write_log_row(const Eigen::VectorXf& q_meas);
    std::ofstream log_file_;
    bool log_enabled_ = false;
    long log_tick_ = 0;

    // A timestamped file is created on enter() so runs do not overwrite each other and
    // constructing this state at startup does not truncate a sibling MindYourStep eval file.
    void open_eval_file(const std::string& path);
    void write_eval_row();
    std::string eval_path_;
    std::ofstream eval_file_;
    bool eval_enabled_ = false;
};

REGISTER_FSM(State_Footstep)
