// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// FSM state that deploys the G1-2d footstep policy (G12DFootEnvCfg).
//
// Mirrors State_Mimic: it owns a ManagerBasedRLEnv (the policy + obs/action
// managers) and a FootstepCommand that reproduces OnlineFootCommand on-device.
// A dedicated thread runs the policy at env.step_dt; State_Footstep::command is
// a static pointer so the command-backed observation terms can read it.

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
        if (log_file_.is_open()) log_file_.close();
        if (eval_file_.is_open()) eval_file_.close();
    }

    // Accessed by the command-backed observation terms (joint_ik_target, phase,
    // foot_commands_2d) defined in State_Footstep.cpp.
    static isaaclab::FootstepCommand* command;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<isaaclab::Kinematics> kin_;
    std::unique_ptr<isaaclab::FootstepCommand> command_;
    // Selectable foot-command source (joystick by default, or csv) - see deploy.yaml.
    std::unique_ptr<isaaclab::FootCommandSource> command_source_;
    // ArUco footstep-target subscriber (command_source == "vision"). The node
    // publishes camera-optical-frame poses; vision_cam_tf_ converts them to
    // the pelvis frame with the controller's own joint state (waist FK).
    isaaclab::VisionFootTargetSubscriber::SharedPtr vision_sub_;
    isaaclab::D435PelvisCamTransform vision_cam_tf_;

    // upper-body joints held at a pose (SDK indices + targets/gains)
    std::vector<int> upper_ids_;
    std::vector<float> upper_default_;
    std::vector<float> upper_kp_;
    std::vector<float> upper_kd_;

    // --- per-goal stop (goal command source) --------------------------------
    // On reaching a goal the gait freezes (standby command), the upper body
    // interpolates to that goal's pose, and after `stop_hold_time_` the robot
    // walks on holding it. The last goal ends in the stopped state.
    // The interpolation runs in run() (1 kHz); the policy thread only picks the
    // pose and waits for upper_motion_done_.
    void update_upper_target();
    std::vector<std::vector<float>> stop_poses_; // per goal (may be shorter/empty)
    float stop_move_time_ = 1.5f;                // [s] upper-body motion time
    float stop_hold_time_ = 1.0f;                // [s] extra pause after it
    std::atomic<int> upper_pose_index_{-1};      // -1: upper_default_
    std::atomic<bool> upper_motion_done_{true};
    std::vector<float> upper_target_;            // what run() sends to lowcmd
    std::vector<float> upper_from_;              // motion start pose
    int upper_pose_active_ = -1;                 // pose run() is moving toward
    std::chrono::steady_clock::time_point upper_t0_;

    // joystick -> command mapping
    isaaclab::FootCommandInput default_input_;
    float joy_x_scale_ = 0.2f;
    float joy_y_scale_ = 0.1f;
    float joy_yaw_scale_ = 0.2f;

    std::thread policy_thread;
    bool policy_thread_running = false;

    // Set by run() on the Y button; the policy thread stands still (standby)
    // until it flips, then resets the planner and starts walking.
    std::atomic<bool> walk_started_{false};

    // sim_odom: MuJoCo ground-truth base position from rt/odommodestate
    bool use_sim_odom_ = false;
    std::shared_ptr<unitree::robot::go2::subscription::SportModeState> odom_sub_;

    // --- data logging (mirrors the tocabi cc.cpp writeFile columns) ----------
    void open_log_file(const std::string& path);
    void write_log_row(const Eigen::VectorXf& q_meas);
    std::ofstream log_file_;
    bool log_enabled_ = false;
    long log_tick_ = 0;

    // --- footstep tracking evaluation (one CSV row per completed step) -------
    // Path template from deploy.yaml (e.g. log/footstep_eval.csv). A timestamped
    // file is created on enter() so runs do not overwrite each other, and so
    // constructing this state at controller start does not truncate a sibling
    // MindYourStep eval file.
    void open_eval_file(const std::string& path);
    void write_eval_row();
    std::string eval_path_;
    std::ofstream eval_file_;
    bool eval_enabled_ = false;
};

REGISTER_FSM(State_Footstep)
