// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// FSM state that deploys the yonglab G1-3d-Joystick tracker (RISE) with joystick footsteps. deploy.yaml
// `footstep.mode`:
//   stage1 - nominal footsteps from the joystick velocity, z = 0 (blind; flat ground)
//   stage2 - the footstep planner picks each footstep from depth (planner.onnx)
// A dedicated thread runs the policy at env.step_dt.

#pragma once

#include <atomic>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/commands/depth_source.h"
#include "isaaclab/envs/mdp/commands/footstep_command.h"
#include "isaaclab/utils/kinematics.h"
#include "unitree/dds_wrapper/robots/go2/go2_sub.h"
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

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
        if (eval_file_.is_open()) eval_file_.close();
    }

    // Static so the command-backed observation terms can read it; set by the state that is entered.
    static isaaclab::FootstepCommand* command;

private:
    using StringPub = unitree::robot::ChannelPublisherPtr<std_msgs::msg::dds_::String_>;

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<isaaclab::Kinematics> kin_;
    std::unique_ptr<isaaclab::FootstepCommand> command_;
    isaaclab::FootstepPlannerConfig planner_cfg_;
    bool stage2_ = false;

    // stage 2: depth image from DDS (unitree_mujoco DepthCamera / realsense_depth_pub)
    std::unique_ptr<isaaclab::DepthSubscriber> depth_sub_;
    std::vector<float> depth_;
    // drawn by unitree_mujoco: nominal footstep, the planner's pick and its score map
    StringPub nominal_pub_, pick_pub_, score_pub_;

    // upper body held at its default pose (SDK indices + targets / gains)
    std::vector<int> upper_ids_;
    std::vector<float> upper_default_, upper_kp_, upper_kd_;
    std::vector<float> default_q_; // all 29 joints, SDK order (planner proprioception)

    std::thread policy_thread;
    std::atomic<bool> policy_thread_running{false};
    std::atomic<bool> policy_ready_{false}; // first env->step() done since enter()
    // Set by run() on Y / 'g'; the policy thread stands still until then.
    std::atomic<bool> walk_started_{false};

    // MuJoCo only (foot_state_source: sim_odom): the true base position for the depth points
    std::shared_ptr<unitree::robot::go2::subscription::SportModeState> odom_sub_;

    // per-footstep landing error: log/footstep_eval_<time>.csv (opened on enter())
    void open_eval_file();
    void write_eval_row();
    std::string eval_path_;
    std::ofstream eval_file_;
};

REGISTER_FSM(State_Footstep)
