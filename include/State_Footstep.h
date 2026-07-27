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

    // upper-body joints held at default (SDK indices + targets/gains)
    std::vector<int> upper_ids_;
    std::vector<float> upper_default_;
    std::vector<float> upper_kp_;
    std::vector<float> upper_kd_;

    // joystick -> command mapping
    isaaclab::FootCommandInput default_input_;
    float joy_x_scale_ = 0.2f;
    float joy_y_scale_ = 0.1f;
    float joy_yaw_scale_ = 0.2f;

    std::thread policy_thread;
    bool policy_thread_running = false;

    // sim_odom: MuJoCo ground-truth base position from rt/odommodestate
    bool use_sim_odom_ = false;
    std::shared_ptr<unitree::robot::go2::subscription::SportModeState> odom_sub_;

    // --- data logging (mirrors the tocabi cc.cpp writeFile columns) ----------
    void open_log_file(const std::string& path);
    void write_log_row(const Eigen::VectorXf& q_meas);
    std::ofstream log_file_;
    bool log_enabled_ = false;
    long log_tick_ = 0;
};

REGISTER_FSM(State_Footstep)
