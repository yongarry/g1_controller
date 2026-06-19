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
#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/utils/kinematics.h"
#include "isaaclab/envs/mdp/commands/footstep_command.h"

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
};

REGISTER_FSM(State_Footstep)
