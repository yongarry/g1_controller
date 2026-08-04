// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// FSM state that deploys the Isaac Lab G1 MindYourStep policy
// (G1MindYourStepFlatEnvCfg). Lower-body 12-DoF actions come from the ONNX
// actor; the upper body is held at default (mirroring LowerJointPositionAction
// in training). The 16-D foot-placement goal is produced on-device by
// MysGaitCommand (joystick or CSV replay); State_MindYourStep::command is a
// static pointer so the command-backed observation term can read it.

#pragma once

#include <fstream>
#include <memory>
#include <thread>
#include <vector>

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/envs/mdp/commands/mys_gait_command.h"
#include "isaaclab/utils/kinematics.h"

class State_MindYourStep : public FSMState
{
public:
    State_MindYourStep(int state_mode, std::string state_string);

    void enter();
    void run();
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) policy_thread.join();
        if (eval_file_.is_open()) eval_file_.close();
    }

    // Read by the mys_foot_command observation term in State_MindYourStep.cpp.
    static isaaclab::MysGaitCommand* command;

private:
    void open_eval_file(const std::string& path);
    void write_eval_row(float meas_x, float meas_y, float meas_yaw);

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::unique_ptr<isaaclab::MysGaitCommand> gait_;
    std::shared_ptr<isaaclab::Kinematics> kin_; // FK for per-step landing eval

    // upper-body joints held at default (SDK indices + targets/gains)
    std::vector<int> upper_ids_;
    std::vector<float> upper_default_;
    std::vector<float> upper_kp_;
    std::vector<float> upper_kd_;

    // Path template (e.g. log/footstep_eval_mys.csv); stamped file opened on enter().
    std::string eval_path_;
    std::ofstream eval_file_;
    bool eval_enabled_ = false;

    std::thread policy_thread;
    bool policy_thread_running = false;
};

REGISTER_FSM(State_MindYourStep)
