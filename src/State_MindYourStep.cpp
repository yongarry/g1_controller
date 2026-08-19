#include "State_MindYourStep.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "isaaclab/utils/math_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <map>

isaaclab::MysGaitCommand* State_MindYourStep::command = nullptr;

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(mys_foot_command)
{
    const auto& c = State_MindYourStep::command->command();
    return std::vector<float>(c.begin(), c.end());
}

} // namespace mdp
} // namespace isaaclab

static std::map<std::string, int> g1_name_to_sdk()
{
    return {
        {"left_hip_pitch_joint", 0}, {"left_hip_roll_joint", 1}, {"left_hip_yaw_joint", 2},
        {"left_knee_joint", 3}, {"left_ankle_pitch_joint", 4}, {"left_ankle_roll_joint", 5},
        {"right_hip_pitch_joint", 6}, {"right_hip_roll_joint", 7}, {"right_hip_yaw_joint", 8},
        {"right_knee_joint", 9}, {"right_ankle_pitch_joint", 10}, {"right_ankle_roll_joint", 11},
        {"waist_yaw_joint", 12}, {"waist_roll_joint", 13}, {"waist_pitch_joint", 14},
        {"left_shoulder_pitch_joint", 15}, {"left_shoulder_roll_joint", 16}, {"left_shoulder_yaw_joint", 17},
        {"left_elbow_joint", 18}, {"left_wrist_roll_joint", 19}, {"left_wrist_pitch_joint", 20},
        {"left_wrist_yaw_joint", 21},
        {"right_shoulder_pitch_joint", 22}, {"right_shoulder_roll_joint", 23}, {"right_shoulder_yaw_joint", 24},
        {"right_elbow_joint", 25}, {"right_wrist_roll_joint", 26}, {"right_wrist_pitch_joint", 27},
        {"right_wrist_yaw_joint", 28},
    };
}

static float yaml_get(const YAML::Node& n, const std::string& key, float def)
{
    return n[key] ? n[key].as<float>() : def;
}

State_MindYourStep::State_MindYourStep(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    YAML::Node deploy = YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string());
    YAML::Node g = deploy["gait"] ? deploy["gait"] : YAML::Node();

    // ---- gait command generator ----
    isaaclab::MysGaitCommand::Config gcfg;
    gcfg.policy_dt = deploy["step_dt"].as<double>();
    if (g)
    {
        gcfg.feet_distance = yaml_get(g, "feet_distance", gcfg.feet_distance);
        if (g["gait_frequency"]) gcfg.gait_frequency = g["gait_frequency"].as<double>();
        gcfg.stop_steps = g["stop_steps"] ? g["stop_steps"].as<int>() : gcfg.stop_steps;
        gcfg.vert_max = yaml_get(g, "vert_max", gcfg.vert_max);
        gcfg.lat_max = yaml_get(g, "lat_max", gcfg.lat_max);
        gcfg.yaw_max = yaml_get(g, "yaw_max", gcfg.yaw_max);
        gcfg.deadzone = yaml_get(g, "deadzone", gcfg.deadzone);
    }
    gait_ = std::make_unique<isaaclab::MysGaitCommand>(gcfg);
    State_MindYourStep::command = gait_.get();

    // ---- CSV command source (optional) ----
    std::string src = (g && g["command_source"]) ? g["command_source"].as<std::string>()
                                                 : std::string("joystick");
    std::transform(src.begin(), src.end(), src.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    if (src == "csv")
    {
        std::filesystem::path csv = (g && g["csv_path"])
            ? g["csv_path"].as<std::string>()
            : std::string("config/footcommands_mys.csv");
        if (csv.is_relative()) csv = param::proj_dir / csv;
        gait_->load_csv(csv.string());
    }
    else if (src != "joystick")
    {
        spdlog::warn("[MindYourStep] unknown gait.command_source '{}', using joystick", src);
    }

    // ---- FK for per-step landing evaluation (csv mode) ----
    if (gait_->csv_mode())
    {
        std::filesystem::path urdf = (g && g["urdf_path"])
            ? g["urdf_path"].as<std::string>()
            : std::string("config/urdf/g1_29dof.urdf");
        if (urdf.is_relative()) urdf = param::proj_dir / urdf;

        auto lleg = (g && g["lleg_joints"])
            ? g["lleg_joints"].as<std::vector<std::string>>()
            : std::vector<std::string>{
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"};
        auto rleg = (g && g["rleg_joints"])
            ? g["rleg_joints"].as<std::vector<std::string>>()
            : std::vector<std::string>{
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
        std::string lee = (g && g["lee_name"]) ? g["lee_name"].as<std::string>()
                                               : "left_ankle_roll_link";
        std::string ree = (g && g["ree_name"]) ? g["ree_name"].as<std::string>()
                                               : "right_ankle_roll_link";
        std::string base = (g && g["base_link"]) ? g["base_link"].as<std::string>() : "pelvis";

        kin_ = std::make_shared<isaaclab::Kinematics>(
            urdf.string(), base, lleg, rleg, lee, ree, g1_name_to_sdk());

        // Opened on enter() with a timestamp suffix (not at construction).
        std::string eval_path = (g && g["eval_file"])
            ? g["eval_file"].as<std::string>()
            : std::string("log/footstep_eval_mys.csv");
        if (!eval_path.empty())
        {
            std::filesystem::path ep = eval_path;
            if (ep.is_relative()) ep = param::proj_dir / ep;
            eval_path_ = ep.string();
        }
    }

    // ---- env (policy + managers); joint_ids_map is the 12 leg joints ----
    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
    const std::string policy_file = deploy["policy_file"]
        ? deploy["policy_file"].as<std::string>()
        : std::string("policy.onnx");
    const auto onnx_path = policy_dir / "exported" / policy_file;
    spdlog::info("[MindYourStep] loading policy '{}'", onnx_path.string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(onnx_path.string());

    // ---- upper body held at default ----
    YAML::Node ub = deploy["upper_body"];
    if (ub)
    {
        upper_ids_ = ub["joint_ids_map"].as<std::vector<int>>();
        upper_default_ = ub["default_joint_pos"].as<std::vector<float>>();
        upper_kp_ = ub["stiffness"].as<std::vector<float>>();
        upper_kd_ = ub["damping"].as<std::vector<float>>();
        if (upper_default_.size() != upper_ids_.size() ||
            upper_kp_.size() != upper_ids_.size() ||
            upper_kd_.size() != upper_ids_.size())
        {
            throw std::runtime_error(
                "State_MindYourStep: upper_body arrays must all match joint_ids_map size.");
        }
    }

    spdlog::info("[MindYourStep] joints={} / act={} ({} held at default); "
                 "command_source={}; gait_freq={:.2f} Hz",
                 env->robot->data.joint_ids_map.size(),
                 env->action_manager->total_action_dim(), upper_ids_.size(),
                 gait_->csv_mode() ? "csv" : "joystick", gcfg.gait_frequency);

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")));
}

static std::filesystem::path stamp_eval_path_mys_(const std::filesystem::path& template_path)
{
    // log/footstep_eval_mys.csv -> log/footstep_eval_mys_YYMMDD_HHMMSS.csv
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tm);
    return template_path.parent_path() /
           (template_path.stem().string() + "_" + buf + template_path.extension().string());
}

void State_MindYourStep::open_eval_file(const std::string& path)
{
    if (eval_file_.is_open()) eval_file_.close();
    eval_enabled_ = false;

    std::filesystem::path stamped = stamp_eval_path_mys_(path);
    if (!stamped.parent_path().empty())
        std::filesystem::create_directories(stamped.parent_path());

    eval_file_.open(stamped.string(), std::ios::out | std::ios::trunc);
    if (!eval_file_.is_open())
    {
        spdlog::warn("[MysEval] could not open '{}': step evaluation disabled.", stamped.string());
        return;
    }
    eval_file_ << std::fixed << std::setprecision(6);
    // Same schema as State_Footstep so cmd/eval_footstep.py can pool both.
    // MindYourStep does not track landing z; write 0 for cmd/meas/err z.
    eval_file_ << "step,foot,"
                  "cmd_x,cmd_y,cmd_z,cmd_yaw,"
                  "meas_x,meas_y,meas_z,meas_yaw,"
                  "err_x,err_y,err_z,err_yaw,"
                  "ssp_t,dsp_t,height\n";
    eval_enabled_ = true;
    spdlog::info("[MysEval] per-step tracking error -> '{}'.", stamped.string());
}

void State_MindYourStep::write_eval_row(float meas_x, float meas_y, float meas_yaw)
{
    if (!eval_enabled_) return;
    const float cmd_x = gait_->last_cmd_x();
    const float cmd_y = gait_->last_cmd_y();
    const float cmd_yaw = gait_->last_cmd_yaw();
    const float err_x = cmd_x - meas_x;
    const float err_y = cmd_y - meas_y;
    const float err_yaw = isaaclab::math::wrap_to_pi(cmd_yaw - meas_yaw);

    eval_file_ << gait_->step_counter() << ","
               << (gait_->last_step_swing_right() ? "R" : "L") << ","
               << cmd_x << "," << cmd_y << "," << 0.0f << "," << cmd_yaw << ","
               << meas_x << "," << meas_y << "," << 0.0f << "," << meas_yaw << ","
               << err_x << "," << err_y << "," << 0.0f << "," << err_yaw << ","
               << gait_->last_ssp_t() << "," << gait_->last_dsp_t() << ","
               << gait_->last_height() << "\n";
    eval_file_.flush();
}

void State_MindYourStep::enter()
{
    if (!eval_path_.empty()) open_eval_file(eval_path_);

    for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
    {
        int sdk = (int)env->robot->data.joint_ids_map[i];
        lowcmd->msg_.motor_cmd()[sdk].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[sdk].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[sdk].dq() = 0;
        lowcmd->msg_.motor_cmd()[sdk].tau() = 0;
    }
    for (int j = 0; j < (int)upper_ids_.size(); ++j)
    {
        int sdk = upper_ids_[j];
        lowcmd->msg_.motor_cmd()[sdk].kp() = upper_kp_[j];
        lowcmd->msg_.motor_cmd()[sdk].kd() = upper_kd_[j];
        lowcmd->msg_.motor_cmd()[sdk].dq() = 0;
        lowcmd->msg_.motor_cmd()[sdk].tau() = 0;
    }

    policy_thread_running = true;
    policy_thread = std::thread([this]{
        using clock = std::chrono::high_resolution_clock;
        const std::chrono::duration<double> desiredDuration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);
        auto sleepTill = clock::now() + dt;

        auto load_full_state = [&](Eigen::VectorXf& q, Eigen::VectorXf& qd) {
            std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
            auto& motors = FSMState::lowstate->msg_.motor_state();
            for (int i = 0; i < 29; ++i) { q(i) = motors[i].q(); qd(i) = motors[i].dq(); }
        };

        Eigen::VectorXf q(29), qd(29);

        auto read_sticks = [this]{
            if (gait_->csv_mode()) return;
            auto& joy = FSMState::lowstate->joystick;
            gait_->set_input(joy.ly(), -joy.lx(), -joy.rx());
        };

        // Measure swing foot in the stance-foot yaw frame (same frame as cmd_*).
        auto measure_landing = [&](int swing /*0 L, 1 R*/,
                                   float& mx, float& my, float& myaw) {
            using Side = isaaclab::Kinematics::Side;
            const Side sw = (swing == 0) ? Side::LEFT : Side::RIGHT;
            const Side st = (swing == 0) ? Side::RIGHT : Side::LEFT;
            const auto st_pos = kin_->foot_pos(st);
            const auto st_yaw_q = isaaclab::math::yaw_quat(kin_->foot_quat(st));
            const auto sw_pos = kin_->foot_pos(sw);
            const auto sw_quat = kin_->foot_quat(sw);
            isaaclab::math::Vec3 t12;
            isaaclab::math::Quat q12;
            isaaclab::math::subtract_frame_transforms(
                st_pos, st_yaw_q, sw_pos, sw_quat, t12, q12);
            mx = t12[0];
            my = t12[1];
            myaw = isaaclab::math::wrap_to_pi(
                isaaclab::math::euler_xyz_from_quat(q12)[2]);
        };

        env->robot->update();
        load_full_state(q, qd);
        if (kin_) kin_->set_state(q, qd);
        read_sticks();
        gait_->reset();
        env->reset();

        auto last_dir = gait_->move_dir();

        while (policy_thread_running)
        {
            env->robot->update();
            load_full_state(q, qd);
            if (kin_) kin_->set_state(q, qd);
            read_sticks();
            gait_->compute();

            if (gait_->step_completed())
            {
                float mx = 0, my = 0, myaw = 0;
                if (kin_) measure_landing(gait_->last_step_swing_right() ? 1 : 0, mx, my, myaw);
                write_eval_row(mx, my, myaw);
                const float ex = gait_->last_cmd_x() - mx;
                const float ey = gait_->last_cmd_y() - my;
                spdlog::info("[MysEval] step {} foot={} |xy|={:.4f} m  "
                             "(ex={:.4f}, ey={:.4f}, eyaw={:.4f})",
                             gait_->step_counter(),
                             gait_->last_step_swing_right() ? "R" : "L",
                             std::sqrt(ex * ex + ey * ey),
                             std::abs(ex), std::abs(ey),
                             std::abs(isaaclab::math::wrap_to_pi(
                                 gait_->last_cmd_yaw() - myaw)));
            }

            env->step();

            if (!gait_->csv_mode() && gait_->move_dir() != last_dir)
            {
                last_dir = gait_->move_dir();
                spdlog::info("[MindYourStep] move: {}", isaaclab::MysGaitCommand::move_name(last_dir));
            }

            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}

void State_MindYourStep::run()
{
    auto action = env->action_manager->processed_actions();
    for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
    {
        int sdk = (int)env->robot->data.joint_ids_map[i];
        lowcmd->msg_.motor_cmd()[sdk].q() = action[i];
    }
    for (int j = 0; j < (int)upper_ids_.size(); ++j)
    {
        lowcmd->msg_.motor_cmd()[upper_ids_[j]].q() = upper_default_[j];
    }
}
