#include "State_Footstep.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

isaaclab::FootstepCommand* State_Footstep::command = nullptr;

// command(): [IK joint target (12), phase cos, sin, footstep x, y, z, roll, pitch, yaw, ssp, dsp, height, com z]
namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(joint_ik_target)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>(c.begin(), c.begin() + 12);
}

REGISTER_OBSERVATION(phase)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>{c[12], c[13]};
}

// yonglab pose_only_command: the footstep pose only (x, y, z, roll, pitch, yaw)
REGISTER_OBSERVATION(foot_commands_pose)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>(c.begin() + 14, c.begin() + 20);
}

} // namespace mdp
} // namespace isaaclab

using isaaclab::g1_name_to_sdk;

static float yaml_get(const YAML::Node& n, const std::string& key, float def)
{
    return n && n[key] ? n[key].as<float>() : def;
}

State_Footstep::State_Footstep(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    YAML::Node deploy = param::load_yaml_with_extends(policy_dir / "params" / "deploy.yaml");
    YAML::Node fs = deploy["footstep"];
    if (!fs) throw std::runtime_error("State_Footstep: deploy.yaml missing 'footstep' section.");

    std::filesystem::path urdf_path = fs["urdf_path"].as<std::string>();
    if (!urdf_path.is_absolute()) urdf_path = param::proj_dir / urdf_path;
    kin_ = std::make_shared<isaaclab::Kinematics>(
        urdf_path.string(), fs["base_link"].as<std::string>(),
        fs["lleg_joints"].as<std::vector<std::string>>(), fs["rleg_joints"].as<std::vector<std::string>>(),
        fs["lee_name"].as<std::string>(), fs["ree_name"].as<std::string>(), g1_name_to_sdk());

    isaaclab::FootstepCommand::Config fcfg = isaaclab::footstep_config_from_yaml(deploy);
    fcfg.start_phase_indicator = fs["start_phase_indicator"].as<int>(0);
    command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
    State_Footstep::command = command_.get(); // visible to obs terms before env build

    // stage1: nominal footsteps (no network, no depth); stage2: the planner
    const std::string mode = fs["mode"] ? fs["mode"].as<std::string>() : std::string("stage2");
    if (mode != "stage1" && mode != "stage2")
        throw std::runtime_error("footstep.mode must be 'stage1' or 'stage2', got: " + mode);
    stage2_ = (mode == "stage2");
    YAML::Node pn = fs["planner"];
    planner_cfg_ = isaaclab::planner_config_from_yaml(pn);
    std::string onnx;
    if (stage2_)
    {
        if (!pn || !pn["planner_file"]) throw std::runtime_error("footstep.mode stage2 needs footstep.planner.planner_file");
        onnx = (policy_dir / "exported" / pn["planner_file"].as<std::string>()).string();
        YAML::Node dn = pn["depth"];
        const std::string topic = dn && dn["topic"] ? dn["topic"].as<std::string>() : std::string("rt/depth");
        depth_sub_ = std::make_unique<isaaclab::DepthSubscriber>(topic, (uint32_t)(yaml_get(dn, "max_age", 0.2f) * 1000.0f));
        spdlog::info("[Footstep] stage2: planner '{}', depth topic {}", onnx, topic);
    }
    else
        spdlog::info("[Footstep] stage1: nominal footsteps from the joystick, z = 0 (flat ground only)");
    command_->enable_planner_mode(planner_cfg_, onnx);

    // drawn by the simulator: yellow = nominal footstep, green = the planner's pick, blue tiles = its score map
    auto make_pub = [](const char* topic) {
        auto p = std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(topic);
        p->InitChannel();
        return p;
    };
    nominal_pub_ = make_pub("rt/footstep_nominal");
    if (stage2_)
    {
        pick_pub_ = make_pub("rt/footstep_pick");
        score_pub_ = make_pub("rt/footstep_scores");
    }

    // fk_odometry: foot world poses dead-reckoned from FK landings, heading from the IMU (the robot).
    // sim_odom: MuJoCo's true base position (rt/odommodestate) + FK foot offset.
    const std::string foot_src = fs["foot_state_source"] ? fs["foot_state_source"].as<std::string>() : std::string("fk_odometry");
    if (foot_src == "sim_odom")
    {
        const std::string topic = fs["sim_odom_topic"] ? fs["sim_odom_topic"].as<std::string>() : std::string("rt/odommodestate");
        odom_sub_ = std::make_shared<unitree::robot::go2::subscription::SportModeState>(topic);
        odom_sub_->wait_for_connection();
        command_->set_foot_state_source(isaaclab::FootStateSource::SIM_ODOM);
    }
    else if (foot_src == "fk_odometry")
        command_->set_foot_state_source(isaaclab::FootStateSource::FK_ODOMETRY);
    else
        throw std::runtime_error("footstep.foot_state_source must be 'fk_odometry' or 'sim_odom', got: " + foot_src);
    spdlog::info("[Footstep] foot_state_source = {}", foot_src);

    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
    const auto onnx_path = policy_dir / "exported" / deploy["policy_file"].as<std::string>("policy.onnx");
    spdlog::info("[Footstep] loading policy '{}'", onnx_path.string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(onnx_path.string());

    YAML::Node ub = deploy["upper_body"];
    upper_ids_ = ub["joint_ids_map"].as<std::vector<int>>();
    upper_default_ = ub["default_joint_pos"].as<std::vector<float>>();
    upper_kp_ = ub["stiffness"].as<std::vector<float>>();
    upper_kd_ = ub["damping"].as<std::vector<float>>();
    default_q_.assign(29, 0.0f);
    for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
        default_q_[(int)env->robot->data.joint_ids_map[i]] = env->robot->data.default_joint_pos[i];
    for (size_t j = 0; j < upper_ids_.size(); ++j) default_q_[upper_ids_[j]] = upper_default_[j];

    this->registered_checks.emplace_back(std::make_pair(
        [&]() -> bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); }, FSMStringMap.right.at("Passive")));

    const std::string eval_path = fs["eval_file"] ? fs["eval_file"].as<std::string>() : std::string("log/footstep_eval.csv");
    if (!eval_path.empty())
    {
        std::filesystem::path ep = eval_path;
        if (ep.is_relative()) ep = param::proj_dir / ep;
        eval_path_ = ep.string();
    }
}

// log/footstep_eval.csv -> log/footstep_eval_YYMMDD_HHMMSS.csv; rows in the stance-foot frame
void State_Footstep::open_eval_file()
{
    if (eval_file_.is_open()) eval_file_.close();
    if (eval_path_.empty()) return;
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tm);
    const std::filesystem::path p = eval_path_;
    const std::filesystem::path stamped = p.parent_path() / (p.stem().string() + "_" + buf + p.extension().string());
    std::filesystem::create_directories(stamped.parent_path());
    eval_file_.open(stamped.string(), std::ios::out | std::ios::trunc);
    if (!eval_file_.is_open())
    {
        spdlog::warn("[FootEval] could not open '{}'", stamped.string());
        return;
    }
    eval_file_ << std::fixed << std::setprecision(6)
               << "step,foot,cmd_x,cmd_y,cmd_z,cmd_yaw,meas_x,meas_y,meas_z,meas_yaw,err_x,err_y,err_z,err_yaw,ssp_t,dsp_t,height\n";
    spdlog::info("[FootEval] per-step landing error -> '{}'", stamped.string());
}

void State_Footstep::write_eval_row()
{
    if (!eval_file_.is_open()) return;
    const auto& c = *command_;
    const auto &cmd = c.last_step_command(), &meas = c.last_step_measured(), &err = c.last_step_error(), &tim = c.last_step_timing();
    eval_file_ << c.step_counter() << "," << (c.last_step_swing_right() ? "R" : "L") << ","
               << cmd[0] << "," << cmd[1] << "," << cmd[2] << "," << c.last_step_command_yaw() << ","
               << meas[0] << "," << meas[1] << "," << meas[2] << "," << c.last_step_measured_yaw() << ","
               << err[0] << "," << err[1] << "," << err[2] << "," << c.last_step_yaw_error() << ","
               << tim[0] << "," << tim[1] << "," << tim[2] << "\n";
    eval_file_.flush(); // a run is often ended with Ctrl+C
}

void State_Footstep::enter()
{
    // Stage1 and Stage2 are two instances of this state: the observation terms read the command of the active one
    State_Footstep::command = command_.get();
    walk_started_ = false; // stand still until Y / 'g'
    open_eval_file();

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

    command_->clear_standby(); // re-entry: re-solve the standby IK from the current pose
    policy_ready_ = false;
    policy_thread_running = true;
    policy_thread = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const auto dt = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(env->step_dt));
        auto sleepTill = clock::now() + dt;
        Eigen::VectorXf q(29), qd(29);

        auto read_state = [&]() {
            env->robot->update();
            {
                std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
                auto& motors = FSMState::lowstate->msg_.motor_state();
                for (int i = 0; i < 29; ++i) { q(i) = motors[i].q(); qd(i) = motors[i].dq(); }
            }
            kin_->set_state(q, qd);
            command_->robot_quat_w_ = env->robot->data.root_quat_w;
            if (odom_sub_)
            {
                std::lock_guard<std::mutex> lock(odom_sub_->mutex_);
                const auto& p = odom_sub_->msg_.position();
                command_->set_base_pos_world(isaaclab::math::Vec3(p[0], p[1], p[2]));
            }
        };

        // joystick velocity and planner proprioception: read before the footstep update of this tick
        auto feed_inputs = [&]() {
            float ly = 0.f, lx = 0.f, rx = 0.f;
            {
                std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
                ly = FSMState::lowstate->joystick.ly();
                lx = FSMState::lowstate->joystick.lx();
                rx = FSMState::lowstate->joystick.rx();
            }
            if (FSMState::keyboard) FSMState::keyboard->apply_stick_axes(ly, lx, rx);
            const auto& pc = planner_cfg_;
            command_->set_joystick_velocity(ly * (ly >= 0.f ? pc.vx_max : -pc.vx_min), -lx * pc.vy_max, -rx * pc.wz_max);
            std::array<float, 33> p;
            const auto& d = env->robot->data;
            for (int i = 0; i < 3; ++i) { p[i] = d.root_ang_vel_b[i]; p[3 + i] = d.projected_gravity_b[i]; }
            for (int i = 0; i < 12; ++i) { p[6 + i] = q(i) - default_q_[i]; p[18 + i] = qd(i); }
            for (int i = 0; i < 3; ++i) p[30 + i] = q(12 + i) - default_q_[12 + i];
            command_->set_planner_proprio(p);
        };

        // depth image of this tick (after the footstep update)
        long depth_missing = 0;
        auto observe_depth = [&]() {
            if (!depth_sub_) return;
            const bool ok = depth_sub_->latest(depth_, planner_cfg_.width, planner_cfg_.height);
            command_->planner_observe(ok ? depth_.data() : nullptr);
            if (!ok && (depth_missing++ % 250) == 0)
                spdlog::warn("[FootPlanner] no fresh {}x{} depth image: the planner sees nothing new", planner_cfg_.width,
                             planner_cfg_.height);
        };

        // markers for the simulator, after every new footstep
        int marker_seq = command_->plan_seq();
        auto publish_markers = [&]() {
            if (command_->plan_seq() == marker_seq) return;
            marker_seq = command_->plan_seq();
            const auto& mc = command_->marker_config();
            auto write = [&](StringPub& pub, const std::string& payload) {
                std_msgs::msg::dds_::String_ msg;
                msg.data(payload);
                pub->Write(msg);
            };
            auto soles = [&](const std::vector<std::array<float, 4>>& v) {
                std::ostringstream os;
                os << mc.frame << ' ' << mc.sole_x << ' ' << mc.sole_y;
                for (const auto& p : v) os << ' ' << p[0] << ' ' << p[1] << ' ' << p[2] << ' ' << p[3];
                return os.str();
            };
            write(nominal_pub_, soles(command_->nominal_soles()));
            if (!stage2_) return;
            write(pick_pub_, soles(command_->picked_soles()));
            std::ostringstream os;
            os << mc.frame << ' ' << command_->cell_size() << ' ' << command_->score_tiles_yaw();
            for (const auto& t : command_->score_tiles()) os << ' ' << t[0] << ' ' << t[1] << ' ' << t[2] << ' ' << (int)t[3];
            write(score_pub_, os.str());
        };

        // Stand on a frozen-phase command until Y; command_->reset() then anchors on the state the robot is in.
        read_state();
        feed_inputs();
        command_->hold_standby(0.0f);
        env->reset();
        bool walking = false;

        while (policy_thread_running)
        {
            read_state();
            feed_inputs();
            if (!walking)
            {
                command_->hold_standby(0.0f);
                if (walk_started_)
                {
                    command_->reset();
                    // seed the observation history with the first walking frame (episodes start from a reset)
                    env->reset();
                    walking = true;
                    spdlog::info("[Footstep] walk start");
                }
            }
            if (walking)
            {
                command_->compute();
                if (command_->step_completed())
                {
                    write_eval_row();
                    const auto& cmd = command_->last_step_command();
                    const auto& e = command_->last_step_error();
                    spdlog::info("Executed foot command : x={:.4f} y={:.4f} z={:.4f} [m], yaw={:.4f} [rad] ({}), landing error {:.4f} m",
                                 cmd[0], cmd[1], cmd[2], command_->last_step_command_yaw(),
                                 command_->last_step_swing_right() ? "R" : "L", std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]));
                }
            }
            observe_depth();
            publish_markers();
            env->step();
            policy_ready_ = true;

            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}

void State_Footstep::run()
{
    // Y / 'g' starts walking (checked here: on_pressed is a single-tick edge of the 1 kHz FSM loop)
    if (FSMState::lowstate->joystick.Y.on_pressed
        || (FSMState::keyboard && FSMState::keyboard->on_pressed && FSMState::keyboard->key() == "g"))
        walk_started_ = true;

    // legs keep the previous state's targets until the policy has acted once
    if (policy_ready_)
    {
        auto action = env->action_manager->processed_actions();
        for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
            lowcmd->msg_.motor_cmd()[(int)env->robot->data.joint_ids_map[i]].q() = action[i];
    }
    for (int j = 0; j < (int)upper_ids_.size(); ++j)
        lowcmd->msg_.motor_cmd()[upper_ids_[j]].q() = upper_default_[j];
}
