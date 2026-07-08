#include "State_Footstep.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

#include <chrono>
#include <cmath>
#include <map>
#include <iomanip>

isaaclab::FootstepCommand* State_Footstep::command = nullptr;

// ---------------------------------------------------------------------------
// Command-backed observation terms (read State_Footstep::command).
// The 23-dim command is [ik_target(12), phase_cos, phase_sin, foot_command0(9)].
// ---------------------------------------------------------------------------
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

// foot_commands_2d = [x, y, yaw, ssp_t, dsp_t, height]
REGISTER_OBSERVATION(foot_commands_2d)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>{c[14], c[15], c[19], c[20], c[21], c[22]};
}

REGISTER_OBSERVATION(foot_commands_3d)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>{c.begin() + 14, c.end()};
}

} // namespace mdp
} // namespace isaaclab

// ---------------------------------------------------------------------------
// Full G1 29-dof joint name -> Unitree SDK motor index map (for FK).
// ---------------------------------------------------------------------------
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

State_Footstep::State_Footstep(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    YAML::Node deploy = YAML::LoadFile((policy_dir / "params" / "deploy.yaml").string());
    YAML::Node fs = deploy["footstep"];
    if (!fs) throw std::runtime_error("State_Footstep: deploy.yaml missing 'footstep' section.");

    // ---- kinematics (Pinocchio) ----
    std::filesystem::path urdf_path = fs["urdf_path"].as<std::string>();
    if (!urdf_path.is_absolute()) urdf_path = param::proj_dir / urdf_path;

    auto lleg = fs["lleg_joints"].as<std::vector<std::string>>();
    auto rleg = fs["rleg_joints"].as<std::vector<std::string>>();
    kin_ = std::make_shared<isaaclab::Kinematics>(
        urdf_path.string(), fs["base_link"].as<std::string>(),
        lleg, rleg,
        fs["lee_name"].as<std::string>(), fs["ree_name"].as<std::string>(),
        g1_name_to_sdk());

    // ---- command generator config ----
    isaaclab::FootstepCommand::Config fcfg;
    fcfg.future_foot_step_num = fs["future_foot_step_num"].as<int>(2);
    fcfg.step_dt = deploy["step_dt"].as<float>();
    fcfg.vrp_height = fs["vrp_height"].as<float>();
    fcfg.pelv_com_offset = fs["pelv_com_offset"].as<float>();
    fcfg.vrp_horizon_length = fs["vrp_horizon_length"].as<float>(5.0f);
    fcfg.preview_horizon_length = fs["preview_horizon_length"].as<float>(2.0f);
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
        pair("foot_ssp_time", fcfg.foot_ssp_min, fcfg.foot_ssp_max);
        pair("foot_dsp_time", fcfg.foot_dsp_min, fcfg.foot_dsp_max);
        pair("foot_height", fcfg.foot_height_min, fcfg.foot_height_max);
    }
    const int default_start_phase = fs["start_phase_indicator"].as<int>(0);

    // default operator input + joystick scaling
    if (fs["default_input"])
    {
        auto di = fs["default_input"];
        default_input_.step_y = yaml_get(di, "step_y", default_input_.step_y);
        default_input_.ssp_t = yaml_get(di, "ssp_t", default_input_.ssp_t);
        default_input_.dsp_t = yaml_get(di, "dsp_t", default_input_.dsp_t);
        default_input_.height = yaml_get(di, "height", default_input_.height);
        default_input_.com_z = yaml_get(di, "com_z", default_input_.com_z);
    }
    if (fs["joystick"])
    {
        auto js = fs["joystick"];
        joy_x_scale_ = yaml_get(js, "x_scale", joy_x_scale_);
        joy_y_scale_ = yaml_get(js, "y_scale", joy_y_scale_);
        joy_yaw_scale_ = yaml_get(js, "yaw_scale", joy_yaw_scale_);
    }

    // ---- foot world-position source ----
    // fk_odometry: FK landing accumulation (default, hardware)
    // sim_odom:    MuJoCo rt/odommodestate base truth + FK foot offset
    {
        std::string foot_src = fs["foot_state_source"] ? fs["foot_state_source"].as<std::string>()
                                                       : std::string("fk_odometry");
        std::transform(foot_src.begin(), foot_src.end(), foot_src.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        use_sim_odom_ = (foot_src == "sim_odom");
        if (use_sim_odom_)
        {
            std::string topic = fs["sim_odom_topic"] ? fs["sim_odom_topic"].as<std::string>()
                                                     : std::string("rt/odommodestate");
            odom_sub_ = std::make_shared<unitree::robot::go2::subscription::SportModeState>(topic);
            odom_sub_->wait_for_connection();
            spdlog::info("[Footstep] foot_state_source = sim_odom (topic={})", topic);
        }
        else
        {
            if (foot_src != "fk_odometry")
                spdlog::warn("[Footstep] unknown foot_state_source '{}', using fk_odometry", foot_src);
            spdlog::info("[Footstep] foot_state_source = fk_odometry");
        }
    }

    // ---- foot-command source ----
    // "csv_global": follow absolute world-frame foot targets (footcommands_global.csv);
    // the planner recomputes the local command from the accumulated stance foot each step.
    // Otherwise: per-tick local input from joystick (default) or csv.
    std::string src = fs["command_source"] ? fs["command_source"].as<std::string>() : "joystick";
    std::transform(src.begin(), src.end(), src.begin(), [](unsigned char c){ return std::tolower(c); });

    if (src == "csv_global")
    {
        std::filesystem::path csv = fs["global_csv_path"] ? fs["global_csv_path"].as<std::string>()
                                                          : std::string("config/footcommands_global.csv");
        if (csv.is_relative()) csv = param::proj_dir / csv;

        isaaclab::GlobalFootTarget gdef;
        gdef.ssp_t = default_input_.ssp_t;
        gdef.dsp_t = default_input_.dsp_t;
        gdef.height = default_input_.height;
        gdef.com_z = default_input_.com_z;
        auto plan = isaaclab::load_global_foot_plan(csv.string(), gdef);

        fcfg.start_phase_indicator = plan.front().phase;

        // Anchor of the global plan frame: world pose of the initial stance foot
        // (the foot NOT swinging first). Must match the frame the plan CSV was
        // generated in (cmd/convert_footcommand_2_global.py). Defaults to the G1
        // spawn-keyframe foot poses.
        auto foot_pose = [&](const std::string& key, std::array<float, 4> def) -> isaaclab::WorldPose {
            std::array<float, 4> v = def;
            if (fs[key]) { auto n = fs[key].as<std::vector<float>>(); for (int i = 0; i < 4 && i < (int)n.size(); ++i) v[i] = n[i]; }
            return isaaclab::WorldPose{v[0], v[1], v[2], v[3]};
        };
        const isaaclab::WorldPose lfoot = foot_pose("global_init_lfoot", {-0.02179f,  0.118506f, 0.0f, 0.0f});
        const isaaclab::WorldPose rfoot = foot_pose("global_init_rfoot", {-0.02179f, -0.118506f, 0.0f, 0.0f});
        // start phase 0 => right foot swings first => left foot is the initial stance.
        const isaaclab::WorldPose init_stance = (plan.front().phase == 0) ? lfoot : rfoot;

        command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
        command_->set_global_plan(std::move(plan), init_stance);
        State_Footstep::command = command_.get(); // visible to obs terms before env build
        spdlog::info("[FootCommand] command_source = csv_global (init stance x={:.4f} y={:.4f} z={:.4f} yaw={:.4f})",
                     init_stance.x, init_stance.y, init_stance.z, init_stance.yaw);
        // command_source_ stays null: the global plan drives the planner internally.
    }
    else if (src == "vision")
    {
        // ArUco footstep targets from cmd/aruco_footstep_perception.py.
        // The perception node publishes pelvis-frame target poses; the planner
        // keeps a world-frame memory and picks the two nearest feasible
        // targets at every step boundary (steps in place while none are seen).
        YAML::Node vn = fs["vision"];
        isaaclab::FootstepCommand::VisionConfig vc;
        std::string topic = "rt/footstep_vision";
        if (vn)
        {
            if (vn["topic"]) topic = vn["topic"].as<std::string>();
            vc.memory_s = yaml_get(vn, "memory", vc.memory_s);
            vc.max_range = yaml_get(vn, "max_range", vc.max_range);
            vc.min_forward = yaml_get(vn, "min_forward", vc.min_forward);
            vc.exclude_radius = yaml_get(vn, "exclude_radius", vc.exclude_radius);
            vc.side_margin = yaml_get(vn, "side_margin", vc.side_margin);
        }
        vision_sub_ = std::make_shared<isaaclab::VisionFootTargetSubscriber>(topic);

        fcfg.start_phase_indicator = default_start_phase;
        command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
        command_->enable_vision_mode(vc);
        State_Footstep::command = command_.get(); // visible to obs terms before env build
        command_->set_input(default_input_); // timing defaults / station-keep width
        spdlog::info("[FootCommand] command_source = vision (topic={}, memory={:.1f}s, "
                     "max_range={:.2f}m)", topic, vc.memory_s, vc.max_range);
    }
    else
    {
        command_source_ = isaaclab::make_foot_command_source(
            fs, param::proj_dir, &FSMState::lowstate->joystick,
            default_input_, joy_x_scale_, joy_y_scale_, joy_yaw_scale_, default_start_phase);
        // CSV may impose the starting swing foot; honor it so phase alternation aligns.
        fcfg.start_phase_indicator = command_source_->start_phase_indicator();

        command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
        State_Footstep::command = command_.get(); // visible to obs terms before env build
        command_->set_input(command_source_->input());
    }

    command_->set_foot_state_source(use_sim_odom_ ? isaaclab::FootStateSource::SIM_ODOM
                                                  : isaaclab::FootStateSource::FK_ODOMETRY);

    // ---- env (policy + managers) ----
    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
    env->alg = std::make_unique<isaaclab::OrtRunner>((policy_dir / "exported" / "policy.onnx").string());

    // ---- upper-body hold config ----
    YAML::Node ub = deploy["upper_body"];
    if (ub)
    {
        upper_ids_ = ub["joint_ids_map"].as<std::vector<int>>();
        upper_default_ = ub["default_joint_pos"].as<std::vector<float>>();
        upper_kp_ = ub["stiffness"].as<std::vector<float>>();
        upper_kd_ = ub["damping"].as<std::vector<float>>();
    }

    // ---- transitions ----
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")));

    // ---- data logging (optional) ----
    // Set `footstep: log_file: <path>` in deploy.yaml to enable. Relative paths
    // are resolved against the project dir. Defaults to "footstep_log.txt".
    std::string log_path = fs["log_file"] ? fs["log_file"].as<std::string>()
                                          : std::string("footstep_log.txt");
    if (!log_path.empty())
    {
        std::filesystem::path lp = log_path;
        if (lp.is_relative()) lp = param::proj_dir / lp;
        open_log_file(lp.string());
    }
}

void State_Footstep::open_log_file(const std::string& path)
{
    log_file_.open(path, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open())
    {
        spdlog::warn("[FootLog] could not open log file '{}': logging disabled.", path);
        return;
    }
    log_file_ << std::fixed << std::setprecision(6);
    // Column layout (tab separated), mirroring tocabi cc.cpp writeFile:
    log_file_ << "tick\twalking_tick"
                 "\tref_zmp_x\tref_zmp_y\tref_zmp_z"
                 "\ttarget_com_stance_x\ttarget_com_stance_y\ttarget_com_stance_z"
                 "\tcom_stance_x\tcom_stance_y\tcom_stance_z"
                 "\tcom_global_x\tcom_global_y\tcom_global_z"
                 "\tlfoot_x\tlfoot_y\tlfoot_z\trfoot_x\trfoot_y\trfoot_z"
                 "\ttarget_com_global_x\ttarget_com_global_y\ttarget_com_global_z"
                 "\tq_leg_desired[0..11]\tq_leg_meas[0..11]\n";
    log_enabled_ = true;
    spdlog::info("[FootLog] logging to '{}'.", path);
}

void State_Footstep::write_log_row(const Eigen::VectorXf& q_meas)
{
    if (!log_enabled_) return;
    const auto& c = *command_;
    const isaaclab::math::Vec3 ref_zmp = c.ref_zmp();
    const isaaclab::math::Vec3 tgt_com_st = c.target_com_stance();
    const isaaclab::math::Vec3 com_st = c.com_stance();
    const isaaclab::math::Vec3 com_gl = c.com_global();
    const isaaclab::math::Vec3 lfoot = c.left_foot_pos();
    const isaaclab::math::Vec3 rfoot = c.right_foot_pos();
    const isaaclab::math::Vec3 tgt_com_gl = c.target_com_global();
    const Eigen::VectorXf& q_des = c.target_joint_pos();

    log_file_ << log_tick_ << "\t" << c.walking_tick() << "\t";
    log_file_ << ref_zmp[0] << "\t" << ref_zmp[1] << "\t" << ref_zmp[2] << "\t";
    log_file_ << tgt_com_st[0] << "\t" << tgt_com_st[1] << "\t" << tgt_com_st[2] << "\t";
    log_file_ << com_st[0] << "\t" << com_st[1] << "\t" << com_st[2] << "\t";
    log_file_ << com_gl[0] << "\t" << com_gl[1] << "\t" << com_gl[2] << "\t";
    log_file_ << lfoot[0] << "\t" << lfoot[1] << "\t" << lfoot[2] << "\t" << rfoot[0] << "\t" << rfoot[1] << "\t" << rfoot[2] << "\t";
    log_file_ << tgt_com_gl[0] << "\t" << tgt_com_gl[1] << "\t" << tgt_com_gl[2] << "\t";
    for (int i = 0; i < q_des.size(); ++i) log_file_ << q_des(i) << "\t";
    for (int i = 0; i < 12 && i < q_meas.size(); ++i) log_file_ << q_meas(i) << "\t";
    log_file_ << "\n";
    ++log_tick_;
}

void State_Footstep::enter()
{
    // gains for the 12 lower-body (action) joints
    for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
    {
        int sdk = (int)env->robot->data.joint_ids_map[i];
        lowcmd->msg_.motor_cmd()[sdk].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[sdk].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[sdk].dq() = 0;
        lowcmd->msg_.motor_cmd()[sdk].tau() = 0;
    }
    // gains for held upper-body joints
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

        auto update_base_from_odom = [&]() {
            if (!use_sim_odom_ || !odom_sub_) return;
            std::lock_guard<std::mutex> lock(odom_sub_->mutex_);
            const auto& p = odom_sub_->msg_.position();
            command_->set_base_pos_world(isaaclab::math::Vec3(p[0], p[1], p[2]));
        };

        // vision mode: forward the latest perception frame (ingested inside
        // FootstepCommand::compute() with this tick's stance-foot state).
        auto feed_vision = [&]() {
            if (!vision_sub_) return;
            command_->set_vision_clock(std::chrono::duration<double>(
                clock::now().time_since_epoch()).count());
            std::vector<isaaclab::VisionTargetPelvis> ts;
            if (vision_sub_->take(ts)) command_->set_vision_targets(ts);
        };

        // initial reset
        env->robot->update();
        load_full_state(q, qd);
        kin_->set_state(q, qd);
        command_->robot_quat_w_ = env->robot->data.root_quat_w;
        update_base_from_odom();
        if (command_source_) command_->set_input(command_source_->input());
        feed_vision();
        command_->reset();
        env->reset();

        while (policy_thread_running)
        {
            env->robot->update();
            load_full_state(q, qd);
            kin_->set_state(q, qd);
            command_->robot_quat_w_ = env->robot->data.root_quat_w;
            update_base_from_odom();
            if (command_source_) command_->set_input(command_source_->input());
            feed_vision();
            command_->compute();
            // advance the (csv) command source when a footstep completes
            if (command_source_ && command_->step_completed()) command_source_->advance();
            // per-step landing error report (mirrors tocabi cc.cpp)
            if (command_->step_completed())
            {
                const auto& e = command_->last_step_error();
                spdlog::info("Foot Position error : {:.4f} [m]", std::sqrt(e[0]*e[0] + e[1]*e[1]));
                spdlog::info(">> X error : {:.4f} [m]", std::abs(e[0]));
                spdlog::info(">> Y error : {:.4f} [m]", std::abs(e[1]));
                spdlog::info("Foot Yaw error : {:.4f} [rad]", std::abs(e[2]));
                const auto& fc = command_->foot_command0();
                spdlog::info("Next foot step command : {:.4f} [m], {:.4f} [m], {:.4f} [rad]", fc[0], fc[1], fc[5]);
                // spdlog::info("t_total: {:.3f}", command_->last_step_total_time());
            }
            env->step();

            write_log_row(q);

            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}

void State_Footstep::run()
{
    auto action = env->action_manager->processed_actions(); // 12 lower joint targets
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
