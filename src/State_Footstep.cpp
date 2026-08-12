#include "State_Footstep.h"
#include "LinearInterpolator.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <map>
#include <iomanip>

isaaclab::FootstepCommand* State_Footstep::command = nullptr;

// ---------------------------------------------------------------------------
// Command-backed observation terms (read State_Footstep::command).
// The 24-dim command is
//   [ik_target(12), phase_cos, phase_sin, foot_command0(9), com_z_command(1)].
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

// Alias of foot_commands (9-dim, no com_z) for existing deploy.yaml policies.
REGISTER_OBSERVATION(foot_commands_3d)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>(c.begin() + 14, c.begin() + 23);
}

// foot_commands_w_comz = foot_command0(9) + com_z(1)  [indices 14:24]
REGISTER_OBSERVATION(foot_commands_3d_w_comz)
{
    const auto& c = State_Footstep::command->command();
    return std::vector<float>(c.begin() + 14, c.begin() + 24);
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

// World pose of the initial stance foot (the foot NOT swinging first), read
// from the deploy.yaml global_init_lfoot / global_init_rfoot keys. It anchors
// the world frame of the csv_global plan and MUST match the frame the plan was
// authored in. Defaults to the G1 spawn-keyframe poses. (The goal command
// source does not use this: it anchors on the robot's own stance at walk start.)
static isaaclab::WorldPose init_stance_pose(const YAML::Node& fs, int start_phase)
{
    auto foot_pose = [&](const std::string& key, std::array<float, 4> def) {
        std::array<float, 4> v = def;
        if (fs[key])
        {
            auto n = fs[key].as<std::vector<float>>();
            for (int i = 0; i < 4 && i < (int)n.size(); ++i) v[i] = n[i];
        }
        return isaaclab::WorldPose{v[0], v[1], v[2], v[3]};
    };
    // start phase 0 => right foot swings first => left foot is the initial stance.
    return (start_phase == 0)
        ? foot_pose("global_init_lfoot", {-0.02179f,  0.118506f, 0.0f, 0.0f})
        : foot_pose("global_init_rfoot", {-0.02179f, -0.118506f, 0.0f, 0.0f});
}

State_Footstep::State_Footstep(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    YAML::Node deploy = param::load_yaml_with_extends(policy_dir / "params" / "deploy.yaml");
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
    fcfg.vrpx_offset = fs["vrpx_offset"].as<float>(0.03f);
    fcfg.vrpy_offset = fs["vrpy_offset"].as<float>(0.02f);
    fcfg.vrp_horizon_length = fs["vrp_horizon_length"].as<float>(5.0f);
    fcfg.preview_horizon_length = fs["preview_horizon_length"].as<float>(2.0f);
    // Ablation cases: must match the com_generate_type the policy was trained
    // with (G13DFootEnvCfg / _heuri1 / _heuri2).
    {
        const std::string cgt = fs["com_generate_type"].as<std::string>(std::string("prev"));
        if (cgt == "prev") fcfg.com_generate_type = isaaclab::ComGenerateType::PREV;
        else if (cgt == "heuri") fcfg.com_generate_type = isaaclab::ComGenerateType::HEURI;
        else if (cgt == "heuri2") fcfg.com_generate_type = isaaclab::ComGenerateType::HEURI2;
        else throw std::runtime_error("footstep.com_generate_type must be one of "
                                      "'prev', 'heuri', 'heuri2', got: " + cgt);
        spdlog::info("[Footstep] com_generate_type: {}", cgt);
    }
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
    // sim_odom:    MuJoCo rt/odommodestate base + FK foot offset
    // mujoco:      MuJoCo ankle-roll body xpos/yaw via odom.foot_position_body
    {
        std::string foot_src = fs["foot_state_source"] ? fs["foot_state_source"].as<std::string>()
                                                       : std::string("fk_odometry");
        std::transform(foot_src.begin(), foot_src.end(), foot_src.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        use_sim_odom_ = (foot_src == "sim_odom");
        use_mujoco_foot_ = (foot_src == "mujoco");
        if (use_sim_odom_ || use_mujoco_foot_)
        {
            std::string topic = fs["sim_odom_topic"] ? fs["sim_odom_topic"].as<std::string>()
                                                     : std::string("rt/odommodestate");
            odom_sub_ = std::make_shared<unitree::robot::go2::subscription::SportModeState>(topic);
            odom_sub_->wait_for_connection();
            spdlog::info("[Footstep] foot_state_source = {} (topic={})", foot_src, topic);
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

        // Anchor of the global plan frame: world pose of the initial stance foot.
        // Must match the frame the plan CSV was generated in
        // (cmd/convert_footcommand_2_global.py).
        const isaaclab::WorldPose init_stance = init_stance_pose(fs, plan.front().phase);

        command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
        command_->set_global_plan(std::move(plan), init_stance);
        State_Footstep::command = command_.get(); // visible to obs terms before env build
        spdlog::info("[FootCommand] command_source = csv_global (init stance x={:.4f} y={:.4f} z={:.4f} yaw={:.4f})",
                     init_stance.x, init_stance.y, init_stance.z, init_stance.yaw);
        // command_source_ stays null: the global plan drives the planner internally.
    }
    else if (src == "goal")
    {
        // Walk to a list of world-frame goal points (deploy.yaml footstep.goal).
        // The planner re-measures the active goal from the stance foot at every
        // step boundary, so the last step before a goal is the residual
        // (goal - current position).
        YAML::Node gn = fs["goal"];
        if (!gn) throw std::runtime_error("State_Footstep: command_source 'goal' needs a 'goal:' section in deploy.yaml.");

        isaaclab::FootstepCommand::GoalConfig gc;
        gc.step_x_max   = yaml_get(gn, "step_x_max", gc.step_x_max);
        gc.step_y       = yaml_get(gn, "step_y", gc.step_y);
        gc.step_yaw_max = yaml_get(gn, "step_yaw_max", gc.step_yaw_max);
        gc.ssp_t        = yaml_get(gn, "ssp_t", gc.ssp_t);
        gc.dsp_t        = yaml_get(gn, "dsp_t", gc.dsp_t);
        gc.height       = yaml_get(gn, "height", gc.height);
        gc.reach_radius = yaml_get(gn, "reach_radius", gc.reach_radius);
        gc.reach_yaw    = yaml_get(gn, "reach_yaw", gc.reach_yaw);
        gc.align_radius = yaml_get(gn, "align_radius", gc.align_radius);
        gc.com_z_rate   = yaml_get(gn, "com_z_rate", gc.com_z_rate);

        std::vector<isaaclab::FootstepCommand::Goal> goals;
        if (gn["points"])
        {
            // [x, y] leaves the arrival heading free; [x, y, yaw] fixes it;
            // [x, y, yaw, com_z] also sets the CoM height while walking there.
            for (const auto& p : gn["points"])
            {
                auto v = p.as<std::vector<float>>();
                if (v.size() < 2)
                    throw std::runtime_error("State_Footstep: each footstep.goal.points entry needs "
                                             "[x, y], [x, y, yaw] or [x, y, yaw, com_z].");
                goals.push_back(isaaclab::FootstepCommand::Goal{
                    v[0], v[1], v.size() > 2 ? v[2] : 0.0f, v.size() > 2,
                    v.size() > 3 ? v[3] : 0.0f});
            }
        }
        if (goals.empty())
            throw std::runtime_error("State_Footstep: footstep.goal.points is empty.");

        // Per-goal stop: upper-body pose to move to on arrival (validated
        // against upper_ids_ once the upper-body config is loaded below).
        if (gn["stop"])
        {
            auto sn = gn["stop"];
            stop_move_time_ = std::max(yaml_get(sn, "move_time", stop_move_time_), 0.01f);
            stop_hold_time_ = std::max(yaml_get(sn, "hold_time", stop_hold_time_), 0.0f);
            if (sn["upper_body_pose"])
                for (const auto& p : sn["upper_body_pose"])
                    stop_poses_.push_back(p.as<std::vector<float>>());
        }

        fcfg.start_phase_indicator = default_start_phase;
        command_ = std::make_unique<isaaclab::FootstepCommand>(fcfg, kin_);
        command_->enable_goal_mode(gc, std::move(goals));
        State_Footstep::command = command_.get(); // visible to obs terms before env build
        command_->set_input(default_input_);      // com_z only; the goal config drives the steps
        spdlog::info("[FootCommand] command_source = goal ({} points, reach_radius={:.2f}m, "
                     "step_x_max={:.2f}m, step_yaw_max={:.2f}rad)",
                     command_->goals().size(), gc.reach_radius, gc.step_x_max, gc.step_yaw_max);
        for (size_t i = 0; i < command_->goals().size(); ++i)
        {
            const auto& g = command_->goals()[i];
            if (g.has_yaw)
                spdlog::info("[FootGoal]   goal {}: x={:.3f} y={:.3f} yaw={:.3f}", i + 1, g.x, g.y, g.yaw);
            else
                spdlog::info("[FootGoal]   goal {}: x={:.3f} y={:.3f} (heading free)", i + 1, g.x, g.y);
        }
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
        // Optional hand-eye extrinsic (D435i camera_link in the head_link
        // frame); defaults to the official URDF mount otherwise.
        if (vn && vn["camera"] && vn["camera"]["extrinsic"])
        {
            auto ex = vn["camera"]["extrinsic"];
            auto vec3 = [](const YAML::Node& n) {
                return isaaclab::math::Vec3(n[0].as<float>(), n[1].as<float>(), n[2].as<float>());
            };
            const isaaclab::math::Vec3 pos = ex["pos"] ? vec3(ex["pos"])
                : isaaclab::math::Vec3(0.05366f, 0.01753f, 0.47387f);
            const isaaclab::math::Vec3 rpy = ex["rpy"] ? vec3(ex["rpy"])
                : isaaclab::math::Vec3(0.0f, 0.8307767239493009f, 0.0f);
            vision_cam_tf_.set_extrinsic(pos, rpy);
            spdlog::info("[FootVision] camera extrinsic override: pos=({:.4f}, {:.4f}, {:.4f}) "
                         "rpy=({:.4f}, {:.4f}, {:.4f})", pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]);
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

    if (use_mujoco_foot_)
        command_->set_foot_state_source(isaaclab::FootStateSource::MUJOCO);
    else if (use_sim_odom_)
        command_->set_foot_state_source(isaaclab::FootStateSource::SIM_ODOM);
    else
        command_->set_foot_state_source(isaaclab::FootStateSource::FK_ODOMETRY);

    // ---- env (policy + managers) ----
    auto articulation = std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(deploy, articulation);
    const std::string policy_file = deploy["policy_file"]
        ? deploy["policy_file"].as<std::string>()
        : std::string("policy.onnx");
    const auto onnx_path = policy_dir / "exported" / policy_file;
    spdlog::info("[Footstep] loading policy '{}'", onnx_path.string());
    env->alg = std::make_unique<isaaclab::OrtRunner>(onnx_path.string());

    // ---- upper-body hold config ----
    YAML::Node ub = deploy["upper_body"];
    if (ub)
    {
        upper_ids_ = ub["joint_ids_map"].as<std::vector<int>>();
        upper_default_ = ub["default_joint_pos"].as<std::vector<float>>();
        upper_kp_ = ub["stiffness"].as<std::vector<float>>();
        upper_kd_ = ub["damping"].as<std::vector<float>>();
    }
    upper_target_ = upper_default_;
    upper_from_ = upper_default_;
    for (size_t i = 0; i < stop_poses_.size(); ++i)
        if (stop_poses_[i].size() != upper_ids_.size())
            throw std::runtime_error("State_Footstep: footstep.goal.stop.upper_body_pose[" +
                                     std::to_string(i) + "] has " + std::to_string(stop_poses_[i].size()) +
                                     " values, expected " + std::to_string(upper_ids_.size()) +
                                     " (one per upper_body joint).");

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

    // Per-step tracking evaluation. Set `footstep: eval_file: ""` to disable.
    // The file itself is opened on enter() with a timestamp suffix so startup
    // (which constructs every FSM state) does not truncate sibling logs.
    std::string eval_path = fs["eval_file"] ? fs["eval_file"].as<std::string>()
                                            : std::string("log/footstep_eval.csv");
    if (!eval_path.empty())
    {
        std::filesystem::path ep = eval_path;
        if (ep.is_relative()) ep = param::proj_dir / ep;
        eval_path_ = ep.string();
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
    // Column layout (tab separated). Existing columns preserved for plotting.ipynb;
    // world-frame verification columns appended (stance_world_ odometry frame),
    // matching tocabi commands.py data.txt logging.
    log_file_ << "tick\twalking_tick"
                 "\tref_zmp_x\tref_zmp_y\tref_zmp_z"
                 "\ttarget_com_stance_x\ttarget_com_stance_y\ttarget_com_stance_z"
                 "\tcom_stance_x\tcom_stance_y\tcom_stance_z"
                 "\tcom_global_x\tcom_global_y\tcom_global_z"
                 "\tlfoot_x\tlfoot_y\tlfoot_z\trfoot_x\trfoot_y\trfoot_z"
                 "\ttarget_com_global_x\ttarget_com_global_y\ttarget_com_global_z"
                 "\tq_leg_desired[0..11]\tq_leg_meas[0..11]"
                 "\tphase_indicator"
                 "\tswing_target_x\tswing_target_y\tswing_target_z"
                 "\tref_vrp_world_x\tref_vrp_world_y\tref_vrp_world_z"
                 "\ttarget_com_world_x\ttarget_com_world_y\ttarget_com_world_z"
                 "\tcom_world_x\tcom_world_y\tcom_world_z"
                 "\tstance_foot_x\tstance_foot_y\tstance_foot_z"
                 "\tswing_foot_x\tswing_foot_y\tswing_foot_z\n";
    log_enabled_ = true;
    spdlog::info("[FootLog] logging to '{}'.", path);
}

// One row per completed footstep: what was commanded, what the foot actually
// landed on, and the difference - all in the stance-foot frame, the frame the
// command itself is expressed in. This is the data cmd/eval_footstep.py reads.
static std::filesystem::path stamp_eval_path_(const std::filesystem::path& template_path)
{
    // log/footstep_eval.csv -> log/footstep_eval_YYMMDD_HHMMSS.csv
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tm);
    return template_path.parent_path() /
           (template_path.stem().string() + "_" + buf + template_path.extension().string());
}

void State_Footstep::open_eval_file(const std::string& path)
{
    if (eval_file_.is_open()) eval_file_.close();
    eval_enabled_ = false;

    std::filesystem::path stamped = stamp_eval_path_(path);
    if (!stamped.parent_path().empty())
        std::filesystem::create_directories(stamped.parent_path());

    eval_file_.open(stamped.string(), std::ios::out | std::ios::trunc);
    if (!eval_file_.is_open())
    {
        spdlog::warn("[FootEval] could not open '{}': step evaluation disabled.", stamped.string());
        return;
    }
    eval_file_ << std::fixed << std::setprecision(6);
    eval_file_ << "step,foot,"
                  "cmd_x,cmd_y,cmd_yaw,"
                  "meas_x,meas_y,meas_yaw,"
                  "err_x,err_y,err_yaw,"
                  "ssp_t,dsp_t,height\n";
    eval_enabled_ = true;
    spdlog::info("[FootEval] per-step tracking error -> '{}'.", stamped.string());
}

void State_Footstep::write_eval_row()
{
    if (!eval_enabled_) return;
    const auto& c = *command_;
    const isaaclab::math::Vec3& cmd = c.last_step_command();
    const isaaclab::math::Vec3& meas = c.last_step_measured();
    const isaaclab::math::Vec3& err = c.last_step_error();
    const isaaclab::math::Vec3& tim = c.last_step_timing(); // [ssp_t, dsp_t, height]

    eval_file_ << c.step_counter() << "," << (c.last_step_swing_right() ? "R" : "L") << ","
               << cmd[0] << "," << cmd[1] << "," << cmd[2] << ","
               << meas[0] << "," << meas[1] << "," << meas[2] << ","
               << err[0] << "," << err[1] << "," << err[2] << ","
               << tim[0] << "," << tim[1] << "," << tim[2] << "\n";
    eval_file_.flush(); // a run is often ended with Ctrl+C
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
    const isaaclab::math::Vec3 swing_tgt = c.swing_target_world();
    const isaaclab::math::Vec3 vrp_w = c.ref_vrp_world();
    const isaaclab::math::Vec3 tgt_com_w = c.target_com_world();
    const isaaclab::math::Vec3 com_w = c.com_world();
    const isaaclab::math::Vec3 stance_f = c.stance_foot_world();
    const isaaclab::math::Vec3 swing_f = c.swing_foot_world();
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
    log_file_ << c.phase_indicator0() << "\t";
    log_file_ << swing_tgt[0] << "\t" << swing_tgt[1] << "\t" << swing_tgt[2] << "\t";
    log_file_ << vrp_w[0] << "\t" << vrp_w[1] << "\t" << vrp_w[2] << "\t";
    log_file_ << tgt_com_w[0] << "\t" << tgt_com_w[1] << "\t" << tgt_com_w[2] << "\t";
    log_file_ << com_w[0] << "\t" << com_w[1] << "\t" << com_w[2] << "\t";
    log_file_ << stance_f[0] << "\t" << stance_f[1] << "\t" << stance_f[2] << "\t";
    log_file_ << swing_f[0] << "\t" << swing_f[1] << "\t" << swing_f[2] << "\n";
    ++log_tick_;
}

void State_Footstep::enter()
{
    walk_started_ = false; // stand still until the operator presses Y
    upper_pose_index_ = -1;
    upper_pose_active_ = -1;
    upper_motion_done_ = true;
    upper_target_ = upper_default_;

    if (!eval_path_.empty()) open_eval_file(eval_path_);

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
            if (!odom_sub_) return;
            std::lock_guard<std::mutex> lock(odom_sub_->mutex_);
            if (use_sim_odom_)
            {
                const auto& p = odom_sub_->msg_.position();
                command_->set_base_pos_world(isaaclab::math::Vec3(p[0], p[1], p[2]));
            }
            if (use_mujoco_foot_)
            {
                // Layout from unitree_sdk2_bridge: [Lxyz, Rxyz, Lyaw, Ryaw]
                const auto& fp = odom_sub_->msg_.foot_position_body();
                command_->set_mujoco_foot_pose(
                    isaaclab::Kinematics::LEFT,
                    isaaclab::WorldPose{fp[0], fp[1], fp[2], fp[6]});
                command_->set_mujoco_foot_pose(
                    isaaclab::Kinematics::RIGHT,
                    isaaclab::WorldPose{fp[3], fp[4], fp[5], fp[7]});
            }
        };

        // vision mode: take the latest perception frame (camera optical
        // frame), convert it to the pelvis frame with the waist FK, and hand
        // it to FootstepCommand (which ingests it inside compute() with this
        // tick's stance-foot state).
        //
        // The FK runs on the waist angles the frame was CAPTURED at, which the
        // perception node ships with it. Using this tick's joints instead
        // would place the targets through a camera pose the head has already
        // moved away from - only the fallback path does that, when the
        // perception node has no robot state of its own.
        auto feed_vision = [&]() {
            if (!vision_sub_) return;
            command_->set_vision_clock(std::chrono::duration<double>(
                clock::now().time_since_epoch()).count());
            isaaclab::VisionFrame vf;
            if (vision_sub_->take(vf))
            {
                const isaaclab::math::Vec3 w =
                    vf.has_waist ? vf.waist
                                 : isaaclab::math::Vec3(q(12), q(13), q(14));
                vision_cam_tf_.to_pelvis(vf.targets, w[0], w[1], w[2]);
                command_->set_vision_targets(vf.targets);
            }
        };

        // initial reset. Except for the scripted sources (see auto_start below)
        // the planner is NOT reset here: the state enters in standby and
        // command_->reset() runs at the moment walking starts, so it anchors on
        // the state the robot is in then.
        env->robot->update();
        load_full_state(q, qd);
        kin_->set_state(q, qd);
        command_->robot_quat_w_ = env->robot->data.root_quat_w;
        update_base_from_odom();
        if (command_source_) command_->set_input(command_source_->input());
        feed_vision();
        // A scripted plan (csv, csv_global) needs no aiming, so it starts walking
        // as soon as the state is entered and stops once the plan is done. The
        // operator-driven sources wait in standby for the Y press.
        const int plan_steps = command_source_ ? command_source_->size() : 0;
        const bool auto_start = command_->global_mode() || plan_steps > 0;
        if (auto_start) command_->reset();
        else            command_->hold_standby(0.0f);
        env->reset();

        // STANDBY -> (Y) -> WALKING -> (goal reached) -> AT_GOAL -> WALKING ...
        // and FINISHED once the last goal / the global plan is done. Everything
        // but WALKING holds the standby command, i.e. the robot stands still.
        enum class Mode { STANDBY, WALKING, AT_GOAL, FINISHED };
        Mode mode = auto_start ? Mode::WALKING : Mode::STANDBY;
        auto resume_at = clock::now();
        // CoM height to stand at: 0 before the first step, then the com_z of the
        // goal just reached, so stopping does not undo the crouch the robot
        // walked there with.
        float standby_com_z = 0.0f;

        while (policy_thread_running)
        {
            env->robot->update();
            load_full_state(q, qd);
            kin_->set_state(q, qd);
            command_->robot_quat_w_ = env->robot->data.root_quat_w;
            update_base_from_odom();
            if (command_source_) command_->set_input(command_source_->input());
            feed_vision();

            // Standby / stopped: the policy runs on a frozen-phase, stand-still
            // command so it holds the stance instead of stepping.
            if (mode == Mode::STANDBY)
            {
                command_->hold_standby(standby_com_z);
                if (walk_started_)
                {
                    command_->reset();
                    // Seed the observation history with the first WALKING frame.
                    // Without this the policy starts a step with up to
                    // history_length * skip_history_tick frames of standby
                    // observations (frozen phase, zero foot command) still in
                    // the buffer - a transient it never saw in training, where
                    // every episode begins from a reset history.
                    env->reset();
                    mode = Mode::WALKING;
                    spdlog::info("[Footstep] walk start");
                }
            }
            else if (mode == Mode::AT_GOAL)
            {
                command_->hold_standby(standby_com_z);
                // Hold until the upper body has finished moving, then pause.
                if (!upper_motion_done_)
                    resume_at = clock::now() + std::chrono::duration_cast<clock::duration>(
                        std::chrono::duration<double>(stop_hold_time_));
                else if (clock::now() >= resume_at)
                {
                    command_->reset(/*keep_goal_progress=*/true);
                    env->reset(); // seed the history with the walking frame (see above)
                    mode = Mode::WALKING;
                }
            }
            else if (mode == Mode::FINISHED)
            {
                command_->hold_standby(standby_com_z);
            }

            if (mode == Mode::WALKING)
            {
                command_->compute();
                // advance the (csv) command source when a footstep completes
                if (command_source_ && command_->step_completed()) command_source_->advance();
                // per-step landing error report (mirrors tocabi cc.cpp)
                if (command_->step_completed())
                {
                    write_eval_row();
                    const auto& e = command_->last_step_error();
                    spdlog::info("Foot Position error : {:.4f} [m]", std::sqrt(e[0]*e[0] + e[1]*e[1]));
                    spdlog::info(">> X error : {:.4f} [m]", std::abs(e[0]));
                    spdlog::info(">> Y error : {:.4f} [m]", std::abs(e[1]));
                    spdlog::info("Foot Yaw error : {:.4f} [rad]", std::abs(e[2]));
                    const auto& fc = command_->foot_command0();
                    // foot_command0: [x, y, z, roll, pitch, yaw, ssp_t, dsp_t, height]
                    spdlog::info("Next foot step command : x={:.4f} y={:.4f} z={:.4f} [m], yaw={:.4f} [rad]",
                                 fc[0], fc[1], fc[2], fc[5]);
                    // spdlog::info("t_total: {:.3f}", command_->last_step_total_time());
                }

                // Reached a goal: stop right here (the standby command replaces
                // the step that was just planned) and move the upper body.
                if (command_->goal_arrived())
                {
                    const int reached = command_->goal_index() - 1;
                    const bool last = command_->goal_index() >= (int)command_->goals().size();
                    if (reached >= 0 && reached < (int)stop_poses_.size())
                    {
                        upper_motion_done_ = false; // run() takes it from here
                        upper_pose_index_ = reached;
                    }
                    // Stand at the CoM height the robot actually walked here with
                    // (the rate-limited value, which may still be short of that
                    // goal's com_z), so the stop does not step the height either
                    // way.
                    standby_com_z = command_->com_z();
                    resume_at = clock::now() + std::chrono::duration_cast<clock::duration>(
                        std::chrono::duration<double>(stop_hold_time_));
                    mode = last ? Mode::FINISHED : Mode::AT_GOAL;
                    command_->hold_standby(standby_com_z);
                    spdlog::info("[FootGoal] stopped at goal {} (com_z={:.3f}){}", reached + 1,
                                 standby_com_z, last ? ", last goal: staying stopped" : "");
                }
                else if (command_->global_plan_done() ||
                         (plan_steps > 0 && command_->step_counter() >= plan_steps))
                {
                    // Plan finished: stop on the foot that just landed and hold
                    // the default pose rather than marching in place forever.
                    mode = Mode::FINISHED;
                    command_->hold_standby(standby_com_z);
                    spdlog::info("[FootCommand] plan complete after {} steps: stopping, "
                                 "holding the default pose.", command_->step_counter());
                }
                else
                {
                    write_log_row(q);
                }
            }

            env->step();

            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
    });
}

// Move the upper body toward the pose the policy thread selected. Runs at the
// 1 kHz FSM rate so the arms move smoothly regardless of the policy step_dt.
void State_Footstep::update_upper_target()
{
    const int want = upper_pose_index_.load();
    if (want != upper_pose_active_)
    {
        upper_pose_active_ = want;
        upper_from_ = upper_target_; // start wherever the arms are commanded now
        upper_t0_ = std::chrono::steady_clock::now();
        upper_motion_done_ = false;
    }
    if (upper_motion_done_) return;

    const std::vector<float>& to = (upper_pose_active_ >= 0)
        ? stop_poses_[upper_pose_active_] : upper_default_;
    const float t = std::chrono::duration<float>(
        std::chrono::steady_clock::now() - upper_t0_).count();
    upper_target_ = linear_interpolate(t, {0.0f, stop_move_time_}, {upper_from_, to});
    if (t >= stop_move_time_) upper_motion_done_ = true;
}

void State_Footstep::run()
{
    // Y releases the standby hold and starts walking (checked here, not in the
    // policy thread: on_pressed is a single-tick edge of the 1 kHz FSM loop).
    if (FSMState::lowstate->joystick.Y.on_pressed) walk_started_ = true;

    update_upper_target();

    auto action = env->action_manager->processed_actions(); // 12 lower joint targets
    for (int i = 0; i < (int)env->robot->data.joint_ids_map.size(); ++i)
    {
        int sdk = (int)env->robot->data.joint_ids_map[i];
        lowcmd->msg_.motor_cmd()[sdk].q() = action[i];
    }
    for (int j = 0; j < (int)upper_ids_.size(); ++j)
    {
        lowcmd->msg_.motor_cmd()[upper_ids_[j]].q() = upper_target_[j];
    }
}
