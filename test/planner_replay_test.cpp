// Replays a recording of the yonglab deploy configuration in Isaac (tools/dump_isaac.py: tracker + planner in closed
// loop, noise off) through FootstepCommand in planner mode, with the deploy.yaml the controller uses, and compares
//   - the depth camera pose (kinematics + offset) with Isaac's camera,
//   - the picked candidate cell of every footstep,
//   - the command of every step: IK joint target (12), phase (2), footstep pose (6).
//
//   ./planner_replay_test <golden dir> [policy dir = ../config/policy/stage2]

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "param.h"
#include "isaaclab/envs/mdp/commands/footstep_command.h"

using namespace isaaclab;

struct Array
{
    std::vector<int> shape;
    std::vector<float> data;
    int rows() const { return shape[0]; }
    int cols() const { return (int)data.size() / shape[0]; }
    const float* row(int i) const { return data.data() + (size_t)i * cols(); }
};

static Array load(const std::filesystem::path& p)
{
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + p.string());
    Array a;
    int nd = 0;
    f.read((char*)&nd, 4);
    a.shape.resize(nd);
    f.read((char*)a.shape.data(), 4 * nd);
    size_t n = 1;
    for (int d : a.shape) n *= d;
    a.data.resize(n);
    f.read((char*)a.data.data(), 4 * n);
    return a;
}

int main(int argc, char** argv)
{
    if (argc < 2) { std::cerr << "usage: planner_replay_test <golden dir> [policy dir]\n"; return 2; }
    const std::filesystem::path g = argv[1];
    const std::filesystem::path src = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path policy_dir = argc > 2 ? std::filesystem::path(argv[2]) : src / "config/policy/stage2";
    YAML::Node deploy = param::load_yaml_with_extends(policy_dir / "params" / "deploy.yaml");
    YAML::Node fs = deploy["footstep"];

    const Array q = load(g / "step_q.bin"), qd = load(g / "step_qd.bin"), R = load(g / "step_R.bin"), p = load(g / "step_p.bin");
    const Array w = load(g / "step_w_b.bin"), cmd = load(g / "step_command.bin"), phase = load(g / "step_phase.bin");
    const Array joy = load(g / "step_joystick.bin");
    const Array ft = load(g / "frame_t.bin"), fd = load(g / "frame_depth_raw.bin"), fp = load(g / "frame_cam_p.bin"), fR = load(g / "frame_cam_R.bin");
    const Array lt = load(g / "latch_t.bin"), lpick = load(g / "latch_pick.bin");
    const Array ref = load(g / "step_ref.bin"); // com, target com, target pelvis, target swing, swing (stance frame)
    double ref_max[4] = {0, 0, 0, 0}, ref_sum[4] = {0, 0, 0, 0};
    const Array lpatch = load(g / "latch_patch.bin"), lseen = load(g / "latch_seen.bin"), lprop = load(g / "latch_proprio.bin");
    const Array lsp = load(g / "latch_stance_p.bin"), lsyaw = load(g / "latch_stance_yaw.bin"), lscore = load(g / "latch_score.bin");

    auto kin = std::make_shared<Kinematics>(
        (src / fs["urdf_path"].as<std::string>()).string(), fs["base_link"].as<std::string>(),
        fs["lleg_joints"].as<std::vector<std::string>>(), fs["rleg_joints"].as<std::vector<std::string>>(),
        fs["lee_name"].as<std::string>(), fs["ree_name"].as<std::string>(), g1_name_to_sdk());
    FootstepCommand::Config fcfg = footstep_config_from_yaml(deploy);
    fcfg.start_phase_indicator = (int)phase.row(0)[0];
    FootstepCommand command(fcfg, kin);
    FootstepPlannerConfig pc = planner_config_from_yaml(fs["planner"]);
    pc.blur_zero_pad = false; // the recording is made with the current yonglab depth model (tools/dump_isaac.py)
    command.enable_planner_mode(pc, (policy_dir / "exported" / fs["planner"]["planner_file"].as<std::string>()).string());
    command.set_foot_state_source(FootStateSource::SIM_ODOM);

    std::vector<float> dq(29, 0.0f);
    const auto lower = deploy["default_joint_pos"].as<std::vector<float>>();
    const auto upper = deploy["upper_body"]["default_joint_pos"].as<std::vector<float>>();
    const auto upper_ids = deploy["upper_body"]["joint_ids_map"].as<std::vector<int>>();
    for (int i = 0; i < 12; ++i) dq[i] = lower[i];
    for (size_t j = 0; j < upper_ids.size(); ++j) dq[upper_ids[j]] = upper[j];

    const pinocchio::FrameIndex cam_fid = kin->frame_index(pc.camera_parent);
    double cam_p_err = 0, cam_R_err = 0, ik_max = 0, ik_sum = 0, ph_max = 0, pose_max = 0;
    double patch_err = 0, proprio_err = 0, score_err = 0;
    float stance_err = 0;
    int seen_diff = 0;
    int frame = 0, latch = 0, picks_same = 0, picks = 0, pose_rows = 0, last_seq = command.planner_pick_seq();
    while (frame < ft.rows() && ft.row(frame)[0] < 1) ++frame;
    const int N = q.rows();
    for (int t = 0; t < N; ++t)
    {
        Eigen::VectorXf qv = Eigen::Map<const Eigen::VectorXf>(q.row(t), 29), qdv = Eigen::Map<const Eigen::VectorXf>(qd.row(t), 29);
        kin->set_state(qv, qdv);
        Eigen::Matrix3f Rw = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(R.row(t));
        command.robot_quat_w_ = math::Quat(Rw).normalized();
        command.set_base_pos_world(Eigen::Map<const Eigen::Vector3f>(p.row(t)));
        command.set_joystick_velocity(joy.row(t)[0], joy.row(t)[1], joy.row(t)[2]);
        std::array<float, 33> pr;
        const Eigen::Vector3f grav = Rw.transpose() * Eigen::Vector3f(0, 0, -1);
        for (int i = 0; i < 3; ++i) { pr[i] = w.row(t)[i]; pr[3 + i] = grav[i]; }
        for (int i = 0; i < 12; ++i) { pr[6 + i] = qv[i] - dq[i]; pr[18 + i] = qdv[i]; }
        for (int i = 0; i < 3; ++i) pr[30 + i] = qv[12 + i] - dq[12 + i];
        command.set_planner_proprio(pr);

        // Isaac computes the command term once more in the step that resets it; State_Footstep does the same at walk
        // start (reset() and compute() in one tick)
        if (t == 0) command.reset();
        command.compute();

        // the image Isaac un-projected after the step that led to this state
        const float* depth = nullptr;
        if (frame < ft.rows() && (int)ft.row(frame)[0] == t)
        {
            depth = fd.row(frame);
            Eigen::Vector3f cp;
            Eigen::Matrix3f cR;
            planner_camera_in_pelvis(*kin, cam_fid, pc, cp, cR);
            cp = Eigen::Map<const Eigen::Vector3f>(p.row(t)) + Rw * cp;
            cR = Rw * cR;
            cam_p_err = std::max(cam_p_err, (double)(cp - Eigen::Map<const Eigen::Vector3f>(fp.row(frame))).norm());
            cam_R_err = std::max(cam_R_err, (double)(cR - Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(fR.row(frame))).cwiseAbs().maxCoeff());
            ++frame;
        }
        command.planner_observe(depth);

        if (command.planner_pick_seq() != last_seq)
        {
            last_seq = command.planner_pick_seq();
            const int want = latch < lt.rows() ? (int)lpick.row(latch)[0] : -2;
            std::printf("[replay] footstep %2d at step %3d (Isaac: step %3d): cell %3d, Isaac %3d%s\n", latch, t,
                        latch < lt.rows() ? (int)lt.row(latch)[0] + 1 : -1, command.planner_pick(), want,
                        command.planner_pick() == want ? "" : "   <-- differs");
            if (latch < lt.rows())
            {
                double dp = 0, dprop = 0, dscore = 0;
                int dseen = 0;
                const auto& pa = command.planner_patch();
                const auto& se = command.planner_seen();
                for (size_t i = 0; i < pa.size(); ++i)
                {
                    dseen += se[i] != lseen.row(latch)[i];
                    if (se[i] == lseen.row(latch)[i]) dp = std::max(dp, (double)std::abs(pa[i] - lpatch.row(latch)[i]));
                }
                for (size_t i = 0; i < command.planner_proprio().size(); ++i)
                {
                    const double e = std::abs(command.planner_proprio()[i] - lprop.row(latch)[i]);
                    dprop = std::max(dprop, e);
                }
                for (size_t i = 0; i < command.planner_score().size(); ++i)
                    dscore = std::max(dscore, (double)std::abs(command.planner_score()[i] - lscore.row(latch)[i]));
                const auto& sw = command.stance_world();
                patch_err = std::max(patch_err, dp);
                seen_diff += dseen;
                proprio_err = std::max(proprio_err, dprop);
                score_err = std::max(score_err, dscore);
                stance_err = std::max(stance_err, std::hypot(std::hypot(sw.x - lsp.row(latch)[0], sw.y - lsp.row(latch)[1]), sw.z - lsp.row(latch)[2])
                                                      + std::abs(math::wrap_to_pi(sw.yaw - lsyaw.row(latch)[0])));
            }
            picks_same += command.planner_pick() == want;
            ++picks;
            ++latch;
        }
        {
            const math::Vec3 mine[4] = {command.com_stance(), command.target_com_stance(), command.target_swing_stance(), command.swing_stance()};
            const int col[4] = {0, 3, 9, 12};
            for (int k = 0; k < 4; ++k)
            {
                const double e = (mine[k] - Eigen::Map<const Eigen::Vector3f>(ref.row(t) + col[k])).norm();
                ref_max[k] = std::max(ref_max[k], e);
                ref_sum[k] += e;
            }
        }
        const auto& c = command.command();
        const float* gc = cmd.row(t);
        double ik = 0;
        for (int i = 0; i < 12; ++i) ik = std::max(ik, (double)std::abs(c[i] - gc[i]));
        ik_max = std::max(ik_max, ik);
        ik_sum += ik;
        for (int i = 12; i < 14; ++i) ph_max = std::max(ph_max, (double)std::abs(c[i] - gc[i]));
    }
    std::printf("[replay] %d steps, %d depth images\n", N, frame);
    std::printf("[replay] camera pose vs Isaac: position %.2e m, rotation %.2e\n", cam_p_err, cam_R_err);
    std::printf("[replay] planner inputs vs Isaac: patch %.1e m (%d cells seen differently), proprioception %.1e, stance pose %.1e; "
                "scores %.3f\n", patch_err, seen_diff, proprio_err, stance_err, score_err);
    std::printf("[replay] picked cell equal to Isaac's: %d of %d footsteps\n", picks_same, picks);
    const char* ref_name[4] = {"measured CoM", "CoM target", "swing foot target", "measured swing foot"};
    for (int k = 0; k < 4; ++k)
        std::printf("[replay] %s vs Isaac (stance frame): max %.4f m, mean %.4f m\n", ref_name[k], ref_max[k], ref_sum[k] / N);
    std::printf("[replay] command vs Isaac: IK target max %.4f rad (mean of the per-step max %.4f), phase max %.2e, "
                "footstep pose max %.4f (%d steps differ)\n", ik_max, ik_sum / N, ph_max, pose_max, pose_rows);
    // The measured CoM differs by a few mm (URDF vs simulated masses), and with it the CoM target and the IK target.
    const bool ok = cam_p_err < 1e-4 && cam_R_err < 1e-4 && patch_err < 1e-3 && seen_diff == 0 && proprio_err < 1e-4
                    && picks_same >= picks - 1 && ph_max < 1e-4 && ref_sum[1] / N < 0.003 && ik_sum / N < 0.01 && ref_max[2] < 1e-3;
    std::printf("[replay] %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
