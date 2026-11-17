// Stage-2 footstep planner of yonglab (supermimic/planner.py, candidate area "reach"), for command_source: planner.
//
// PlannerPatch = TerrainPatch("unprojected"): the pixels of one depth image every `every` policy ticks, kept for
// `frames` images, un-projected to world points and averaged per 2 cm cell of a patch in front of the stance foot.
// PlannerNet   = PatchScorer exported by tools/export_onnx.py: patch + proprioception -> a score per candidate cell
// and the height step of the two footsteps.
//
// Both are checked against data recorded in Isaac by tools/planner_golden_test.cpp.

#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "onnxruntime_cxx_api.h"

#include "isaaclab/utils/kinematics.h"

namespace isaaclab
{

struct FootstepPlannerConfig
{
    // nominal footstep from the joystick velocity (JoystickFootCommandCfg)
    std::array<float, 3> joystick_max = {0.5f, 0.15f, 0.5f}; // |vx|, |vy|, |yaw rate| of the shortest footstep
    float step_time_slow = 1.0f;   // footstep time SSP + 2 DSP [s] at zero velocity
    float step_time_fast = 0.65f;  // ... at joystick_max
    float ssp_ratio = 0.8f;
    float stance_width = 0.237f;
    float foot_height = 0.08f;
    // the two in-place footsteps after a reset (FootCommandCfg.ranges of the training env)
    float reset_ssp = 0.6f;
    float reset_dsp = 0.1f;
    // stick -> velocity command (the training ranges)
    float vx_max = 0.5f, vx_min = -0.1f, vy_max = 0.15f, wz_max = 0.5f;

    // candidate cells and patch
    float res = 0.02f;
    float sole_offset_x = 0.035f; // sole centre ahead of the ankle
    float sole_z = -0.035f;       // sole bottom relative to the ankle

    // depth camera (yonglab d435_cfg: ray-cast pinhole camera on the torso)
    int width = 64, height = 36;
    float fov_x_deg = 87.0f, fov_y_deg = 58.0f;
    float max_depth = 3.0f;
    bool blur = true; // the 3x3 mean (over the valid pixels) of the training's depth model
    bool blur_zero_pad = false; // planners trained before 2026-11-16: plain mean, pixels outside the image count as 0
    int every = 5;    // policy ticks between kept images
    int frames = 20;  // kept images (2 s)
    std::string camera_parent = "torso_link";
    Eigen::Vector3f camera_pos = Eigen::Vector3f(0.0576235f, 0.01753f, 0.41987f);
    float camera_pitch = 0.8307767239493009f;
};

// `footstep.planner` of deploy.yaml; missing keys keep the training values above.
inline FootstepPlannerConfig planner_config_from_yaml(const YAML::Node& n)
{
    FootstepPlannerConfig c;
    if (!n) return c;
    auto f = [&](const char* key, float& v) { if (n[key]) v = n[key].as<float>(); };
    if (n["joystick_max"]) c.joystick_max = n["joystick_max"].as<std::array<float, 3>>();
    if (n["step_time"])
    {
        c.step_time_slow = n["step_time"][0].as<float>();
        c.step_time_fast = n["step_time"][1].as<float>();
    }
    f("ssp_ratio", c.ssp_ratio);
    f("stance_width", c.stance_width);
    f("foot_height", c.foot_height);
    f("reset_ssp", c.reset_ssp);
    f("reset_dsp", c.reset_dsp);
    if (n["velocity"])
    {
        const YAML::Node v = n["velocity"];
        if (v["vx"]) { c.vx_min = v["vx"][0].as<float>(); c.vx_max = v["vx"][1].as<float>(); }
        if (v["vy"]) c.vy_max = v["vy"].as<float>();
        if (v["wz"]) c.wz_max = v["wz"].as<float>();
    }
    f("res", c.res);
    f("sole_offset_x", c.sole_offset_x);
    f("sole_z", c.sole_z);
    if (const YAML::Node d = n["depth"])
    {
        if (d["width"]) c.width = d["width"].as<int>();
        if (d["height"]) c.height = d["height"].as<int>();
        if (d["fov"]) { c.fov_x_deg = d["fov"][0].as<float>(); c.fov_y_deg = d["fov"][1].as<float>(); }
        if (d["max_depth"]) c.max_depth = d["max_depth"].as<float>();
        if (d["blur"]) c.blur = d["blur"].as<bool>();
        if (d["blur_zero_pad"]) c.blur_zero_pad = d["blur_zero_pad"].as<bool>();
        if (d["every"]) c.every = d["every"].as<int>();
        if (d["frames"]) c.frames = d["frames"].as<int>();
        if (d["camera_parent"]) c.camera_parent = d["camera_parent"].as<std::string>();
        if (d["camera_pos"])
        {
            const auto v = d["camera_pos"].as<std::array<float, 3>>();
            c.camera_pos = Eigen::Vector3f(v[0], v[1], v[2]);
        }
        if (d["camera_pitch"]) c.camera_pitch = d["camera_pitch"].as<float>();
    }
    return c;
}

// Pose of the depth camera in the pelvis frame: position and rotation whose columns are the optical axes
// (x right, y down, z forward). Needs kin.set_state() of this tick.
inline void planner_camera_in_pelvis(const Kinematics& kin, pinocchio::FrameIndex parent, const FootstepPlannerConfig& cfg,
                                     Eigen::Vector3f& p, Eigen::Matrix3f& R)
{
    const Eigen::Matrix3f Rt = math::matrix_from_quat(kin.frame_quat(parent));
    p = kin.frame_pos(parent) + Rt * cfg.camera_pos;
    Eigen::Matrix3f link_from_optical; // camera link: x forward, y left, z up
    link_from_optical << 0, 0, 1,
                        -1, 0, 0,
                         0, -1, 0;
    R = Rt * Eigen::AngleAxisf(cfg.camera_pitch, Eigen::Vector3f::UnitY()).toRotationMatrix() * link_from_optical;
}

class PlannerPatch
{
public:
    // (x_min, x_max), (y_min, y_max): reachable area of the first footstep (x, |y|), stance-foot frame.
    PlannerPatch(const FootstepPlannerConfig& cfg, float x_min, float x_max, float y_min, float y_max)
    : cfg_(cfg)
    {
        const int ix0 = (int)std::lround(x_min / cfg.res), ix1 = (int)std::lround(x_max / cfg.res);
        const int iy0 = (int)std::lround(y_min / cfg.res), iy1 = (int)std::lround(y_max / cfg.res);
        nx_ = ix1 - ix0 + 1;
        ny_ = iy1 - iy0 + 1;
        cx0_ = ix0 * cfg.res;
        cy0_ = iy0 * cfg.res;
        H_ = nx_ + 12;
        W_ = ny_ + 6;
        x0_ = (ix0 - 6) * cfg.res + cfg.sole_offset_x;
        y0_ = (iy0 - 3) * cfg.res;
        fx_ = cfg.width / (2.0f * std::tan(0.5f * cfg.fov_x_deg * (float)M_PI / 180.0f));
        fy_ = cfg.height / (2.0f * std::tan(0.5f * cfg.fov_y_deg * (float)M_PI / 180.0f));
    }

    int H() const { return H_; }
    int W() const { return W_; }
    int cells() const { return nx_ * ny_; } // + 1: the in-place step
    // candidate cell -> first footstep (x, |y|)
    float cell_x(int cell) const { return cx0_ + (cell / ny_) * cfg_.res; }
    float cell_y(int cell) const { return cy0_ + (cell % ny_) * cfg_.res; }
    size_t num_frames() const { return cloud_.size(); }

    void clear() { cloud_.clear(); }

    // depth: height x width, row-major, distance to the image plane [m]. cam_R: columns = the optical axes
    // (x right, y down, z forward) in the world.
    void add_frame(const float* depth, const Eigen::Vector3f& cam_p, const Eigen::Matrix3f& cam_R)
    {
        const int Wd = cfg_.width, Hd = cfg_.height;
        std::vector<float> d(Wd * Hd), b;
        for (int i = 0; i < Wd * Hd; ++i)
        {
            float v = depth[i];
            if (std::isnan(v)) v = 0.0f;
            d[i] = std::min(std::max(v, 0.0f), cfg_.max_depth);
        }
        if (cfg_.blur)
        {
            // 3 x 3 mean over the valid pixels only (inside the image, depth > 0), as the training's depth model
            // since 2026-11-16; invalid pixels stay invalid
            b.assign(Wd * Hd, 0.0f);
            for (int v = 0; v < Hd; ++v)
                for (int u = 0; u < Wd; ++u)
                {
                    if (!cfg_.blur_zero_pad && !(d[v * Wd + u] > 0.0f)) continue;
                    float s = 0.0f;
                    int n = 0;
                    for (int dv = -1; dv <= 1; ++dv)
                        for (int du = -1; du <= 1; ++du)
                        {
                            const int vv = v + dv, uu = u + du;
                            if (vv < 0 || vv >= Hd || uu < 0 || uu >= Wd || !(d[vv * Wd + uu] > 0.0f)) continue;
                            s += d[vv * Wd + uu];
                            ++n;
                        }
                    b[v * Wd + u] = cfg_.blur_zero_pad ? s / 9.0f : s / n;
                }
            d.swap(b);
        }
        std::vector<Eigen::Vector3f> pts;
        pts.reserve(Wd * Hd);
        for (int v = 0; v < Hd; ++v)
            for (int u = 0; u < Wd; ++u)
            {
                const float z = d[v * Wd + u];
                if (!(z > 0.05f && z < cfg_.max_depth - 0.05f)) continue;
                // The training un-projects with the pixel index (not its centre) although the rays go through the
                // centres; kept so that the patch is what the planner was trained on.
                const Eigen::Vector3f p(z * (u - 0.5f * Wd) / fx_, z * (v - 0.5f * Hd) / fy_, z);
                pts.push_back(cam_p + cam_R * p);
            }
        cloud_.push_back(std::move(pts));
        while ((int)cloud_.size() > cfg_.frames) cloud_.pop_front();
    }

    // Patch [m] (H x W, row-major) and seen mask at a support transition. (sx, sy, sz, syaw): world pose of the
    // stance ankle; side: +1 left swing, -1 right swing (the patch is mirrored so that W grows away from the stance foot).
    void build(float sx, float sy, float sz, float syaw, float side, std::vector<float>& patch, std::vector<float>& seen) const
    {
        const float c = std::cos(syaw), s = std::sin(syaw), ground0 = sz + cfg_.sole_z;
        std::vector<float> tot(H_ * W_, 0.0f), cnt(H_ * W_, 0.0f);
        for (const auto& frame : cloud_)
            for (const auto& p : frame)
            {
                const float dx = p[0] - sx, dy = p[1] - sy;
                const int ix = (int)std::nearbyint((c * dx + s * dy - x0_) / cfg_.res);
                const int iy = (int)std::nearbyint((side * (-s * dx + c * dy) - y0_) / cfg_.res);
                if (ix < 0 || ix >= H_ || iy < 0 || iy >= W_) continue;
                tot[ix * W_ + iy] += p[2] - ground0;
                cnt[ix * W_ + iy] += 1.0f;
            }
        patch.assign(H_ * W_, 0.0f);
        seen.assign(H_ * W_, 0.0f);
        for (int i = 0; i < H_ * W_; ++i)
            if (cnt[i] > 0.0f)
            {
                patch[i] = std::min(std::max(tot[i] / cnt[i], -0.3f), 0.3f);
                seen[i] = 1.0f;
            }
    }

private:
    FootstepPlannerConfig cfg_;
    int nx_ = 0, ny_ = 0, H_ = 0, W_ = 0;
    float cx0_ = 0, cy0_ = 0, x0_ = 0, y0_ = 0, fx_ = 1, fy_ = 1;
    std::deque<std::vector<Eigen::Vector3f>> cloud_;
};

class PlannerNet
{
public:
    explicit PlannerNet(const std::string& path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "footstep_planner")
    {
        Ort::SessionOptions opt;
        opt.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(env_, path.c_str(), opt);
        auto dim = [&](size_t i) { return session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape(); };
        patch_shape_ = dim(0);
        proprio_shape_ = dim(1);
    }

    int H() const { return (int)patch_shape_[1]; }
    int W() const { return (int)patch_shape_[2]; }
    int proprio_dim() const { return (int)proprio_shape_[1]; }

    // score (cells + 1) and z (cells + 1, 2) [m]; the last entry is the in-place step.
    void run(std::vector<float>& patch, std::vector<float>& proprio, std::vector<float>& seen,
             std::vector<float>& score, std::vector<float>& z)
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::array<Ort::Value, 3> in = {
            Ort::Value::CreateTensor<float>(mem, patch.data(), patch.size(), patch_shape_.data(), patch_shape_.size()),
            Ort::Value::CreateTensor<float>(mem, proprio.data(), proprio.size(), proprio_shape_.data(), proprio_shape_.size()),
            Ort::Value::CreateTensor<float>(mem, seen.data(), seen.size(), patch_shape_.data(), patch_shape_.size()),
        };
        const char* in_names[] = {"patch", "proprio", "seen"};
        const char* out_names[] = {"score", "z"};
        auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, in.data(), 3, out_names, 2);
        const size_t n = out[0].GetTensorTypeAndShapeInfo().GetElementCount();
        score.assign(out[0].GetTensorData<float>(), out[0].GetTensorData<float>() + n);
        z.assign(out[1].GetTensorData<float>(), out[1].GetTensorData<float>() + 2 * n);
    }

private:
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<int64_t> patch_shape_, proprio_shape_;
};

} // namespace isaaclab
