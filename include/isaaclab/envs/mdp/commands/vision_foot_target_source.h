// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// DDS subscriber + camera-to-pelvis transform for the ArUco footstep-target
// perception node (cmd/aruco_footstep_perception.py).
//
// The node publishes RAW camera-frame estimates as a JSON payload on a
// std_msgs String topic (default rt/footstep_vision):
//
//   {"stamp": <unix time>, "frame": "camera_optical", "targets": [
//      {"id": 0, "pos": [x, y, z], "quat": [w, x, y, z], "nmk": 4, "err": 0.3},
//      ... ]}
//
// Poses are the footstep-target frames (origin = footstep center on the top
// surface, x = footstep yaw direction, z = up) expressed in the D435i COLOR
// OPTICAL frame (OpenCV convention: x right, y down, z forward - the direct
// solvePnP output). The controller converts them to the pelvis frame with
// D435PelvisCamTransform (analytic waist FK below) using its own joint state,
// then FootstepCommand (vision mode) takes them to the stance-foot frame /
// accumulated world frame and plans on them.
//
// JSON is parsed with yaml-cpp (JSON is a YAML subset), so no extra
// dependency is needed.

#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/ros2/String_.hpp>

#include "isaaclab/utils/math_utils.h"

namespace isaaclab
{

// One footstep target measured by the perception node. The subscriber fills
// it in the CAMERA OPTICAL frame (raw PnP); D435PelvisCamTransform::to_pelvis
// rewrites pos/quat in place into the PELVIS frame before it is handed to
// FootstepCommand.
struct VisionTarget
{
    int id = -1;              // footstep/board target index
    math::Vec3 pos = math::Vec3::Zero();
    math::Quat quat = math::Quat::Identity();
    int num_markers = 0;      // markers used for the PnP (1..4)
    float reproj_err = 0.0f;  // mean reprojection error [px]
};

// ---------------------------------------------------------------------------
// Camera-in-pelvis FK (analytic; mirrors the constants in g1_29dof.xml / the
// official URDF). Only the 3 waist joints (SDK 12: yaw, 13: roll, 14: pitch)
// sit between the pelvis and the HEAD-mounted D435i:
//   pelvis -Rz(q12)-> (-0.0039635, 0, 0.035) -Rx(q13)->
//   (0, 0, 0.019) -Ry(q14)-> head_link (0.0039635, 0, -0.054) ->
//   d435 (0.05366, 0.01753, 0.47387) pitched down 0.8307767 rad.
// An optional hand-eye correction (deploy.yaml vision.camera.extrinsic: the
// D435i camera_link pose in the HEAD_LINK frame, RealSense convention x
// forward / z up) overrides the default mount.
// ---------------------------------------------------------------------------
class D435PelvisCamTransform
{
public:
    D435PelvisCamTransform()
    {
        set_extrinsic(math::Vec3(0.05366f, 0.01753f, 0.47387f),
                      math::Vec3(0.0f, 0.8307767239493009f, 0.0f));
    }

    // pos/rpy: camera_link pose in the head_link frame.
    void set_extrinsic(const math::Vec3& pos, const math::Vec3& rpy)
    {
        ext_pos_ = pos;
        const Eigen::Matrix3f R_link =
            (Eigen::AngleAxisf(rpy[2], math::Vec3::UnitZ()) *
             Eigen::AngleAxisf(rpy[1], math::Vec3::UnitY()) *
             Eigen::AngleAxisf(rpy[0], math::Vec3::UnitX())).toRotationMatrix();
        // camera_link (x fwd, z up) -> OpenCV optical (x right, y down,
        // z fwd): columns = optical axes expressed in camera_link coords.
        Eigen::Matrix3f R_link_cv;
        R_link_cv << 0.f,  0.f, 1.f,
                    -1.f,  0.f, 0.f,
                     0.f, -1.f, 0.f;
        ext_R_cv_ = R_link * R_link_cv;
    }

    // (pos, R) of the camera OPTICAL frame in the pelvis frame.
    void cam_in_pelvis(float q_yaw, float q_roll, float q_pitch,
                       math::Vec3& pos, Eigen::Matrix3f& R) const
    {
        Eigen::Matrix3f Rw =
            Eigen::AngleAxisf(q_yaw, math::Vec3::UnitZ()).toRotationMatrix();
        math::Vec3 p = Rw * P_ROLL_;
        Rw = Rw * Eigen::AngleAxisf(q_roll, math::Vec3::UnitX()).toRotationMatrix();
        p += Rw * P_TORSO_;
        Rw = Rw * Eigen::AngleAxisf(q_pitch, math::Vec3::UnitY()).toRotationMatrix();
        p += Rw * P_HEAD_; // head_link frame reached
        pos = p + Rw * ext_pos_;
        R = Rw * ext_R_cv_;
    }

    // Rewrite camera-optical-frame targets into the pelvis frame, using the
    // waist joint positions of the current control tick.
    void to_pelvis(std::vector<VisionTarget>& ts,
                   float q_yaw, float q_roll, float q_pitch) const
    {
        math::Vec3 cp;
        Eigen::Matrix3f cR;
        cam_in_pelvis(q_yaw, q_roll, q_pitch, cp, cR);
        const math::Quat cq(cR);
        for (auto& t : ts)
        {
            t.pos = cp + cR * t.pos;
            t.quat = math::quat_mul(cq, t.quat);
        }
    }

private:
    const math::Vec3 P_ROLL_ = math::Vec3(-0.0039635f, 0.0f, 0.035f);
    const math::Vec3 P_TORSO_ = math::Vec3(0.0f, 0.0f, 0.019f);
    const math::Vec3 P_HEAD_ = math::Vec3(0.0039635f, 0.0f, -0.054f);
    math::Vec3 ext_pos_ = math::Vec3::Zero();
    Eigen::Matrix3f ext_R_cv_ = Eigen::Matrix3f::Identity();
};

class VisionFootTargetSubscriber
    : public unitree::robot::SubscriptionBase<std_msgs::msg::dds_::String_>
{
public:
    using SharedPtr = std::shared_ptr<VisionFootTargetSubscriber>;

    explicit VisionFootTargetSubscriber(const std::string& topic = "rt/footstep_vision")
    : SubscriptionBase<std_msgs::msg::dds_::String_>(topic) {}

    // Parse the latest payload into `out`. Returns true only when a message
    // newer than the previously taken one was parsed successfully (out may be
    // empty: a valid frame with no detected targets).
    bool take(std::vector<VisionTarget>& out)
    {
        std::string payload;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (seq_ == taken_seq_) return false;
            taken_seq_ = seq_;
            payload = msg_.data();
        }
        if (payload.empty()) return false;

        try
        {
            YAML::Node n = YAML::Load(payload);
            out.clear();
            if (n["targets"])
            {
                for (const auto& t : n["targets"])
                {
                    VisionTarget v;
                    v.id = t["id"].as<int>();
                    v.pos = math::Vec3(t["pos"][0].as<float>(),
                                       t["pos"][1].as<float>(),
                                       t["pos"][2].as<float>());
                    v.quat = math::Quat(t["quat"][0].as<float>(),
                                        t["quat"][1].as<float>(),
                                        t["quat"][2].as<float>(),
                                        t["quat"][3].as<float>());
                    v.quat.normalize();
                    if (t["nmk"]) v.num_markers = t["nmk"].as<int>();
                    if (t["err"]) v.reproj_err = t["err"].as<float>();
                    out.push_back(v);
                }
            }
            parse_error_logged_ = false;
            return true;
        }
        catch (const std::exception& e)
        {
            if (!parse_error_logged_)
            {
                parse_error_logged_ = true;
                spdlog::warn("[FootVision] failed to parse payload: {}", e.what());
            }
            return false;
        }
    }

protected:
    void post_communication() override { ++seq_; }

private:
    uint64_t seq_ = 0;
    uint64_t taken_seq_ = 0;
    bool parse_error_logged_ = false;
};

} // namespace isaaclab
