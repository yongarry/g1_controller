// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// DDS subscriber for the ArUco footstep-target perception node
// (cmd/aruco_footstep_perception.py).
//
// The node publishes a JSON payload on a std_msgs String topic
// (default rt/footstep_vision):
//
//   {"stamp": <unix time>, "targets": [
//      {"id": 0, "pos": [x, y, z], "quat": [w, x, y, z], "nmk": 4, "err": 0.3},
//      ... ]}
//
// Poses are the footstep-target frames expressed in the PELVIS frame
// (target origin = footstep center on the top surface, x = footstep yaw
// direction, z = up). FootstepCommand (vision mode) converts them to the
// stance-foot frame / accumulated world frame and plans on them.
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

// One footstep target measured by the perception node (pelvis frame).
struct VisionTargetPelvis
{
    int id = -1;              // footstep/board target index
    math::Vec3 pos = math::Vec3::Zero();
    math::Quat quat = math::Quat::Identity();
    int num_markers = 0;      // markers used for the PnP (1..4)
    float reproj_err = 0.0f;  // mean reprojection error [px]
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
    bool take(std::vector<VisionTargetPelvis>& out)
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
                    VisionTargetPelvis v;
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
