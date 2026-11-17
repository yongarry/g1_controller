// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Depth image of the stage-2 footstep planner over DDS (unitree_go HeightMap_ used as an image container), as
// published by unitree_mujoco's DepthCamera or realsense_depth_pub.

#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/go2/HeightMap_.hpp>

namespace isaaclab
{

// Depth image of the footstep planner over DDS, in the same message type: width x height, row-major (row 0 = top),
// data = distance to the image plane [m] (0 / NaN = no return), as published by unitree_mujoco's DepthCamera.
class DepthSubscriber
    : public unitree::robot::SubscriptionBase<unitree_go::msg::dds_::HeightMap_>
{
public:
    DepthSubscriber(const std::string& topic, uint32_t max_age_ms)
    : SubscriptionBase<unitree_go::msg::dds_::HeightMap_>(topic)
    {
        set_timeout_ms(max_age_ms);
    }

    // Copies the newest image; false when none arrived within max_age or its size is not width x height.
    bool latest(std::vector<float>& out, int width, int height)
    {
        if (isTimeout()) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if ((int)msg_.width() != width || (int)msg_.height() != height || msg_.data().size() != (size_t)width * height)
            return false;
        out = msg_.data();
        return true;
    }
};

} // namespace isaaclab
