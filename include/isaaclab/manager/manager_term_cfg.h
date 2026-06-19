// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <deque>
#include <vector>
#include <functional>
#include <numeric>

namespace isaaclab
{

class ManagerBasedRLEnv;

using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv*, YAML::Node)>;

struct ObservationTermCfg
{
    YAML::Node params;
    ObsFunc func;
    std::vector<float> clip;
    std::vector<float> scale;
    int history_length = 1;
    // Down-samples the history buffer: keeps `history_length` frames spaced
    // `skip_history_tick` ticks apart (matches DyrosObservationManager).
    int skip_history_tick = 1;
    bool scale_first = false;

    int max_length() const { return history_length * std::max(1, skip_history_tick); }

    void reset(std::vector<float> obs)
    {
        buff_.clear();
        for(int i(0); i < max_length(); ++i) add(obs);
    }

    void add(std::vector<float> obs)
    {
        for(int j = 0; j < obs.size(); ++j)
        {
            if(scale_first) {
                if(!scale.empty()) obs[j] *= scale[j];
                if (!clip.empty()) {
                    obs[j] = std::clamp(obs[j], clip[0], clip[1]);
                }
            } else {
                if (!clip.empty()) {
                    obs[j] = std::clamp(obs[j], clip[0], clip[1]);
                }
                if(!scale.empty()) obs[j] *= scale[j];
            }
        }
        buff_.push_back(obs);

        if ((int)buff_.size() > max_length()) buff_.pop_front();
    }

    // Returns the h-th history frame (h in [0, history_length)), oldest first,
    // selecting every `skip_history_tick`-th frame aligned to the newest entry.
    const std::vector<float> & get(int h) const
    {
        const int skip = std::max(1, skip_history_tick);
        const int idx = ((int)buff_.size() - 1) - (history_length - 1 - h) * skip;
        return buff_[std::clamp(idx, 0, (int)buff_.size() - 1)];
    }

    const std::vector<float> get() const
    {
        std::vector<float> concatenated;
        for (const auto& entry : buff_) {
            concatenated.insert(concatenated.end(), entry.begin(), entry.end());
        }
        return concatenated;
    }

    const std::size_t size() const { return std::accumulate(buff_.begin(), buff_.end(), 0,
        [](std::size_t sum, const auto& v) { return sum + v.size(); }); }

private:
    // Complete circular buffer with most recent entry at the end and oldest entry at the beginning.
    std::deque<std::vector<float>> buff_;
};

};