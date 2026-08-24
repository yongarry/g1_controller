// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-robot teleop / CSV generator for the 16-dim foot goal consumed by the
// Isaac Lab G1 MindYourStep policy (same layout as MindYourStepFootCommand /
// GoalDoubleFootPlacement):
//
//   [ l_pos_offset(3), l_orn_offset(4), r_pos_offset(3), r_orn_offset(4),
//     gait_info(2) ]
//
// Joystick mode follows the original mind-your-step deploy GaitGenerator.
// CSV mode replays per-step local footholds from footcommands_mys.csv
// with the same semantics as Footstep csv mode: step_* is in the current
// stance-foot yaw frame. step_yaw is that row's relative heading, not a
// running sum -- see cmd/convert2mys.py.

#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <filesystem>
#include <map>

#include <spdlog/spdlog.h>

namespace isaaclab
{

class MysGaitCommand
{
public:
    enum class Move { STILL, FWD, LEFT, RIGHT };

    struct Config
    {
        float feet_distance = 0.2f;
        int stop_steps = 4;
        double gait_frequency = 1.0;
        double policy_dt = 0.02;
        float vert_max = 0.5f;
        float lat_max = 0.3f;
        float yaw_max = 1.5707963f;
        float deadzone = 0.1f;
    };

    // One scripted foothold (stance-frame local command), matching footcommands.csv.
    struct StepCmd
    {
        int swing = 0;       // 0: left swings, 1: right swings
        float step_x = 0.f;
        float step_y = 0.f;  // positive magnitude
        float step_z = 0.f;
        float step_yaw = 0.f;
        float ssp_t = 0.7f;
        float dsp_t = 0.15f;
        float height = 0.08f;
    };

    explicit MysGaitCommand(const Config& cfg) : cfg_(cfg) { reset(); }

    // Load a local foot-command CSV (same columns as footcommands.csv) and
    // switch this generator into CSV replay mode.
    void load_csv(const std::string& path)
    {
        load_csv_(path);
        csv_mode_ = true;
        reset();
        spdlog::info("[MysGait/CSV] loaded {} steps from '{}' (start foot={})",
                     steps_.size(), path, steps_.front().swing == 0 ? "L" : "R");
    }

    bool csv_mode() const { return csv_mode_; }
    int plan_size() const { return (int)steps_.size(); }
    bool plan_done() const { return csv_mode_ && plan_done_; }

    // True on the tick a scripted foothold just completed (landing evaluated).
    bool step_completed() const { return step_completed_; }
    int step_counter() const { return step_counter_; }
    bool last_step_swing_right() const { return last_swing_ == 1; }
    float last_cmd_x() const { return last_cmd_x_; }
    float last_cmd_y() const { return last_cmd_y_; }
    float last_cmd_yaw() const { return last_cmd_yaw_; }
    float last_ssp_t() const { return last_ssp_; }
    float last_dsp_t() const { return last_dsp_; }
    float last_height() const { return last_height_; }
    // Active command (stance-frame) while a step is in progress; used by the
    // state to form landing error against FK when step_completed() is true.
    float active_cmd_x() const { return active_cmd_x_; }
    float active_cmd_y() const { return active_cmd_y_; }
    float active_cmd_yaw() const { return active_cmd_yaw_; }
    int active_swing() const { return active_swing_; } // foot that just landed
    bool has_active_step() const { return has_active_; }

    void reset()
    {
        gait_process_ = 0.0;
        swing_foot_idx_ = 0;
        vertical_dist_ = 0.0f;
        lateral_dist_ = 0.0f;
        steering_angle_ = 0.0f;
        gaits_to_still_ = 0;
        move_dir_ = Move::STILL;
        next_dir_ = Move::STILL;
        cursor_ = 0;
        step_counter_ = 0;
        step_completed_ = false;
        plan_done_ = false;
        end_logged_ = false;
        has_active_ = false;
        set_offsets_(still_offsets_());
        cmd_[14] = 0.0f;
        cmd_[15] = 0.0f;

        // Park one tick before the half-cycle boundary that matches the CSV's
        // first swing foot, so the first compute() samples at gp = 0.0 (left)
        // or 0.5 (right) rather than 0.02 / 0.52.
        if (csv_mode_ && !steps_.empty())
        {
            const double off = gp_off_();
            if (steps_.front().swing == 1) // right swings first
            {
                gait_process_ = 0.5 - off;
                swing_foot_idx_ = 0;
            }
            else
            {
                gait_process_ = 0.0 - off;
                swing_foot_idx_ = 1;
            }
            if (gait_process_ < 0.0) gait_process_ += 1.0;
        }
    }

    void set_input(float fwd, float lat, float turn)
    {
        if (csv_mode_) return; // sticks ignored while replaying a plan
        vertical_dist_ = clampf(fwd, -1.0f, 1.0f) * cfg_.vert_max;
        lateral_dist_ = clampf(lat, -1.0f, 1.0f) * cfg_.lat_max;
        steering_angle_ = clampf(turn, -1.0f, 1.0f) * cfg_.yaw_max;

        const float f = std::abs(fwd), l = std::abs(lat);
        if (f < cfg_.deadzone && l < cfg_.deadzone) next_dir_ = Move::STILL;
        else if (f >= l)                            next_dir_ = Move::FWD;
        else                                        next_dir_ = (lat >= 0.0f) ? Move::LEFT : Move::RIGHT;
    }

    // Call once per policy tick. When a foothold completes, step_completed() is
    // true for that tick and active_cmd_* hold the command that just finished
    // so the caller can measure the landing and write the eval row.
    void compute()
    {
        step_completed_ = false;

        gait_process_ = std::fmod(gait_process_ + gp_off_(), 1.0);
        if (gait_process_ < 0.0) gait_process_ += 1.0;
        const int swing = (gait_process_ < 0.5) ? 0 : 1;
        const bool sample_goal = (swing != swing_foot_idx_);
        const int completed_swing = swing_foot_idx_; // foot that just finished
        swing_foot_idx_ = swing;

        if (csv_mode_)
        {
            compute_csv_(sample_goal, completed_swing);
            return;
        }

        // Switch direction only on a half-cycle boundary so gait_info and the
        // 14-D foot target jump together (training resamples at the same tick).
        // Applying move_dir_ immediately used to spin gp_info mid-step while the
        // offsets stayed on the previous command, which the policy never saw.
        if (sample_goal && next_dir_ != move_dir_)
        {
            if (next_dir_ != Move::STILL) gaits_to_still_ = cfg_.stop_steps;
            move_dir_ = next_dir_;
        }

        const bool gait_clock_on = (move_dir_ != Move::STILL) || (gaits_to_still_ > 0);
        if (gait_clock_on)
        {
            const double a = 2.0 * M_PI * gait_process_;
            cmd_[14] = static_cast<float>(std::cos(a));
            cmd_[15] = static_cast<float>(std::sin(a));
        }
        else
        {
            cmd_[14] = 0.0f;
            cmd_[15] = 0.0f;
        }

        if (!sample_goal) return;
        switch (move_dir_)
        {
        case Move::STILL:
            set_offsets_(still_offsets_());
            gaits_to_still_ = std::max(0, gaits_to_still_ - 1);
            break;
        case Move::FWD:
            set_offsets_(vertical_offsets_());
            break;
        case Move::LEFT:
        case Move::RIGHT:
            set_offsets_(lateral_offsets_());
            break;
        }
    }

    const std::array<float, 16>& command() const { return cmd_; }

    double gait_process() const { return gait_process_; }
    int swing_foot_idx() const { return swing_foot_idx_; }
    Move move_dir() const { return move_dir_; }
    static const char* move_name(Move m)
    {
        switch (m) {
        case Move::FWD: return "FWD";
        case Move::LEFT: return "LEFT";
        case Move::RIGHT: return "RIGHT";
        default: return "STILL";
        }
    }

private:
    struct Offsets { std::array<float, 3> lp, rp; std::array<float, 4> lq, rq; };

    static constexpr std::array<float, 4> kIdentity{1.0f, 0.0f, 0.0f, 0.0f};
    static constexpr std::array<float, 3> kZero{0.0f, 0.0f, 0.0f};
    static float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
    double gp_off_() const { return cfg_.policy_dt * cfg_.gait_frequency; }

    static std::array<float, 4> yaw_quat_wxyz_(float a)
    {
        return {std::cos(0.5f * a), 0.0f, 0.0f, std::sin(0.5f * a)};
    }

    // Signed lateral target in the stance frame (matches FootstepCommand).
    static float signed_y_(int swing /*0 L, 1 R*/, float y_mag)
    {
        return (swing == 1) ? -std::abs(y_mag) : std::abs(y_mag);
    }

    Offsets offsets_from_step_(const StepCmd& s) const
    {
        Offsets o{kZero, kZero, kIdentity, kIdentity};
        const float sy = signed_y_(s.swing, s.step_y);
        const auto q = yaw_quat_wxyz_(s.step_yaw);
        if (s.swing == 0)
        {
            o.lp = {s.step_x, sy, s.step_z};
            o.lq = q;
        }
        else
        {
            o.rp = {s.step_x, sy, s.step_z};
            o.rq = q;
        }
        return o;
    }

    void compute_csv_(bool sample_goal, int completed_swing)
    {
        if (sample_goal)
        {
            // A previous scripted foothold just landed.
            if (has_active_)
            {
                last_swing_ = active_swing_;
                last_cmd_x_ = active_cmd_x_;
                last_cmd_y_ = active_cmd_y_;
                last_cmd_yaw_ = active_cmd_yaw_;
                last_ssp_ = active_ssp_;
                last_dsp_ = active_dsp_;
                last_height_ = active_height_;
                ++step_counter_;
                step_completed_ = true;
                has_active_ = false;
                (void)completed_swing;
            }

            if (cursor_ < (int)steps_.size())
            {
                const StepCmd& s = steps_[cursor_];
                if (s.swing != swing_foot_idx_)
                {
                    spdlog::warn("[MysGait/CSV] step {} foot mismatch (csv swing={}, gait swing={}); applying anyway",
                                 cursor_ + 1, s.swing == 0 ? "L" : "R",
                                 swing_foot_idx_ == 0 ? "L" : "R");
                }
                set_offsets_(offsets_from_step_(s));
                active_swing_ = s.swing;
                active_cmd_x_ = s.step_x;
                active_cmd_y_ = signed_y_(s.swing, s.step_y);
                active_cmd_yaw_ = s.step_yaw;
                active_ssp_ = s.ssp_t;
                active_dsp_ = s.dsp_t;
                active_height_ = s.height;
                has_active_ = true;
                spdlog::info("[MysGait/CSV] step {}/{} (foot={}, x={:.3f} y={:.3f} yaw={:.3f})",
                             cursor_ + 1, steps_.size(),
                             s.swing == 0 ? "L" : "R",
                             active_cmd_x_, active_cmd_y_, active_cmd_yaw_);
                ++cursor_;
            }
            else
            {
                // Last foothold done: settle like joystick (swing-only still + gait
                // clock) for stop_steps half-cycles, then fully-still both feet.
                // Jumping straight to both-feet still with gp=[0,0] is an input
                // the policy never sees at a walking→stand transition.
                if (!plan_done_)
                {
                    plan_done_ = true;
                    gaits_to_still_ = cfg_.stop_steps;
                    if (!end_logged_)
                    {
                        end_logged_ = true;
                        spdlog::info("[MysGait/CSV] plan complete after {} steps; settling {} half-steps.",
                                     step_counter_, cfg_.stop_steps);
                    }
                }
                set_offsets_(still_offsets_());
                if (gaits_to_still_ > 0)
                    gaits_to_still_ = std::max(0, gaits_to_still_ - 1);
                if (gaits_to_still_ == 0)
                    set_offsets_(still_offsets_()); // both feet, gp off below
            }
        }

        // After the command for this tick is known, so gp_info cannot stay on
        // for a frame of fully-still offsets (or off during an active step).
        const bool gait_clock_on = has_active_ || (gaits_to_still_ > 0);
        if (gait_clock_on)
        {
            const double a = 2.0 * M_PI * gait_process_;
            cmd_[14] = static_cast<float>(std::cos(a));
            cmd_[15] = static_cast<float>(std::sin(a));
        }
        else
        {
            cmd_[14] = 0.0f;
            cmd_[15] = 0.0f;
        }
    }

    // Hold-still goal.
    // Settling (gaits_to_still > 0): swing foot at nominal stance width, stance at 0
    //   -- same structure as a walking command, matches GaitGenerator._gen_still_cmd
    //   while the gait clock is still running.
    // Fully still: BOTH feet at +-feet_distance and gp_info = [0,0], matching
    //   GoalDoubleFootPlacement's steady_still_flag (still_phase and num_gaits >= 2).
    // The previous swing-only command in the fully-still case swapped which foot
    // was non-zero every 0.5 s and produced a twitch at each hidden step boundary.
    Offsets still_offsets_() const
    {
        Offsets o{kZero, kZero, kIdentity, kIdentity};
        const std::array<float, 3> l{0.0f, cfg_.feet_distance, 0.0f};
        const std::array<float, 3> r{0.0f, -cfg_.feet_distance, 0.0f};
        if (gaits_to_still_ > 0)
        {
            o.lp = (swing_foot_idx_ == 0) ? l : kZero;
            o.rp = (swing_foot_idx_ == 1) ? r : kZero;
        }
        else
        {
            o.lp = l;
            o.rp = r;
        }
        return o;
    }

    Offsets vertical_offsets_() const
    {
        Offsets o{kZero, kZero, kIdentity, kIdentity};
        const float sa = clampf(steering_angle_, -static_cast<float>(M_PI), static_cast<float>(M_PI));
        const int steering_foot = (sa >= 0.0f) ? 0 : 1;
        const auto sq = yaw_quat_wxyz_(sa);

        o.lp = (swing_foot_idx_ == 0)
            ? std::array<float, 3>{vertical_dist_, cfg_.feet_distance, 0.0f} : kZero;
        o.rp = (swing_foot_idx_ == 1)
            ? std::array<float, 3>{vertical_dist_, -cfg_.feet_distance, 0.0f} : kZero;
        o.lq = (steering_foot == 0) ? sq : kIdentity;
        o.rq = (steering_foot == 1) ? sq : kIdentity;
        return o;
    }

    Offsets lateral_offsets_() const
    {
        Offsets o{kZero, kZero, kIdentity, kIdentity};
        int dir = (lateral_dist_ >= 0.0f) ? 1 : -1;
        if (std::abs(lateral_dist_) < 1e-4f) dir = 0;

        const float lat = cfg_.feet_distance * dir + lateral_dist_;
        const float evil = -cfg_.feet_distance * dir / 2.0f;

        if (dir == 1)
        {
            o.lp = (swing_foot_idx_ == 0) ? std::array<float, 3>{0.0f, lat, 0.0f} : kZero;
            o.rp = (swing_foot_idx_ == 1) ? std::array<float, 3>{0.0f, evil, 0.0f} : kZero;
        }
        else if (dir == -1)
        {
            o.lp = (swing_foot_idx_ == 0) ? std::array<float, 3>{0.0f, evil, 0.0f} : kZero;
            o.rp = (swing_foot_idx_ == 1) ? std::array<float, 3>{0.0f, lat, 0.0f} : kZero;
        }
        else
        {
            o.lp = {0.0f, cfg_.feet_distance, 0.0f};
            o.rp = {0.0f, -cfg_.feet_distance, 0.0f};
        }
        return o;
    }

    void set_offsets_(const Offsets& o)
    {
        for (int i = 0; i < 3; ++i) cmd_[i] = o.lp[i];
        for (int i = 0; i < 4; ++i) cmd_[3 + i] = o.lq[i];
        for (int i = 0; i < 3; ++i) cmd_[7 + i] = o.rp[i];
        for (int i = 0; i < 4; ++i) cmd_[10 + i] = o.rq[i];
    }

    static std::string trim_(std::string s)
    {
        auto notspace = [](int c){ return !std::isspace(c); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
        s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
        return s;
    }
    static std::string lower_(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return s;
    }
    static std::vector<std::string> split_(const std::string& line)
    {
        std::vector<std::string> out;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) out.push_back(trim_(cell));
        return out;
    }

    void load_csv_(const std::string& path)
    {
        if (!std::filesystem::exists(path))
            throw std::runtime_error("MysGaitCommand: CSV not found: " + path);
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("MysGaitCommand: cannot open CSV: " + path);

        std::string line;
        if (!std::getline(f, line))
            throw std::runtime_error("MysGaitCommand: empty CSV: " + path);
        std::vector<std::string> header = split_(line);
        std::map<std::string, int> col;
        for (int i = 0; i < (int)header.size(); ++i) col[lower_(header[i])] = i;

        for (const char* c : {"foot", "step_x", "step_y", "step_yaw"})
            if (!col.count(c))
                throw std::runtime_error(std::string("MysGaitCommand: missing column '") + c + "' in " + path);

        const bool has_z = col.count("step_z");
        const bool has_ssp = col.count("ssp_t");
        const bool has_dsp = col.count("dsp_t");
        const bool has_h = col.count("height");

        auto getf = [&](const std::vector<std::string>& cells, const std::string& name, int ln) -> float {
            try {
                size_t pos = 0;
                float v = std::stof(cells[col[name]], &pos);
                if (std::isnan(v)) throw std::runtime_error("NaN");
                return v;
            } catch (...) {
                throw std::runtime_error("MysGaitCommand: bad '" + name + "' at line " +
                                         std::to_string(ln) + " in " + path);
            }
        };

        steps_.clear();
        int ln = 1;
        while (std::getline(f, line))
        {
            ++ln;
            if (trim_(line).empty()) continue;
            auto cells = split_(line);
            if ((int)cells.size() < (int)header.size())
                throw std::runtime_error("MysGaitCommand: short row at line " + std::to_string(ln));

            const std::string foot = lower_(cells[col["foot"]]);
            StepCmd s;
            if (foot == "r" || foot == "right") s.swing = 1;
            else if (foot == "l" || foot == "left") s.swing = 0;
            else throw std::runtime_error("MysGaitCommand: bad foot label at line " + std::to_string(ln));

            s.step_x = getf(cells, "step_x", ln);
            s.step_y = std::abs(getf(cells, "step_y", ln));
            s.step_yaw = getf(cells, "step_yaw", ln);
            if (has_z) s.step_z = getf(cells, "step_z", ln);
            if (has_ssp) s.ssp_t = getf(cells, "ssp_t", ln);
            if (has_dsp) s.dsp_t = getf(cells, "dsp_t", ln);
            if (has_h) s.height = getf(cells, "height", ln);
            steps_.push_back(s);
        }
        if (steps_.empty())
            throw std::runtime_error("MysGaitCommand: no data rows in " + path);
        end_logged_ = false;
    }

    Config cfg_;
    std::array<float, 16> cmd_{};

    double gait_process_ = 0.0;
    int swing_foot_idx_ = 0;
    float vertical_dist_ = 0.0f;
    float lateral_dist_ = 0.0f;
    float steering_angle_ = 0.0f;
    int gaits_to_still_ = 0;
    Move move_dir_ = Move::STILL;
    Move next_dir_ = Move::STILL;

    // CSV replay
    bool csv_mode_ = false;
    std::vector<StepCmd> steps_;
    int cursor_ = 0;
    bool plan_done_ = false;
    bool end_logged_ = false;
    bool step_completed_ = false;
    int step_counter_ = 0;

    bool has_active_ = false;
    int active_swing_ = 0;
    float active_cmd_x_ = 0, active_cmd_y_ = 0, active_cmd_yaw_ = 0;
    float active_ssp_ = 0.7f, active_dsp_ = 0.15f, active_height_ = 0.08f;

    int last_swing_ = 0;
    float last_cmd_x_ = 0, last_cmd_y_ = 0, last_cmd_yaw_ = 0;
    float last_ssp_ = 0.7f, last_dsp_ = 0.15f, last_height_ = 0.08f;
};

} // namespace isaaclab
