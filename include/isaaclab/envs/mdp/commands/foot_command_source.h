// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Foot-command source abstraction for State_Footstep.
//
// The footstep planner (FootstepCommand) consumes a single nominal
// FootCommandInput each control tick, sampled at step boundaries. This file
// wraps "where that input comes from" behind a small interface so the source
// can be selected from deploy.yaml without touching the planner / policy:
//
//   * JoystickFootCommandSource - the original, default behaviour (unchanged).
//   * CSVFootCommandSource       - replays foot commands from footcommands.csv.
//
// Selected via `footstep.command_source` ("joystick" | "csv") in deploy.yaml.

#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

namespace isaaclab
{

// Operator-driven nominal foot-step command (clamped to trained ranges by the
// planner). This is the single unit both command sources produce.
struct FootCommandInput
{
    float step_x = 0.0f;   // forward step length [m]
    float step_y = 0.237f; // lateral step width (base, always positive) [m]
    float step_z = 0.0f;   // step height change (0 for 2d) [m]
    float step_yaw = 0.0f; // per-step turn [rad]
    float ssp_t = 0.75f;   // single support time [s]
    float dsp_t = 0.15f;   // double support time [s]
    float height = 0.075f; // swing apex height [m]
    float com_z = 0.0f;    // com height offset [m]
};

// Absolute (world-frame) foot-step target used by the "global" command mode.
// One entry per planned footstep: the SWING foot's landing pose plus the same
// timing/height/com fields carried by FootCommandInput. Positions are expressed
// in the initial-stance-foot frame (the first stance foot is the world origin),
// so the planner accumulates the achieved stance pose and recomputes the local
// command from the measured stance foot to the indexed target every step.
struct GlobalFootTarget
{
    float x = 0.0f;      // world swing-landing x [m]
    float y = 0.0f;      // world swing-landing y [m]
    float z = 0.0f;      // world swing-landing z [m]
    float yaw = 0.0f;    // world swing-landing yaw [rad]
    float ssp_t = 0.75f; // single support time [s]
    float dsp_t = 0.15f; // double support time [s]
    float height = 0.075f; // swing apex height [m]
    float com_z = 0.0f;  // com height offset [m]
    int phase = 0;       // 0: right foot swings, 1: left foot swings
};

// ---------------------------------------------------------------------------
// Interface
// ---------------------------------------------------------------------------
class FootCommandSource
{
public:
    virtual ~FootCommandSource() = default;
    // Nominal command for the current tick (read every control tick).
    virtual FootCommandInput input() = 0;
    // Called once when a footstep completes (step boundary). No-op by default.
    virtual void advance() {}
    // Initial swing-foot phase (0: right swings first, 1: left swings first).
    virtual int start_phase_indicator() const { return 0; }
    virtual const char* name() const = 0;
};

// ---------------------------------------------------------------------------
// Joystick source (original behaviour, unchanged mapping)
// ---------------------------------------------------------------------------
class JoystickFootCommandSource : public FootCommandSource
{
public:
    JoystickFootCommandSource(unitree::common::UnitreeJoystick* joy,
                              const FootCommandInput& def,
                              float x_scale, float y_scale, float yaw_scale,
                              int start_phase)
    : joy_(joy), def_(def), x_(x_scale), y_(y_scale), yaw_(yaw_scale), start_phase_(start_phase) {}

    FootCommandInput input() override
    {
        FootCommandInput in = def_;
        if (joy_)
        {
            in.step_x = joy_->ly() * x_;        // forward
            in.step_yaw = -joy_->rx() * yaw_;   // turn
            // lateral intent widens/narrows the (sign-applied) step width slightly
            in.step_y = def_.step_y + std::abs(joy_->lx()) * y_;
        }
        return in;
    }

    int start_phase_indicator() const override { return start_phase_; }
    const char* name() const override { return "joystick"; }

private:
    unitree::common::UnitreeJoystick* joy_;
    FootCommandInput def_;
    float x_, y_, yaw_;
    int start_phase_;
};

// ---------------------------------------------------------------------------
// CSV source (replays footcommands.csv in order, advancing per completed step)
// ---------------------------------------------------------------------------
class CSVFootCommandSource : public FootCommandSource
{
public:
    CSVFootCommandSource(const std::string& path, const FootCommandInput& def)
    : def_(def)
    {
        load_(path);
        spdlog::info("[FootCommand/CSV] loaded {} foot commands from '{}' (start foot={})",
                     rows_.size(), path, labels_.front());
    }

    FootCommandInput input() override
    {
        return rows_[std::min(cursor_, (int)rows_.size() - 1)];
    }

    void advance() override
    {
        if (cursor_ + 1 < (int)rows_.size())
        {
            ++cursor_;
            const auto& r = rows_[cursor_];
            spdlog::info("[FootCommand/CSV] step {}/{} (foot={}, x={:.3f} y={:.3f} yaw={:.3f})",
                         cursor_ + 1, rows_.size(), labels_[cursor_], r.step_x, r.step_y, r.step_yaw);
        }
        else if (!end_logged_)
        {
            end_logged_ = true;
            spdlog::info("[FootCommand/CSV] reached end ({} steps); holding last command.", rows_.size());
        }
    }

    int start_phase_indicator() const override { return start_phase_; }
    const char* name() const override { return "csv"; }

private:
    static std::string trim_(std::string s)
    {
        auto notspace = [](int c){ return !std::isspace(c); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
        s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
        return s;
    }
    static std::string lower_(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
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

    void load_(const std::string& path)
    {
        if (!std::filesystem::exists(path))
            throw std::runtime_error("CSVFootCommandSource: file not found: " + path);

        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("CSVFootCommandSource: cannot open file: " + path);

        std::string line;
        // header
        if (!std::getline(f, line))
            throw std::runtime_error("CSVFootCommandSource: empty file: " + path);
        std::vector<std::string> header = split_(line);
        std::map<std::string, int> col;
        for (int i = 0; i < (int)header.size(); ++i) col[lower_(header[i])] = i;

        const std::vector<std::string> required =
            {"foot", "step_x", "step_y", "step_yaw", "ssp_t", "dsp_t", "height"};
        for (const auto& c : required)
            if (col.find(c) == col.end())
                throw std::runtime_error("CSVFootCommandSource: missing required column '" + c + "' in " + path);
        const bool has_step_z = col.count("step_z");
        const bool has_com_z = col.count("com_z");

        auto get_f = [&](const std::vector<std::string>& cells, const std::string& name, int line_no) -> float {
            const std::string& s = cells[col[name]];
            try {
                size_t pos = 0;
                float v = std::stof(s, &pos);
                if (std::isnan(v)) throw std::runtime_error("NaN");
                return v;
            } catch (const std::exception&) {
                throw std::runtime_error("CSVFootCommandSource: invalid/NaN value for '" + name +
                                         "' at line " + std::to_string(line_no) + " in " + path);
            }
        };

        int line_no = 1;
        while (std::getline(f, line))
        {
            ++line_no;
            const std::string trimmed = trim_(line);
            if (trimmed.empty()) continue; // skip blank lines
            std::vector<std::string> cells = split_(line);
            if ((int)cells.size() < (int)header.size())
                throw std::runtime_error("CSVFootCommandSource: too few columns at line " +
                                         std::to_string(line_no) + " in " + path);

            // foot label
            const std::string foot = lower_(cells[col["foot"]]);
            int phase;
            if (foot == "r" || foot == "right") phase = 0;       // right swing
            else if (foot == "l" || foot == "left") phase = 1;   // left swing
            else throw std::runtime_error("CSVFootCommandSource: invalid foot label '" +
                                          cells[col["foot"]] + "' at line " + std::to_string(line_no) +
                                          " (expected L/R/left/right) in " + path);

            FootCommandInput in = def_;
            in.step_x = get_f(cells, "step_x", line_no);
            in.step_y = get_f(cells, "step_y", line_no);
            in.step_yaw = get_f(cells, "step_yaw", line_no);
            in.ssp_t = get_f(cells, "ssp_t", line_no);
            in.dsp_t = get_f(cells, "dsp_t", line_no);
            in.height = get_f(cells, "height", line_no);
            if (has_step_z) in.step_z = get_f(cells, "step_z", line_no);
            if (has_com_z) in.com_z = get_f(cells, "com_z", line_no);

            rows_.push_back(in);
            labels_.push_back(foot == "r" || foot == "right" ? "R" : "L");
            phases_.push_back(phase);
        }

        if (rows_.empty())
            throw std::runtime_error("CSVFootCommandSource: no data rows in " + path);

        start_phase_ = phases_.front();

        // sanity: footsteps should alternate feet
        for (size_t i = 1; i < phases_.size(); ++i)
            if (phases_[i] == phases_[i - 1])
            {
                spdlog::warn("[FootCommand/CSV] foot labels do not strictly alternate "
                             "(row {} and {} are both '{}'); the planner alternates phase internally.",
                             i, i + 1, labels_[i]);
                break;
            }
    }

    std::vector<FootCommandInput> rows_;
    std::vector<std::string> labels_;
    std::vector<int> phases_;
    FootCommandInput def_;
    int cursor_ = 0;
    int start_phase_ = 0;
    bool end_logged_ = false;
};

// ---------------------------------------------------------------------------
// Global-plan loader (footcommands_global.csv)
//
// Reads a per-row CSV of absolute (world-frame) swing-foot landing targets:
//   foot,pos_x,pos_y,pos_z,yaw,ssp_t,dsp_t,height[,com_z]
// `foot` (L/R) sets the swing phase per step (matches the local CSV). The
// returned plan is consumed by FootstepCommand in "global" command mode.
// ---------------------------------------------------------------------------
inline std::vector<GlobalFootTarget> load_global_foot_plan(const std::string& path,
                                                           const GlobalFootTarget& def = GlobalFootTarget{})
{
    auto trim = [](std::string s) {
        auto notspace = [](int c){ return !std::isspace(c); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
        s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
        return s;
    };
    auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        return s;
    };
    auto split = [&](const std::string& line) {
        std::vector<std::string> out;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) out.push_back(trim(cell));
        return out;
    };

    if (!std::filesystem::exists(path))
        throw std::runtime_error("load_global_foot_plan: file not found: " + path);
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("load_global_foot_plan: cannot open file: " + path);

    std::string line;
    if (!std::getline(f, line))
        throw std::runtime_error("load_global_foot_plan: empty file: " + path);
    std::vector<std::string> header = split(line);
    std::map<std::string, int> col;
    for (int i = 0; i < (int)header.size(); ++i) col[lower(header[i])] = i;

    const std::vector<std::string> required =
        {"foot", "pos_x", "pos_y", "pos_z", "yaw", "ssp_t", "dsp_t", "height"};
    for (const auto& c : required)
        if (col.find(c) == col.end())
            throw std::runtime_error("load_global_foot_plan: missing required column '" + c + "' in " + path);
    const bool has_com_z = col.count("com_z");

    std::vector<GlobalFootTarget> plan;
    int line_no = 1;
    while (std::getline(f, line))
    {
        ++line_no;
        if (trim(line).empty()) continue;
        std::vector<std::string> cells = split(line);
        if ((int)cells.size() < (int)header.size())
            throw std::runtime_error("load_global_foot_plan: too few columns at line " +
                                     std::to_string(line_no) + " in " + path);

        auto get_f = [&](const std::string& name) -> float {
            try { return std::stof(cells[col[name]]); }
            catch (const std::exception&) {
                throw std::runtime_error("load_global_foot_plan: invalid value for '" + name +
                                         "' at line " + std::to_string(line_no) + " in " + path);
            }
        };

        const std::string foot = lower(cells[col["foot"]]);
        GlobalFootTarget t = def;
        if (foot == "r" || foot == "right") t.phase = 0;
        else if (foot == "l" || foot == "left") t.phase = 1;
        else throw std::runtime_error("load_global_foot_plan: invalid foot label '" +
                                      cells[col["foot"]] + "' at line " + std::to_string(line_no) + " in " + path);

        t.x = get_f("pos_x");
        t.y = get_f("pos_y");
        t.z = get_f("pos_z");
        t.yaw = get_f("yaw");
        t.ssp_t = get_f("ssp_t");
        t.dsp_t = get_f("dsp_t");
        t.height = get_f("height");
        if (has_com_z) t.com_z = get_f("com_z");
        plan.push_back(t);
    }

    if (plan.empty())
        throw std::runtime_error("load_global_foot_plan: no data rows in " + path);

    spdlog::info("[FootCommand/Global] loaded {} global foot targets from '{}' (start foot={})",
                 plan.size(), path, plan.front().phase == 0 ? "R" : "L");
    return plan;
}

// ---------------------------------------------------------------------------
// Factory (selects the source from the deploy.yaml `footstep` node)
// ---------------------------------------------------------------------------
inline std::unique_ptr<FootCommandSource> make_foot_command_source(
    const YAML::Node& fs,
    const std::filesystem::path& proj_dir,
    unitree::common::UnitreeJoystick* joy,
    const FootCommandInput& def,
    float x_scale, float y_scale, float yaw_scale,
    int default_start_phase)
{
    std::string src = fs["command_source"] ? fs["command_source"].as<std::string>() : "joystick";
    std::transform(src.begin(), src.end(), src.begin(), [](unsigned char c){ return std::tolower(c); });

    if (src == "csv")
    {
        std::filesystem::path csv = fs["csv_path"] ? fs["csv_path"].as<std::string>()
                                                   : std::string("config/footcommands.csv");
        if (csv.is_relative()) csv = proj_dir / csv;
        auto s = std::make_unique<CSVFootCommandSource>(csv.string(), def);
        spdlog::info("[FootCommand] command_source = csv");
        return s;
    }

    if (src != "joystick")
        spdlog::warn("[FootCommand] unknown command_source '{}', defaulting to joystick", src);
    spdlog::info("[FootCommand] command_source = joystick");
    return std::make_unique<JoystickFootCommandSource>(joy, def, x_scale, y_scale, yaw_scale, default_start_phase);
}

} // namespace isaaclab
