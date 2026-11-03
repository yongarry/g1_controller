// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Foot-command sources for State_Footstep, selected via `footstep.command_source`
// ("joystick" | "csv") in deploy.yaml.

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
#include "isaaclab/devices/keyboard/keyboard.h"

namespace isaaclab
{

// Nominal foot-step command (clamped to trained ranges by the planner).
struct FootCommandInput
{
    float step_x = 0.0f;   // forward step length [m]
    float step_y = 0.237f; // nominal lateral step width (magnitude, always positive) [m]
    float step_z = 0.0f;   // step height change (0 for 2d) [m]
    float step_yaw = 0.0f; // per-step turn [rad]
    float ssp_t = 0.75f;   // single support time [s]
    float dsp_t = 0.15f;   // double support time [s]
    float height = 0.075f; // swing apex height [m]
    float com_z = 0.0f;    // com height offset [m]
    // Lateral crab-walk bias [m], positive = +y (left): left swing widens, right swing narrows.
    float lateral_bias = 0.0f;
};

// SWING-foot landing pose for the global command mode, in the initial-stance-foot
// frame (the first stance foot is the world origin).
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

class FootCommandSource
{
public:
    virtual ~FootCommandSource() = default;
    virtual FootCommandInput input() = 0;
    // One row per footstep; empty for an operator-driven source.
    virtual std::vector<FootCommandInput> plan() const { return {}; }
    // 0 for an operator-driven source. A scripted source starts walking on state
    // entry and stops once this many footsteps have been completed.
    virtual int size() const { return 0; }
    // Initial swing-foot phase (0: right swings first, 1: left swings first).
    virtual int start_phase_indicator() const { return 0; }
    virtual const char* name() const = 0;
};

class JoystickFootCommandSource : public FootCommandSource
{
public:
    JoystickFootCommandSource(unitree::common::UnitreeJoystick* joy,
                              const FootCommandInput& def,
                              float x_scale, float y_scale, float yaw_scale,
                              int start_phase,
                              Keyboard* kbd = nullptr)
    : joy_(joy), kbd_(kbd), def_(def), x_(x_scale), y_(y_scale), yaw_(yaw_scale), start_phase_(start_phase) {}

    FootCommandInput input() override
    {
        FootCommandInput in = def_;
        float ly = 0.f, lx = 0.f, rx = 0.f;
        if (joy_)
        {
            ly = joy_->ly();
            lx = joy_->lx();
            rx = joy_->rx();
        }
        if (kbd_) kbd_->apply_stick_axes(ly, lx, rx);
        in.step_x = ly * x_;
        in.step_yaw = -rx * yaw_;
        in.lateral_bias = -lx * y_;
        return in;
    }

    int start_phase_indicator() const override { return start_phase_; }
    const char* name() const override { return "joystick"; }

private:
    unitree::common::UnitreeJoystick* joy_;
    Keyboard* kbd_;
    FootCommandInput def_;
    float x_, y_, yaw_;
    int start_phase_;
};

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

    FootCommandInput input() override { return rows_.front(); }
    std::vector<FootCommandInput> plan() const override { return rows_; }

    int size() const override { return (int)rows_.size(); }

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
                if (!std::isfinite(v) || pos != s.size()) throw std::runtime_error("bad value");
                return v;
            } catch (const std::exception&) {
                throw std::runtime_error("CSVFootCommandSource: invalid/non-finite value for '" + name +
                                         "' at line " + std::to_string(line_no) + " in " + path);
            }
        };

        int line_no = 1;
        while (std::getline(f, line))
        {
            ++line_no;
            const std::string trimmed = trim_(line);
            if (trimmed.empty()) continue;
            std::vector<std::string> cells = split_(line);
            if ((int)cells.size() < (int)header.size())
                throw std::runtime_error("CSVFootCommandSource: too few columns at line " +
                                         std::to_string(line_no) + " in " + path);

            const std::string foot = lower_(cells[col["foot"]]);
            int phase;
            if (foot == "r" || foot == "right") phase = 0;
            else if (foot == "l" || foot == "left") phase = 1;
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
    int start_phase_ = 0;
};

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
            try {
                const std::string& s = cells[col[name]];
                size_t pos = 0;
                const float v = std::stof(s, &pos);
                if (!std::isfinite(v) || pos != s.size()) throw std::runtime_error("bad value");
                return v;
            }
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

inline std::unique_ptr<FootCommandSource> make_foot_command_source(
    const YAML::Node& fs,
    const std::filesystem::path& proj_dir,
    unitree::common::UnitreeJoystick* joy,
    const FootCommandInput& def,
    float x_scale, float y_scale, float yaw_scale,
    int default_start_phase,
    Keyboard* kbd = nullptr)
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
    return std::make_unique<JoystickFootCommandSource>(joy, def, x_scale, y_scale, yaw_scale, default_start_phase, kbd);
}

} // namespace isaaclab
