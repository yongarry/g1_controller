// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-environment C++ port of
//   tocabi_3d_footstep/mdp/preview_controller/vrp_generator.py :: generate_vrp_online
//
// Generates the VRP (Virtual Repellent Point) reference trajectory and the CoM
// yaw reference in the "first stance foot" frame, used by the preview controller
// to produce the CoM/pelvis reference.

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "isaaclab/utils/math_utils.h"

namespace isaaclab
{

class VrpGenerator
{
public:
    VrpGenerator(float vrp_length, float dt, int num_lookahead_steps, float vrp_height,
                 float vrpx_offset = 0.03f, float vrpy_offset = 0.02f,
                 float cube_diagonal_length = 0.361f)
    : dt_(dt), num_lookahead_(num_lookahead_steps), vrp_height_(vrp_height),
      vrpx_offset_(vrpx_offset), vrpy_offset_(vrpy_offset),
      cube_diagonal_length_(cube_diagonal_length)
    {
        NL_ = static_cast<int>(vrp_length / dt_);
        vrp_ref_firststance.assign(NL_, math::Vec3::Zero());
        CoM_yaw_ref_firststance.assign(NL_, 0.0f);
        CoM_yawvel_ref_firststance.assign(NL_, 0.0f);
        step_z.assign(num_lookahead_, 0.0f);
    }

    int NL() const { return NL_; }

    // footstep_cmd: num_lookahead x 9  (x,y,z,r,p,yaw,ssp_t,dsp_t,height)
    // vrp_state_init: 3   (zmp x,y; com z) in first-stance frame
    // com_yaw_init: scalar
    // swing_foot_start: 3 (current swing foot pos in first-stance frame)
    // com_z_command: num_lookahead+1
    void generate_vrp_online(const std::vector<std::array<float, 9>>& footstep_cmd,
                             const math::Vec3& vrp_state_init,
                             float com_yaw_init,
                             const math::Vec3& swing_foot_start,
                             const std::vector<float>& com_z_command)
    {
        const int LA = num_lookahead_;
        std::vector<float> step_x(LA), step_y(LA), step_yaw(LA), ssp_t(LA), dsp_t(LA), phase_ind(LA);
        std::vector<float> com_z(LA + 1);
        for (int s = 0; s < LA; ++s)
        {
            step_x[s]   = footstep_cmd[s][0];
            step_y[s]   = footstep_cmd[s][1];
            step_z[s]   = footstep_cmd[s][2];
            step_yaw[s] = footstep_cmd[s][5];
            ssp_t[s]    = footstep_cmd[s][6];
            dsp_t[s]    = footstep_cmd[s][7];
            phase_ind[s] = (step_y[s] > 0.0f) ? 1.0f : ((step_y[s] < 0.0f) ? -1.0f : 0.0f);
        }
        for (int i = 0; i <= LA; ++i) com_z[i] = com_z_command[i];

        // target foot positions in first-stance frame: [x,y,z,yaw]
        std::vector<Eigen::Vector4f> stance(LA, Eigen::Vector4f::Zero());
        std::vector<Eigen::Vector4f> swing(LA, Eigen::Vector4f::Zero());

        // first step
        stance[0].setZero();
        stance[0][2] = vrp_height_ + com_z[0];
        swing[0][0] = stance[0][0] + step_x[0];
        swing[0][1] = stance[0][1] + step_y[0];
        {
            const float d = std::hypot(swing[0][0] - swing_foot_start[0], swing[0][1] - swing_foot_start[1]);
            if (d < cube_diagonal_length_) step_z[0] = swing_foot_start[2];
        }
        swing[0][2] = stance[0][2] + step_z[0];
        swing[0][3] = stance[0][3] + step_yaw[0];

        for (int s = 1; s < LA; ++s)
        {
            stance[s] = swing[s - 1];
            const float c = std::cos(stance[s][3]);
            const float sn = std::sin(stance[s][3]);
            swing[s][0] = stance[s][0] + c * step_x[s] - sn * step_y[s];
            swing[s][1] = stance[s][1] + sn * step_x[s] + c * step_y[s];
            swing[s][2] = stance[s][2] + com_z[s + 1];
            float d;
            if (s == 1)
                d = std::hypot(swing[s][0], swing[s][1]);
            else
                d = std::hypot(swing[s][0] - stance[s - 2][0], swing[s][1] - stance[s - 2][1]);
            if (d < cube_diagonal_length_) step_z[s] = -step_z[s - 1];
            swing[s][2] = stance[s][2] + step_z[s];
            swing[s][3] = stance[s][3] + step_yaw[s];
        }

        // VRP horizontal offsets (per step)
        for (int s = 0; s < LA; ++s)
        {
            stance[s][0] -= phase_ind[s] * vrpy_offset_ * std::sin(stance[s][3]);
            stance[s][1] += phase_ind[s] * vrpy_offset_ * std::cos(stance[s][3]);
            stance[s][0] += vrpx_offset_ * std::cos(stance[s][3]);
            stance[s][1] += vrpx_offset_ * std::sin(stance[s][3]);

            swing[s][0] -= -phase_ind[s] * vrpy_offset_ * std::sin(swing[s][3]);
            swing[s][1] += -phase_ind[s] * vrpy_offset_ * std::cos(swing[s][3]);
            swing[s][0] += vrpx_offset_ * std::cos(swing[s][3]);
            swing[s][1] += vrpx_offset_ * std::sin(swing[s][3]);
        }

        // build the per-tick reference over the VRP horizon
        std::fill(vrp_ref_firststance.begin(), vrp_ref_firststance.end(), math::Vec3::Zero());
        std::fill(CoM_yaw_ref_firststance.begin(), CoM_yaw_ref_firststance.end(), 0.0f);
        std::fill(CoM_yawvel_ref_firststance.begin(), CoM_yawvel_ref_firststance.end(), 0.0f);

        int dsp1_start = 0, dsp2_end = 0;
        auto mid3 = [](const Eigen::Vector4f& a, const Eigen::Vector4f& b) {
            return math::Vec3((a[0] + b[0]) * 0.5f, (a[1] + b[1]) * 0.5f, (a[2] + b[2]) * 0.5f);
        };
        for (int s = 0; s < LA; ++s)
        {
            const int dsp1_end = dsp1_start + static_cast<int>(dsp_t[s] / dt_);
            const int ssp_end  = dsp1_end + static_cast<int>(ssp_t[s] / dt_);
            dsp2_end = ssp_end + static_cast<int>(dsp_t[s] / dt_);

            const math::Vec3 stance_p(stance[s][0], stance[s][1], stance[s][2]);
            const math::Vec3 swing_p(swing[s][0], swing[s][1], swing[s][2]);
            const math::Vec3 prev_mid = (s == 0) ? vrp_state_init : mid3(stance[s - 1], swing[s - 1]);
            const float prev_yaw_mid = (s == 0) ? com_yaw_init : 0.5f * (stance[s - 1][3] + swing[s - 1][3]);
            const float cur_yaw_mid = 0.5f * (stance[s][3] + swing[s][3]);

            // DSP1: VRP moves prev_mid -> stance, CoM yaw stays at prev_yaw_mid
            for (int tk = dsp1_start; tk < dsp1_end && tk < NL_; ++tk)
            {
                const float ct = (tk - dsp1_start) * dt_;
                math::Vec3 p;
                for (int i = 0; i < 3; ++i)
                    p[i] = math::cubic(prev_mid[i], 0.0f, stance_p[i], 0.0f, dsp_t[s], ct);
                vrp_ref_firststance[tk] = p;
                CoM_yaw_ref_firststance[tk] = prev_yaw_mid;
                CoM_yawvel_ref_firststance[tk] = 0.0f;
            }
            // SSP: VRP stays at stance, CoM yaw moves prev_yaw_mid -> cur_yaw_mid
            for (int tk = dsp1_end; tk < ssp_end && tk < NL_; ++tk)
            {
                const float ct = (tk - dsp1_end) * dt_;
                vrp_ref_firststance[tk] = stance_p;
                float yv;
                CoM_yaw_ref_firststance[tk] = math::cubic(prev_yaw_mid, 0.0f, cur_yaw_mid, 0.0f, ssp_t[s], ct, &yv);
                CoM_yawvel_ref_firststance[tk] = yv;
            }
            // DSP2: VRP moves stance -> mid(stance,swing), CoM yaw stays at cur_yaw_mid
            for (int tk = ssp_end; tk < dsp2_end && tk < NL_; ++tk)
            {
                const float ct = (tk - ssp_end) * dt_;
                const math::Vec3 mid = mid3(stance[s], swing[s]);
                math::Vec3 p;
                for (int i = 0; i < 3; ++i)
                    p[i] = math::cubic(stance_p[i], 0.0f, mid[i], 0.0f, dsp_t[s], ct);
                vrp_ref_firststance[tk] = p;
                CoM_yaw_ref_firststance[tk] = cur_yaw_mid;
                CoM_yawvel_ref_firststance[tk] = 0.0f;
            }
            dsp1_start = dsp2_end;
        }
        // remaining horizon: hold the last mid point
        const math::Vec3 last_mid = mid3(stance[LA - 1], swing[LA - 1]);
        const float last_yaw_mid = 0.5f * (stance[LA - 1][3] + swing[LA - 1][3]);
        for (int tk = dsp2_end; tk < NL_; ++tk)
        {
            vrp_ref_firststance[tk] = last_mid;
            CoM_yaw_ref_firststance[tk] = last_yaw_mid;
            CoM_yawvel_ref_firststance[tk] = 0.0f;
        }
    }

    // outputs
    std::vector<math::Vec3> vrp_ref_firststance;       // NL
    std::vector<float> CoM_yaw_ref_firststance;        // NL
    std::vector<float> CoM_yawvel_ref_firststance;     // NL
    std::vector<float> step_z;                         // num_lookahead (overlap-adjusted)

private:
    float dt_;
    int num_lookahead_;
    float vrp_height_;
    float vrpx_offset_, vrpy_offset_, cube_diagonal_length_;
    int NL_;
};

} // namespace isaaclab
