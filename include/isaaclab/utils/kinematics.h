// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Pinocchio-backed kinematics for the G1 footstep deploy controller.
//
// The model is loaded as a FIXED-BASE model rooted at `pelvis`, so every frame
// placement / velocity / Jacobian returned by this helper is expressed in the
// PELVIS (base) frame. The command generator anchors its "global" frame at the
// pelvis each control tick (see footstep_command.h), so this is exactly what it
// needs.
//
// State-estimation assumption (no floating-base estimator on hardware):
//   - the pelvis is treated as fixed when computing frame/CoM velocities, i.e.
//     base linear & angular velocity are taken to be zero. Reported velocities
//     therefore reflect joint motion only. This is the standard assumption when
//     no base velocity estimate is available; the planted stance foot makes it
//     a good approximation for the quantities the command generator consumes.

#pragma once

#include <map>
#include <string>
#include <vector>
#include <stdexcept>
#include <eigen3/Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "isaaclab/utils/math_utils.h"

namespace isaaclab
{

class Kinematics
{
public:
    enum Side { LEFT = 0, RIGHT = 1 };

    // name_to_sdk: maps every actuated joint URDF name -> Unitree SDK motor index.
    Kinematics(const std::string& urdf_path,
               const std::string& base_link,
               const std::vector<std::string>& lleg_joints,
               const std::vector<std::string>& rleg_joints,
               const std::string& lee_name,
               const std::string& ree_name,
               const std::map<std::string, int>& name_to_sdk)
    {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        ik_data_ = pinocchio::Data(model_);

        q_ = Eigen::VectorXd::Zero(model_.nq);
        v_ = Eigen::VectorXd::Zero(model_.nv);

        // Build pinocchio-config-index -> sdk-motor-index map for all 1-dof joints.
        // model_.joints[1..] are the movable joints (index 0 is the universe).
        for (pinocchio::JointIndex jid = 1; jid < (pinocchio::JointIndex)model_.njoints; ++jid)
        {
            const std::string& jname = model_.names[jid];
            auto it = name_to_sdk.find(jname);
            if (it == name_to_sdk.end())
                continue; // skip joints that aren't actuated motors (e.g. fixed)
            const int qidx = model_.idx_qs[jid];
            qidx_to_sdk_[qidx] = it->second;
        }

        lee_fid_ = model_.getFrameId(lee_name);
        ree_fid_ = model_.getFrameId(ree_name);
        if (lee_fid_ >= (pinocchio::FrameIndex)model_.nframes ||
            ree_fid_ >= (pinocchio::FrameIndex)model_.nframes)
            throw std::runtime_error("Kinematics: end-effector frame not found in URDF.");

        lleg_q_ = resolve_joint_qindices(lleg_joints);
        rleg_q_ = resolve_joint_qindices(rleg_joints);
        (void)base_link; // base is the URDF root; kept for documentation/parity
    }

    // q_sdk / qd_sdk are full 29-dof vectors in Unitree SDK motor order.
    void set_state(const Eigen::VectorXf& q_sdk, const Eigen::VectorXf& qd_sdk)
    {
        for (const auto& kv : qidx_to_sdk_)
        {
            q_(kv.first) = static_cast<double>(q_sdk(kv.second));
            v_(kv.first) = static_cast<double>(qd_sdk(kv.second));
        }
        pinocchio::forwardKinematics(model_, data_, q_, v_);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::centerOfMass(model_, data_, q_, v_, false);
    }

    // ---- queries (all in the pelvis/base frame) ----
    math::Vec3 foot_pos(Side s) const
    {
        const auto& M = data_.oMf[s == LEFT ? lee_fid_ : ree_fid_];
        return M.translation().cast<float>();
    }
    math::Quat foot_quat(Side s) const
    {
        const auto& M = data_.oMf[s == LEFT ? lee_fid_ : ree_fid_];
        Eigen::Quaterniond q(M.rotation());
        return math::Quat(q.w(), q.x(), q.y(), q.z()).cast<float>().normalized();
    }
    math::Vec3 foot_lin_vel(Side s) const
    {
        auto vel = pinocchio::getFrameVelocity(
            model_, data_, s == LEFT ? lee_fid_ : ree_fid_, pinocchio::LOCAL_WORLD_ALIGNED);
        return vel.linear().cast<float>();
    }
    math::Vec3 foot_ang_vel(Side s) const
    {
        auto vel = pinocchio::getFrameVelocity(
            model_, data_, s == LEFT ? lee_fid_ : ree_fid_, pinocchio::LOCAL_WORLD_ALIGNED);
        return vel.angular().cast<float>();
    }
    math::Vec3 com_pos() const { return data_.com[0].cast<float>(); }
    math::Vec3 com_vel() const { return data_.vcom[0].cast<float>(); }

    // Current leg joint angles (6) in the configured leg-joint order.
    Eigen::VectorXf leg_q(Side s) const
    {
        const auto& idx = (s == LEFT) ? lleg_q_ : rleg_q_;
        Eigen::VectorXf out(idx.size());
        for (size_t i = 0; i < idx.size(); ++i) out(i) = static_cast<float>(q_(idx[i]));
        return out;
    }

    // Differential IK (DLS) for one leg, mirroring DifferentialIKController("dls").
    // Targets are in the pelvis/base frame. Returns the 6 joint angles.
    Eigen::VectorXf diff_ik_leg(Side s,
                                const math::Vec3& target_pos_b,
                                const math::Quat& target_quat_b,
                                const Eigen::VectorXf& q_init,
                                int iters, float lambda, float pos_tol) const
    {
        const auto& idx = (s == LEFT) ? lleg_q_ : rleg_q_;
        const pinocchio::FrameIndex fid = (s == LEFT) ? lee_fid_ : ree_fid_;
        const int n = static_cast<int>(idx.size());

        // local working configuration; reuse preallocated ik_data_ (do not
        // disturb the cached query state in data_).
        Eigen::VectorXd q = q_;
        for (int i = 0; i < n; ++i) q(idx[i]) = static_cast<double>(q_init(i));

        pinocchio::Data& data = ik_data_;
        const double lam2 = static_cast<double>(lambda) * static_cast<double>(lambda);

        for (int it = 0; it < iters; ++it)
        {
            pinocchio::forwardKinematics(model_, data, q);
            pinocchio::updateFramePlacements(model_, data);
            const auto& M = data.oMf[fid];

            const Eigen::Vector3d cur_pos = M.translation();
            Eigen::Quaterniond cur_quat(M.rotation());

            Eigen::Vector3d pos_err = target_pos_b.cast<double>() - cur_pos;
            if (pos_err.norm() < pos_tol) break;

            // orientation error as axis-angle (matches compute_pose_error axis_angle)
            Eigen::Quaterniond des_q(target_quat_b.w(), target_quat_b.x(),
                                     target_quat_b.y(), target_quat_b.z());
            Eigen::Quaterniond q_err = des_q * cur_quat.conjugate();
            if (q_err.w() < 0.0) { q_err.coeffs() *= -1.0; }
            Eigen::Vector3d rot_err = axis_angle_from_quat(q_err);

            Eigen::Matrix<double, 6, 1> err;
            err.head<3>() = pos_err;
            err.tail<3>() = rot_err;

            // Full-model frame Jacobian (6 x nv), LOCAL_WORLD_ALIGNED: [linear; angular]
            pinocchio::Data::Matrix6x Jfull(6, model_.nv);
            Jfull.setZero();
            pinocchio::computeJointJacobians(model_, data, q);
            pinocchio::getFrameJacobian(model_, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, Jfull);

            // restrict to leg columns
            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
            for (int i = 0; i < n; ++i) J.col(i) = Jfull.col(idx[i]);

            // DLS:  dq = J^T (J J^T + lam^2 I)^-1 err
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            JJt.diagonal().array() += lam2;
            Eigen::Matrix<double, Eigen::Dynamic, 1> dq = J.transpose() * JJt.ldlt().solve(err);

            for (int i = 0; i < n; ++i) q(idx[i]) += dq(i);
        }

        Eigen::VectorXf out(n);
        for (int i = 0; i < n; ++i) out(i) = math::wrap_to_pi(static_cast<float>(q(idx[i])));
        return out;
    }

private:
    std::vector<int> resolve_joint_qindices(const std::vector<std::string>& names) const
    {
        std::vector<int> idx;
        for (const auto& nm : names)
        {
            if (!model_.existJointName(nm))
                throw std::runtime_error("Kinematics: joint '" + nm + "' not in URDF.");
            const pinocchio::JointIndex jid = model_.getJointId(nm);
            idx.push_back(model_.idx_qs[jid]);
        }
        return idx;
    }

    static Eigen::Vector3d axis_angle_from_quat(const Eigen::Quaterniond& q_in)
    {
        Eigen::Quaterniond q = q_in.normalized();
        if (q.w() < 0.0) q.coeffs() *= -1.0;
        const double mag = q.vec().norm();
        const double half_angle = std::atan2(mag, q.w());
        const double angle = 2.0 * half_angle;
        double factor;
        if (std::abs(angle) > 1e-6) factor = angle / std::sin(half_angle);
        else factor = 2.0; // small-angle limit of angle/sin(half_angle)
        return q.vec() * factor;
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    mutable pinocchio::Data ik_data_; // scratch for differential IK
    Eigen::VectorXd q_, v_;

    std::map<int, int> qidx_to_sdk_; // pinocchio config index -> sdk motor index
    std::vector<int> lleg_q_, rleg_q_;
    pinocchio::FrameIndex lee_fid_, ree_fid_;
};

} // namespace isaaclab
