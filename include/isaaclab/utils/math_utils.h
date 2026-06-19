// Copyright (c) 2025, DYROS.
// All rights reserved.
//
// Single-environment (num_envs == 1) C++ ports of the subset of
// `isaaclab.utils.math` used by the G1 footstep command generator.
//
// IMPORTANT: All quaternions use the Isaac Lab convention (w, x, y, z) and are
// stored in Eigen::Quaternionf, which is constructed as Quaternionf(w, x, y, z)
// and whose .coeffs() are (x, y, z, w). Helpers below take/return Quaternionf.

#pragma once

#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

namespace isaaclab
{
namespace math
{

using Vec3 = Eigen::Vector3f;
using Quat = Eigen::Quaternionf;

inline float wrap_to_pi(float a)
{
    // maps angle to (-pi, pi]
    a = std::fmod(a + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (a < 0.0f) a += 2.0f * static_cast<float>(M_PI);
    return a - static_cast<float>(M_PI);
}

// Rotate vector by quaternion: q (x) v
inline Vec3 quat_apply(const Quat& q, const Vec3& v)
{
    return q.normalized() * v;
}

// Rotate vector by inverse quaternion: q^-1 (x) v
inline Vec3 quat_apply_inverse(const Quat& q, const Vec3& v)
{
    return q.normalized().conjugate() * v;
}

inline Quat quat_mul(const Quat& a, const Quat& b)
{
    return (a * b).normalized();
}

inline Quat quat_conjugate(const Quat& q)
{
    return q.conjugate();
}

// Returns the yaw-only quaternion of q (zeroes roll & pitch).
inline Quat yaw_quat(const Quat& q)
{
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const float yaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                 1.0f - 2.0f * (qy * qy + qz * qz));
    const float half = 0.5f * yaw;
    return Quat(std::cos(half), 0.0f, 0.0f, std::sin(half)).normalized();
}

// Intrinsic xyz Euler -> quaternion (matches isaaclab.utils.math.quat_from_euler_xyz)
inline Quat quat_from_euler_xyz(float roll, float pitch, float yaw)
{
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    Quat q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
    return q.normalized();
}

// Returns (roll, pitch, yaw) from a quaternion (matches euler_xyz_from_quat,
// not wrapped). Caller should wrap_to_pi where needed.
inline Vec3 euler_xyz_from_quat(const Quat& q)
{
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const float roll = std::atan2(2.0f * (qw * qx + qy * qz),
                                  1.0f - 2.0f * (qx * qx + qy * qy));
    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = std::clamp(sinp, -1.0f, 1.0f);
    const float pitch = std::asin(sinp);
    const float yaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                 1.0f - 2.0f * (qy * qy + qz * qz));
    return Vec3(roll, pitch, yaw);
}

inline Eigen::Matrix3f matrix_from_quat(const Quat& q)
{
    return q.normalized().toRotationMatrix();
}

// t02, q02 = combine_frame_transforms(t01, q01, t12, q12)
inline void combine_frame_transforms(const Vec3& t01, const Quat& q01,
                                     const Vec3& t12, const Quat& q12,
                                     Vec3& t02, Quat& q02)
{
    t02 = t01 + quat_apply(q01, t12);
    q02 = quat_mul(q01, q12);
}

// position-only variant
inline Vec3 combine_frame_transforms_pos(const Vec3& t01, const Quat& q01, const Vec3& t12)
{
    return t01 + quat_apply(q01, t12);
}

// t12, q12 = subtract_frame_transforms(t01, q01, t02, q02)
inline void subtract_frame_transforms(const Vec3& t01, const Quat& q01,
                                      const Vec3& t02, const Quat& q02,
                                      Vec3& t12, Quat& q12)
{
    const Quat q01_inv = q01.conjugate();
    t12 = quat_apply(q01_inv, t02 - t01);
    q12 = quat_mul(q01_inv, q02);
}

// position-only variant
inline Vec3 subtract_frame_transforms_pos(const Vec3& t01, const Quat& q01, const Vec3& t02)
{
    return quat_apply(q01.conjugate(), t02 - t01);
}

// Cubic spline scalar interpolation (matches preview_controller/vrp_generator cubic()).
// f(0)=start_pos, f(end_time)=end_pos, f'(0)=start_vel, f'(end_time)=end_vel.
// Returns position; optionally fills velocity.
inline float cubic(float start_pos, float start_vel, float end_pos, float end_vel,
                   float end_time, float current_time, float* vel_out = nullptr)
{
    if (end_time <= 1e-9f)
    {
        if (vel_out) *vel_out = end_vel;
        return end_pos;
    }
    const float t = std::clamp(current_time, 0.0f, end_time);
    const float a = (2.0f * (start_pos - end_pos) + start_vel * end_time + end_vel * end_time) / (end_time * end_time * end_time);
    const float b = (3.0f * (end_pos - start_pos) - 2.0f * start_vel * end_time - end_vel * end_time) / (end_time * end_time);
    const float c = start_vel;
    const float d = start_pos;
    if (vel_out) *vel_out = 3.0f * a * t * t + 2.0f * b * t + c;
    return a * t * t * t + b * t * t + c * t + d;
}

} // namespace math
} // namespace isaaclab
