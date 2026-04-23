#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <unistd.h>
#include <thread>

#include <Eigen/Core>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

inline const std::string HG_CMD_TOPIC = "rt/lowcmd";
inline const std::string HG_IMU_TORSO = "rt/secondary_imu";
inline const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

inline constexpr int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
inline const std::array<float, G1_NUM_MOTOR> Kp{
    200, 100, 40, 200, 228.5, 28.5,      // legs
    200, 100, 40, 200, 228.5, 28.5,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
inline const std::array<float, G1_NUM_MOTOR> Kd{
    12.5, 6.3, 2.5, 16.3, 10.81, 1.81,     // legs
    12.5, 6.3, 2.5, 16.3, 10.81, 1.81,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

enum G1JointIndex {
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,
    WaistYaw = 12,
    WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
    LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,  // NOTE INVALID for g1 23dof
    RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

enum class Mode : uint8_t {
    PR = 0,
    AB = 1
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit) CRC32 ^= dwPolynomial;
  
        xbit >>= 1;
      }
    }
    return CRC32;
}

class G1Controller {
public:
    explicit G1Controller(std::string networkInterface);

    void imuDataHandler(const void *message);
    void robotDataHandler(const void *message);
    void Compute();

private:
    bool initializePinocchioModel(const std::string &urdf_path);
    void updateKinematics(const MotorState &motor_state, const ImuState *imu_state);
    void logFramePoseAndJacobian(const std::string &frame_name,
                                 pinocchio::FrameIndex frame_id,
                                 Eigen::MatrixXd &jacobian_buffer);

    double time_;
    double control_dt_;  // [2ms]
    uint64_t control_us_;
    double duration_;    // [3 s]
    int counter_;
    Mode mode_pr_;
    uint8_t mode_machine_;

    DataBuffer<MotorState> motor_state_buffer_;
    DataBuffer<ImuState> imu_state_buffer_;
  
    ChannelPublisherPtr<LowCmd_> motor_cmd_publisher_;
    ChannelSubscriberPtr<LowState_> motor_state_subscriber_;
    ChannelSubscriberPtr<IMUState_> imu_state_subscriber_;
    ThreadPtr compute_thread_ptr_;
  
    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    pinocchio::Model pin_model_;
    std::unique_ptr<pinocchio::Data> pin_data_;
    Eigen::VectorXd pin_q_;
    Eigen::MatrixXd frame_jacobian_buffer_;
    bool pinocchio_ready_ = false;
    std::array<int, G1_NUM_MOTOR> pin_q_index_from_motor_{};
    pinocchio::FrameIndex left_foot_frame_id_ = 0;
    pinocchio::FrameIndex right_foot_frame_id_ = 0;
    pinocchio::FrameIndex left_hand_frame_id_ = 0;
    pinocchio::FrameIndex right_hand_frame_id_ = 0;
    pinocchio::FrameIndex jacobian_target_frame_id_ = 0;
};