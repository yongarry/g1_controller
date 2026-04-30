#include <filesystem>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "g1_controller/g1_controller.h"

namespace {
constexpr std::array<const char *, G1_NUM_MOTOR> kG1MotorJointNames = {
    "left_hip_pitch_joint",  "left_hip_roll_joint",      "left_hip_yaw_joint",
    "left_knee_joint",       "left_ankle_pitch_joint",   "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint",     "right_hip_yaw_joint",
    "right_knee_joint",      "right_ankle_pitch_joint",  "right_ankle_roll_joint",
    "waist_yaw_joint",       "waist_roll_joint",         "waist_pitch_joint",
    "left_shoulder_pitch_joint",  "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",           "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",       "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",  "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",     "right_wrist_pitch_joint",  "right_wrist_yaw_joint"};
}

G1Controller::G1Controller(std::string networkInterface)
    : time_(0.0),
      control_dt_(0.002),
      duration_(3.0),
      counter_(0),
      mode_pr_(Mode::PR),
      mode_machine_(0),
      control_us_(static_cast<uint64_t>(std::lround(control_dt_ * 1e6)))
{
    ChannelFactory::Instance()->Init(0, networkInterface);
    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(0.5f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
    if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
    sleep(0.5);
    }
    // create publisher
    motor_cmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    motor_cmd_publisher_->InitChannel();
    // create subscriber
    motor_state_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    motor_state_subscriber_->InitChannel(std::bind(&G1Controller::robotDataHandler, this, std::placeholders::_1), 1);
    imu_state_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imu_state_subscriber_->InitChannel(std::bind(&G1Controller::imuDataHandler, this, std::placeholders::_1), 1);
    custom_ctrl_cmd_subscriber_.reset(new ChannelSubscriber<std_msgs::msg::dds_::String_>(HG_CUSTOM_CTRL_CMD_TOPIC));
    custom_ctrl_cmd_subscriber_->InitChannel(std::bind(&G1Controller::customCtrlCmdHandler, this, std::placeholders::_1), 1);
    // create threads
    compute_thread_ptr_ = CreateRecurrentThreadEx("compute", 11, control_us_, &G1Controller::Compute, this);

    pin_q_index_from_motor_.fill(-1);
    // g1_controller/urdf/g1_29dof.urdf — resolved from this source file (../.. from src/)
    const std::filesystem::path urdf_path =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "urdf" / "g1_29dof.urdf";
    pinocchio_ready_ = initializePinocchioModel(urdf_path.string());
    if (!pinocchio_ready_) {
      std::cerr << "[WARN] Pinocchio is disabled because URDF model init failed." << std::endl;
    }
}

bool G1Controller::initializePinocchioModel(const std::string &urdf_path) {
    if (!std::filesystem::exists(urdf_path)) {
      std::cerr << "[ERROR] URDF file does not exist: " << urdf_path << std::endl;
      return false;
    }

    try {
      pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), pin_model_);
      pin_data_ = std::make_unique<pinocchio::Data>(pin_model_);
      pin_q_ = pinocchio::neutral(pin_model_);
      frame_jacobian_buffer_ = Eigen::MatrixXd::Zero(6, pin_model_.nv);

      for (size_t motor_idx = 0; motor_idx < kG1MotorJointNames.size(); ++motor_idx) {
        const std::string joint_name = kG1MotorJointNames[motor_idx];
        const pinocchio::JointIndex joint_id = pin_model_.getJointId(joint_name);
        if (joint_id == 0) {
          std::cerr << "[ERROR] Joint not found in URDF: " << joint_name << std::endl;
          return false;
        }

        pin_q_index_from_motor_[motor_idx] = static_cast<int>(pin_model_.joints[joint_id].idx_q());
      }

      left_foot_frame_id_ = pin_model_.getFrameId("left_ankle_roll_link");
      right_foot_frame_id_ = pin_model_.getFrameId("right_ankle_roll_link");
      left_hand_frame_id_ = pin_model_.getFrameId("left_hand_palm_link");
      right_hand_frame_id_ = pin_model_.getFrameId("right_hand_palm_link");
      jacobian_target_frame_id_ = left_hand_frame_id_;

      std::cout << "[INFO] Pinocchio initialized: nq=" << pin_model_.nq
                << ", nv=" << pin_model_.nv
                << ", mapped motors=" << G1_NUM_MOTOR << std::endl;
      return true;
    } catch (const std::exception &e) {
      std::cerr << "[ERROR] Failed to initialize Pinocchio model: " << e.what() << std::endl;
      return false;
    }
}

void G1Controller::updateKinematics(const MotorState &motor_state, const ImuState *imu_state) {
    if (!pinocchio_ready_ || !pin_data_) {
      return;
    }

    // Floating base position is kept at origin.
    pin_q_.head<3>().setZero();
    if (imu_state) {
      const Eigen::Vector3d rpy(imu_state->rpy[0], imu_state->rpy[1], imu_state->rpy[2]);
      const Eigen::Matrix3d base_rotation = pinocchio::rpy::rpyToMatrix(rpy);
      const Eigen::Quaterniond base_quat(base_rotation);
      pin_q_.segment<4>(3) << base_quat.x(), base_quat.y(), base_quat.z(), base_quat.w();
    } else {
      pin_q_.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;
    }

    for (size_t i = 0; i < motor_state.q.size(); ++i) {
      const int q_idx = pin_q_index_from_motor_[i];
      if (q_idx >= 0 && q_idx < static_cast<int>(pin_q_.size())) {
        pin_q_[q_idx] = static_cast<double>(motor_state.q[i]);
      }
    }

    pinocchio::forwardKinematics(pin_model_, *pin_data_, pin_q_);
    pinocchio::updateFramePlacements(pin_model_, *pin_data_);
}

void G1Controller::logFramePoseAndJacobian(const std::string &frame_name,
                                           pinocchio::FrameIndex frame_id,
                                           Eigen::MatrixXd &jacobian_buffer) {
    if (frame_id >= pin_model_.frames.size()) {
      std::cerr << "[WARN] frame not found: " << frame_name << std::endl;
      return;
    }

    const auto &frame_pose = pin_data_->oMf[frame_id];
    std::cout << "[Pinocchio] " << frame_name << " xyz: "
              << frame_pose.translation().transpose() << std::endl;

    pinocchio::computeFrameJacobian(pin_model_, *pin_data_, pin_q_, frame_id,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                    jacobian_buffer);
}

void G1Controller::imuDataHandler(const void *message) {
    IMUState_ imu_torso = *(const IMUState_ *)message;
    auto &rpy = imu_torso.rpy();
}

void G1Controller::robotDataHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();  
    imu_state_buffer_.SetData(imu_tmp);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }
}

void G1Controller::customCtrlCmdHandler(const void *message) {
    const auto cmd = *static_cast<const std_msgs::msg::dds_::String_ *>(message);
    if (cmd.data() == "custom_controller_start") {
      std::cout << "custom controller start!" << std::endl;
    }
}

void G1Controller::Compute() {
	auto start_time = std::chrono::steady_clock::now();
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    const std::shared_ptr<const ImuState> imu = imu_state_buffer_.GetData();
    if (!ms) {
        return;
    }

    updateKinematics(*ms, imu.get());
    if (pinocchio_ready_ && (counter_++ % 500 == 0)) {
      logFramePoseAndJacobian("left_foot", left_foot_frame_id_, frame_jacobian_buffer_);
      logFramePoseAndJacobian("right_foot", right_foot_frame_id_, frame_jacobian_buffer_);
    //   logFramePoseAndJacobian("left_hand", left_hand_frame_id_, frame_jacobian_buffer_);
    //   logFramePoseAndJacobian("right_hand", right_hand_frame_id_, frame_jacobian_buffer_);
    //   logFramePoseAndJacobian("jacobian_target", jacobian_target_frame_id_, frame_jacobian_buffer_);
    }

    time_ += control_dt_;
    mode_pr_ = Mode::PR;

    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = 0.0f;
        dds_low_command.motor_cmd().at(i).q() = 0.0f;
        dds_low_command.motor_cmd().at(i).dq() = 0.0f;
        dds_low_command.motor_cmd().at(i).kp() = Kp[i];
        dds_low_command.motor_cmd().at(i).kd() = Kd[i];
    }
    // Keep a symmetric, slightly bent lower-body posture.
    dds_low_command.motor_cmd().at(LeftHipPitch).q() = -0.2f;
    dds_low_command.motor_cmd().at(LeftKnee).q() = 0.5f;
    dds_low_command.motor_cmd().at(LeftAnklePitch).q() = -0.3f;
    dds_low_command.motor_cmd().at(RightHipPitch).q() = -0.2f;
    dds_low_command.motor_cmd().at(RightKnee).q() = 0.5f;
    dds_low_command.motor_cmd().at(RightAnklePitch).q() = -0.3f;

    dds_low_command.motor_cmd().at(LeftShoulderPitch).q() = 0.1f;
    dds_low_command.motor_cmd().at(RightShoulderPitch).q() = 0.1f;
    dds_low_command.motor_cmd().at(LeftShoulderRoll).q() = 0.2f;
    dds_low_command.motor_cmd().at(RightShoulderRoll).q() = -0.2f;
    dds_low_command.motor_cmd().at(LeftElbow).q() = 1.1f;
    dds_low_command.motor_cmd().at(RightElbow).q() = 1.1f;

    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    motor_cmd_publisher_->Write(dds_low_command);
	auto end_time = std::chrono::steady_clock::now();
	if (std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() > static_cast<int64_t>(control_us_)) {
		std::cout << "[ERROR] Control period exceeded: " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() << " us" << std::endl;
	}
}
