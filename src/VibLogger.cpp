#include "VibLogger.h"

#include "FSM/FSMState.h"
#include "param.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace
{

constexpr int kFsmHz = 1000; // CtrlFSM::dt
constexpr int kQueueCap = 4096;

// SDK motor index order (g1_name_to_sdk).
const char* kJoint[29] = {
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
};

struct Sample
{
    double t = 0;
    uint32_t tick = 0;
    char state[16]{};
    float gyro[3]{};
    float acc[3]{};
    float rpy[3]{};
    float q[29]{};
    float dq[29]{};
    float qdes[29]{};
    float tau[29]{};
};

} // namespace

struct VibLogger::Impl
{
    std::atomic<bool> running{false};
    int decimation = 2;
    int tick_div = 0;
    uint32_t dropped = 0;
    std::chrono::steady_clock::time_point t0{};

    std::mutex mtx;
    std::condition_variable cv;
    std::vector<Sample> queue;
    std::thread writer;
    std::ofstream file;
};

VibLogger& VibLogger::instance()
{
    static VibLogger logger;
    return logger;
}

VibLogger::~VibLogger()
{
    stop();
}

void VibLogger::start()
{
    if (impl_ && impl_->running.load()) return;

    std::string path = "log/vibration.csv";
    int hz = 500;
    const YAML::Node n = param::config["vibration_log"];
    if (n)
    {
        if (n["file"]) path = n["file"].as<std::string>();
        if (n["hz"]) hz = n["hz"].as<int>();
    }
    if (path.empty())
    {
        spdlog::info("[Vib] logging disabled (vibration_log.file is empty)");
        return;
    }
    if (hz < 1) hz = 1;
    if (hz > kFsmHz) hz = kFsmHz;

    std::filesystem::path p = path;
    if (p.is_relative()) p = param::proj_dir / p;
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tm);
    const std::filesystem::path stamped = p.parent_path() / (p.stem().string() + "_" + buf + p.extension().string());
    std::filesystem::create_directories(stamped.parent_path());

    auto impl = std::make_unique<Impl>();
    impl->decimation = std::max(1, (int)std::lround(static_cast<double>(kFsmHz) / hz));
    impl->file.open(stamped.string(), std::ios::out | std::ios::trunc);
    if (!impl->file.is_open())
    {
        spdlog::warn("[Vib] could not open '{}'", stamped.string());
        return;
    }
    impl->file << std::fixed << std::setprecision(5);
    impl->file << "t,tick,state,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,roll,pitch,yaw";
    for (const char* name : kJoint)
    {
        impl->file << ',' << name << "_q," << name << "_dq," << name << "_qdes," << name << "_tau";
    }
    impl->file << '\n';

    impl->queue.reserve(kQueueCap);
    impl->t0 = std::chrono::steady_clock::now();
    const int actual_hz = kFsmHz / impl->decimation;
    impl->running.store(true);
    impl->writer = std::thread([impl_ptr = impl.get()] {
        std::vector<Sample> batch;
        batch.reserve(kQueueCap);
        while (true)
        {
            uint32_t dropped = 0;
            {
                std::unique_lock<std::mutex> lock(impl_ptr->mtx);
                if (impl_ptr->running.load() && impl_ptr->queue.empty())
                    impl_ptr->cv.wait_for(lock, std::chrono::milliseconds(20));
                if (!impl_ptr->running.load() && impl_ptr->queue.empty()) break;
                batch.swap(impl_ptr->queue);
                if (impl_ptr->queue.capacity() < kQueueCap) impl_ptr->queue.reserve(kQueueCap);
                dropped = impl_ptr->dropped;
                impl_ptr->dropped = 0;
            }
            if (dropped > 0)
                spdlog::warn("[Vib] dropped {} samples (disk fell behind)", dropped);
            for (const Sample& s : batch)
            {
                impl_ptr->file << s.t << ',' << s.tick << ',' << s.state << ','
                               << s.gyro[0] << ',' << s.gyro[1] << ',' << s.gyro[2] << ','
                               << s.acc[0] << ',' << s.acc[1] << ',' << s.acc[2] << ','
                               << s.rpy[0] << ',' << s.rpy[1] << ',' << s.rpy[2];
                for (int i = 0; i < 29; ++i)
                {
                    impl_ptr->file << ',' << s.q[i] << ',' << s.dq[i] << ',' << s.qdes[i] << ',' << s.tau[i];
                }
                impl_ptr->file << '\n';
            }
            if (!batch.empty()) impl_ptr->file.flush();
            batch.clear();
        }
    });

    spdlog::info("[Vib] {} Hz (FSM {} Hz / {}) IMU gyro, acc, rpy + joint q/dq/qdes/tau -> '{}'",
                 actual_hz, kFsmHz, impl->decimation, stamped.string());
    spdlog::info("[Vib] units: gyro rad/s, acc m/s^2, rpy rad, q rad, dq rad/s, tau N.m (tau_est). qdes is the motor command.");
    impl_ = impl.release();
}

void VibLogger::sample(int state_id)
{
    Impl* impl = impl_;
    if (!impl || !impl->running.load(std::memory_order_acquire)) return;
    if (!FSMState::lowstate || !FSMState::lowcmd) return;
    if (++impl->tick_div < impl->decimation) return;
    impl->tick_div = 0;

    Sample s;
    s.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - impl->t0).count();
    if (auto it = FSMStringMap.left.find(state_id); it != FSMStringMap.left.end())
        std::strncpy(s.state, it->second.c_str(), sizeof(s.state) - 1);
    {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        const auto& msg = FSMState::lowstate->msg_;
        s.tick = msg.tick();
        const auto& imu = msg.imu_state();
        for (int i = 0; i < 3; ++i)
        {
            s.gyro[i] = imu.gyroscope()[i];
            s.acc[i] = imu.accelerometer()[i];
            s.rpy[i] = imu.rpy()[i];
        }
        const auto& motors = msg.motor_state();
        for (int i = 0; i < 29; ++i)
        {
            s.q[i] = motors[i].q();
            s.dq[i] = motors[i].dq();
            s.tau[i] = motors[i].tau_est();
        }
    }
    for (int i = 0; i < 29; ++i)
        s.qdes[i] = FSMState::lowcmd->msg_.motor_cmd()[i].q();

    {
        std::lock_guard<std::mutex> lock(impl->mtx);
        if ((int)impl->queue.size() >= kQueueCap)
            ++impl->dropped;
        else
            impl->queue.push_back(s);
    }
}

void VibLogger::stop()
{
    if (!impl_) return;
    impl_->running.store(false);
    impl_->cv.notify_all();
    if (impl_->writer.joinable()) impl_->writer.join();
    if (impl_->file.is_open()) impl_->file.close();
    delete impl_;
    impl_ = nullptr;
}
