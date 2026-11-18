// High-rate log for vibration. The policy runs at step_dt (50 Hz), which aliases
// anything above 25 Hz; arm chatter has been seen near 70-80 Hz. This samples the
// 1 kHz FSM loop (decimated) and writes on a side thread so the control tick
// only copies one LowState under its mutex.

#pragma once

class VibLogger
{
public:
    static VibLogger& instance();

    // Reads config.yaml `vibration_log`. Empty file path disables logging.
    void start();
    // state_id is the FSM id (FSMStringMap). Called from the 1 kHz loop.
    void sample(int state_id);
    void stop();

private:
    VibLogger() = default;
    ~VibLogger();
    VibLogger(const VibLogger&) = delete;
    VibLogger& operator=(const VibLogger&) = delete;

    struct Impl;
    Impl* impl_ = nullptr;
};
