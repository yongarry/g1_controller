#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "State_Footstep.h"

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // shutdown() only releases Unitree's own controller; another user process keeps publishing.
        usleep(1.5 * 1e6); // isTimeout() needs >1 s without messages
        if(!lowcmd_sub->isTimeout())
        {
            spdlog::critical("lowcmd is still being published by another process. Exiting.");
            exit(1);
        }
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller (footstep planner based on WM) \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    
    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    if (FSMState::keyboard && !FSMState::keyboard->has_tty())
        spdlog::warn("No controlling TTY; keyboard FSM keys will not work.");

    std::cout << "Joystick: [L2+Up] FixStand, then [R1+X] Stage1 / [R1+Y] Stage2, [Y] walk, [L2+B] Passive.\n";
    std::cout << "Keyboard (type in THIS terminal, not the MuJoCo window):\n";
    std::cout << "  [f] FixStand  [j] Stage1  [k] Stage2  [g] walk  [p] Passive.  Velocity: WASD, yaw: QE.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}

