#include <iostream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_SPORT_STATE "rt/odommodestate"//high frequency
#define TOPIC_SPORT_LF_STATE "rt/lf/odommodestate"//low frequency

class Custom
{
public:
    explicit Custom()
    {}

    ~Custom()
    {}

    void Init();

private:

    /*high frequency message handler function for subscriber*/
    void HighFreOdomMessageHandler(const void* messages);
    /*low frequency message handler function for subscriber*/
    void LowFreOdomMessageHandler(const void* messages);

private:

    unitree_go::msg::dds_::SportModeState_ estimator_state{};
    unitree_go::msg::dds_::SportModeState_ lf_estimator_state{};

    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> estimate_state_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> lf_estimate_state_subscriber;
};


void Custom::Init()
{
    /*create subscriber*/
    estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_STATE));
    estimate_state_subscriber->InitChannel(std::bind(&Custom::HighFreOdomMessageHandler, this, std::placeholders::_1), 1);

    /*create subscriber*/
    lf_estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_LF_STATE));
    lf_estimate_state_subscriber->InitChannel(std::bind(&Custom::LowFreOdomMessageHandler, this, std::placeholders::_1), 1);
}

void Custom::HighFreOdomMessageHandler(const void* message)
{
    estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

    std::cout << "position info: " << std::endl;
    std::cout << "x: " << estimator_state.position()[0] << std::endl;
    std::cout << "y: " << estimator_state.position()[1] << std::endl;
    std::cout << "z: " << estimator_state.position()[2] << std::endl;

    std::cout << "velocity info: " << std::endl;
    std::cout << "x: " << estimator_state.velocity()[0] << std::endl;
    std::cout << "y: " << estimator_state.velocity()[1] << std::endl;
    std::cout << "z: " << estimator_state.velocity()[2] << std::endl;

    std::cout << "eular angle info: " << std::endl;
    std::cout << "x: " << estimator_state.imu_state().rpy()[0] << std::endl;
    std::cout << "y: " << estimator_state.imu_state().rpy()[1] << std::endl;
    std::cout << "z: " << estimator_state.imu_state().rpy()[2] << std::endl;

    std::cout << "yaw speed info: " << std::endl;
    std::cout << estimator_state.yaw_speed() << std::endl;

    std::cout << "Quaternion info: " << std::endl;
    std::cout << "w: " << estimator_state.imu_state().quaternion()[0] << std::endl;
    std::cout << "x: " << estimator_state.imu_state().quaternion()[1] << std::endl;
    std::cout << "y: " << estimator_state.imu_state().quaternion()[2] << std::endl;
    std::cout << "z: " << estimator_state.imu_state().quaternion()[3] << std::endl;
}

void Custom::LowFreOdomMessageHandler(const void* message)
{
    lf_estimator_state = *(unitree_go::msg::dds_::SportModeState_*)message;

    std::cout << "position info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.position()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.position()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.position()[2] << std::endl;

    std::cout << "velocity info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.velocity()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.velocity()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.velocity()[2] << std::endl;

    std::cout << "eular angle info: " << std::endl;
    std::cout << "x: " << lf_estimator_state.imu_state().rpy()[0] << std::endl;
    std::cout << "y: " << lf_estimator_state.imu_state().rpy()[1] << std::endl;
    std::cout << "z: " << lf_estimator_state.imu_state().rpy()[2] << std::endl;

    std::cout << "yaw speed info: " << std::endl;
    std::cout << lf_estimator_state.yaw_speed() << std::endl;

    std::cout << "Quaternion info: " << std::endl;
    std::cout << "w: " << lf_estimator_state.imu_state().quaternion()[0] << std::endl;
    std::cout << "x: " << lf_estimator_state.imu_state().quaternion()[1] << std::endl;
    std::cout << "y: " << lf_estimator_state.imu_state().quaternion()[2] << std::endl;
    std::cout << "z: " << lf_estimator_state.imu_state().quaternion()[3] << std::endl;
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}