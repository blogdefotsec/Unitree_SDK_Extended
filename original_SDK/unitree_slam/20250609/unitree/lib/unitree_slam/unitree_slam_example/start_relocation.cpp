#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/ros2_idl/QtCommand_.hpp>

#define TOPIC "rt/qt_command"
using namespace unitree::robot;

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    ChannelFactory::Instance()->Init(0, argv[1]); // argv[1]：The name of the network card with network segment 123
    ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_> publisher = ChannelPublisherPtr<unitree_interfaces::msg::dds_::QtCommand_>(new ChannelPublisher<unitree_interfaces::msg::dds_::QtCommand_>(TOPIC));
    publisher->InitChannel();
    unitree_interfaces::msg::dds_::QtCommand_ send_msg;

    send_msg.seq_().data() = "index:123;"; // The value 123 is set by the user (1-10000).
    send_msg.command_() = 6;               // 6 is to start relocation command
    send_msg.attribute_() = 0; //GO2: When this value is 1, the XT16 LiDAR node is enabled, and when it is 2, the MID360 LiDAR node is enabled. Please confirm the value based on the actual LiDAR being.  B2:Please provide a fixed value of 0.
    publisher->Write(send_msg);
    std::cout << "send start relocation command" << std::endl;
    sleep(2);

    return 0;
}
