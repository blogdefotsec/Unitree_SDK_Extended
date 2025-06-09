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

    // Go to the location with node name 1
    send_msg.seq_().data() = "index:123;";   // The value 123 is set by the user (1-10000).
    send_msg.command_() = 9;                 // 9 is a single node navigation command
    send_msg.attribute_() = 1;
    send_msg.translation_x_() = 1.0;	      //X
    send_msg.translation_y_() = 0.1;	      //Y
    send_msg.translation_z_() = 0.2;         //Z
    send_msg.euler_yaw_() = 0.3;             //Yaw
        
    send_msg.node_edge_name_().push_back(1); // Target node name

    publisher->Write(send_msg);
    std::cout << "send single nav command" << std::endl;
    sleep(2);

    return 0;
}
