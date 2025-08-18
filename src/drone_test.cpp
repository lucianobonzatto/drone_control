#include "../include/drone_control.h"
#include "../include/ros_client.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ros_client_node");

    ROSClient ros_client(node);
    DroneControl drone_control(&ros_client);
    rclcpp::spin_some(node);

    drone_control.set_OFFBOARD_Mode();
    drone_control.takeOff();
    drone_control.hover(5);
    drone_control.land();

    return 0;
}
